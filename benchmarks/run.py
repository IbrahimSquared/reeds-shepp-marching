#!/usr/bin/env python3
"""Serial, resumable JSONL benchmark orchestrator. It does not score paths."""

import argparse
import csv
import gzip
import hashlib
import json
import os
import queue
import signal
import subprocess
import sys
import platform
import threading
import time
from collections import OrderedDict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "benchmarks"
METHODS = ("marching", "hybrid", "lattice")


def sha(path):
    if not path.is_file(): return None
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""): h.update(block)
    return h.hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def tree_hash(roots):
    files = []
    for root in roots:
        if root.is_file(): files.append(root)
        elif root.is_dir(): files.extend(p for p in root.rglob("*") if p.is_file())
    return digest([(str(p.relative_to(ROOT)), sha(p)) for p in sorted(set(files))])


def _cpu_counters():
    result = {}
    try:
        for line in Path("/proc/stat").read_text().splitlines():
            fields = line.split()
            if fields and fields[0].startswith("cpu") and fields[0][3:].isdigit():
                result[int(fields[0][3:])] = list(map(int, fields[1:]))
    except (OSError, ValueError):
        return None
    return result


def telemetry(gpu=False):
    out = {}
    try: out["proc_stat_cpu"] = list(map(int, next(x for x in Path("/proc/stat").read_text().splitlines()
        if x.startswith("cpu ")).split()[1:]))
    except (OSError, StopIteration, ValueError): out["proc_stat_cpu"] = None
    out["proc_stat_cpus"] = _cpu_counters()
    try: out["proc_meminfo_kib"] = {x[0].rstrip(":"): int(x[1]) for line in Path("/proc/meminfo").read_text().splitlines()
        if len(x := line.split()) > 1 and x[1].isdigit()}
    except OSError: out["proc_meminfo_kib"] = None
    try: out["proc_vmstat_pages"] = {x[0]: int(x[1]) for line in Path("/proc/vmstat").read_text().splitlines()
        if len(x := line.split()) == 2 and x[0] in ("pswpin", "pswpout")}
    except OSError: out["proc_vmstat_pages"] = None
    if gpu:
        try:
            p = subprocess.run(["nvidia-smi", "--query-gpu=index,name,memory.used,memory.total,utilization.gpu",
                "--format=csv,noheader"], capture_output=True, text=True, timeout=3)
            out["nvidia_smi"] = {"returncode": p.returncode, "stdout": p.stdout, "stderr": p.stderr}
        except (OSError, subprocess.TimeoutExpired) as e: out["nvidia_smi"] = {"available": False, "error": str(e)}
    return out


def _cpu_list(spec):
    cpus = set()
    for part in spec.strip().split(","):
        bounds = part.split("-", 1)
        try:
            lo = int(bounds[0]); hi = int(bounds[-1])
            cpus.update(range(lo, hi + 1))
        except ValueError:
            return []
    return sorted(cpus)


def _siblings(cpu):
    try:
        value = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology/thread_siblings_list").read_text()
        return _cpu_list(value)
    except OSError:
        return []


class Sampler:
    def __init__(self, pid, selected_cpu):
        self.pid, self.selected_cpu, self.peak_hwm = pid, selected_cpu, 0
        self.siblings = _siblings(selected_cpu)
        self.stop = threading.Event()
        self.samples = []
        self.thread = threading.Thread(target=self._run, daemon=True); self.thread.start()

    def _run(self):
        path = Path(f"/proc/{self.pid}/status")
        while True:
            try:
                for line in path.read_text().splitlines():
                    if line.startswith("VmHWM:"): self.peak_hwm = max(self.peak_hwm, int(line.split()[1])); break
            except (OSError, ValueError): return
            try:
                read_begin = time.monotonic()
                stat = Path(f"/proc/{self.pid}/stat").read_text().split(") ", 1)[1].split()
                load = telemetry()
                self.samples.append({"time": time.monotonic(), "process_ticks": int(stat[11])+int(stat[12]),
                                     "read_span_s": time.monotonic()-read_begin, **load})
            except (OSError, ValueError): pass
            if self.stop.wait(.05): return

    def close(self):
        self.stop.set(); self.thread.join(1)
        samples = self.samples
        result = {"samples": samples, "selected_cpu": self.selected_cpu,
                  "physical_core_siblings": self.siblings,
                  "sampled_peak_hwm_kib": self.peak_hwm,
                  "peak_hwm_is_lower_bound": True,
                  "measurement_quality": "insufficient", "measurement_status": "unknown",
                  "contaminated": None, "load_ok": True}
        if len(samples) < 2:
            result["reason"] = "insufficient load samples"
            return result
        first, last = samples[0], samples[-1]
        hz = os.sysconf("SC_CLK_TCK")
        def external(a,b):
            dt = b["time"]-a["time"]
            ca, cb = a["proc_stat_cpu"], b["proc_stat_cpu"]
            if dt <= 0 or ca is None or cb is None: return None
            busy = sum(cb[:8])-sum(ca[:8])-(cb[3]-ca[3])-(cb[4]-ca[4])
            return max(0., (busy-(b["process_ticks"]-a["process_ticks"]))/hz/dt)
        def core_external(a, b):
            dt = b["time"] - a["time"]
            ca, cb = a.get("proc_stat_cpus"), b.get("proc_stat_cpus")
            if dt <= 0 or not self.siblings or self.selected_cpu not in self.siblings or not ca or not cb or any(c not in ca or c not in cb for c in self.siblings): return None
            busy_ticks = 0
            for cpu in self.siblings:
                before, after = ca[cpu], cb[cpu]
                busy_ticks += sum(after[:8]) - sum(before[:8]) - (after[3]-before[3]) - (after[4]-before[4])
            own = b["process_ticks"] - a["process_ticks"]
            return max(0., (busy_ticks-own)/hz/dt)
        average = external(first,last)
        external_windows = [external(a,b) for a,b in zip(samples,samples[20:])]
        external_windows = [x for x in external_windows if x is not None]
        peak = max(external_windows) if external_windows else average
        core_windows = []
        j = 0
        for i, sample in enumerate(samples):
            while j < len(samples) and samples[j]["time"]-sample["time"] < 1.: j += 1
            if j < len(samples) and samples[j]["time"]-sample["time"] <= 1.1:
                value = core_external(sample, samples[j])
                if value is not None: core_windows.append(value)
        core_mean = core_external(first, last)
        core_peak = max(core_windows) if core_windows else None
        mem = [x.get("proc_meminfo_kib") for x in samples]
        available_values = [x["MemAvailable"] for x in mem if x and "MemAvailable" in x]
        available = min(available_values) if available_values else None
        swap_a, swap_b = first.get("proc_vmstat_pages"), last.get("proc_vmstat_pages")
        swap = swap_a is not None and swap_b is not None and swap_b != swap_a
        def interval(a,b,value,cpu_count,aggregate=False):
            if value is None:return None
            dt=b['time']-a['time']
            # USER_HZ counters are independently rounded. Six busy CPU fields
            # and two process fields each contribute up to one tick of error
            # to a difference; also cover the non-atomic read span.
            fields=6 if aggregate else 6*cpu_count
            error=((fields+2)/hz+(cpu_count+1)*(a.get('read_span_s',0.)+b.get('read_span_s',0.)))/dt
            return [max(0.,value-error),value+error]
        host_count=os.cpu_count() or 1
        checks=[('host_mean',interval(first,last,average,host_count,True),2.0),
                ('core_mean',interval(first,last,core_mean,len(self.siblings)),.05)]
        for a,b in zip(samples,samples[20:]):
            checks.append(('host_window',interval(a,b,external(a,b),host_count,True),4.0))
        j=0
        for a in samples:
            while j<len(samples) and samples[j]['time']-a['time']<1.:j+=1
            if j<len(samples) and samples[j]['time']-a['time']<=1.1:
                b=samples[j]
                checks.append(('core_window',interval(a,b,core_external(a,b),len(self.siblings)),.15))
        contaminated=any(bounds is not None and bounds[0]>limit for _,bounds,limit in checks)
        contaminated=contaminated or (available is not None and available<8*1024*1024) or swap
        complete_data=all(x is not None for x in [available,swap_a,swap_b])
        windows_observed=last['time']-first['time']<1. or (
            any(name=='core_window' for name,_,_ in checks) and
            all(b['time']-a['time']<=.1 for a,b in zip(samples,samples[1:])))
        clean=complete_data and windows_observed and all(bounds is not None and bounds[1]<=limit for _,bounds,limit in checks)
        quality='sufficient' if contaminated or clean else 'insufficient'
        result['cpu_resolution_intervals']=[dict(metric=name,lower=bounds[0] if bounds else None,
            upper=bounds[1] if bounds else None,limit=limit) for name,bounds,limit in checks]
        result.update(external_cpu_mean_cores=average, external_cpu_peak_1s_cores=peak,
                      selected_core_external_mean_cores=core_mean,
                      selected_core_external_peak_1s_cores=core_peak,
                      minimum_available_kib=available, swap_changed=swap,
                      measurement_quality=quality,
                      measurement_status="contaminated" if contaminated else ("clean" if quality == "sufficient" else "unknown"),
                      contaminated=contaminated, load_ok=not contaminated)
        return result


class Child:
    def __init__(self, argv, log_path, container_id_file=None):
        self.started = time.monotonic()
        self.p = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1, start_new_session=True)
        self.lines, self.log, self.lock = queue.Queue(), log_path.open("a", encoding="utf-8"), threading.Lock()
        self.threads = [threading.Thread(target=self._stdout, daemon=True),
                        threading.Thread(target=self._stderr, daemon=True)]
        for thread in self.threads: thread.start()
        docker = Path(argv[0]).name in ("docker", "podman")
        self.container_id_file = container_id_file if docker else None
        self.sampler = None  # Attached to the native process after readiness.

    def _stdout(self):
        try:
            for line in self.p.stdout: self.lines.put(line.rstrip("\r\n"))
        finally: self.lines.put(None)

    def _stderr(self):
        for line in self.p.stderr:
            with self.lock: self.log.write(line); self.log.flush()
            sys.stderr.write(line); sys.stderr.flush()

    def request(self, request, timeout):
        began = time.monotonic(); raw_request = json.dumps(request, separators=(",", ":"))
        self.p.stdin.write(raw_request + "\n"); self.p.stdin.flush(); deadline = began + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0: raise TimeoutError(f"request exceeded {timeout:g}s watchdog")
            try: line = self.lines.get(timeout=remaining)
            except queue.Empty: raise TimeoutError(f"request exceeded {timeout:g}s watchdog") from None
            if line is None: raise RuntimeError(f"child exited before reply (status {self.p.poll()})")
            try: reply = json.loads(line)
            except json.JSONDecodeError:
                message = f"[non-JSON stdout saved as stderr] {line}\n"
                with self.lock: self.log.write(message); self.log.flush()
                sys.stderr.write(message); sys.stderr.flush(); continue
            return {"raw_request": raw_request, "raw_reply": line, "reply": reply,
                    "ipc_ms": (time.monotonic() - began) * 1000}

    def kill(self):
        if self.p.poll() is None:
            try: os.killpg(self.p.pid, signal.SIGKILL)
            except ProcessLookupError: pass
        try: self.p.wait(2)
        except subprocess.TimeoutExpired: self.p.kill(); self.p.wait()
        self.cleanup_container()

    def cleanup_container(self):
        if not self.container_id_file or not self.container_id_file.exists(): return
        try: cid = self.container_id_file.read_text().strip()
        except OSError: cid = ""
        if cid:
            try:
                subprocess.run([self.p.args[0], "rm", "-f", cid],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=5)
            except (OSError, subprocess.TimeoutExpired): pass
        self.container_id_file.unlink(missing_ok=True)

    def close(self):
        if self.p.poll() is None:
            try: self.p.stdin.close(); self.p.wait(2)
            except (BrokenPipeError, subprocess.TimeoutExpired): self.kill()
        else:
            try: self.p.stdin.close()
            except (BrokenPipeError, OSError): pass
        for thread in self.threads: thread.join(1)
        self.cleanup_container()
        load = self.sampler.close() if self.sampler else {"load_ok": True, "contaminated": None,
            "measurement_quality": "missing", "measurement_status": "unknown", "reason": "missing process monitor"}
        hwm = self.sampler.peak_hwm if self.sampler else None
        self.log.close()
        return {"exit_code": self.p.poll(), "sampled_peak_hwm_kib": hwm,
                "peak_hwm_is_lower_bound": True, "load": load}


class Recorder:
    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path, self.tmp = path, path.with_suffix(path.suffix + ".partial")
        self.stream = gzip.open(self.tmp, "wt", encoding="utf-8", newline="\n")

    def add(self, value): self.stream.write(json.dumps(value, separators=(",", ":"), ensure_ascii=False) + "\n")
    def finish(self): self.stream.close(); os.replace(self.tmp, self.path)


def completed(path, input_hash):
    try:
        last = None
        with gzip.open(path, "rt", encoding="utf-8") as f:
            for line in f:
                if line.strip(): last = json.loads(line)
        return last if last and last.get("event") == "group_complete" and last.get("input_hash") == input_hash and not last.get("measured_contamination", False) else None
    except (OSError, EOFError, json.JSONDecodeError): return None


def read_cases():
    maps_file, cases_file = BENCH / "maps.json", BENCH / "cases.csv"
    manifest = json.loads(maps_file.read_text())
    if sha(cases_file) != manifest["cases_sha256"]: raise ValueError("cases.csv hash mismatch")
    maps = {x["map_id"]: x for x in manifest["maps"]}; groups = OrderedDict()
    with cases_file.open(newline="") as f:
        for row in csv.DictReader(f):
            for k in ("radius_cells", "start_x", "start_y", "goal_x", "goal_y"): row[k] = int(row[k])
            for k in ("start_yaw_deg", "goal_yaw_deg"): row[k] = float(row[k])
            groups.setdefault((row["map_id"], row["radius_cells"], row["start_id"]), []).append(row)
    if not groups or any(len(rows) != 10 for rows in groups.values()): raise ValueError("expected 10 goals per group")
    for (mid, radius, _), rows in groups.items():
        m = maps[mid]
        for path, expected in ((ROOT / m["source_path"], m["source_sha256"]),
                               (BENCH / m["normalized_path"], m["normalized_sha256"])):
            if sha(path) != expected: raise ValueError(f"map hash mismatch: {path}")
    return manifest, groups, maps


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--api", required=True, help="absolute RS JSONL executable")
    p.add_argument("--nav2-command", help="JSON-array child command (default: isolated Docker)")
    p.add_argument("--output", required=True, type=Path); p.add_argument("--trial", required=True, type=int, choices=range(3))
    p.add_argument("--cpu", type=int, default=14, help="logical CPU for serial planner processes")
    p.add_argument("--limit-groups", type=int); p.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS))
    a = p.parse_args(); api = Path(a.api)
    if not api.is_absolute() or not api.is_file() or not os.access(api, os.X_OK): p.error("--api must be an absolute executable")
    if a.limit_groups is not None and a.limit_groups < 1: p.error("--limit-groups must be positive")
    nav = ["docker", "run", "--rm", "-i", "--network", "none", "--read-only", "--tmpfs", "/tmp",
           "--shm-size=256m", "--cpuset-cpus", str(a.cpu), "-e", "ROS_LOG_DIR=/tmp/roslogs", "-v", f"{ROOT}:{ROOT}:ro", "rs-marching-benchmark:nav2-observed-1.1.20"]
    if a.nav2_command:
        try: nav = json.loads(a.nav2_command)
        except json.JSONDecodeError as e: p.error(f"invalid --nav2-command: {e}")
        if not isinstance(nav, list) or not nav or not all(isinstance(x, str) for x in nav): p.error("--nav2-command must be a JSON string array")

    frozen, groups, maps = read_cases(); items = list(groups.items())[:a.limit_groups]
    order = list(a.methods); n = a.trial % len(order); order = order[n:] + order[:n]
    primitives = {str(BENCH / "primitives" / f"r{r}.json"): sha(BENCH / "primitives" / f"r{r}.json")
                  for (mid, r, sid), rows in items}
    if 'lattice' in a.methods:
        from generate_primitives import validate_heading_connectivity
        for name,value in primitives.items():
            if value is None:raise SystemExit(f'missing lattice input: {name}')
            data=json.loads(Path(name).read_text());metadata=data['lattice_metadata']
            if len(metadata['heading_angles'])!=16:
                raise SystemExit(f'expected validated 16-heading lattice input: {name}')
            graph=validate_heading_connectivity(16,[(p['start_angle_index'],p['end_angle_index']) for p in data['primitives']])
            if not graph['strongly_connected']:
                raise SystemExit(f'disconnected lattice heading graph: {name}')
    binding = {"command": nav}
    if Path(nav[0]).is_file(): binding["binary_sha256"] = sha(Path(nav[0]))
    if Path(nav[0]).name == "docker":
        image = next((x for x in nav if x.startswith("rs-marching-benchmark:") and ":" in x), None)
        if image:
            try:
                q = subprocess.run([nav[0], "image", "inspect", image, "--format", "{{.Id}}"], capture_output=True, text=True, timeout=5)
                binding.update(image=image, image_id=q.stdout.strip() if q.returncode == 0 else None,
                               image_inspect_error=q.stderr.strip() if q.returncode else None)
            except (OSError, subprocess.TimeoutExpired) as e: binding.update(image=image, image_inspect_error=str(e))
    settings = {"cpu": a.cpu, "trial": a.trial, "methods": a.methods, "method_order": order, "limit_groups": a.limit_groups,
        "cases_sha256": frozen["cases_sha256"], "maps_json_sha256": sha(BENCH / "maps.json"),
        "source_tree_sha256": tree_hash([ROOT / "src", ROOT / "include", ROOT / "CMakeLists.txt", BENCH / "nav2", BENCH / "run.py"]),
        "api": str(api.resolve()), "api_sha256": sha(api), "nav2": binding,
        "map_hashes": {k: {"source": v["source_sha256"], "normalized": v["normalized_sha256"]} for k, v in maps.items()},
        "primitive_hashes": primitives}
    run_hash = digest(settings); out = a.output.resolve(); out.mkdir(parents=True, exist_ok=True); mp = out / "manifest.json"
    if mp.exists() and json.loads(mp.read_text()).get("run_hash") != run_hash: raise SystemExit("output directory has different inputs/settings")
    report = {"schema": "rs-marching-benchmark-run-v1", "run_hash": run_hash, "settings": settings,
              "planned_groups": len(items), "completed_group_methods": [], "group_telemetry": [], "status": "running",
              "host": {"platform": __import__("platform").platform(), "cpu": Path("/proc/cpuinfo").read_text(), "logical_cpus": os.cpu_count()}}
    def save():
        tmp = mp.with_suffix(".json.partial"); tmp.write_text(json.dumps(report, indent=2) + "\n"); os.replace(tmp, mp)
    def map_path(mid): return str((BENCH / maps[mid]["normalized_path"]).resolve())
    def march(mid, r, row):
        return {"command": "march", "map_path": map_path(mid), "radius": r,
            "start": {"x": row["start_x"], "y": row["start_y"], "heading_deg": row["start_yaw_deg"]}}
    def query(method, mid, r, row):
        if method == "marching": return {"command": "query", "x": row["goal_x"], "y": row["goal_y"],
            "heading_deg": row["goal_yaw_deg"], "check_collisions": True, "sample_step": .1}
        q = {"method": method, "map": map_path(mid), "radius_cells": r,
            "start": [row["start_x"], row["start_y"], row["start_yaw_deg"] * 3.141592653589793 / 180],
            "goal": [row["goal_x"], row["goal_y"], row["goal_yaw_deg"] * 3.141592653589793 / 180],
            "max_time_s": 10, "bins": 72}
        if method == "lattice": q["primitive_file"] = str((BENCH / "primitives" / f"r{r}.json").resolve())
        return q

    save(); all_failures = []
    for gi, ((mid, radius, sid), rows) in enumerate(items):
        gid = f"{mid}-r{radius:02d}-{sid}"; before = telemetry(gpu=True); method_rows = []
        for oi, method in enumerate(order):
            rel = f"trial-{a.trial}/{gid}-{method}.jsonl.gz"; dest = out / rel
            ih = digest({"rows": rows, "method": method, "trial": a.trial, "order": oi, "run_hash": run_hash})
            old = completed(dest, ih)
            if old:
                errs = old.get("failures", []); all_failures += [{"group_id": gid, "method": method, **e} for e in errs]
                method_rows.append({"group_id": gid, "method": method, "record": rel, "input_hash": ih,
                                    "status": "resumed", "failure_count": len(errs)}); continue
            if dest.exists():
                excluded = out / "excluded" / f"{dest.stem}-{time.time_ns()}.gz"
                excluded.parent.mkdir(exist_ok=True); os.replace(dest, excluded)
            rec = Recorder(dest); log = out / "trial-logs" / f"{gid}-{method}.stderr.log"; log.parent.mkdir(parents=True, exist_ok=True)
            b = {"trial": a.trial, "group_id": gid, "group_index": gi, "method": method,
                 "method_order_index": oi, "input_hash": ih}; errs, proc_summaries, child = [], [], None
            def close(kill=False):
                nonlocal child
                if child:
                    if kill: child.kill()
                    proc_summaries.append(child.close()); child = None
            def start():
                nonlocal child
                if child is not None:
                    close()
                argv = ["taskset", "-c", str(a.cpu), str(api.resolve())] if method == "marching" else list(nav)
                cidfile = None
                if method != "marching" and Path(argv[0]).name in ("docker", "podman"):
                    cidfile = out / "trial-containers" / f"{gid}-{method}.cid"
                    cidfile.parent.mkdir(parents=True, exist_ok=True); cidfile.unlink(missing_ok=True)
                    argv[2:2] = ["--cidfile", str(cidfile)]
                try: child = Child(argv, log, cidfile)
                except Exception as e:
                    errs.append({"request_id": "process_start", "failure": str(e)}); return False
                rec.add({**b, "event": "process_start", "argv": argv,
                         "process_startup_ms": (time.monotonic() - child.started) * 1000})
                if child:
                    # A rejected empty request confirms readiness without planning.
                    warm = {}
                    try:
                        response = child.request(warm, 15); rec.add({**b, "event": "warmup", "request": warm, **response})
                        if response["reply"].get("status") != "error" and response["reply"].get("ok") is not False: raise RuntimeError("readiness request unexpectedly accepted")
                        if cidfile:
                            cid = cidfile.read_text().strip()
                            pid = int(subprocess.check_output([argv[0],"inspect","--format","{{.State.Pid}}",cid],text=True))
                        else: pid=child.p.pid
                        child.sampler=Sampler(pid, a.cpu)
                    except Exception as e:
                        rec.add({**b, "event": "warmup_failure", "request": warm, "failure": str(e)})
                        errs.append({"request_id": "warmup", "failure": str(e)}); close(kill=True); return False
                return True
            missing = method == "lattice" and primitives[str(BENCH / "primitives" / f"r{radius}.json")] is None
            if missing:
                errs.append({"request_id": "primitive_file", "failure": "missing primitive file"})
                for row in rows: rec.add({**b, "event": "goal_result", "case_id": row["case_id"], "status": "missing_primitive_file"})
            else:
                started = start(); field_ok = method != "marching" and started
                field_failure = "field_failed"
                if method == "marching" and started:
                    request = march(mid, radius, rows[0]); began = time.monotonic()
                    try:
                        reply = child.request(request, 100); rec.add({**b, "event": "march", "request": request, **reply})
                        field_ok = bool(reply["reply"].get("ok"))
                        if not field_ok: errs.append({"request_id": "march", "failure": reply["reply"]})
                    except Exception as e:
                        field_failure = "field_timeout" if isinstance(e, TimeoutError) else "field_failed"
                        errs.append({"request_id": "march", "failure": str(e)})
                        rec.add({**b, "event": "march", "request": request, "failure": str(e), "ipc_ms": (time.monotonic()-began)*1000})
                        close(kill=True)
                if method == "marching" and not field_ok:
                    for row in rows:
                        errs.append({"request_id": row["case_id"], "failure": field_failure})
                        rec.add({**b, "event": "goal_result", "case_id": row["case_id"], "status": field_failure})
                else:
                    for row in rows:
                        if child is None or child.p.poll() is not None:
                            if not start():
                                errs.append({"request_id": row["case_id"], "failure": "process_start_failed"})
                                rec.add({**b, "event": "goal_result", "case_id": row["case_id"], "status": "process_start_failed"}); continue
                            if method == "marching":
                                request = march(mid, radius, row)
                                began = time.monotonic()
                                recorded = False
                                try:
                                    reply = child.request(request, 100)
                                    if not reply["reply"].get("ok"):
                                        rec.add({**b, "event": "march_restart", "case_id": row["case_id"], "request": request, **reply,
                                                 "failure": f"field rebuild returned {reply['reply']}"})
                                        recorded = True
                                        raise RuntimeError(reply["reply"])
                                    rec.add({**b, "event": "march_restart", "case_id": row["case_id"], "request": request, **reply})
                                    recorded = True
                                except Exception as e:
                                    if not recorded:
                                        rec.add({**b, "event": "march_restart", "case_id": row["case_id"], "request": request,
                                                 "failure": str(e), "ipc_ms": (time.monotonic()-began)*1000})
                                    errs.append({"request_id": row["case_id"], "failure": f"field_restart: {e}"})
                                    rec.add({**b, "event": "goal_result", "case_id": row["case_id"], "status": "field_restart_failed"}); close(kill=True); continue
                        request = query(method, mid, radius, row); began = time.monotonic()
                        try:
                            reply = child.request(request, 100 if method == "marching" else 15)
                            rec.add({**b, "event": "goal_result", "case_id": row["case_id"], "request": request, **reply})
                            value = reply["reply"]
                            if value.get("status") in ("error", "no_path", "timeout", "max_iterations") or value.get("ok") is False or value.get("success") is False:
                                errs.append({"request_id": row["case_id"], "failure": value})
                        except Exception as e:
                            rec.add({**b, "event": "goal_result", "case_id": row["case_id"], "request": request,
                                     "failure": str(e), "ipc_ms": (time.monotonic()-began)*1000})
                            errs.append({"request_id": row["case_id"], "failure": str(e)}); close(kill=True)
            close()
            status = "failed" if errs else "complete"
            measured_contamination = any(x["load"].get("contaminated") is True for x in proc_summaries)
            load_ok = not measured_contamination
            qualities = [x["load"].get("measurement_quality", "missing") for x in proc_summaries]
            measurement_quality = "insufficient" if not proc_summaries or any(q != "sufficient" for q in qualities) else "sufficient"
            rec.add({**b, "event": "group_complete", "status": status, "input_hash": ih, "load_ok": load_ok,
                     "measured_contamination": measured_contamination,
                     "measurement_quality": measurement_quality,
                     "processes": proc_summaries, "stderr_log": str(log.relative_to(out)), "failures": errs})
            rec.finish(); all_failures += [{"group_id": gid, "method": method, **e} for e in errs]
            method_rows.append({"group_id": gid, "method": method, "record": rel, "input_hash": ih,
                                "status": status, "failure_count": len(errs), "load_ok": load_ok,
                                "measured_contamination": measured_contamination,
                                "measurement_quality": measurement_quality})
            if measured_contamination:
                report["completed_group_methods"].extend(method_rows)
                report["status"]="stopped_for_host_load"; save()
                print(f"Host-load gate failed: {gid} {method}; record retained. Resume after host is quiet.",flush=True)
                return 2
        report["completed_group_methods"].extend(method_rows)
        report["group_telemetry"].append({"group_id": gid, "before": before, "after": telemetry()})
        report["failures"] = all_failures; save()
        print(f"trial {a.trial}: {gi+1}/{len(items)} groups complete ({gid})", flush=True)
    report["status"] = "complete"; save()
    (out / "failures.json").write_text(json.dumps({"count": len(all_failures), "failures": all_failures}, indent=2) + "\n")
    print(json.dumps({"status": "complete", "groups": len(items), "group_methods": len(report["completed_group_methods"]),
                      "failure_count": len(all_failures), "output": str(out)}))
    return 0


if __name__ == "__main__": raise SystemExit(main())
