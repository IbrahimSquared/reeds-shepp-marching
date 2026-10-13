#!/usr/bin/env python3
"""Serve the local Reeds–Shepp Marching GUI and HTTP API."""
from __future__ import annotations

import argparse
import atexit
from collections import OrderedDict
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from typing import Any
import uuid
import webbrowser

import contourpy
from flask import Flask, Response, jsonify, request, send_file
from matplotlib import colormaps
import numpy as np
from PIL import Image, UnidentifiedImageError
from werkzeug.exceptions import HTTPException

REPO = Path(__file__).resolve().parents[1]
if __package__:
    from .field_metrics import field_metrics, five_point_laplacian
else:
    from field_metrics import field_metrics, five_point_laplacian

MAX_MAP_CELLS = 1_048_576
MAX_MAP_DIMENSION = 2048
MAX_UPLOAD_BYTES = 16 * 1024 * 1024
MAX_SESSION_MAPS = 32
MAX_CONTOUR_POINTS = 600_000
PALETTES = ("jet", "viridis", "turbo", "plasma", "magma", "inferno", "cividis", "gray", "coolwarm")


class ApiError(Exception):
    def __init__(self, code: str, message: str, status: int = 400):
        super().__init__(message)
        self.code, self.message, self.status = code, message, status


def finite_number(value: Any, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ApiError("invalid_input", f"{name} must be a number.")
    try:
        number = float(value)
    except (OverflowError, ValueError):
        raise ApiError("invalid_input", f"{name} must be finite.") from None
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ApiError("invalid_input", f"{name} must be between {minimum:g} and {maximum:g}.")
    return number


def normalized_degrees(value: Any, name: str) -> float:
    heading = math.remainder(finite_number(value, name, -1e9, 1e9), 360.0)
    return -180.0 if heading == 180.0 else heading


def cell_number(value: Any, name: str, extent: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < extent:
        raise ApiError("invalid_cell", f"{name} must be an integer cell in [0, {extent - 1}].")
    return value


def json_object() -> dict:
    if not request.is_json:
        raise ApiError("invalid_content_type", "Send an application/json object.", 415)
    data = request.get_json()
    if not isinstance(data, dict):
        raise ApiError("invalid_input", "Expected a JSON object.")
    return data


@dataclass
class MapRecord:
    id: str
    name: str
    path: Path
    occupancy: np.ndarray
    preparation_ms: float

    def metadata(self) -> dict:
        height, width = self.occupancy.shape
        ys, xs = np.nonzero(self.occupancy)
        center = int(np.argmin((xs - width / 2) ** 2 + (ys - height / 2) ** 2))
        return {"id": self.id, "name": self.name, "width": width, "height": height,
                "free_cells": int(len(xs)), "image_url": f"/api/maps/{self.id}/image",
                "suggested_start": {"x": int(xs[center]), "y": int(ys[center])}}


class MapStore:
    def __init__(self, root: Path):
        self.root = root
        root.mkdir(parents=True, exist_ok=True)
        self.records: dict[str, MapRecord] = {}
        self.lock = threading.RLock()
        self.default = self.add((REPO / "images/waffle_inflated.jpeg").read_bytes(), "Waffle")
        for path in sorted((REPO / "images/examples").glob("[0-9][0-9]_*.png")):
            self.add(path.read_bytes(), path.stem[3:].replace("_", " ").title())

    def add(self, data: bytes, name: str) -> MapRecord:
        started = time.perf_counter()
        if len(data) > MAX_UPLOAD_BYTES:
            raise ApiError("map_too_large", "Map upload exceeds 16 MiB.", 413)
        try:
            with Image.open(io.BytesIO(data)) as image:
                width, height = image.size
                if image.format not in {"PNG", "JPEG", "BMP"}:
                    raise ApiError("invalid_map", "Choose a PNG, JPEG or BMP image.")
                if min(width, height) < 3 or max(width, height) > MAX_MAP_DIMENSION or width * height > MAX_MAP_CELLS:
                    raise ApiError("map_too_large", "Maps must be at least 3×3, at most 2048 per side and at most 1,048,576 cells.", 413)
                pixels = np.asarray(image.convert("RGB"), dtype=np.float64)
        except (UnidentifiedImageError, OSError, ValueError, Image.DecompressionBombError):
            raise ApiError("invalid_map", "The uploaded file is not a supported, readable image.") from None
        # Match Environment::loadImage: integer luminance, strictly greater than 128.
        gray = (0.3 * pixels[:, :, 0] + 0.59 * pixels[:, :, 1] + 0.11 * pixels[:, :, 2]).astype(np.int32)
        occupancy = gray > 128
        if not occupancy.any():
            raise ApiError("invalid_map", "The map contains no free cells.")
        identity = hashlib.sha256(f"{width}x{height}:".encode() + occupancy.tobytes()).hexdigest()[:20]
        with self.lock:
            if identity in self.records:
                return self.records[identity]
            if len(self.records) >= MAX_SESSION_MAPS:
                raise ApiError("map_limit", "This session already contains 32 maps. Restart the GUI for a new session.", 413)
            path = self.root / f"{identity}.png"
            Image.fromarray(occupancy.astype(np.uint8) * 255).save(path)
            record = MapRecord(identity, name[:100] or "Uploaded map", path, occupancy,
                               (time.perf_counter() - started) * 1000)
            self.records[identity] = record
            return record

    def get(self, identity: Any) -> MapRecord:
        if not isinstance(identity, str):
            raise ApiError("invalid_map", "Select a map.")
        with self.lock:
            if identity not in self.records:
                raise ApiError("unknown_map", "This map is not available in the current session.", 404)
            return self.records[identity]


class SolverProcess:
    """One serialized request stream; termination can interrupt a waiting call."""
    def __init__(self, binary: Path, work: Path):
        self.lock = threading.Lock()
        self.close_lock = threading.Lock()
        self.messages: queue.Queue = queue.Queue()
        self.log = (work / "solver.log").open("w")
        environment = {key: value for key, value in os.environ.items() if not key.startswith("RS_")}
        try:
            self.process = subprocess.Popen([str(binary)], cwd=work, env=environment,
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=self.log, text=True, encoding="utf-8", bufsize=1)
        except Exception:
            self.log.close()
            raise
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            for line in self.process.stdout:
                self.messages.put(json.loads(line))
        except Exception as error:
            self.messages.put(error)
        finally:
            self.messages.put(RuntimeError("The solver process stopped."))

    def call(self, message: dict, timeout: float = 120) -> dict:
        with self.lock:
            if self.process.poll() is not None:
                raise ApiError("solver_stopped", "The solver process stopped; march again.", 409)
            try:
                self.process.stdin.write(json.dumps(message, allow_nan=False, separators=(",", ":")) + "\n")
                self.process.stdin.flush()
                response = self.messages.get(timeout=timeout)
            except queue.Empty:
                self.close()
                raise ApiError("solver_timeout", "The solver exceeded its time limit; try a smaller map or another configuration.", 504) from None
            except (BrokenPipeError, OSError, ValueError):
                raise ApiError("solver_stopped", "The solver process stopped; march again.", 409) from None
            if isinstance(response, Exception) or not isinstance(response, dict):
                # A malformed reply or reader failure invalidates the serialized
                # stream; never let a later request consume an ambiguous reply.
                self.close()
                raise ApiError("solver_protocol", "The solver returned an invalid response; see the server log.", 500)
            if not response.get("ok"):
                error = response.get("error", {})
                raise ApiError(error.get("code", "solver_error"), error.get("message", "Solver request failed."))
            return response

    def close(self):
        # Cancellation deliberately does not acquire the request lock: it must
        # interrupt a call waiting for a solver reply. EOF releases the reader.
        with self.close_lock:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=1)
            self.reader.join()
            self.process.stdin.close()
            self.process.stdout.close()
            self.log.close()


@dataclass
class RunRecord:
    id: str
    config: dict
    map: MapRecord
    work: Path
    status: str = "running"
    phase: str = "Initializing and marching"
    error: dict | None = None
    engine: SolverProcess | None = None
    summary: dict | None = None
    distance: np.ndarray | None = None
    fields: bytes | None = None
    contours: OrderedDict = field(default_factory=OrderedDict)
    contour_lock: threading.Lock = field(default_factory=threading.Lock)


class RunManager:
    def __init__(self, root: Path, binary: Path, maps: MapStore, engine_factory=SolverProcess):
        self.root, self.binary, self.maps = root, binary, maps
        self.engine_factory = engine_factory
        self.current: RunRecord | None = None
        self.lock = threading.RLock()
        # Direct one-to-one queries use their own serialized solver process, so
        # they never replace or wait on the retained run.
        self.direct_engine: SolverProcess | None = None
        self.direct_lock = threading.Lock()
        self.workers: set[threading.Thread] = set()
        self.closed = False

    def validate_config(self, data: dict) -> tuple[dict, MapRecord]:
        record = self.maps.get(data.get("map_id"))
        height, width = record.occupancy.shape
        start = data.get("start")
        if not isinstance(start, dict):
            raise ApiError("invalid_input", "Specify a start configuration.")
        x = cell_number(start.get("x"), "Start x", width)
        y = cell_number(start.get("y"), "Start y", height)
        if not record.occupancy[y, x]:
            raise ApiError("occupied_start", "Choose a start in free space.")
        heading = normalized_degrees(start.get("heading_deg"), "Orientation")
        radius = finite_number(data.get("radius"), "Turning radius", 0.05, 10000)
        return {"map_id": record.id, "start": {"x": x, "y": y, "heading_deg": heading}, "radius": radius}, record

    def start(self, data: dict) -> RunRecord:
        config, map_record = self.validate_config(data)
        if not self.binary.is_file():
            raise ApiError("missing_solver", "Build the RS_marching_api target before marching.", 503)
        with self.lock:
            if self.closed:
                raise ApiError("server_closed", "The workspace is shutting down.", 503)
            if self.current and self.current.status == "running":
                raise ApiError("busy", "A march is already running. Cancel it first.", 409)
            if self.current and self.current.engine:
                self.current.engine.close()
            identity = uuid.uuid4().hex[:16]
            work = self.root / "runs" / identity
            work.mkdir(parents=True)
            run = RunRecord(identity, config, map_record, work)
            self.current = run
            (work / "request.json").write_text(json.dumps(config, indent=2) + "\n")
            worker = threading.Thread(target=self._march, args=(run,), daemon=True)
            self.workers.add(worker)
            worker.start()
            return run

    def _march(self, run: RunRecord):
        started = time.perf_counter()
        try:
            engine = self.engine_factory(self.binary, run.work)
            with self.lock:
                if self.current is not run or run.status != "running":
                    engine.close()
                    return
                run.engine = engine
            payload = engine.call({"command": "march", "map_path": str(run.map.path),
                                   "start": run.config["start"], "radius": run.config["radius"]})
            transported_ms = (time.perf_counter() - started) * 1000
            with self.lock:
                if self.current is not run or run.status != "running":
                    return
                run.phase = "Preparing fields and metrics"
            metrics_started = time.perf_counter()
            height, width = run.map.occupancy.shape
            if (payload["width"], payload["height"]) != (width, height):
                raise RuntimeError("Solver map dimensions differ from the selected map.")
            distance = np.asarray(payload.pop("distance"), dtype=np.float64).reshape(height, width)
            distance[~run.map.occupancy] = np.nan
            distance[~np.isfinite(distance) | (np.abs(distance) >= 1e300)] = np.nan
            metrics = field_metrics(distance, run.map.occupancy, ridge_threshold=5.0)
            laplacian, stencil = five_point_laplacian(distance, run.map.occupancy)
            values = distance[np.isfinite(distance)]
            lap_values = np.abs(laplacian[stencil])
            fields = distance.astype("<f8").tobytes() + laplacian.astype("<f8").tobytes()
            cpp_times = payload["timings_ms"]
            times = {key: cpp_times[key] for key in ("initialization", "march", "snapshot")}
            times["transport"] = max(0, transported_ms - cpp_times["total"])
            times["metrics"] = (time.perf_counter() - metrics_started) * 1000
            times["total"] = (time.perf_counter() - started) * 1000
            times["map_preparation"] = run.map.preparation_ms
            summary = {"run_id": run.id, "status": "ready", "phase": "Ready",
                       "map_id": run.map.id, "config": run.config,
                       "width": width, "height": height,
                       "fields_url": f"/api/runs/{run.id}/fields",
                       "distance_range": [float(values.min()), float(values.max())] if values.size else [0, 1],
                       "laplacian_limit": max(1.0, float(np.percentile(lap_values, 98))) if lap_values.size else 1.0,
                       "pivots": payload["pivots"], "effective_radius": payload.get("radius", run.config["radius"]),
                       "metrics": {"timings_ms": times, "field": metrics, "work": payload["metrics"]}}
            (run.work / "result.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
            with self.lock:
                if self.current is run and run.status == "running":
                    run.summary, run.distance, run.fields = summary, distance, fields
                    run.status, run.phase = "ready", "Ready"
        except Exception as error:
            with self.lock:
                if self.current is run and run.status == "running":
                    run.status, run.phase = "failed", "March failed"
                    run.error = {"code": error.code if isinstance(error, ApiError) else "solver_error",
                                 "message": str(error)}
            if run.engine:
                run.engine.close()
            print(f"March {run.id} failed: {error}", file=sys.stderr, flush=True)
        finally:
            with self.lock:
                self.workers.discard(threading.current_thread())

    def get(self, identity: str, ready: bool = False) -> RunRecord:
        with self.lock:
            run = self.current
            if run is None or run.id != identity:
                raise ApiError("stale_run", "This run is no longer active. March the current configuration.", 409)
            if ready and run.status != "ready":
                raise ApiError("not_ready", "Wait for marching to finish before querying this run.", 409)
            return run

    def cancel(self, identity: str):
        with self.lock:
            run = self.get(identity)
            run.status, run.phase = "cancelled", "Cancelled"
            if run.engine:
                run.engine.close()
            run.fields, run.distance, run.summary = None, None, None
            run.contours.clear()

    def query(self, identity: str, data: dict) -> dict:
        started = time.perf_counter()
        run = self.get(identity, ready=True)
        height, width = run.map.occupancy.shape
        x = cell_number(data.get("x"), "Target x", width)
        y = cell_number(data.get("y"), "Target y", height)
        if not run.map.occupancy[y, x]:
            raise ApiError("occupied_target", "The selected target cell is occupied.")
        checking = data.get("check_collisions", True)
        if not isinstance(checking, bool):
            raise ApiError("invalid_input", "check_collisions must be true or false.")
        step = finite_number(data.get("sample_step", 0.5), "Sample step", 0.1, 5)
        command = {"command": "query", "x": x, "y": y,
                   "check_collisions": checking, "sample_step": step}
        if "heading_deg" in data:
            command["heading_deg"] = normalized_degrees(data["heading_deg"], "Goal orientation")
        try:
            response = run.engine.call(command, timeout=15)
        except ApiError as error:
            if error.code in {"solver_timeout", "solver_stopped", "solver_protocol"}:
                engine = run.engine
                with self.lock:
                    if self.current is run and run.status == "ready":
                        run.status, run.phase = "failed", "Solver failed"
                        run.error = {"code": error.code, "message": error.message}
                        run.fields, run.distance, run.summary = None, None, None
                        run.contours.clear()
                if engine:
                    engine.close()
            raise
        self.get(identity, ready=True)
        response["run_id"] = identity
        response["timings_ms"]["api_total"] = (time.perf_counter() - started) * 1000
        return response

    def direct(self, data: dict) -> dict:
        """One start-to-goal query without a retained one-to-all run."""
        started = time.perf_counter()
        config, map_record = self.validate_config(data)
        height, width = map_record.occupancy.shape
        goal = data.get("goal")
        if not isinstance(goal, dict):
            raise ApiError("invalid_input", "Specify a goal configuration.")
        x = cell_number(goal.get("x"), "Goal x", width)
        y = cell_number(goal.get("y"), "Goal y", height)
        if not map_record.occupancy[y, x]:
            raise ApiError("occupied_target", "The selected goal cell is occupied.")
        heading = normalized_degrees(goal.get("heading_deg"), "Goal orientation")
        checking = data.get("check_collisions", True)
        if not isinstance(checking, bool):
            raise ApiError("invalid_input", "check_collisions must be true or false.")
        step = finite_number(data.get("sample_step", 0.5), "Sample step", 0.1, 5)
        order = data.get("order", "dijkstra")
        if order not in ("dijkstra", "astar"):
            raise ApiError("invalid_input", "order must be dijkstra or astar.")
        if not self.binary.is_file():
            raise ApiError("missing_solver", "Build the RS_marching_api target before querying.", 503)
        command = {"command": "direct_query", "map_path": str(map_record.path), "start": config["start"],
                   "radius": config["radius"], "goal": {"x": x, "y": y, "heading_deg": heading},
                   "check_collisions": checking, "sample_step": step, "order": order}
        with self.direct_lock:
            # Publish under the lifecycle lock so close cannot miss an engine
            # whose constructor is still running. Release it before the query:
            # shutdown must be able to interrupt a pending solver response.
            with self.lock:
                if self.closed:
                    raise ApiError("server_closed", "The workspace is shutting down.", 503)
                if self.direct_engine is None:
                    work = self.root / "direct"
                    work.mkdir(parents=True, exist_ok=True)
                    self.direct_engine = self.engine_factory(self.binary, work)
                engine = self.direct_engine
            try:
                response = engine.call(command, timeout=600)
            except ApiError as error:
                if error.code in {"solver_timeout", "solver_stopped", "solver_protocol"}:
                    engine.close()
                    with self.lock:
                        if self.direct_engine is engine:
                            self.direct_engine = None
                raise
        response["config"] = config
        response["timings_ms"]["api_total"] = (time.perf_counter() - started) * 1000
        return response

    def contour_data(self, identity: str, count: int) -> dict:
        run = self.get(identity, ready=True)
        with run.contour_lock:
            if count in run.contours:
                run.contours.move_to_end(count)
                return run.contours[count]
            distance = run.distance
            if distance is None:
                raise ApiError("stale_run", "This run is no longer active.", 409)
            values = distance[np.isfinite(distance)]
            levels = np.linspace(values.min(), values.max(), count) if values.size and values.max() > values.min() else np.empty(0)
            generator = contourpy.contour_generator(z=np.ma.masked_invalid(distance), line_type="Separate", corner_mask=False)
            paths, points = [], 0
            for level in levels:
                for path in generator.lines(float(level)):
                    points += len(path)
                    if points > MAX_CONTOUR_POINTS:
                        raise ApiError("contours_too_large", "Too many contour vertices for interactive display. Reduce the contour count.", 413)
                    paths.append(path.tolist())
            response = {"run_id": identity, "levels": levels.tolist(), "paths": paths}
            self.get(identity, ready=True)
            run.contours[count] = response
            while len(run.contours) > 4:
                run.contours.popitem(last=False)
            return response

    def close(self):
        with self.lock:
            self.closed = True
            if self.current:
                self.cancel(self.current.id)
            workers = tuple(self.workers)
            engine, self.direct_engine = self.direct_engine, None
        # SolverProcess.close interrupts a pending call, so shutdown does not
        # wait for a running direct query to release direct_lock.
        if engine:
            engine.close()
        # A cancelled/replaced run can still be inside Popen before it publishes
        # its engine. Let every worker close that engine before Python exits.
        for worker in workers:
            worker.join()


def create_app(run_root: Path, binary: Path | None = None, engine_factory=SolverProcess) -> Flask:
    run_root = run_root.resolve()
    run_root.mkdir(parents=True, exist_ok=True)
    app = Flask(__name__, static_folder=str(REPO / "gui/static"), static_url_path="/static")
    app.config.update(MAX_CONTENT_LENGTH=MAX_UPLOAD_BYTES, TRUSTED_HOSTS=["localhost", "127.0.0.1", "[::1]"])
    maps = MapStore(run_root / "maps")
    manager = RunManager(run_root, (binary or REPO / "build/RS_marching_api").resolve(), maps, engine_factory)
    app.extensions["rs_manager"] = manager

    @app.before_request
    def local_origin():
        origin = request.headers.get("Origin")
        if origin and origin != f"{request.scheme}://{request.host}":
            raise ApiError("foreign_origin", "This local GUI accepts requests from its own origin.", 403)
        if request.headers.get("Sec-Fetch-Site") == "cross-site":
            raise ApiError("foreign_origin", "This local GUI accepts same-site requests only.", 403)

    @app.after_request
    def headers(response):
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Content-Security-Policy"] = "default-src 'self'; img-src 'self' data: blob:; style-src 'self' 'unsafe-inline'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'"
        if request.path.startswith("/api/"):
            response.headers["Cache-Control"] = "no-store"
        return response

    @app.errorhandler(ApiError)
    def api_error(error):
        return jsonify(error={"code": error.code, "message": error.message}), error.status

    @app.errorhandler(HTTPException)
    def http_error(error):
        return jsonify(error={"code": error.name.lower().replace(" ", "_"), "message": error.description}), error.code

    @app.get("/")
    def index():
        return app.send_static_file("index.html")

    @app.get("/api/bootstrap")
    def bootstrap():
        palettes = [{"name": name, "colors": np.rint(colormaps[name](np.linspace(0, 1, 256))[:, :3] * 255).astype(int).tolist()} for name in PALETTES]
        with maps.lock:
            listing = [item.metadata() for item in maps.records.values()]
        return jsonify(maps=listing, palettes=palettes,
                       defaults={"map_id": maps.default.id, "start": {"x": 40, "y": 600, "heading_deg": 0}, "radius": 10},
                       limits={"map_cells": MAX_MAP_CELLS, "map_dimension": MAX_MAP_DIMENSION,
                               "upload_bytes": MAX_UPLOAD_BYTES, "map_count": MAX_SESSION_MAPS,
                               "contour_count": [2, 150], "radius": [0.05, 10000]})

    @app.post("/api/maps")
    def upload_map():
        record = maps.add(request.get_data(cache=False), request.args.get("name", "Uploaded map"))
        return jsonify(record.metadata()), 201

    @app.get("/api/maps/<identity>/image")
    def map_image(identity):
        return send_file(maps.get(identity).path, mimetype="image/png")

    @app.post("/api/runs")
    def start_run():
        run = manager.start(json_object())
        return jsonify(run_id=run.id, status="running"), 202

    @app.get("/api/runs/<identity>")
    def run_status(identity):
        with manager.lock:
            run = manager.get(identity)
            return jsonify(run.summary or {"run_id": run.id, "status": run.status, "phase": run.phase, "error": run.error})

    @app.delete("/api/runs/<identity>")
    def cancel_run(identity):
        manager.cancel(identity)
        return jsonify(run_id=identity, status="cancelled")

    @app.get("/api/runs/<identity>/fields")
    def fields(identity):
        with manager.lock:
            return Response(manager.get(identity, ready=True).fields, mimetype="application/octet-stream")

    @app.get("/api/runs/<identity>/contours")
    def contours(identity):
        try:
            count = int(request.args.get("count", "45"))
        except ValueError:
            raise ApiError("invalid_input", "Contour count must be an integer.") from None
        if not 2 <= count <= 150:
            raise ApiError("invalid_input", "Contour count must be between 2 and 150.")
        return jsonify(manager.contour_data(identity, count))

    @app.post("/api/runs/<identity>/query")
    def query_path(identity):
        return jsonify(manager.query(identity, json_object()))

    @app.post("/api/direct")
    def direct_query():
        return jsonify(manager.direct(json_object()))

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--binary", type=Path, default=REPO / "build/RS_marching_api")
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--open", action="store_true", help="open the GUI in the default browser")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    root = args.run_dir or REPO / "generated_runs" / (datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S") + "_gui")
    app = create_app(root, args.binary)
    atexit.register(app.extensions["rs_manager"].close)
    url = f"http://127.0.0.1:{args.port}"
    print(f"Reeds–Shepp Marching GUI: {url}\nSession files: {root.resolve()}", flush=True)
    if args.open:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    app.run(host="127.0.0.1", port=args.port, threaded=True, use_reloader=False, debug=False)


if __name__ == "__main__":
    main()
