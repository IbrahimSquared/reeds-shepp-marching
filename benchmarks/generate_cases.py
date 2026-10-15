#!/usr/bin/env python3
"""Generate the frozen, deterministic Reeds-Shepp benchmark case set."""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent
SEED = 20261015
THRESHOLD = 128
RADII = (5, 10, 20)
CSV_FIELDS = (
    "case_id", "map_id", "start_id", "goal_id", "radius_cells",
    "start_x", "start_y", "start_yaw_deg", "goal_x", "goal_y",
    "goal_yaw_deg",
)
MAP_SOURCES = (
    ("waffle", "images/waffle_inflated.jpeg"),
    ("01_u_traps", "images/examples/01_u_traps.png"),
    ("02_t_junctions", "images/examples/02_t_junctions.png"),
    ("03_switchbacks", "images/examples/03_switchbacks.png"),
    ("04_pillar_grid", "images/examples/04_pillar_grid.png"),
    ("05_noisy_pillars", "images/examples/05_noisy_pillars.png"),
    ("06_nonconvex_islands", "images/examples/06_nonconvex_islands.png"),
    ("07_rooms_and_doors", "images/examples/07_rooms_and_doors.png"),
    ("08_large_maze", "images/examples/08_large_maze.png"),
    ("09_large_corrals", "images/examples/09_large_corrals.png"),
)


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def normalized_gray(source: Image.Image) -> np.ndarray:
    rgb = np.asarray(source.convert("RGB"), dtype=np.float64)
    # Match Environment.cpp's integer conversion and strict gray > 128 test.
    gray = 0.3 * rgb[:, :, 0] + 0.59 * rgb[:, :, 1] + 0.11 * rgb[:, :, 2]
    return gray.astype(np.uint8)


def png_bytes(gray: np.ndarray) -> bytes:
    stream = io.BytesIO()
    Image.fromarray(gray, mode="L").save(stream, format="PNG", optimize=False)
    return stream.getvalue()


def largest_component(free: np.ndarray) -> list[tuple[int, int]]:
    height, width = free.shape
    seen = np.zeros(free.shape, dtype=np.bool_)
    best: list[tuple[int, int]] = []
    for y in range(height):
        for x in range(width):
            if not free[y, x] or seen[y, x]:
                continue
            seen[y, x] = True
            queue = deque([(x, y)])
            component = []
            while queue:
                cx, cy = queue.popleft()
                component.append((cx, cy))
                for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                    if 0 <= nx < width and 0 <= ny < height and free[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        queue.append((nx, ny))
            if len(component) > len(best):
                best = component
    return sorted(best, key=lambda point: (point[1], point[0]))


def rows_and_maps() -> tuple[list[dict[str, object]], dict[str, bytes], dict[str, object]]:
    rng = np.random.default_rng(SEED)
    rows: list[dict[str, object]] = []
    normalized: dict[str, bytes] = {}
    map_records = []

    for map_id, source_rel in MAP_SOURCES:
        source_path = ROOT / source_rel
        source_bytes = source_path.read_bytes()
        with Image.open(io.BytesIO(source_bytes)) as source:
            gray = normalized_gray(source)
        height, width = gray.shape
        norm_name = f"maps/{map_id}.png"
        norm_data = png_bytes(gray)
        normalized[norm_name] = norm_data
        free = gray > THRESHOLD
        component = largest_component(free)
        points = np.asarray(component, dtype=np.int32)
        # Uniform rejection sampling avoids a quadratic all-cell eligibility
        # scan while preserving a uniform distribution over eligible starts.
        selected_starts = []
        while len(selected_starts) < 3:
            candidate_index = int(rng.integers(len(component)))
            sx, sy = component[candidate_index]
            if (sx, sy) in selected_starts:
                continue
            distance2 = np.sum((points - np.array([sx, sy])) ** 2, axis=1)
            if int(np.count_nonzero(distance2 >= 20 * 20)) >= 10:
                selected_starts.append((sx, sy))
        for start_number, (sx, sy) in enumerate(selected_starts, start=1):
            distance2 = np.sum((points - np.array([sx, sy])) ** 2, axis=1)
            goal_pool = points[distance2 >= 20 * 20]
            goal_indices = rng.choice(len(goal_pool), size=10, replace=False)
            start_yaw = round(float(rng.uniform(0.0, 360.0)), 6)
            goals = []
            for goal_index in goal_indices:
                gx, gy = map(int, goal_pool[int(goal_index)])
                goal_yaw = round(float(rng.uniform(0.0, 360.0)), 6)
                goals.append((gx, gy, goal_yaw))
            for radius in RADII:
                for goal_number, (gx, gy, goal_yaw) in enumerate(goals, start=1):
                    rows.append({
                        "case_id": f"{map_id}-s{start_number:02d}-g{goal_number:02d}-r{radius:02d}",
                        "map_id": map_id,
                        "start_id": f"{map_id}-s{start_number:02d}",
                        "goal_id": f"{map_id}-s{start_number:02d}-g{goal_number:02d}",
                        "radius_cells": radius,
                        "start_x": sx,
                        "start_y": sy,
                        "start_yaw_deg": f"{start_yaw:.6f}",
                        "goal_x": gx,
                        "goal_y": gy,
                        "goal_yaw_deg": f"{goal_yaw:.6f}",
                    })
        map_records.append({
            "map_id": map_id,
            "source_path": source_rel,
            "source_sha256": sha256(source_bytes),
            "normalized_path": norm_name,
            "normalized_sha256": sha256(norm_data),
            "width": width,
            "height": height,
            "resolution_m_per_cell": 0.05,
            "free_threshold": "gray > 128",
            "largest_4_connected_free_component_cells": len(component),
        })

    csv_stream = io.StringIO(newline="")
    writer = csv.DictWriter(csv_stream, fieldnames=CSV_FIELDS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    csv_data = csv_stream.getvalue().encode("utf-8")
    manifest = {
        "schema": "rs-marching-benchmark-cases-v1",
        "generator_seed": SEED,
        "rng": "numpy.random.default_rng (PCG64)",
        "coordinates": {"x": "right", "y": "down", "yaw_degrees": "0 right, 90 down"},
        "selection_rules": {
            "free_space": "normalized grayscale > 128, matching src/Environment.cpp",
            "component": "largest 4-connected free component; ties use first row-major component",
            "positions": "integer cell centers; no map inflation or occupancy changes",
            "starts": "3 distinct cells per map, uniformly sampled from eligible cells in the selected component",
            "goals": "10 distinct cells per start, sampled uniformly without replacement from the selected component and at least 20 cells Euclidean from the start",
            "yaw": "independent uniform [0,360), rounded to 6 decimal places",
            "radii_cells": list(RADII),
            "pose_pairs": "shared across all radii",
            "counts": {"maps": 10, "starts_per_map": 3, "goals_per_start": 10, "rows": 900},
        },
        "cases_path": "cases.csv",
        "cases_sha256": sha256(csv_data),
        "maps": map_records,
    }
    return rows, normalized, {"csv": csv_data, "manifest": manifest}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify artifacts without writing")
    args = parser.parse_args()
    rows, normalized, generated = rows_and_maps()
    manifest_bytes = (json.dumps(generated["manifest"], indent=2, sort_keys=False) + "\n").encode()
    expected = {OUT / "cases.csv": generated["csv"], OUT / "maps.json": manifest_bytes}
    expected.update({OUT / name: data for name, data in normalized.items()})
    if args.check:
        mismatches = [str(path.relative_to(OUT)) for path, data in expected.items()
                      if not path.is_file() or path.read_bytes() != data]
        if mismatches:
            raise SystemExit("benchmark artifacts differ: " + ", ".join(mismatches))
        print(f"verified {len(rows)} rows, 10 maps, {len(expected)} deterministic artifacts")
    else:
        (OUT / "maps").mkdir(parents=True, exist_ok=True)
        for path, data in expected.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        print(f"wrote {len(rows)} rows, 10 maps, {len(expected)} deterministic artifacts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
