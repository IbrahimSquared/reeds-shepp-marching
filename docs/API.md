# Reeds–Shepp Marching API

Coordinates are integer cell centers: x right, y down, orientation in degrees with
0 right and 90 down. Radius is in cell units. The interactive interface accepts
radii in `[0.05,10000]` and images up to 1,048,576 cells, with at most 2048 cells
per side. The solver checks the path of a point vehicle; inflate obstacles to
account for the vehicle footprint.

The API uses `heading_deg` and `heading_rad` for orientation in degrees and
radians, respectively. These field names are retained for compatibility.

**Query type** describes the goal: a **goal position (free orientation)** or a **goal configuration (fixed
orientation)**. The response field `query_mode` uses `free_heading` and
`fixed_heading`, respectively. **Solver mode** describes how planning runs:
**One-to-all**, **One-to-one**, or **One-to-one (approximate)**.

## Native JSON-line process

Start `build/RS_marching_api`. Send one JSON object per line; read one response
per line. Keep the process alive for repeated queries. Close stdin to exit.
Diagnostics are written to stderr. Paths in commands must be absolute.

Compute a one-to-all distance field:

```json
{"command":"march","map_path":"/absolute/path/to/map.png","start":{"x":40,"y":600,"heading_deg":0},"radius":10}
```

A successful reply contains `ok`, `width`, `height`, `radius`, `start`,
`distance`, `pivots`, `metrics` and `timings_ms`. `distance` is a row-major array
of `width*height` values; nonfinite entries are `null`. Pivots carry continuous
positions, orientation `heading_rad` and parent IDs. The process uses one start,
fixed deterministic settings and no text-file exports.

Query a goal position with free final orientation:

```json
{"command":"query","x":312,"y":290,"check_collisions":true,"sample_step":0.5}
```

Add `"heading_deg":90` to specify the final orientation. For a free final
orientation, the query compares the stored path with a limited set of
alternatives. For a specified final orientation, it searches a limited set of
connections to reach the goal configuration. Both reuse the distance field
without modifying it.

Run a one-to-one query:

```json
{"command":"direct_query","map_path":"/absolute/path/to/map.png","start":{"x":40,"y":600,"heading_deg":0},"radius":10,"goal":{"x":312,"y":290,"heading_deg":90},"order":"dijkstra","check_collisions":true,"sample_step":0.5}
```

`order` is `dijkstra` for **One-to-one** or `astar` for
**One-to-one (approximate)**. Each query runs a separate march that can stop
before computing the full distance field. It preserves an existing one-to-all
field. These solver modes can return a longer path or miss a path found by a full
one-to-all march. The `direct` object reports cells processed and stopping
information.

`sample_step` must be in `[0.1,5]`. It controls the spacing of points in the
displayed path, without changing continuous collision checking. The native
process accepts finite numeric orientations and reduces them periodically
before converting to radians.

### Query results

A successful request is not necessarily a successful path. Read `status`:

- `clear`: the returned path passed collision checking. For queries with a
  specified final orientation, the required path checks can produce this status
  even when the additional `check_collisions` audit is disabled;
- `reconstructed`: a path is returned without the additional collision audit.
  `collision_checked` reports whether that audit was performed;
- other statuses describe goals outside free space, paths not found by the
  search, paths blocked by obstacles, or numerical uncertainty. They do not
  prove that the goal is unreachable.

Path replies expose `stored_distance`, `reconstructed_length`, `waypoints`,
`primitives`, `polyline`, `reversal_count`, `collision_checked`, collision
results and `timings_ms` as applicable. Queries also report the search limits
and the type of path selected. Orientations in waypoints and sampled geometry
are radians. The actual query length can differ
from the distance-field value at the goal position.

Errors use `{"ok":false,"error":{"code":"...","message":"..."}}`.
Malformed commands do not terminate a healthy process. A `query` before a
successful `march` returns `no_active_run`. Starting a new march replaces the
one-to-all run; callers should treat a failed replacement as requiring a fresh
successful march before further queries.

## HTTP interface

Run `.venv/bin/python gui/server.py`. Options `--port`, `--binary` and `--run-dir`
set the server port, solver executable and session directory. Session files
default to `generated_runs/`. The service binds to loopback and accepts a single
one-to-all run at a time. The browser and solver use the same normalized
binary occupancy. Responses are JSON except image/field routes. Errors use
`{"error":{"code":"...","message":"..."}}` with an HTTP error status.

| Endpoint | Purpose |
|---|---|
| `GET /api/bootstrap` | Available maps, palettes, defaults and limits |
| `POST /api/maps?name=...` | Upload raw PNG/JPEG/BMP bytes; receive map metadata |
| `GET /api/maps/<id>/image` | Normalized map PNG |
| `POST /api/runs` | Start `{map_id,start:{x,y,heading_deg},radius}`; returns HTTP 202 and `run_id` |
| `GET /api/runs/<id>` | Poll status; ready results include metrics, pivots and field URLs |
| `DELETE /api/runs/<id>` | Cancel/discard the one-to-all run |
| `GET /api/runs/<id>/fields` | Distance then signed five-point Laplacian as little-endian Float64 planes |
| `GET /api/runs/<id>/contours?count=45` | Masked field contours, with count in `[2,150]` |
| `POST /api/runs/<id>/query` | `{x,y,heading_deg?,check_collisions,sample_step}` |
| `POST /api/direct` | `{map_id,start,radius,goal,order,check_collisions,sample_step}` |

Field planes each contain `width*height` row-major entries. Invalid, occupied
and unreached values are NaN. Display settings do not remarch the field.
Uploads are limited to 16 MiB; a session keeps at most 32 distinct maps,
including bundled maps. HTTP orientations must be finite numbers in `[-1e9,1e9]`.

A run ID binds responses to their inputs. Busy runs cannot be queried; starting
another run invalidates the previous one. One-to-one queries use a separate
process from the one-to-all run. During dragging, the browser keeps only the
latest pending query in each solver mode and ignores replies for previous inputs.
If the solver process fails, the distance field is discarded and must be
computed again.

Timings separate initialization, marching, path reconstruction, collision
checking, sampling points for display and communication where applicable.

## Command-line configuration and output

`build/RS_marching config/settings.config` computes a one-to-all distance field.
Here, `mapSource` selects the **map source**: `random` generates a random map from
the dimensions, obstacle ranges and seed settings; `image` loads `imagePath`.
It does not select a one-to-one solver mode. The GUI and native JSON-line process take map and query
inputs directly and do not read `settings.config`.

The configuration requires one starting configuration in free space. Relative
image paths are resolved from the working directory. Numeric settings must be
complete values and orientations must be finite. All settings except `imagePath`
allow inline `#` comments; `imagePath` is literal, so put its comments on a
separate line. Replace the old `mode=1` with `mapSource=random` and `mode=2` with
`mapSource=image`; `mode` now produces an error with these replacements. Remove
`bumps`: it has no effect and produces an obsolete-setting warning.

With `saveResults=1`, files are written under `output/`:

| File | Contents |
|---|---|
| `distance.txt` | Distance field |
| `occupancy.txt` | Free/occupied grid |
| `cameFrom.txt`, `origin.txt`, `thetas_.txt` | Data used to reconstruct paths through pivots |
| `lightSources.txt` | Pivot x/y positions and orientations |

Pivot positions may be fractional. The third column of `lightSources.txt` is
physical orientation minus pi/2, in radians. `saveResults=0` preserves existing
output files. Use path queries through the API to obtain motion primitives and
waypoints. A waypoint is a configuration on the returned path; waypoints include
the start, intermediate configurations and the goal. `reversal_count` counts
gear changes between forward and reverse motion.
