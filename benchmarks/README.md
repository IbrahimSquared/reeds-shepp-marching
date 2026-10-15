# Reproducible planner benchmark

This benchmark compares Reeds–Shepp Marching with the **Nav2 Smac Hybrid A***
and **Nav2 Smac State Lattice** search implementations ([Nav2 1.1.20](https://github.com/ros-navigation/navigation2/tree/1.1.20/nav2_smac_planner)). It measures
path quality and the cost of answering ten goals from each start. All planners
allow forward and reverse motion; no path smoother is applied.

## Results

The [per-case CSV](results/results.csv) contains all **8,100 attempts**; the
[batch CSV](results/batch_timings.csv) records **810 ten-goal batches**. Quality
columns describe validated paths; failures retain their reason and timing.
[Summary and provenance](results/summary.json) bind inputs, code, binaries and data.
All outcomes and recovered paths were identical across the three trials. Every
attempt could be classified as a validated path or a failure.

| Method | Valid paths / 900 | Valid within 4° / 900 | Length (cells) | Length / best observed | Mean cusps | Roughness |
|---|---:|---:|---:|---:|---:|---:|
| Reeds–Shepp Marching | 900 (100.0%) | 900 (100.0%) | 188.70 | 1.000 | 0.594 | 0.171 |
| Smac Hybrid A* | 428 (47.6%) | 428 (47.6%) | 192.61 | 1.017 | 0.675 | 2.803 |
| Smac State Lattice | 443 (49.2%) | 43 (4.8%) | 195.48 | 1.021 | 0.741 | 0.603 |

Length, length ratio and roughness are medians over **320 common valid cases**
in trial 0. Cusps are means over those same cases.
“Valid within 4°” additionally requires start and goal orientation errors of at
most 4° relative to the requested poses. Maximum observed endpoint errors on valid paths were approximately 0°, 2.50° and 13.16°, respectively.
The lengths therefore do not always solve exactly the same boundary conditions.

| Method | Collision | Endpoint | Native motion join | Uncertain clearance | Timeout |
|---|---:|---:|---:|---:|---:|
| Reeds–Shepp Marching | 0 | 0 | 0 | 0 | 0 |
| Smac Hybrid A* | 471 | 1 | 0 | 0 | 0 |
| Smac State Lattice | 433 | 17 | 3 | 3 | 1 |

These are failures of the documented point-vehicle, zero-inflation, unsmoothed
configuration under continuous validation. They do not characterize a typical
Nav2 deployment with footprint checking, inflation and smoothing.

### Timing and memory

Measured on an Intel Core i9-13980HX running Ubuntu 24.04. Marching was built
with GCC 13.3; Nav2 used GCC 11.4 in its pinned Ubuntu 22.04 container. Both
used Release builds (`-O3 -DNDEBUG`).
The planner used P-core CPU14 in trials 0–1 and same-type CPU4 in trial 2.
The harness was initially OS-scheduled and later pinned to CPU0. Four batches
with measured contention were retained separately and rerun. **All 810 accepted
batches still have unknown load classification**: counter resolution could not
certify quiet execution. Treat the following as indicative timings, not a
speedup claim.

| Method | Mean native work (ms/goal) | Mean request/reply (ms/goal) | Median request/reply (ms/goal) | Sampled peak memory (MiB) |
|---|---:|---:|---:|---:|
| Reeds–Shepp Marching | 36.61 | 54.14 | 48.00 | 146.0 |
| Smac Hybrid A* | 48.69 | 51.19 | 33.82 | 233.8 |
| Smac State Lattice | 163.37 | 166.41 | 22.95 | 773.5 |

All costs are amortized over ten goals, including shared setup and failures.
Native work is initialization + marching + native query total for Marching;
for Smac it is setup + search/backtrace. Marching's query total also includes
its collision check and sampling. Request/reply times additionally include
field/trace export, JSON serialization and IPC; they are not pure search times.
Process/container launch and the common post-run audit are excluded. Memory
is a sampled high-water lower bound, including the observer and retained state.

Total request/reply times across the three trials were **48.43/48.72/49.03 s**
for Marching, **45.92/45.73/46.57 s** for Hybrid A*, and
**149.31/148.65/151.35 s** for State Lattice. The low lattice median and higher mean
reflect expensive cases; neither failures nor their runtime were dropped.

## Cases

The ten bundled maps are Waffle and examples 01–09. Each map has three starts and
ten goals per start, reused at minimum turning radii of **5, 10 and 20
cells**: **900 cases per planner**. `cases.csv` fixes every pose before any
planner results are collected. Seed 20261015 selects positions in the largest
four-connected free component and independent continuous orientations. Selection
uses occupancy and a minimum start–goal separation of 20 cells; it does not test
whether a planner can solve a case.

`maps.json` records map dimensions, hashes, normalization and generation rules.
Coordinates are pixel centers: x increases right, y down, and 90° points down.
One cell represents 0.05 m. All methods use a point vehicle and the same occupied
pixel squares, without additional inflation. Nav2 uses two search cells per map
cell to align its integer search states and truncating collision grid with these
pixel centers; obstacle boundaries do not move.

## Planner settings and validation

Marching builds one retained one-to-all field per start/radius and answers the ten
fully specified queries from it. Hybrid A* uses 72 uniform heading bins. State
Lattice uses 16 nonuniform headings and Ackermann primitives generated by Nav2's
pinned generator, with reverse expansion enabled. Requested orientations remain
continuous in the case file. Both Nav2 planners quantize their start and goal
orientations; their actual errors are recorded.

The Nav2 adapters call the search classes directly, without ROS navigation,
footprint inflation or smoothing. Reverse and non-straight penalties are 1,
change and retrospective penalties are 0, obstacle heuristic caching is off,
and analytic expansion is limited to five turning radii. Lookup tables and map
setup are retained across the ten queries, with their initialization charged to
the first query. Hybrid A* retains its native chord-based
search-cost approximation; reported path lengths use physical arcs and straights.
These settings compare unsmoothed paths with distance as the objective; they are not a comparison of
complete navigation stacks or all possible planner configurations. Search has a
10-second limit per Nav2 query. Marching has a 100-second watchdog for its shared
field, matching the total search allowance for ten goals.

A successful result must have a faithfully recovered native motion sequence,
continuous obstacle clearance, the requested minimum turning radius, position
error at most 0.002 cells and faithful orientation at both native endpoints.
For Nav2 these endpoints use the nearest heading bins; Marching uses the requested
continuous orientations. We report start and goal orientation errors against the
original requests, and a second success rate requiring both errors to be at most
4°. Sixteen-heading lattice rounding can reach approximately 13.3°; Hybrid A*
rounding can reach 2.5°. These accuracy differences matter when comparing lengths. Float replay checks use 0.002 cells and 0.0002 radians.
The same continuous collision checker audits all methods, including obstacle
boundary contact. Uncertain checks do not count as successes.

The Nav2 build includes a small observer that preserves original control metadata
before analytic expansion clears it. It does not alter search decisions; its time
and storage are included. Captured controls are usable only while their original
parent, pose and direction remain unchanged. Conflicting final controls are never
replaced. Observable endpoint or motion-join failures retain their witnesses;
physical quality metrics remain unset when the full path is unavailable.

For Nav2 analytic expansions, the evaluator accepts the original OMPL curve only
when it reproduces the native sequence of sampled positions. It does not join
arbitrary adjacent samples with newly planned paths. Search primitives are checked
against native parent records. Unverifiable motion is reported separately from planner failures. Missing or
contradictory records are not repaired by choosing a different path.

## Measurements

- **Success:** geometrically valid paths to native endpoints divided by all
  attempted cases, alongside success within 4° of both requested orientations.
  Timeouts, collision failures and other failures remain in the denominator.
- **Length:** physical arc and straight lengths in map cells. Comparative length
  statistics use cases valid for all three methods, with the reported endpoint
  orientation differences. A shorter path to a quantized pose is not necessarily
  shorter to the original requested pose. The best observed valid length
  is a reference, not a proof of distance optimality.
- **Cusps:** gear changes between forward and reverse motion, ignoring
  primitives of at most `1e-8` cells.
- **Path roughness:** `R²/L × Σ|κᵢ₊₁ − κᵢ|`, summed over adjacent primitives
  in the same gear. Here `R` is the requested minimum turning radius, `L` is
  path length, and steering curvature `κ` is `+1/r`, `0` or `−1/r` for a left
  turn, straight or right turn of radius `r`. Primitives of at most `1e-8` cells
  are ignored. This dimensionless measure is computed from the motion primitives,
  not from plotted samples. It measures steering changes per path length;
  cusps are counted separately. It is not a distance-field Laplacian or a comfort
  model, and should be read alongside path length and cusp count.
- **Runtime:** three serial trials with rotated method order. Field construction
  is charged once to each ten-query batch. Search/setup timings and complete
  request/reply wall time are kept separately; exports and validation costs are
  not described as search time. Marching query search, reconstruction, collision
  checking and sampling are also reported separately. Its native query total
  includes those phases; Nav2 search/setup time excludes adapter output assembly.
  Process/container launch is excluded; map and search setup are included.
  Failures retain their consumed runtime.
- **Memory:** the largest sampled process resident-memory high-water mark,
  including retained fields and search storage. The same 50 ms sampling applies
  to all methods; a process that exits between samples can leave an underestimated
  peak. Missing measurements remain explicit. CPU, memory and swap samples accompany each batch; GPU snapshots are advisory.

Trial 0 defines the quality table; subsequent trials check repeatability and
provide timing samples. The report retains per-trial total elapsed times and
flags cases whose outcomes or recovered paths change between trials. Mean
amortized time includes every batch, including expensive failures; the median
is reported separately. Individual cases and failures remain available for
inspection. No cases are removed because a method performs poorly.

## Reproduce a run

Start from the repository root on Ubuntu 24.04 with CMake, a C++20 compiler,
Boost headers, SFML 2.5+, Python 3.12 with `venv`, and Docker. The root
README's installation section lists the native packages. Benchmark scoring
uses the exact Python versions in `benchmarks/requirements.txt`.

```sh
sudo apt-get update
sudo apt-get install -y build-essential cmake libboost-dev libsfml-dev python3-venv
```

Build the marching JSON API, the native geometry-audit executable, install the
pinned Python packages, and build the Nav2 1.1.20 adapter image:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target RS_marching_api -j2

cmake -S benchmarks -B build/benchmark-audit -DCMAKE_BUILD_TYPE=Release
cmake --build build/benchmark-audit -j2

python3 -m venv build/benchmark-venv
build/benchmark-venv/bin/python -m pip install -r benchmarks/requirements.txt

docker build -f benchmarks/nav2/Dockerfile \
  -t rs-marching-benchmark:nav2-observed-1.1.20 benchmarks/nav2
```

The Dockerfile pins the ROS Humble base image and Nav2 package versions. The
runner uses this image by default; it starts an isolated container for each
group and planner. The committed lattice primitive files are the run inputs.
The generator and control checks verify their provenance and geometry. Before
timing, verify the cases and run the controls:

```sh
build/benchmark-venv/bin/python benchmarks/generate_cases.py --check
build/benchmark-venv/bin/python benchmarks/controls.py --api "$PWD/build/RS_marching_api"
```

Run all three serial trials. `--api` must be an absolute executable path;
`--cpu` selects the logical CPU used by the planner process (the default is
14, which must be available on the host). Different trials can use another quiet
core of the same type; each result records its CPU number. A trial resumes with
its original CPU setting. Measured host or selected-core
contention stops the run; repeat the same command once the machine is quiet.
Ambiguous short measurements retain their cases and are marked unknown:

```sh
for trial in 0 1 2; do
  build/benchmark-venv/bin/python benchmarks/run.py \
    --api "$PWD/build/RS_marching_api" \
    --output "build/benchmark-runs/trial-$trial" \
    --trial "$trial" --cpu 14 || break
done
```

Each trial directory contains a manifest, host telemetry, failure summary,
stderr logs and compressed JSONL records for each map/start/radius/planner
group. The records preserve requests, replies and per-call IPC wall time. Run
the scorer over the three run directories after all trials complete:

```sh
build/benchmark-venv/bin/python benchmarks/score.py \
  --runs build/benchmark-runs/trial-0 \
         build/benchmark-runs/trial-1 \
         build/benchmark-runs/trial-2 \
  --output build/benchmark-results
```

The scorer writes `results.csv` with one row per case, method and trial,
including planner status, validation, path-quality measures, timing and memory.
`batch_timings.csv` reports shared field-build and ten-query batch timings,
separating native timings from request/reply IPC time. The shared field time is
repeated in query rows for reference; use the batch table to charge it once.
`summary.json` records
input bindings and the trial-0 aggregate table. These generated files stay under
`build/`, which is ignored by Git. A limited run is for development only; the
scorer requires all three complete trials for a full comparison.

## Reproduce the figures

The main README shows one case per map where all three methods pass validation,
selected by the longest Marching path within that group. Two additional cases
show collision failures for both Nav2 methods and valid Marching paths. These
selections illustrate paths and failure locations; use the full results table
for success rates. Arrows show requested orientations, including for Nav2 paths
whose actual endpoint orientations are quantized.

[`examples.json`](examples.json) stores the 36 plotted motion sequences, case
IDs, validation results, collision witnesses and source hashes from trial 0.
Rendering checks each path hash against `results/results.csv`; paths are sampled
from their recorded arcs and straights, without replanning or smoothing.
With the benchmark Python dependencies installed:

```sh
build/benchmark-venv/bin/python benchmarks/plot_examples.py
```

PNG, SVG and PDF files are written to `build/benchmark-figures/`. To rebuild the
small figure dataset from a complete recorded trial, use:

```sh
build/benchmark-venv/bin/python benchmarks/plot_examples.py \
  --raw-trial build/benchmark-runs/trial-0/trial-0
```

Rebuilding also requires the benchmark Docker image and audit executable. It
repeats native-motion decoding and continuous validation, checking the results
against the committed CSV; it does not rerun the planners or their timings.
