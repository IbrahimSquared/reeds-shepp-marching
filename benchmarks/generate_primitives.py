#!/usr/bin/env python3
"""Generate Nav2's Ackermann control sets using the unmodified pinned generator."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys
import urllib.request

NAV2 = 'a097086719c88f781aa59788eca29ac6ca5e56db'
STOPPING_THRESHOLDS = {5: 5, 10: 5, 20: 9}
FILES = ['constants.py', 'helper.py', 'trajectory.py', 'trajectory_generator.py',
         'lattice_generator.py', 'generate_motion_primitives.py']


def validate_heading_connectivity(num_headings, directed_edges):
    """Check strong connectivity for forward and index-shifted reverse edges."""
    if num_headings < 2 or num_headings % 2:
        raise ValueError('heading connectivity requires an even heading count')
    graph = [set() for _ in range(num_headings)]
    forward = set()
    for start, end in directed_edges:
        if start not in range(num_headings) or end not in range(num_headings):
            raise ValueError(f'heading edge outside [0, {num_headings}): {(start, end)}')
        forward.add((start, end))
    for start, end in forward:
        graph[start].add(end)
        graph[(start + num_headings // 2) % num_headings].add(
            (end + num_headings // 2) % num_headings)
    unreachable = {}
    for source in range(num_headings):
        reached = {source}
        pending = [source]
        while pending:
            for target in sorted(graph[pending.pop()] - reached):
                reached.add(target)
                pending.append(target)
        if len(reached) != num_headings:
            unreachable[str(source)] = sorted(set(range(num_headings)) - reached)
    return {
        'num_headings': num_headings,
        'forward_edges': len(forward),
        'directed_edges_including_reverse': sum(len(edges) for edges in graph),
        'strongly_connected': not unreachable,
        'unreachable_by_start_heading': unreachable,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--radii', nargs='+', type=int, default=[5, 10, 20])
    ap.add_argument('--stopping-threshold', type=int, help='override the radius-specific generation threshold')
    ap.add_argument('--rejected-output', type=Path, default=Path('build/benchmark-generator/rejected'))
    ap.add_argument('--cache', type=Path, default=Path('build/benchmark-generator'))
    ap.add_argument('--output', type=Path, default=Path('benchmarks/primitives'))
    args = ap.parse_args()
    args.cache.mkdir(parents=True, exist_ok=True)
    args.output.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for name in FILES:
        p = args.cache / name
        url = f'https://raw.githubusercontent.com/ros-navigation/navigation2/{NAV2}/nav2_smac_planner/lattice_primitives/{name}'
        if not p.exists():
            p.write_bytes(urllib.request.urlopen(url).read())
        hashes[name] = hashlib.sha256(p.read_bytes()).hexdigest()
    sys.path.insert(0, str(args.cache.resolve()))
    generator = importlib.import_module('generate_motion_primitives')
    from lattice_generator import LatticeGenerator
    candidates = []
    connectivity = {}
    for radius in args.radii:
        config = dict(motion_model='ackermann', turning_radius=radius * .05,
                      grid_resolution=.025, stopping_threshold=args.stopping_threshold or STOPPING_THRESHOLDS[radius], num_of_headings=16)
        print(f'Generating R={radius} cells', flush=True)
        primitives = LatticeGenerator(config).run()
        headings = generator.create_heading_angle_list(primitives)
        if len(headings) != config['num_of_headings']:
            raise RuntimeError(f'R={radius}: expected 16 headings, got {len(headings)}')
        heading_index = {angle: index for index, angle in enumerate(headings)}
        edges = [(heading_index[trajectory.parameters.start_angle],
                  heading_index[trajectory.parameters.end_angle])
                 for trajectories in primitives.values() for trajectory in trajectories]
        report = validate_heading_connectivity(config['num_of_headings'], edges)
        connectivity[str(radius)] = report
        print(f'R={radius}: heading connectivity {report}', flush=True)
        if not report['strongly_connected']:
            rejected = args.rejected_output / f'primitives16_rejected_r{radius}.json'
            args.rejected_output.mkdir(parents=True, exist_ok=True)
            generator.write_to_json(rejected, primitives, config)
            data = json.loads(rejected.read_text())
            data.pop('date_generated', None)
            rejected.write_text(json.dumps(data, separators=(',', ':')) + '\n')
            (args.rejected_output / f'primitives16_rejected_r{radius}.connectivity.json').write_text(
                json.dumps(report, indent=2) + '\n')
            raise SystemExit(f'R={radius}: directed heading graph is not strongly connected; '
                             f'rejected output saved to {rejected}; stopping without raising threshold')
        candidates.append((radius, config, primitives))

    # Accept output only after all requested radii pass the structural gate.
    for radius, config, primitives in candidates:
        path = args.output / f'r{radius}.json'
        generator.write_to_json(path, primitives, config)
        data = json.loads(path.read_text())
        data.pop('date_generated', None)
        path.write_text(json.dumps(data, separators=(',', ':')) + '\n')
        print(f'{path}: {len(data["primitives"])} primitives', flush=True)
    (args.output / 'source.json').write_text(json.dumps(dict(nav2_commit=NAV2,
        generator_sha256=hashes, resolution_m=.05, search_resolution_m=.025,
        headings=16, stopping_thresholds={str(r):c['stopping_threshold'] for r,c,p in candidates},
        directed_heading_connectivity=connectivity), indent=2) + '\n')


if __name__ == '__main__':
    main()
