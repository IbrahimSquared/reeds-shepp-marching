#!/usr/bin/env python3
"""Render saved benchmark motions. --raw-trial optionally rebuilds the small snapshot."""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / 'benchmarks'
METHODS = ('marching', 'hybrid', 'lattice')
LABELS = ('Marching', 'Hybrid A*', 'State Lattice')
COLORS = ('#0072b2', '#d55e00', '#9b479f')
STYLES = ('-', '--', ':')
FAILURES = ('03_switchbacks-s02-g02-r20', '07_rooms_and_doors-s03-g06-r20')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def path_hash(ps):
    return hashlib.sha256(json.dumps(ps, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def rows_by_case():
    groups = {}
    with (BENCH / 'results/results.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['trial'] == '0':
                groups.setdefault(row['case_id'], {})[row['method']] = row
    return groups


def extract(raw):
    # Decode using the same native-motion reader, then independently recheck the
    # saved CSV path hash and continuous-validation result. No planner is rerun.
    from controls import Channel, nav2_command
    from geometry import marching_primitives, nav2_primitives
    from score import validate
    groups = rows_by_case()
    overview = []
    for mid in ['waffle'] + sorted({g['marching']['map_id'] for g in groups.values()} - {'waffle'}):
        candidates = [g for g in groups.values() if g['marching']['map_id'] == mid
                      and all(g[m]['success'] == 'True' for m in METHODS)]
        chosen = max(candidates, key=lambda g: (float(g['marching']['length_cells']), g['marching']['case_id']))
        overview.append(chosen['marching']['case_id'])
    reference = Channel(nav2_command(ROOT))
    audit = Channel([str(ROOT / 'build/benchmark-audit/benchmark_audit')])
    examples = {}
    try:
        for cid in overview + list(FAILURES):
            rows = groups[cid]
            case = {k: rows['marching'][k] for k in ('case_id', 'map_id', 'start_id', 'goal_id', 'radius_cells',
                    'start_x', 'start_y', 'start_yaw_deg', 'goal_x', 'goal_y', 'goal_yaw_deg')}
            mid, radius, sid = case['map_id'], float(case['radius_cells']), case['start_id']
            example = dict(case=case, map_sha256=sha(BENCH / 'maps' / f'{mid}.png'), methods={})
            for method in METHODS:
                file = raw / f'{mid}-r{int(radius):02d}-{sid}-{method}.jsonl.gz'
                with gzip.open(file, 'rt') as stream:
                    record = next(r for r in map(json.loads, stream) if r.get('case_id') == cid and r.get('event') == 'goal_result')
                reply = record['reply']
                headings = json.loads((BENCH / f'primitives/r{int(radius)}.json').read_text())['lattice_metadata']['heading_angles'] if method == 'lattice' else None
                ps = marching_primitives(reply, radius) if method == 'marching' else nav2_primitives(reply, radius, reference, headings)[0]
                assert path_hash(ps) == rows[method]['path_sha256'], (cid, method, 'path hash')
                check = validate(ps, case, audit, method, reply, headings)
                assert check['validation'] == rows[method]['validation'], (cid, method, check)
                for key in ('witness_x', 'witness_y'):
                    if key in check:
                        assert abs(check[key] - float(rows[method][key])) < 1e-10
                example['methods'][method] = dict(primitives=ps, result=rows[method], raw_record_sha256=sha(file))
            if cid in FAILURES:
                assert rows['marching']['validation'] == 'clear'
                assert all(rows[m]['validation'] == 'blocked' for m in METHODS[1:])
            examples[cid] = example
            print('Verified', cid, flush=True)
    finally:
        audit.close()
        reference.close()
    snapshot = dict(schema='rs-marching-figure-examples-v1', trial=0,
                    results_sha256=sha(BENCH / 'results/results.csv'), cases_sha256=sha(BENCH / 'cases.csv'),
                    overview_selection='Longest Marching path among cases valid for all three methods on each map; case ID breaks ties.',
                    failure_selection='Two selected collision failures of both Nav2 methods; illustrative, not a frequency estimate.',
                    overview=overview, failures=list(FAILURES), examples=examples)
    (BENCH / 'examples.json').write_text(json.dumps(snapshot, separators=(',', ':')) + '\n')


def points(ps):
    chunks = []
    for p in ps:
        length = np.linspace(0, p['length'], max(2, math.ceil(p['length'] / .08) + 1)) * p['gear']
        x, y, theta = p['start']
        k = {'L': 1., 'R': -1., 'S': 0.}[p['type']] / p['radius']
        if k:
            chunk = np.column_stack((x + (np.sin(theta + length*k) - math.sin(theta))/k,
                                     y + (math.cos(theta) - np.cos(theta + length*k))/k))
        else:
            chunk = np.column_stack((x + length*math.cos(theta), y + length*math.sin(theta)))
        chunks.extend((chunk, np.array([[np.nan, np.nan]])))
    return np.concatenate(chunks)


def map_axes(ax, example):
    bitmap = np.asarray(Image.open(BENCH / 'maps' / f"{example['case']['map_id']}.png").convert('L'))
    h, w = bitmap.shape
    ax.imshow(bitmap, cmap='gray', vmin=0, vmax=255, origin='upper', interpolation='nearest',
              extent=(-.5, w-.5, h-.5, -.5))
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color('#9ca3af')
    return w, h


def paths(ax, example, methods=METHODS, width=2):
    for method in methods:
        i = METHODS.index(method)
        xy = points(example['methods'][method]['primitives'])
        ax.plot(xy[:, 0], xy[:, 1], color=COLORS[i], linestyle=STYLES[i],
                linewidth=width*(1.7, 1.15, .9)[i], zorder=3+i)


def poses(ax, case, scale):
    for prefix, color in [('start', '#009e50'), ('goal', '#d7191c')]:
        x, y, theta = (float(case[prefix + suffix]) for suffix in ('_x', '_y', '_yaw_deg'))
        theta = math.radians(theta)
        ax.annotate('', (x+scale*math.cos(theta), y+scale*math.sin(theta)), (x, y),
                    arrowprops=dict(arrowstyle='-|>', color=color, lw=2, mutation_scale=12), zorder=8)
        ax.plot(x, y, 'o', color=color, markersize=3.5, markeredgecolor='white', markeredgewidth=.6, zorder=9)


def save(fig, output, name):
    for extension in ('png', 'svg', 'pdf'):
        fig.savefig(output / f'{name}.{extension}', dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def render(output):
    snapshot = json.loads((BENCH / 'examples.json').read_text())
    assert sha(BENCH / 'results/results.csv') == snapshot['results_sha256']
    assert sha(BENCH / 'cases.csv') == snapshot['cases_sha256']
    rows = rows_by_case()
    for cid, example in snapshot['examples'].items():
        assert sha(BENCH / 'maps' / f"{example['case']['map_id']}.png") == example['map_sha256']
        for method, item in example['methods'].items():
            assert item['result'] == rows[cid][method]
            assert path_hash(item['primitives']) == rows[cid][method]['path_sha256']
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11})
    output.mkdir(parents=True, exist_ok=True)
    legend = [Line2D([], [], color=c, ls=s, lw=2.5, label=l) for c, s, l in zip(COLORS, STYLES, LABELS)]
    legend += [Line2D([], [], marker='o', ls='', color=c, label=l) for c, l in [('#009e50', 'Start'), ('#d7191c', 'Goal')]]
    fig, axes = plt.subplots(2, 5, figsize=(16, 8.2), gridspec_kw=dict(wspace=.12, hspace=.25))
    for ax, cid in zip(axes.flat, snapshot['overview']):
        example = snapshot['examples'][cid]
        w, h = map_axes(ax, example)
        paths(ax, example, width=1.45)
        poses(ax, example['case'], max(w, h)*.06)
        ax.set_xlabel(cid, fontsize=9)
    fig.legend(handles=legend, loc='upper center', ncol=5, frameon=False)
    fig.subplots_adjust(top=.91, bottom=.05)
    save(fig, output, 'benchmark-maps')

    fig, axes = plt.subplots(2, 3, figsize=(13, 8.8), gridspec_kw=dict(wspace=.20, hspace=.24))
    for axs, cid in zip(axes, snapshot['failures']):
        example = snapshot['examples'][cid]
        ax = axs[0]
        w, h = map_axes(ax, example)
        paths(ax, example, width=1.6)
        poses(ax, example['case'], max(w, h)*.055)
        ax.set_xlabel(cid, fontsize=10)
        for ax, method in zip(axs[1:], METHODS[1:]):
            map_axes(ax, example)
            paths(ax, example, methods=(method,), width=2)
            row = example['methods'][method]['result']
            x, y = float(row['witness_x']), float(row['witness_y'])
            ax.set_xlim(x-2, x+2)
            ax.set_ylim(y+2, y-2)
            ax.plot(x, y, 'x', color='#e31a1c', mew=2, ms=10, zorder=10)
            ax.set_xlabel(f'{LABELS[METHODS.index(method)]} · collision\n4 × 4 cells', fontsize=11)
            axs[0].plot(x, y, 'x', color='#e31a1c', mew=1.5, ms=6, zorder=10)
    fig.legend(handles=legend + [Line2D([], [], marker='x', ls='', color='#e31a1c', label='Collision witness')],
               loc='upper center', ncol=6, frameon=False, fontsize=10)
    fig.subplots_adjust(top=.91, bottom=.07)
    save(fig, output, 'benchmark-failures')
    print('Rendered figures; all 36 path hashes match the benchmark CSV.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-trial', type=Path, help='trial-0 directory containing the recorded *.jsonl.gz groups')
    parser.add_argument('--output', type=Path, default=ROOT/'build/benchmark-figures')
    args = parser.parse_args()
    if args.raw_trial:
        extract(args.raw_trial)
    render(args.output)


if __name__ == '__main__':
    main()
