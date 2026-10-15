#!/usr/bin/env python3
"""Small independent controls for coordinates, primitive fidelity, clearance and metrics."""
import argparse
import json
import math
from pathlib import Path
import selectors
import subprocess
import tempfile
import numpy as np
from PIL import Image
from geometry import (replay,primitive,endpoint,materialize,nav2_primitives,
                      marching_primitives,quality,wrap,lattice_edge,analytic_suffix,check_marching_metrics)
from generate_primitives import validate_heading_connectivity

class Channel:
    def __init__(self,command):
        self.p=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,bufsize=1)
    def __call__(self,request):
        self.p.stdin.write(json.dumps(request)+'\n');self.p.stdin.flush()
        while True:
            ready=selectors.DefaultSelector();ready.register(self.p.stdout,selectors.EVENT_READ)
            if not ready.select(30):
                self.p.kill();raise TimeoutError('adapter did not reply in 30 seconds')
            line=self.p.stdout.readline();ready.close()
            if not line:raise RuntimeError('adapter exited')
            try:return json.loads(line)
            except json.JSONDecodeError:continue
    def close(self):
        self.p.stdin.close()
        try:self.p.wait(timeout=3)
        except subprocess.TimeoutExpired:self.p.kill();self.p.wait()


def nav2_command(root):
    return ['docker','run','--rm','-i','--network','none','--read-only','--tmpfs','/tmp',
            '-e','ROS_LOG_DIR=/tmp/roslogs','-v',f'{root}:{root}:ro',
            'rs-marching-benchmark:nav2-observed-1.1.20']


def analytic_heading_control(root,reference):
    bin_size=np.float32(2)*np.float32(math.pi)/np.float32(72)
    assert float(np.float32(65)*bin_size)==5.672320365905762
    fixture=json.loads((root/'benchmarks/controls/analytic_heading_wrap.json').read_text())
    ps,error=analytic_suffix(fixture['chain'],0,fixture['radius_cells'],fixture['method'],
        dict(angle_bins=fixture['angle_bins']),fixture['headings'],reference)
    assert error<.002, error
    return dict(control='native_heading_representation',case_id=fixture['case_id'],
                max_sample_error_cells=error,length_cells=sum(p['length'] for p in ps))


def lattice_orientation_accuracy_control(root, output, audit):
    # Import locally because score imports Channel from this module.
    import shutil
    import score
    from score import validate

    validation_root = output / 'orientation-validation-root'
    map_path = validation_root / 'benchmarks/maps/empty.png'
    map_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(output / 'empty.png', map_path)
    previous_root = score.ROOT
    score.ROOT = validation_root
    try:
        headings = json.loads((root / 'benchmarks/primitives/r5.json').read_text())['lattice_metadata']['heading_angles']
        assert len(headings) == 16
        case = dict(map_id='empty', radius_cells='5', start_x='20', start_y='20',
                    start_yaw_deg='10', goal_x='60', goal_y='20', goal_yaw_deg='10')
        reply = dict(effective_start_yaw_rad=0.0, effective_goal_yaw_rad=0.0)
        straight = [primitive([20.0, 20.0, 0.0], 'S', 1, 40.0, 5.0)]
        accepted = validate(straight, case, audit, method='lattice', reply=reply, headings=headings)
        assert accepted['validation'] == 'clear', accepted
        assert math.isclose(accepted['start_orientation_error_deg'],10.,abs_tol=1e-10)
        assert math.isclose(accepted['goal_orientation_error_deg'],10.,abs_tol=1e-10)
        assert accepted['orientation_within_4_deg'] is False

        wrong_reply = dict(reply, effective_goal_yaw_rad=1.0)
        try:
            validate(straight, case, audit, method='lattice', reply=wrong_reply, headings=headings)
        except ValueError as error:
            assert 'nearest native bin' in str(error), error
        else:
            raise AssertionError('mismatched effective lattice yaw was accepted')

        wrong_edge = [primitive([20.0, 20.0, 0.0], 'L', 1, 0.5, 5.0)]
        arc_end = endpoint(wrong_edge[-1])
        wrong_case = dict(case, goal_x=str(arc_end[0]), goal_y=str(arc_end[1]))
        rejected = validate(wrong_edge, wrong_case, audit, method='lattice', reply=reply, headings=headings)
        assert rejected['validation'] == 'endpoint_error', rejected
        return dict(control='lattice_requested_orientation_accuracy',
                    accepted_validation=accepted['validation'],
                    requested_start_goal_error_deg=[accepted['start_orientation_error_deg'],
                                                    accepted['goal_orientation_error_deg']],
                    orientation_within_4_deg=accepted['orientation_within_4_deg'],
                    mismatched_effective_yaw='rejected', actual_primitive_heading_mismatch=rejected['validation'])
    finally:
        score.ROOT = previous_root
        shutil.rmtree(validation_root, ignore_errors=True)


def coverage_controls(root, output):
    import csv,gzip,hashlib,shutil
    from score import validated_groups
    def inspect_inputs(paths,partial):return list(validated_groups(paths,partial,{},{}))
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    directory=output/'coverage';directory.mkdir(exist_ok=True)
    run=directory/'trial-0';run.mkdir(exist_ok=True)
    cases=list(csv.DictReader((root/'benchmarks/cases.csv').open()))[:10]
    settings=dict(trial=0,methods=['marching','hybrid','lattice'],limit_groups=1,
        cases_sha256=digest(root/'benchmarks/cases.csv'),maps_json_sha256=digest(root/'benchmarks/maps.json'),
        primitive_hashes={str(root/'benchmarks/primitives/r5.json'):digest(root/'benchmarks/primitives/r5.json')})
    (directory/'manifest.json').write_text(json.dumps(dict(status='complete',settings=settings,run_hash='control')))
    gid=f"{cases[0]['map_id']}-r{int(cases[0]['radius_cells']):02d}-{cases[0]['start_id']}"
    for method in settings['methods']:
        with gzip.open(run/f'{method}.jsonl.gz','wt') as f:
            for case in cases:f.write(json.dumps(dict(event='goal_result',trial=0,method=method,group_id=gid,case_id=case['case_id']))+'\n')
            f.write(json.dumps(dict(event='group_complete',trial=0,method=method,group_id=gid,load_ok=True))+'\n')
    inspect_inputs([directory],True)
    def rejected(paths,partial):
        try:inspect_inputs(paths,partial)
        except ValueError:return
        raise AssertionError('invalid coverage accepted')
    rejected([directory,directory],True)
    rejected([directory],False)
    missing=run/'lattice.jsonl.gz';saved=missing.read_bytes();missing.unlink()
    rejected([directory],True);missing.write_bytes(saved)
    # Header-only identity changes must fail through the very same reader used
    # by scoring, even though all goal-level keys still cover the expected set.
    with gzip.open(missing,'rt') as f:rows=[json.loads(line) for line in f]
    rows[-1]['method']='marching'
    with gzip.open(missing,'wt') as f:
        for row in rows:f.write(json.dumps(row)+'\n')
    rejected([directory],True);missing.write_bytes(saved)
    settings['cases_sha256']='wrong' 
    (directory/'manifest.json').write_text(json.dumps(dict(status='complete',settings=settings,run_hash='control')))
    rejected([directory],True)
    shutil.rmtree(directory)


def native_failure_controls(root):
    from score import native_failure
    import copy
    captures=json.loads((root/'benchmarks/controls/captured_prefix_controls.json').read_text())
    assert len(captures)==5
    for fixture in captures:
        a,b=fixture['parent'],fixture['node']
        ps=lattice_edge(a,b)
        assert np.linalg.norm(endpoint(ps[-1])[:2]-[b['x'],b['y']])<.002
        missing={k:v for k,v in b.items() if not k.startswith('saved_prefix_')}
        try:lattice_edge(a,missing)
        except ValueError:pass
        else:raise AssertionError('missing native control was guessed')
        conflicting=copy.deepcopy(b)
        conflicting['primitive']=dict(b['saved_prefix_primitive'],trajectory_length=999.)
        try:lattice_edge(a,conflicting)
        except ValueError:pass
        else:raise AssertionError('captured control replaced conflicting final metadata')
    fixtures=json.loads((root/'benchmarks/controls/native_failure_witnesses.json').read_text())
    statuses=[]
    for fixture in fixtures:
        case=fixture['case'];reply=fixture['reply'];radius=int(case['radius_cells'])
        headings=json.loads((root/f'benchmarks/primitives/r{radius}.json').read_text())['lattice_metadata']['heading_angles']
        result=native_failure(reply,case,headings)
        expected='native_endpoint_error' if case['map_id']=='01_u_traps' else 'native_join_error'
        assert result and result['validation']==expected,(case['case_id'],result)
        statuses.append(dict(case_id=case['case_id'],**result))
        if expected=='native_join_error':
            # Metadata alone does not certify failure: the emitted control must agree.
            altered=copy.deepcopy(reply)
            altered['path']=[reply['path'][0],reply['path'][-1]]
            assert native_failure(altered,case,headings) is None
            i=result['first_failed_node'];b=copy.deepcopy(reply['parent_chain'][i])
            b['saved_prefix_primitive']=b['primitive'];b['saved_prefix_backwards']=b['backwards']
            try:lattice_edge(reply['parent_chain'][i-1],b)
            except ValueError:pass
            else:raise AssertionError('contradictory final primitive was replaced')
        else:
            altered=copy.deepcopy(reply)
            altered['path'][-1][:2]=[float(case['goal_x']),float(case['goal_y'])]
            # Disagreement between representations is not this endpoint certificate.
            other=native_failure(altered,case,headings)
            assert other is None or other['validation']!='native_endpoint_error'
    return statuses


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--api',required=True);args=ap.parse_args()
    root=Path(__file__).resolve().parent.parent
    output=root/'build/benchmark-controls';output.mkdir(parents=True,exist_ok=True)
    # Native API counts all nonzero pieces; comparative quality ignores <=1e-8 cells.
    tiny=[primitive([20.,20.,0.],kind,gear,length,20.) for kind,gear,length in
          [('L',-1,3.6092587301),('R',1,9.9045057914e-14),('L',-1,1.5814543060)]]
    reported=dict(reconstructed_length=sum(p['length'] for p in tiny),reversal_count=2)
    assert check_marching_metrics(reported,tiny)==2 and quality(tiny,20.)['gear_reversals']==0
    try:check_marching_metrics(dict(reported,reversal_count=1),tiny)
    except ValueError:pass
    else:raise AssertionError('incorrect native reversal count accepted')
    native_failure_controls(root)
    coverage_controls(root,output)
    im=np.full((100,100),255,np.uint8);Image.fromarray(im).save(output/'empty.png')
    im[40:51,40:51]=0;Image.fromarray(im).save(output/'blocked.png')
    corridor=np.zeros((100,100),np.uint8);corridor[45:56,2:98]=255
    Image.fromarray(corridor).save(output/'corridor.png')
    audit=Channel([str(root/'build/benchmark-audit/benchmark_audit')])
    reference=Channel(nav2_command(root));results=[]
    results.append(analytic_heading_control(root,reference))
    results.append(lattice_orientation_accuracy_control(root, output, audit))
    # Continuous clearance: an interior crossing fails; an external tangent is legal.
    for y,expected in [(45.,'blocked'),(39.5,'clear'),(39.499,'clear'),(39.501,'blocked')]:
        p=primitive([30.,y,0.],'S',1,30.,5.)
        a=audit(dict(map=str(output/'blocked.png'),primitives=[p]));assert a['status']==expected,a
        results.append(dict(control='clearance',y=y,status=a['status']))
    for start,goal,gear in [([10,20,0],[60,20,0],1),([60,20,0],[10,20,0],-1)]:
        rs=reference(dict(op='rs_path',start=start,goal=goal,radius_cells=5))
        assert abs(rs['total_length']-50)<1e-9 and all(s['gear']==gear for s in rs['segments'])
        ps=materialize(start,rs['segments'],5);assert np.linalg.norm(endpoint(ps[-1])[:2]-goal[:2])<1e-9
    # Transform control: reflection preserves length and gear count, flips turn signs.
    for start,goal in [([30,30,.37],[31,31,2.9]),([30,30,0.],[30,30,math.pi])]:
        a=reference(dict(op='rs_path',start=start,goal=goal,radius_cells=5))
        b=reference(dict(op='rs_path',start=[start[0],-start[1],-start[2]],goal=[goal[0],-goal[1],-goal[2]],radius_cells=5))
        assert abs(a['total_length']-b['total_length'])<1e-8
        ps=materialize(start,a['segments'],5);assert abs(wrap(endpoint(ps[-1])[2]-goal[2]))<1e-8
    scenarios=[([10,20,0],[60,20,0]),([60,20,0],[10,20,0]),
               ([30,30,.37],[60,60,2.9]),([30,30,0.],[30,30,math.pi])]
    for method in ['hybrid','lattice']:
        path=root/'benchmarks/primitives/r5.json'
        headings=json.loads(path.read_text())['lattice_metadata']['heading_angles']
        nav=Channel(nav2_command(root))
        for start,goal in scenarios:
            request=dict(method=method,map=str(output/'empty.png'),radius_cells=5,start=start,goal=goal,primitive_file=str(path))
            reply=nav(request)
            if reply['status']!='success':raise AssertionError(reply)
            for node in reply['parent_chain']:
                native=(np.float32(node['theta_bin'])*(np.float32(2)*np.float32(math.pi)/np.float32(72))
                    if method=='hybrid' else np.float32(headings[node['theta_bin']]))
                assert node['native_theta_rad']==float(native)
            ps,metadata=nav2_primitives(reply,5,reference,headings)
            check=audit(dict(map=str(output/'empty.png'),primitives=ps));assert check['status']=='clear',check
            q=quality(ps,5)
            results.append(dict(method=method,start=start,goal=goal,quality=q,**metadata))
            (output/f'{method}_{len(results)}.json').write_text(json.dumps(dict(request=request,reply=reply,primitives=ps)))
        nav.close()
        nav=Channel(nav2_command(root))
        # A straight path in an eleven-cell-wide corridor must remain valid.
        request=dict(method=method,map=str(output/'corridor.png'),radius_cells=5,
                     start=[20,50,0],goal=[80,50,0],primitive_file=str(path))
        reply=nav(request);assert reply['status']=='success',reply
        ps,meta=nav2_primitives(reply,5,reference,headings)
        check=audit(dict(map=str(output/'corridor.png'),primitives=ps));assert check['status']=='clear',check
        results.append(dict(method=method,control='narrow_corridor',quality=quality(ps,5)))
        nav.close()
    # Exhaust native lattice structure at all radii in both traversal directions.
    import hashlib
    primitive_bindings={}
    for radius in [5,10,20]:
        path=root/f'benchmarks/primitives/r{radius}.json';data=json.loads(path.read_text())
        headings=data['lattice_metadata']['heading_angles']
        num_headings=data['lattice_metadata']['num_of_headings']
        assert num_headings==16 and len(headings)==16
        assert len({p['start_angle_index'] for p in data['primitives']})==16
        connectivity=validate_heading_connectivity(num_headings,
            [(p['start_angle_index'],p['end_angle_index']) for p in data['primitives']])
        assert connectivity['strongly_connected'], (radius,connectivity)
        primitive_bindings[str(radius)]=hashlib.sha256(path.read_bytes()).hexdigest()
        for p in data['primitives']:
            for backwards in [False,True]:
                angle=math.pi if backwards else 0.
                a=dict(x=0.,y=0.,theta_rad=headings[p['start_angle_index']]+angle)
                b=dict(x=p['poses'][-1][0]/.05,y=p['poses'][-1][1]/.05,
                    theta_rad=headings[p['end_angle_index']]+angle,backwards=backwards,
                    primitive=dict(p,turning_radius=p['trajectory_radius']))
                ps=lattice_edge(a,b)
                if p['straight_length']==0:assert all(x['type']!='S' for x in ps)
                if p['arc_length']==0:assert all(x['type']=='S' for x in ps)
                assert abs(sum(x['length'] for x in ps)-p['trajectory_length']/.05)<.004
        results.append(dict(control='all_lattice_primitives',radius=radius,
                            traversals=2*len(data['primitives']),heading_connectivity=connectivity))
    (output/'primitive_bindings.json').write_text(json.dumps(primitive_bindings,indent=2)+'\n')
    api=Channel([args.api]);start=dict(x=30,y=30,heading_deg=21.2)
    reply=api(dict(command='march',map_path=str(output/'empty.png'),start=start,radius=5))
    reply=api(dict(command='query',x=60,y=60,heading_deg=166.2,check_collisions=True,sample_step=.1))
    (output/'marching.json').write_text(json.dumps(reply))
    assert reply['status']=='clear',reply
    ps=marching_primitives(reply,5);check=audit(dict(map=str(output/'empty.png'),primitives=ps));assert check['status']=='clear',check
    q=quality(ps,5);assert abs(q['length_cells']-reply['reconstructed_length'])<1e-7
    assert q['gear_reversals']==reply['reversal_count']
    results.append(dict(method='marching',quality=q))
    api(dict(command='march',map_path=str(output/'corridor.png'),start=dict(x=20,y=50,heading_deg=0),radius=5))
    reply=api(dict(command='query',x=80,y=50,heading_deg=0,check_collisions=True,sample_step=.1))
    assert reply['status']=='clear',reply
    results.append(dict(method='marching',control='narrow_corridor',quality=quality(marching_primitives(reply,5),5)))
    api.close();reference.close();audit.close()
    (output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))

if __name__=='__main__':main()
