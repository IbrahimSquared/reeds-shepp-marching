#!/usr/bin/env python3
"""Score frozen native outputs after timing, using shared continuous clearance."""
import argparse
import csv
import gzip
import hashlib
import json
import math
import subprocess
from pathlib import Path
import numpy as np
from controls import Channel
from geometry import (nav2_primitives,marching_primitives,quality,endpoint,wrap,
                      primitive,POSITION_TOL,ANGLE_TOL,check_marching_metrics)

ROOT=Path(__file__).resolve().parent.parent


def records(path):
    with gzip.open(path,'rt') as f:
        return [json.loads(line) for line in f]


def validate(ps,case,audit,method="marching",reply=None,headings=None):
    radius=float(case['radius_cells'])
    if not ps:return dict(validation='empty_path')
    start=np.asarray(ps[0]['start']);end=endpoint(ps[-1])
    requested_start=np.array([float(case['start_x']),float(case['start_y']),math.radians(float(case['start_yaw_deg']))])
    requested_goal=np.array([float(case['goal_x']),float(case['goal_y']),math.radians(float(case['goal_yaw_deg']))])
    pos=max(float(np.linalg.norm(start[:2]-requested_start[:2])),float(np.linalg.norm(end[:2]-requested_goal[:2])))
    se=abs(math.degrees(wrap(start[2]-requested_start[2])));ge=abs(math.degrees(wrap(end[2]-requested_goal[2])))
    result=dict(start_orientation_error_deg=se,goal_orientation_error_deg=ge,position_error_cells=pos,
                orientation_within_4_deg=max(se,ge)<=4.)
    expected=[requested_start[2],requested_goal[2]]
    if method!='marching':
        bins=np.arange(72)*2*math.pi/72 if method=='hybrid' else np.asarray(headings)
        expected=[min(bins,key=lambda h:abs(wrap(theta-h))) for theta in expected]
        if any(abs(wrap(reply[key]-theta))>ANGLE_TOL for key,theta in
               zip(['effective_start_yaw_rad','effective_goal_yaw_rad'],expected)):
            raise ValueError('adapter heading does not match nearest native bin')
    native_error=max(abs(wrap(start[2]-expected[0])),abs(wrap(end[2]-expected[1])))
    result['native_endpoint_orientation_error_rad']=native_error
    if pos>POSITION_TOL or native_error>ANGLE_TOL:
        return dict(result,validation='endpoint_error')
    if any(p['type']!='S' and p['radius']<radius*(1-1e-5) for p in ps):
        return dict(result,validation='turning_radius_error')
    checked=list(ps);max_join=0.;max_angle=0.
    for a,b in zip(ps,ps[1:]):
        x=endpoint(a);y=np.asarray(b['start']);delta=y[:2]-x[:2]
        gap=float(np.linalg.norm(delta));angle=abs(wrap(x[2]-y[2]))
        max_join=max(max_join,gap);max_angle=max(max_angle,angle)
        if gap>POSITION_TOL or angle>ANGLE_TOL:
            return dict(result,validation='join_error',max_join_gap_cells=max_join,max_join_angle_rad=max_angle)
        # Conservatively check the tiny numerical join too. It is not included in
        # physical length or curvature metrics; its maximum residual is reported.
        if gap>1e-10:
            checked.append(primitive([x[0],x[1],math.atan2(delta[1],delta[0])],'S',1,gap,1.))
    reply=audit(dict(map=str(ROOT/'benchmarks/maps'/f'{case["map_id"]}.png'),primitives=checked))
    result.update(validation=reply['status'],max_join_gap_cells=max_join,max_join_angle_rad=max_angle)
    for index,check in enumerate(reply.get('checks',[])):
        if check['status']!='clear':
            result.update(first_failed_check=index,clearance_feature=check['feature'],
                          witness_x=check['witness'][0],witness_y=check['witness'][1])
            break
    return result


def native_failure(reply,case,headings):
    """A witnessed endpoint/join contradiction suffices without decoding the whole path."""
    chain=reply['parent_chain'];path=np.asarray(reply['path'],dtype=float)
    if len(chain)<2 or len(path)<2:return None
    expected=np.array([[float(case['start_x']),float(case['start_y'])],
                       [float(case['goal_x']),float(case['goal_y'])]])
    actual=np.array([[chain[0]['x'],chain[0]['y']],[chain[-1]['x'],chain[-1]['y']]])
    # Require both independent output representations to exhibit the same endpoint.
    if np.max(np.linalg.norm(path[[0,-1],:2]-actual,axis=1))<=POSITION_TOL:
        errors=np.linalg.norm(actual-expected,axis=1)
        if max(errors)>POSITION_TOL:
            k=int(np.argmax(errors))
            return dict(validation='native_endpoint_error',position_error_cells=float(errors[k]),
                failed_endpoint='start' if k==0 else 'goal',witness_x=float(actual[k,0]),witness_y=float(actual[k,1]))
    if reply['method']!='lattice':return None
    for i,(a,b) in enumerate(zip(chain,chain[1:]),1):
        native=b.get('primitive')
        if native is None:continue  # Never replace conflicting native controls with captured ones.
        poses=np.asarray(native['poses'],dtype=float)
        if len(poses)==0:continue
        origin=np.array([b['x'],b['y']])-poses[-1,:2]/.05
        theta=float(np.float32(headings[int(native['start_angle'])]))+(math.pi if b['backwards'] else 0.)
        gap=float(np.linalg.norm(origin-[a['x'],a['y']]))
        angle=abs(wrap(theta-a['theta_rad']))
        if gap<=POSITION_TOL and angle<=ANGLE_TOL:continue
        samples=poses.copy();samples[:,:2]=poses[:,:2]/.05+origin
        if b['backwards']:samples[:,2]+=math.pi
        # Confirm the contradictory control is actually emitted in the raw backtrace.
        candidates=np.flatnonzero(np.linalg.norm(path[:,:2]-samples[0,:2],axis=1)<=POSITION_TOL)
        for j in candidates:
            part=path[j:j+len(samples)]
            if len(part)!=len(samples):continue
            error=float(np.max(np.linalg.norm(part[:,:2]-samples[:,:2],axis=1)))
            angular=max(abs(wrap(x-y)) for x,y in zip(part[:,2],samples[:,2]))
            if error<=POSITION_TOL and angular<=ANGLE_TOL:
                return dict(validation='native_join_error',first_failed_node=i,
                    max_join_gap_cells=gap,max_join_angle_rad=angle,
                    witness_x=float(origin[0]),witness_y=float(origin[1]),
                    native_primitive_id=native['trajectory_id'],native_sample_start=int(j),
                    native_sample_error_cells=error)
    return None


def csv_write(path,rows):
    columns=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader();writer.writerows(rows)


def validated_groups(directories, partial, bindings, file_hashes):
    """One identity authority for record validation and subsequent scoring.

    Exhaust the iterator before writing results: its final coverage check is a
    mandatory gate, including groups for which every planner query failed.
    """
    digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    cases_path=ROOT/'benchmarks/cases.csv';maps_path=ROOT/'benchmarks/maps.json'
    cases=list(csv.DictReader(cases_path.open()));by_id={r['case_id']:r for r in cases}
    group_id=lambda r:f"{r['map_id']}-r{int(r['radius_cells']):02d}-{r['start_id']}"
    all_groups=list(dict.fromkeys(group_id(r) for r in cases))
    map_manifest=json.loads(maps_path.read_text())
    for m in map_manifest['maps']:
        if digest(ROOT/'benchmarks'/m['normalized_path'])!=m['normalized_sha256']:
            raise ValueError('actual map PNG hash mismatch')
    seen=set()
    for directory in directories:
        manifest=json.loads((directory/'manifest.json').read_text());settings=manifest['settings']
        if manifest.get('status')!='complete':raise ValueError('run is not complete')
        if settings['cases_sha256']!=digest(cases_path) or settings['maps_json_sha256']!=digest(maps_path):
            raise ValueError('frozen input hash mismatch')
        limit=settings['limit_groups'];methods=settings['methods'];trial=settings['trial']
        if limit is not None and not partial:raise ValueError('partial run requires --partial')
        if len(set(methods))!=len(methods) or not set(methods)<=set(['marching','hybrid','lattice']) or trial not in range(3):
            raise ValueError('invalid declared method/trial set')
        selected=set(all_groups[:limit])
        selected_cases=[r for r in cases if group_id(r) in selected]
        required_primitives={f"r{r['radius_cells']}.json" for r in selected_cases}
        hashes={Path(name).name:value for name,value in settings['primitive_hashes'].items()}
        if set(hashes)!=required_primitives:raise ValueError('incomplete primitive bindings')
        for name,value in hashes.items():
            if value!=digest(ROOT/'benchmarks/primitives'/name):raise ValueError('actual primitive hash mismatch')
        expected={(trial,m,r['case_id']) for m in methods for r in selected_cases};local=set()
        bindings[str(directory)]=manifest['run_hash']
        for file in sorted(directory.glob('trial-*/*.jsonl.gz')):
            stream=records(file)
            if not stream or stream[-1].get('event')!='group_complete' or not stream[-1].get('load_ok'):
                raise ValueError('incomplete or load-contaminated group')
            goals=[r for r in stream if r.get('event')=='goal_result']
            if len(goals)!=10:raise ValueError('group must contain ten goal results')
            first=goals[0];identity=(first['trial'],first['method'],first['group_id'])
            t,m,gid=identity
            if t!=trial or m not in methods or gid not in selected:raise ValueError('undeclared group identity')
            # All event headers, including completion, must agree. Scoring uses
            # this canonical identity, never a second interpretation of headers.
            for event in stream:
                if (event.get('trial'),event.get('method'),event.get('group_id'))!=identity:
                    raise ValueError('mismatched event/completion identity')
            expected_ids={r['case_id'] for r in selected_cases if group_id(r)==gid}
            if {r['case_id'] for r in goals}!=expected_ids:raise ValueError('wrong group case membership')
            for r in goals:
                key=(t,m,r['case_id'])
                if key in seen:raise ValueError(f'duplicate case result: {key}')
                seen.add(key);local.add(key)
            file_hashes[str(file)]=digest(file)
            yield dict(file=file,trial=t,method=m,group_id=gid,goals=goals,
                       fields=[r for r in stream if r.get('event') in ('march','march_restart')],final=stream[-1])
        if local!=expected:raise ValueError(f'case coverage mismatch: missing {len(expected-local)}, extra {len(local-expected)}')
    if not partial:
        expected={(t,m,r['case_id']) for t in range(3) for m in ['marching','hybrid','lattice'] for r in cases}
        if seen!=expected:raise ValueError('full scoring requires three complete trials of all methods')

def repeatability_report(results,batches):
    report={}
    for method in ['marching','hybrid','lattice']:
        rows=[r for r in results if r['method']==method]
        trials=sorted({r['trial'] for r in rows});by_case={}
        for r in rows:by_case.setdefault(r['case_id'],[]).append(r)
        outcome_keys=['planner_status','validation','success','success_within_4_deg']
        changed_outcomes=[];changed_paths=[]
        for cid,values in sorted(by_case.items()):
            if len({tuple(r[k] for k in outcome_keys) for r in values})>1:changed_outcomes.append(cid)
            paths={r['path_sha256'] for r in values if 'path_sha256' in r}
            if len(paths)>1:changed_paths.append(cid)
        timings=[]
        for trial in trials:
            bs=[b for b in batches if b['trial']==trial and b['method']==method]
            timings.append(dict(trial=trial,complete_ipc_ms=sum(b['batch_ipc_ms'] for b in bs),
                successes=sum(r['success'] for r in rows if r['trial']==trial),
                unknown_load_batches=sum(b['load_measurement']=='unknown' for b in bs)))
        report[method]=dict(observed_trials=trials,changed_outcome_case_ids=changed_outcomes,
            changed_path_case_ids=changed_paths,trials=timings)
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--runs',nargs='+',required=True,type=Path)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--partial',action='store_true',help='explicit pilot subset; no publication table')
    ap.add_argument('--nav2-command',help='JSON array; defaults to benchmark Docker image')
    args=ap.parse_args();file_hashes={}
    args.output.mkdir(parents=True,exist_ok=True)
    cases={r['case_id']:r for r in csv.DictReader((ROOT/'benchmarks/cases.csv').open())}
    cmd=json.loads(args.nav2_command) if args.nav2_command else ['docker','run','--rm','-i','--read-only','--network','none','--tmpfs','/tmp','-e','ROS_LOG_DIR=/tmp/roslogs','rs-marching-benchmark:nav2-observed-1.1.20']
    rs=Channel(cmd);audit=Channel([str(ROOT/'build/benchmark-audit/benchmark_audit')])
    results=[];batch_results=[];unscorable=[];bindings={}
    try:
        for group in validated_groups(args.runs,args.partial,bindings,file_hashes):
                file=group['file'];method=group['method'];trial=group['trial'];gid=group['group_id']
                final=group['final'];goals=group['goals'];fields=group['fields']
                field_ipc=sum(r.get('ipc_ms',0.) for r in fields)
                field_core=sum(r.get('reply',{}).get('timings_ms',{}).get('initialization',0.)+r.get('reply',{}).get('timings_ms',{}).get('march',0.) for r in fields)
                query_ipc=sum(r.get('ipc_ms',0.) for r in goals)
                query_core=sum((r.get('reply',{}).get('timings_ms',{}).get('total',0.) if method=='marching' else r.get('reply',{}).get('search_ms',0.)+r.get('reply',{}).get('setup_ms',0.)) for r in goals)
                field_native=sum(r.get('reply',{}).get('timings_ms',{}).get('total',0.) for r in fields)
                query_native=sum((r.get('reply',{}).get('timings_ms',{}).get('total',0.) if method=='marching' else r.get('reply',{}).get('complete_ms',0.)) for r in goals)
                query_phases={name:sum(r.get('reply',{}).get('timings_ms',{}).get(name,0.) for r in goals)
                              for name in ['search','reconstruction','collision','sampling']} if method=='marching' else {}
                rss=max([p.get('sampled_peak_hwm_kib') or 0 for p in final.get('processes',[])],default=0) or None
                cpus={p['load']['selected_cpu'] for p in final.get('processes',[]) if p.get('load',{}).get('selected_cpu') is not None}
                cpu=next(iter(cpus)) if len(cpus)==1 else None
                memory_status='sampled_lower_bound' if rss is not None else 'unavailable'
                load_status='unknown' if final.get('measurement_quality')!='sufficient' else 'measured'
                batch_results.append(dict(trial=trial,group_id=gid,method=method,cpu=cpu,field_core_ms=field_core,query_core_ms=query_core,field_native_ms=field_native,query_native_ms=query_native,**{f'query_{k}_ms':v for k,v in query_phases.items()},field_ipc_ms=field_ipc,query_ipc_ms=query_ipc,batch_ipc_ms=field_ipc+query_ipc,sampled_peak_hwm_kib=rss,memory_status=memory_status,load_measurement=load_status))
                for record in goals:
                    cid=record['case_id'];case=cases[cid];radius=float(case['radius_cells'])
                    reply=record.get('reply',{});status=reply.get('status',record.get('status','process_failure'))
                    row=dict(case,trial=trial,method=method,cpu=cpu,planner_status=status,validation='not_returned',success=False,success_within_4_deg=False,query_ipc_ms=record.get('ipc_ms',0.),shared_field_ipc_ms=field_ipc,sampled_peak_hwm_kib=rss,memory_status=memory_status,load_measurement=load_status)
                    if status in ('success','clear'):
                        try:
                            if method=='marching':ps=marching_primitives(reply,radius);meta={};headings=None
                            else:
                                headings=json.loads((ROOT/f'benchmarks/primitives/r{int(radius)}.json').read_text())['lattice_metadata']['heading_angles'] if method=='lattice' else None
                                failure=native_failure(reply,case,headings)
                                if failure:
                                    row.update(failure);results.append(row);continue
                                ps,meta=nav2_primitives(reply,radius,rs,headings)
                            row['path_sha256']=hashlib.sha256(json.dumps(ps,sort_keys=True,separators=(',',':')).encode()).hexdigest()
                            row.update(meta);row.update(validate(ps,case,audit,method,reply,headings))
                            if row['validation']=='clear':
                                row.update(quality(ps,radius));row['success']=True
                                row['success_within_4_deg']=row['orientation_within_4_deg']
                                if method=='marching':
                                    row['native_reported_gear_reversals']=check_marching_metrics(reply,ps)
                        except (ValueError,KeyError,np.linalg.LinAlgError) as e:
                            row['validation']='unscorable';row['success']=False;row['success_within_4_deg']=False;row['scoring_error']=str(e)
                            unscorable.append(dict(case_id=cid,trial=trial,method=method,error=str(e),record=str(file)))
                    results.append(row)
                print(f'scored trial {trial} {gid} {method}',flush=True)
    finally:rs.close();audit.close()
    # Trial 0 is the predeclared quality sample; no best-of-three selection.
    primary=[r for r in results if r['trial']==0]
    valid={m:{r['case_id']:r for r in primary if r['method']==m and r['success']} for m in ['marching','hybrid','lattice']}
    common=set.intersection(*(set(rows) for rows in valid.values()))
    for row in results:
        if row['trial']==0 and row['success']:
            best=min(rows[row['case_id']]['length_cells'] for rows in valid.values() if row['case_id'] in rows)
            row['length_over_best_observed']=row['length_cells']/best
            row['common_three_method_case']=row['case_id'] in common
    table=[]
    for method,good in valid.items():
        rows=[r for r in primary if r['method']==method]
        times=[b['batch_ipc_ms']/10 for b in batch_results if b['method']==method]
        paired=[good[c] for c in sorted(common)]
        median=lambda key:float(np.median([r[key] for r in paired])) if paired else None
        table.append(dict(method=method,cases=len(rows),validated_successes=len(good),successes_within_4_deg=sum(r['success_within_4_deg'] for r in rows),
            max_endpoint_orientation_error_deg=max([max(r['start_orientation_error_deg'],r['goal_orientation_error_deg']) for r in good.values()],default=None),
            common_cases=len(common),
            median_length_cells=median('length_cells'),median_length_over_best_observed=median('length_over_best_observed'),
            median_gear_reversals=median('gear_reversals'),
            mean_gear_reversals=float(np.mean([r['gear_reversals'] for r in paired])) if paired else None,
            median_roughness=median('roughness'),
            median_amortized_ipc_ms=float(np.median(times)) if times else None,
            mean_amortized_ipc_ms=float(np.mean(times)) if times else None,
            sampled_peak_hwm_kib=max([b['sampled_peak_hwm_kib'] or 0 for b in batch_results if b['method']==method],default=0) or None,
            unknown_load_batches=sum(b['load_measurement']=='unknown' for b in batch_results if b['method']==method)))
    csv_write(args.output/'results.csv',results);csv_write(args.output/'batch_timings.csv',batch_results)
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    provenance=dict(code_sha256={str(p.relative_to(ROOT)):digest(p) for p in
        [ROOT/'benchmarks/score.py',ROOT/'benchmarks/geometry.py',ROOT/'benchmarks/controls.py',
         ROOT/'benchmarks/audit.cpp']},
        audit_binary_sha256=digest(ROOT/'build/benchmark-audit/benchmark_audit'),reference_command=cmd)
    if cmd and Path(cmd[0]).name=='docker':
        inspected=subprocess.run(['docker','image','inspect',cmd[-1],'--format','{{.Id}}'],capture_output=True,text=True,check=True)
        provenance['reference_image_id']=inspected.stdout.strip()
    summary=dict(scoring=provenance,repeatability=repeatability_report(results,batch_results),bindings=bindings,raw_file_sha256=file_hashes,partial=args.partial,table=table,unscorable=unscorable,common_case_ids=sorted(common),
                 publication_gate='blocked_unscorable' if unscorable else ('pilot_only' if args.partial else 'pending_repeatability_load_review'))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(table,indent=2))
    return 2 if unscorable else 0

if __name__=='__main__':raise SystemExit(main())
