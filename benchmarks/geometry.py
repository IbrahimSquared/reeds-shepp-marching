"""Replay native primitives; never connect arbitrary adjacent samples with new RS paths."""
import math
import numpy as np

POSITION_TOL = .002  # cells; float Nav2 state and primitive-file serialization
ANGLE_TOL = 2e-4     # radians; fidelity, not requested endpoint tolerance


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def replay(start, kind, gear, length, radius):
    x, y, theta = start
    signed = gear * length
    k = {'L': 1., 'R': -1., 'S': 0.}[kind] / radius
    if not k:
        return np.array([x + signed * math.cos(theta), y + signed * math.sin(theta), theta])
    end = theta + signed * k
    return np.array([x + (math.sin(end) - math.sin(theta)) / k,
                     y + (math.cos(theta) - math.cos(end)) / k, end])


def primitive(start, kind, gear, length, radius):
    return dict(start=list(start), type=kind, gear=int(gear), length=float(length), radius=float(radius))


def endpoint(p):
    return replay(p['start'], p['type'], p['gear'], p['length'], p['radius'])


def materialize(start, segments, radius):
    out = []
    cursor = np.asarray(start, dtype=float)
    for s in segments:
        if s['length'] <= 1e-10:
            continue
        p = primitive(cursor, s['type'], s['gear'], s['length'], radius)
        out.append(p)
        cursor = endpoint(p)
    return out


def at_lengths(primitives, lengths):
    ends = np.cumsum([p['length'] for p in primitives])
    out = []
    for distance in lengths:
        i = min(int(np.searchsorted(ends, distance)), len(primitives)-1)
        p = primitives[i]
        local = max(0., distance - (ends[i-1] if i else 0.))
        out.append(replay(p['start'], p['type'], p['gear'], local, p['radius']))
    return np.asarray(out)


def close_pose(a, b):
    return np.linalg.norm(np.asarray(a)[:2]-np.asarray(b)[:2]) <= POSITION_TOL and abs(wrap(a[2]-b[2])) <= ANGLE_TOL


def node_pose(n):
    return np.array([n['x'], n['y'], n['theta_rad']])


def hybrid_edge(a, b, radius, bins):
    index = b['primitive_index']
    if index not in range(6):
        raise ValueError('no searched hybrid primitive')
    step = 2*math.pi/bins
    turn = math.ceil(2*math.asin(math.sqrt(2)/(4*radius))/step)*step
    kind = ['S','L','R','S','L','R'][index]
    gear = 1 if index < 3 else -1
    length = 2*radius*math.sin(turn/2) if kind == 'S' else radius*turn
    p = primitive(node_pose(a), kind, gear, length, radius)
    if not close_pose(endpoint(p), node_pose(b)):
        raise ValueError('hybrid edge differs from its searched primitive')
    return [p]


def check_lattice_samples(result,native,start,gear):
    total=sum(p['length'] for p in result)
    actual=np.asarray(native['poses'],dtype=float)
    if len(actual)==0:
        raise ValueError('missing lattice sample sequence')
    expected=at_lengths(result,np.arange(1,len(actual)+1)/len(actual)*total)
    actual[:,:2]=actual[:,:2]/.05+start[:2]
    if gear<0: actual[:,2]+=math.pi
    if np.max(np.linalg.norm(expected[:,:2]-actual[:,:2],axis=1))>POSITION_TOL or max(abs(wrap(x-y)) for x,y in zip(expected[:,2],actual[:,2]))>ANGLE_TOL:
        raise ValueError('lattice sample sequence fidelity')


def lattice_edge(a, b):
    native = b.get('primitive')
    if native is None and b.get('saved_prefix_primitive') is not None:
        native=b['saved_prefix_primitive']
        b=dict(b,backwards=b['saved_prefix_backwards'])
    if not native:
        raise ValueError('analytic lattice node')
    start, end = node_pose(a), node_pose(b)
    gear = -1 if b['backwards'] else 1
    radius = native['turning_radius']/.05
    delta = wrap(end[2]-start[2])
    if abs(delta) < 1e-6:
        p = primitive(start, 'S', gear, np.linalg.norm(end[:2]-start[:2]), 1.)
        if not close_pose(endpoint(p), end):
            raise ValueError('lattice straight fidelity')
        if native['arc_length']!=0 or abs(p['length']-native['trajectory_length']/.05)>.004:
            raise ValueError('lattice straight metadata fidelity')
        check_lattice_samples([p],native,start,gear)
        return [p]
    if radius <= 0:
        raise ValueError('non-Ackermann primitive')
    if (delta>0)!=native['left_turn'] or abs(delta)>math.pi/2+ANGLE_TOL:
        raise ValueError('lattice native arc direction fidelity')
    kind = 'L' if delta * gear > 0 else 'R'
    arc_length = abs(delta)*radius
    displacement = replay([0.,0.,start[2]], kind,gear,arc_length,radius)[:2]
    u = gear*np.array([math.cos(start[2]), math.sin(start[2])])
    v = gear*np.array([math.cos(end[2]), math.sin(end[2])])
    first, last = np.linalg.solve(np.column_stack([u,v]), end[:2]-start[:2]-displacement)
    if min(first,last) < -POSITION_TOL:
        raise ValueError('negative lattice straight portion')
    straight = native['straight_length']/.05
    # The generator emits an arc and at most one straight portion. Preserve
    # metadata-zero portions: fitting rounded endpoints must not invent turns.
    if straight == 0.:
        segments = [dict(type=kind,gear=gear,length=arc_length)]
    else:
        if min(abs(first),abs(last)) > POSITION_TOL or abs(max(first,last)-straight) > .004:
            raise ValueError('lattice straight structure fidelity')
        segments = [dict(type=kind,gear=gear,length=arc_length)]
        straight_piece = dict(type='S',gear=gear,length=straight)
        segments.insert(0 if first>last else 1,straight_piece)
    result = materialize(start,segments,radius)
    if not close_pose(endpoint(result[-1]), end):
        raise ValueError('lattice endpoint fidelity')
    total = sum(p['length'] for p in result)
    if abs(total-native['trajectory_length']/.05) > .004 or abs(arc_length-native['arc_length']/.05) > .004:
        raise ValueError('lattice length fidelity')
    check_lattice_samples(result,native,start,gear)
    return result


def analytic_suffix(chain, start_index, radius, method, config, headings, rs):
    """Accept only the OMPL curve reproducing Nav2's full analytic sampling sequence."""
    start, goal = node_pose(chain[start_index]), node_pose(chain[-1])
    def native_pose(node):
        return [2*node['x']+1,2*node['y']+1,node['native_theta_rad']]
    # Preserve the native frame and float bin headings. Wrapping a heading can
    # select another equal-length OMPL word near a tie, losing native fidelity.
    native_start,native_goal=native_pose(chain[start_index]),native_pose(chain[-1])
    answer = rs(dict(op='rs_path',start=native_start,goal=native_goal,radius_cells=2*radius))
    segments=[dict(p,length=p['length']/2) for p in answer['segments']]
    start[2]=native_start[2]
    ps = materialize(start,segments,radius)
    if not ps:
        raise ValueError('empty analytic suffix')
    total = sum(p['length'] for p in ps)
    count = int(np.float32(answer['total_length'])/np.float32(math.sqrt(2)))
    if count < 1:
        raise ValueError('analytic suffix shorter than sample interval')
    # Nav2 samples with float i/N, then stores float positions and heading bins.
    sample = at_lengths(ps, [float(np.float32(i)/np.float32(count))*total for i in range(1,count+1)])
    bins = config['angle_bins']
    def bin_of(theta):
        theta = theta % (2*math.pi)
        if method == 'hybrid':
            return int(math.floor(float(np.float32(theta)/(np.float32(2)*np.float32(math.pi)/np.float32(bins)))+.5)) % bins
        return min(range(len(headings)),key=lambda i:abs(wrap(theta-headings[i])))
    def key(p):
        return int(np.float32(2*p[0]+1)),int(np.float32(2*p[1]+1)),bin_of(p[2])
    goal_key=key(goal)
    # setAnalyticPath omits nodes sharing the goal index, then appends the goal.
    expected=[p for p in sample if key(p)!=goal_key]+[goal]
    actual=np.asarray([node_pose(n) for n in chain[start_index+1:]])
    if len(expected)!=len(actual):
        raise ValueError('analytic sample count fidelity')
    error=float(np.max(np.linalg.norm(np.asarray(expected)[:,:2]-actual[:,:2],axis=1)))
    if error > POSITION_TOL:
        raise ValueError('analytic sample position fidelity')
    if not close_pose(endpoint(ps[-1]),goal):
        raise ValueError('analytic endpoint fidelity')
    return ps,error


def nav2_primitives(reply, radius, rs, headings=None):
    chain=reply['parent_chain']
    if len(chain)<2:
        raise ValueError('empty native chain')
    method=reply['method'];config=reply['effective_config']
    prefixes=[[]]
    captured=[0]
    for i in range(1,len(chain)):
        try:
            edge=hybrid_edge(chain[i-1],chain[i],radius,config['angle_bins']) if method=='hybrid' else lattice_edge(chain[i-1],chain[i])
            prefixes.append(prefixes[-1]+edge)
            captured.append(captured[-1]+int(method=='lattice' and chain[i].get('primitive') is None and chain[i].get('saved_prefix_primitive') is not None))
        except ValueError:
            break
    if len(prefixes)==len(chain):
        return prefixes[-1],dict(analytic_start=None,analytic_max_position_error=0.,captured_prefix_controls=captured[-1])
    for i in range(len(prefixes)-1,-1,-1):
        try:
            suffix,error=analytic_suffix(chain,i,radius,method,config,headings,rs)
            return prefixes[i]+suffix,dict(analytic_start=i,analytic_max_position_error=error,captured_prefix_controls=captured[i])
        except (ValueError,KeyError):
            continue
    raise ValueError('cannot faithfully recover native analytic suffix')


def marching_primitives(reply, radius):
    result=[]
    for p in reply['primitives']:
        a=p['start']
        result.append(primitive([a['x'],a['y'],a['heading_rad']],p['type'],p['gear'],p['length'],radius))
    return result


def check_marching_metrics(reply, primitives):
    """Check API accounting under its exact-zero convention, before quality filtering."""
    native=[p for p in primitives if p['length']!=0.]
    reversals=sum(a['gear']!=b['gear'] for a,b in zip(native,native[1:]))
    if (abs(sum(p['length'] for p in primitives)-reply['reconstructed_length'])>1e-5
            or reversals!=reply['reversal_count']):
        raise ValueError('marching output/metric mismatch')
    return reversals


def quality(primitives, radius):
    ps=[p for p in primitives if p['length']>1e-8]
    length=sum(p['length'] for p in ps)
    cusps=sum(a['gear']!=b['gear'] for a,b in zip(ps,ps[1:]))
    curvature=lambda p: {'L':1.,'R':-1.,'S':0.}[p['type']]/p['radius']
    variation=sum(abs(curvature(a)-curvature(b)) for a,b in zip(ps,ps[1:]) if a['gear']==b['gear'])
    return dict(length_cells=length,gear_reversals=cusps,
                roughness=radius*radius*variation/length if length else 0.,
                mean_abs_curvature_times_radius=radius*sum(p['length']*abs(curvature(p)) for p in ps)/length if length else 0.)
