"""Bounded headland search from field geometry and the selected native curve.

The range is a search envelope, not proof that a width permits a complete path.
Every coarse/refined candidate still passes the normal whole-path checks.
"""
import math


def initial_widths(builder, mode, requested):
    if mode=='MANUAL':
        return [requested],{'schema_version':1,'method':'MANUAL','range_min_m':requested,
            'range_max_m':requested,'initial_widths_m':[requested],'refinement_widths_m':[]}
    builder.check()
    field=builder.field
    # Leave room for a working centreline/vehicle inside the reserved band.
    margin=max(builder.width/2,builder.robot_width/2)
    lower=builder.robot_width/2+1e-4
    x0,y0,x1,y1=field.bounds
    left,right=0.,min(50.,min(x1-x0,y1-y0)/2)
    def usable(h):
        inner=field.buffer(-(h+margin),join_style=2)
        return not inner.is_empty and inner.area>1e-6
    if usable(right):left=right
    else:
        for _ in range(24):
            builder.check();middle=(left+right)/2
            if usable(middle):left=middle
            else:right=middle
    geometric_limit=math.floor((left-1e-4)*1e6)/1e6
    if geometric_limit<lower:raise ValueError('地块过窄，无法自动预留地头并保留车体/作业中心线空间')
    # Probe actual F2C geometry at one fixed vehicle radius. Different strip
    # separations account for adjacent/grouped orderings, not radius candidates.
    probes=[]
    for count in (1,2,4):
        builder.check();gap=builder.width*count
        curve=builder.curve_between((0.,0.,0.),(0.,gap,math.pi))
        if not curve:continue
        extent=max(0.,max(p[0] for p in curve))+builder.robot_width/2
        probes.append({'strip_gap_m':gap,'forward_envelope_m':extent})
    estimate=min((v['forward_envelope_m'] for v in probes),default=builder.radius+margin)
    # An enormous field must not imply an enormous headland. Start around the
    # reference turn envelope plus one working-width allowance; expand only
    # when the coarse search found no feasible complete path.
    native_limit=max((v['forward_envelope_m'] for v in probes),default=2*builder.radius+margin)+max(builder.width,builder.robot_width)
    upper=max(lower,min(geometric_limit,native_limit))
    seed=max(lower,min(upper,estimate))
    widths=list(dict.fromkeys(round(h,6) for h in (seed,lower,upper)))
    return widths,{'schema_version':1,'method':'GEOMETRY_NATIVE_ADAPTIVE',
        'range_min_m':min(widths),'range_max_m':max(widths),
        'geometric_limit_m':geometric_limit,'native_envelope_limit_m':native_limit,
        'initial_widths_m':widths,'refinement_widths_m':[],
        'native_turn_probes':probes,'native_turn_estimate_m':estimate,
        'width_resolution_m':max(.02,min(.1,builder.width/10)),
        'vehicle_radius_m':builder.radius,'robot_width_m':builder.robot_width,
        'operation_width_m':builder.width,'curve_type':builder.curve_type,
        'allow_reverse':builder.allow_reverse,
        'limitations':'几何边界和原生转弯包络只用于建立搜索范围；完整路径仍须校验，有限预算不保证全局最优。'}


def refine_widths(search, chosen_width):
    """Bisect the untested neighbours of the best width; at most two per round."""
    if search['method']=='MANUAL':return []
    known=sorted(set(search['initial_widths_m']+search['refinement_widths_m']))
    if chosen_width is None:
        limit=search.get('geometric_limit_m',search['range_max_m'])
        if search['range_max_m']<limit-1e-6:
            trial=round(min(limit,search['range_max_m']+max(search['range_max_m']-search['range_min_m'],search.get('operation_width_m',.1))),6)
            search['range_max_m']=trial
            search['refinement_widths_m'].append(trial)
            return [trial]
        gaps=[(b-a,a,b) for a,b in zip(known,known[1:]) if b-a>search['width_resolution_m']]
        if not gaps:return []
        _,a,b=max(gaps);trial=round((a+b)/2,6)
        search['refinement_widths_m'].append(trial)
        return [trial]
    lo=max((h for h in known if h<chosen_width-1e-6),default=chosen_width)
    hi=min((h for h in known if h>chosen_width+1e-6),default=chosen_width)
    trials=[]
    for other in (lo,hi):
        if abs(chosen_width-other)>search['width_resolution_m']:
            h=round((chosen_width+other)/2,6)
            if h not in known and h not in trials:trials.append(h)
    search['refinement_widths_m'].extend(trials)
    return trials
