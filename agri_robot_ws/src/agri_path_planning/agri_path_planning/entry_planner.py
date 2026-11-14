"""Reuse OpenNav swaths/F2C orderers and native curves with a manual entry."""
import math
from time import monotonic
from shapely import wkb
from shapely.geometry import Polygon
from . import contour
from .contour import Builder, MAX_POINTS
from .curves import curve_name
from .entry import EntryConnector
from .field import validate_path
from agri_base.path_geometry import directed_poses


def manual_entry_path(field_wkb,rows,p):
    import fields2cover as f
    field=wkb.loads(field_wkb)
    builder=Builder(field,p['operation_width'],p['headland_width'],p['min_turning_radius'],
        p['sampling_distance'],p['robot_width'],p['path_continuity'],p['linear_curv_change'],
        p.get('planning_timeout',120.),p.get('allow_reverse',False))
    entry=EntryConnector(field,p,p['entry'],builder.check)
    swaths=f.Swaths()
    if not rows:raise ValueError('OpenNav 未返回可用作业行')
    for a,b in rows:
        line=f.LineString();line.addPoint(*a);line.addPoint(*b)
        swaths.push_back(f.Swath(line,p['operation_width']))
    orderer={'BOUSTROPHEDON':f.RP_Boustrophedon,'SNAKE':f.RP_Snake,'SPIRAL':f.RP_Spiral}[p['route_mode']]()
    if p['route_mode']=='SPIRAL':orderer.setSpiralSize(4)  # OpenNav RouteMode.spiral_n default.
    candidates=[];best=None
    for variant in range(4):
        builder.check();row={'variant':variant,'state':'REJECTED'}
        try:
            ordered=orderer.genSortedSwaths(swaths,variant)
            points=[]
            for i in range(ordered.size()):
                builder.check();swath=ordered[i]
                a,b=swath.startPoint(),swath.endPoint()
                start=(a.getX(),a.getY(),math.atan2(b.getY()-a.getY(),b.getX()-a.getX()))
                end=(b.getX(),b.getY(),start[2])
                if points:
                    link=builder.curve_between(points[-1],start)
                    if not link:raise ValueError('原生曲线无法连接该起步方式的作业行')
                    points.extend(link[1:])
                elif entry:
                    connection=entry.assess(start)
                count=max(1,math.ceil(math.dist(start[:2],end[:2])/p['sampling_distance']))
                if len(points)+count+1>MAX_POINTS:raise ValueError('入口规划采样点数超限')
                points.extend([(start[0]+(end[0]-start[0])*j/count,start[1]+(end[1]-start[1])*j/count,start[2]) for j in range(count+1)])
            validate_path([v[:2] for v in points],field,p['robot_width'])
            _,directions=directed_poses(points)
            if any(d<0 for d in directions) and not p.get('allow_reverse',False):raise ValueError('该起步方式含未授权倒车')
            length=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))
            row.update(state='EVALUATED',entry_length_m=connection['distance_m'],working_length_m=length)
            rank=(connection['distance_m'],length,variant)
            if best is None or rank<best[0]:best=(rank,points,connection,variant)
        except ValueError as exc:
            builder.check();row['reason']=str(exc)
        candidates.append(row)
    if best is None:raise ValueError('四种起步方式均不可行：'+str(candidates))
    _,points,connection,variant=best
    return {'points':points,'metadata':{'mode':p['route_mode'],'curve_type':curve_name(p['path_continuity']),
        'allow_reverse':p.get('allow_reverse',False),'entry':connection,'entry_variant':variant,
        'entry_candidates':candidates,'swath_source':'OpenNav Coverage','ordering_source':'Fields2Cover native variants'}}


def contour_entry_path(field_wkb,p):
    field=wkb.loads(field_wkb);vertices=list(field.exterior.coords)[:-1]
    deadline=monotonic()+p.get('planning_timeout',120.)
    best=None;failures=[]
    for index in range(len(vertices)):
        if contour._CANCEL is not None and contour._CANCEL.is_set():raise ValueError('入口规划已取消')
        if monotonic()>deadline:raise ValueError('入口回形规划超时，未发布部分路径')
        boundary=vertices[index:]+vertices[:index]
        candidate=Polygon(boundary,[list(h.coords) for h in field.interiors])
        try:
            builder=Builder(candidate,p['operation_width'],p['headland_width'],p['min_turning_radius'],
                p['sampling_distance'],p['robot_width'],p['path_continuity'],p['linear_curv_change'],
                max(.001,deadline-monotonic()),p.get('allow_reverse',False))
            value=builder.generate()
            connection=EntryConnector(field,p,p['entry'],builder.check).assess(value['points'][0])
            rank=(connection['distance_m'],sum(math.dist(a[:2],b[:2]) for a,b in zip(value['points'],value['points'][1:])),index)
            if best is None or rank<best[0]:best=(rank,value,connection,index)
        except ValueError as exc:
            if contour._CANCEL is not None and contour._CANCEL.is_set():raise
            failures.append(str(exc))
    if best is None:raise ValueError('没有满足入口/回形净空的由内向外路径：'+'; '.join(failures))
    if monotonic()>deadline:raise ValueError('入口回形规划超时，未发布部分路径')
    _,value,connection,index=best
    value['metadata'].update(entry=connection,entry_variant=index,entry_seams_tested=len(vertices))
    return value
