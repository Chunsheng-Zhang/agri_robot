"""One selected Fields2Cover curve per connection, at the configured radius."""
import math
from collections import OrderedDict
from shapely.geometry import LineString
from .contour import wrap, MAX_POINTS
ENERGY_DEFAULTS={'energy_forward':1.,'energy_turn':1.5,'energy_reverse':1.3,'energy_switch':.5}
from .curves import native_curve


def energy(points, radius, config=None):
    p={**ENERGY_DEFAULTS,**(config or {})}
    out={'length_m':0.,'forward_m':0.,'reverse_m':0.,'turning_m':0.,'switches':0,
         'straight_cost':0.,'turn_extra_cost':0.,'reverse_cost':0.,'switch_cost':0.}
    previous=None
    for a,b in zip(points,points[1:]):
        direction=int(a[3]) if len(a)>3 else 1
        if direction not in (-1,1):raise ValueError('能耗段方向无效')
        ds=math.dist(a[:2],b[:2])
        if ds<1e-9:continue
        if previous is not None and previous!=direction:out['switches']+=1
        previous=direction
        curvature=abs(wrap(b[2]-a[2]))/ds
        extra=(p['energy_turn']-1)*(curvature*radius)**2
        base=p['energy_forward'] if direction==1 else p['energy_reverse']
        out['length_m']+=ds
        if direction==1:
            out['forward_m']+=ds;out['straight_cost']+=base*ds
        else:
            out['reverse_m']+=ds;out['reverse_cost']+=base*ds
        if curvature>1e-3:out['turning_m']+=ds
        out['turn_extra_cost']+=base*extra*ds
    out['switch_cost']=out['switches']*p['energy_switch']
    out['energy_proxy']=sum(out[k] for k in ('straight_cost','turn_extra_cost','reverse_cost','switch_cost'))
    out['unit']='relative_cost';out['calibrated']=False
    return out


class NativeTurns:
    def __init__(self,builder,config,allow_reverse=False):
        self.builder,self.config=builder,config
        self.allow_reverse=allow_reverse
        self.cache=OrderedDict();self.previews=[];self.selected=[];self.total=0

    def _native(self,a,b):
        builder=self.builder;f=builder.f2c
        radius=builder.radius
        robot=f.Robot(builder.robot_width,builder.width);robot.setMinTurningRadius(radius)
        robot.setMaxDiffCurv(builder.curvature_change)
        curve=native_curve(f,builder.curve_type,builder.step)
        builder.check()
        x,y,yaw=a[:3];c,s=math.cos(yaw),math.sin(yaw);dx,dy=b[0]-x,b[1]-y
        path=curve.createTurn(robot,f.Point(0.,0.),0.,f.Point(c*dx+s*dy,-s*dx+c*dy),wrap(b[2]-yaw))
        if path.size()<2 or path.size()>MAX_POINTS:return None
        points=[]
        for i in range(path.size()):
            state=path.getState(i);direction=int(state.dir)
            if direction not in (-1,1):return None
            xx,yy=state.point.getX(),state.point.getY()
            points.append((x+c*xx-s*yy,y+s*xx+c*yy,wrap(yaw+state.angle),direction))
        if any(not all(math.isfinite(v) for v in p) for p in points):return None
        if math.dist(points[-1][:2],b[:2])>1e-4 or abs(wrap(points[-1][2]-b[2]))>1e-4:return None
        metrics=energy(points,builder.radius,self.config)
        reverse=any(p[3]<0 for p in points)
        clear=bool(builder.safe.covers(LineString([p[:2] for p in points])))
        return {'type':builder.curve_type,'radius_m':radius,'points':points,'metrics':metrics,'clearance_valid':clear,
            'executable':clear and (not reverse or self.allow_reverse),
            'reason':('原生曲线包含倒车段，请勾选允许倒车后重新规划' if reverse and not self.allow_reverse else '') if clear else '车体扫掠净空不足'}

    def connection(self,a,b):
        key=tuple(round(float(v),9) for v in (*a[:3],*b[:3]))
        if key in self.cache:
            self.cache.move_to_end(key);return self.cache[key]
        result=self._native(a,b)
        self.total+=1
        if result is not None and len(self.previews)<24:
            points=result['points'];step=max(1,math.ceil(len(points)/200))
            self.previews.append({k:result[k] for k in ('type','radius_m','metrics','clearance_valid','executable','reason')}|
                {'start_pose':list(a[:3]),'end_pose':list(b[:3]),
                 'points':[list(p) for i,p in enumerate(points) if i%step==0 or i==len(points)-1]})
        self.cache[key]=result
        # Bound retained geometry as well as edge count on small Jetson systems.
        while len(self.cache)>96 or sum(len(c['points']) for c in self.cache.values() if c is not None)>200000:
            self.cache.popitem(last=False)
        return result

    def connect(self,a,b):
        best=self.connection(a,b)
        if best is None:raise ValueError(f'{self.builder.curve_type} 未生成有效连接曲线')
        if not best['executable']:raise ValueError(best['reason'])
        self.selected.append({k:best[k] for k in ('type','radius_m','metrics')})
        return list(best['points'])
