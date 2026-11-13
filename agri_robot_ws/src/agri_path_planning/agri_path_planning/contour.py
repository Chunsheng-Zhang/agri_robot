"""Inside-out contour coverage in the project's fixed local map coordinates.

Inspired by Coordinateself.o_rote/loop in Ming2zun's Farmland-Path-Planning
(Apache-2.0). This implementation uses general polygon offsets, bounded work,
smooth corner curves, Fields2Cover forward connectors, and full-path clearance instead of its four-line
intersection routines. No upstream source code is copied. See docs/回形路径规划.md.
No ROS imports: called in an isolated worker so safety callbacks keep running.
"""
import math
from time import monotonic

from shapely.geometry import LineString, Polygon, MultiPoint
from shapely.geometry.polygon import orient
from shapely import wkb
from .curves import CURVES, curve_name, native_curve

MAX_LAYERS = 500
MAX_POINTS = 150000
MAX_RINGS = 2000
_CANCEL = None


class SpiralConnectionError(ValueError):
    """A seam choice failed; another starting seam may still be feasible."""


def set_cancellation(event):
    global _CANCEL
    _CANCEL = event


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def bezier(control, t):
    n=len(control)-1
    return tuple(sum(math.comb(n,i)*(1-t)**(n-i)*t**i*control[i][j]
                     for i in range(n+1)) for j in (0,1))


def smooth_profile(angle):
    """Unit quintic fillet, zero endpoint curvature, sampled k and dk/ds bounds."""
    c,s=math.cos(angle),math.sin(angle)
    p=[(-1.,0.),(-.6,0.),(-.2,0.),(.2*c,.2*s),(.6*c,.6*s),(c,s)]
    d=[(5*(v[0]-u[0]),5*(v[1]-u[1])) for u,v in zip(p,p[1:])]
    dd=[(4*(v[0]-u[0]),4*(v[1]-u[1])) for u,v in zip(d,d[1:])]
    ddd=[(3*(v[0]-u[0]),3*(v[1]-u[1])) for u,v in zip(dd,dd[1:])]
    kmax=qmax=0.
    for i in range(257):
        t=i/256;x,y=bezier(d,t);xx,yy=bezier(dd,t);xxx,yyy=bezier(ddd,t)
        v=x*x+y*y
        if v<1e-12:raise ValueError('回形拐角存在不可行的掉头尖点')
        cross=x*yy-y*xx
        kmax=max(kmax,abs(cross)/v**1.5)
        qmax=max(qmax,abs((x*yyy-y*xxx)/v**2-3*cross*(x*xx+y*yy)/v**3))
    return p,d,kmax,qmax


def pieces(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == 'Polygon':
        return [geometry]
    return [p for g in geometry.geoms for p in pieces(g)]


class Builder:
    def __init__(self, field, width, headland, radius, spacing, robot_width,
                 continuity='CONTINUOUS', curvature_change=2., timeout=60., allow_reverse=False):
        values = (width, radius, spacing, robot_width, curvature_change, timeout)
        if not all(math.isfinite(v) and v > 0 for v in values) or not math.isfinite(headland) or headland < 0:
            raise ValueError('回形参数必须有限：幅宽、半径、采样间距和车宽须为正，地头宽度非负')
        self.curve_type = curve_name(continuity)
        if not isinstance(allow_reverse,bool):raise ValueError('允许倒车须为布尔值')
        self.allow_reverse=allow_reverse
        continuity = CURVES[self.curve_type][2]
        if not field.is_valid or field.is_empty or field.geom_type != 'Polygon':
            raise ValueError('回形要求有效单地块多边形，可包含禁入区')
        import fields2cover as f2c
        self.f2c, self.field = f2c, field
        self.width, self.headland, self.radius = width, headland, radius
        self.robot_width = robot_width
        # Fine curve sampling bounds chord error even when display spacing is coarse.
        self.step = min(spacing, radius*.08)
        self.safe = field.buffer(-robot_width/2).buffer(1e-6)
        self.robot = f2c.Robot(robot_width, width)
        self.robot.setMinTurningRadius(radius)
        self.robot.setMaxDiffCurv(curvature_change)
        self.curvature_change = curvature_change
        self.profiles = {}
        self.curve = native_curve(f2c, self.curve_type, self.step)
        # F2C's millimetre/milliradian cache keys can alias different corners.
        # Endpoint/tangent matching here is stricter than that cache precision.
        self.curve.setUsingCache(False)
        self.continuity = continuity
        self.trim_radius = radius*1.01
        self.deadline = monotonic()+timeout
        self.samples = self.attempts = 0
        self.rejected = []

    def check(self):
        if _CANCEL is not None and _CANCEL.is_set():
            raise ValueError('回形规划已取消')
        if monotonic() > self.deadline:
            raise ValueError('回形规划超时，未发布部分路径')
        if self.samples > MAX_POINTS or self.attempts > 30000:
            raise ValueError('回形采样/连接预算超限，请增大幅宽或缩小地块')

    def curve_between(self, start, end, forward_only=False):
        self.check(); self.attempts += 1
        x,y,yaw = start[:3]
        dx,dy=end[0]-x,end[1]-y
        c,s=math.cos(yaw),math.sin(yaw)
        local=self.curve.createTurn(self.robot,self.f2c.Point(0.,0.),0.,
            self.f2c.Point(dx*c+dy*s,-dx*s+dy*c),wrap(end[2]-yaw))
        if local.size()<2 or not math.isfinite(local.length()):
            return []
        if local.size()>MAX_POINTS:
            raise ValueError('回形单段曲线采样超限')
        result=[]
        for i in range(local.size()):
            state=local.getState(i)
            direction=int(state.dir)
            if direction not in (-1,1):return []
            if direction<0 and (forward_only or not self.allow_reverse):
                return []  # Try another attachment, never erase reverse direction.
            a,b=state.point.getX(),state.point.getY()
            p=(x+a*c-b*s,y+a*s+b*c,wrap(state.angle+yaw))
            if self.allow_reverse:p=(*p,direction)
            if not all(math.isfinite(v) for v in p):raise ValueError('回形连接含非有限坐标')
            result.append(p)
        # The native curve must join the requested poses, not merely approach them.
        if math.dist(result[-1][:2],end[:2])>1e-4 or abs(wrap(result[-1][2]-end[2]))>1e-4:
            return []
        return result

    def rounded_ring(self, coordinates):
        for scale in ([1.,1.25,1.5,2.] if self.continuity == 'CONTINUOUS' else [1.]):
            result = self._rounded_ring(coordinates,scale)
            if result is not None:return result
        return None

    def _rounded_ring(self, coordinates, scale, closed=True):
        vertices=list(coordinates)[:-1] if closed else list(coordinates)
        # Remove redundant points so a straight boundary needs no corner curve.
        changed=True
        while changed and len(vertices)>3:
            changed=False
            for i in (range(len(vertices)) if closed else range(1,len(vertices)-1)):
                b=vertices[i]
                a,c=vertices[i-1],vertices[(i+1)%len(vertices)]
                u,v=(b[0]-a[0],b[1]-a[1]),(c[0]-b[0],c[1]-b[1])
                if math.hypot(*u)<1e-7 or math.hypot(*v)<1e-7 or (
                    abs(u[0]*v[1]-u[1]*v[0])<1e-8*math.hypot(*u)*math.hypot(*v) and u[0]*v[0]+u[1]*v[1]>0):
                    vertices.pop(i);changed=True;break
        cuts=[]; directions=[]; profiles=[]
        for i,b in enumerate(vertices):
            if not closed and i in (0,len(vertices)-1):
                a,c=(b,vertices[1]) if i==0 else (vertices[-2],b)
                yaw=math.atan2(c[1]-a[1],c[0]-a[0])
                cuts.append(0.);directions.append((yaw,yaw));profiles.append(None)
                continue
            a,c=vertices[i-1],vertices[(i+1)%len(vertices)]
            before=math.atan2(b[1]-a[1],b[0]-a[0]);after=math.atan2(c[1]-b[1],c[0]-b[0])
            angle=wrap(after-before)
            if abs(angle)>math.pi-1e-3:return None
            profile=None
            if self.continuity=='CONTINUOUS':
                key=round(angle,12)
                if key not in self.profiles:self.profiles[key]=smooth_profile(angle)
                profile=self.profiles[key]
                cut=max(self.radius*profile[2],math.sqrt(profile[3]/self.curvature_change))*1.02
            else:
                cut=self.trim_radius*math.tan(abs(angle)/2)
            cuts.append(scale*cut)
            directions.append((before,after))
            profiles.append(profile)
        if any(cuts[i]+cuts[(i+1)%len(vertices)] >= math.dist(b,vertices[(i+1)%len(vertices)])-1e-5
               for i,b in enumerate(vertices if closed else vertices[:-1])):
            return None
        corners=[]
        for i,b in enumerate(vertices):
            self.check()
            before,after=directions[i];cut=cuts[i]
            if cut<1e-7:
                a=(*b,before);c=(*b,after)
                corner=[a,c]
            else:
                corner=None
                # A shallow concave CC turn can need a longer zero-curvature
                # entry than the quintic estimate. Expand this corner only:
                # scaling every corner together unnecessarily consumes all
                # neighbouring edge space on the user's measured five-point field.
                for local_scale in ([1.,1.25,1.5,2.,3.,4.] if self.continuity=='CONTINUOUS' else [1.]):
                    candidate_cut=cut*local_scale
                    a=(b[0]-candidate_cut*math.cos(before),b[1]-candidate_cut*math.sin(before),before)
                    c=(b[0]+candidate_cut*math.cos(after),b[1]+candidate_cut*math.sin(after),after)
                    candidate=self.curve_between(a,c,forward_only=True)
                    if not candidate:continue
                    triangle=Polygon([a[:2],b,c[:2]]).buffer(1e-5)
                    if not triangle.covers(LineString([p[:2] for p in candidate])):continue
                    cuts[i]=candidate_cut;corner=candidate;break
                if corner is None:return None
            corners.append(corner)
        # Expanded native fillets must still leave a nonnegative straight edge.
        if any(cuts[i]+cuts[(i+1)%len(vertices)] >= math.dist(b,vertices[(i+1)%len(vertices)])-1e-5
               for i,b in enumerate(vertices if closed else vertices[:-1])):return None
        result=[];anchors=[];spans=[]
        for i,corner in enumerate(corners):
            start=len(result)
            result.extend(corner)
            spans.append((vertices[i],start,len(result)-1))
            if not closed and i==len(corners)-1:break
            a=corner[-1];b=corners[(i+1)%len(corners)][0]
            length=math.dist(a[:2],b[:2]);count=max(1,math.ceil(length/self.step))
            for j in range(1,count+1):
                t=j/count
                # Keep the shared native endpoint exact: rounding a + (b-a)
                # can create a microscopic backtrack that GEOS calls a crossing.
                result.append(b if j==count else
                    (a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),a[2]))
                if .2<=t<=.8:anchors.append(len(result)-1)
            self.samples+=len(corner)+count;self.check()
        if not self.safe.covers(LineString([p[:2] for p in result])):return None
        if not closed:return result,spans
        if not anchors:return None
        # Rotate closed ring to a straight attachment point (zero curvature for CC).
        base=anchors[0];unique=result[:-1]
        ring=unique[base:]+unique[:base]
        ring.append(ring[0])
        mapped=sorted(set((i-base)%len(unique) for i in anchors if i<len(unique)))
        return ring,mapped

    def reference_spiral(self, rings, start_pose):
        """O ordering from Coordinateself.loop/intersection_points, reversed.

        The join is the intersection of this layer's first edge and the next
        layer's last edge. It replaces a corner rather than closing a lap and
        adding a lateral connector. Vector intersections also support vertical
        edges, unlike the reference slope/intercept implementation.
        """
        boundary=list(orient(self.field,sign=1.).exterior.coords)[:-1]
        axis=(boundary[1][0]-boundary[0][0],boundary[1][1]-boundary[0][1])
        def ordered(ring):
            vertices=ring['vertices'];best=None
            for i,a in enumerate(vertices):
                b=vertices[(i+1)%len(vertices)];dx,dy=b[0]-a[0],b[1]-a[1]
                score=(dx*axis[0]+dy*axis[1])/math.hypot(dx,dy)/math.hypot(*axis)
                if best is None or score>best[0]:best=(score,i)
            i=best[1]
            return vertices[i:]+vertices[:i]
        def intersection(a,b,c,d):
            u=(b[0]-a[0],b[1]-a[1]);v=(d[0]-c[0],d[1]-c[1])
            cross=u[0]*v[1]-u[1]*v[0]
            if abs(cross)<1e-10:raise ValueError('回形相邻边平行，无法生成O形交点')
            t=((c[0]-a[0])*v[1]-(c[1]-a[1])*v[0])/cross
            return a[0]+t*u[0],a[1]+t*u[1]
        # The innermost reference pass is one terminal edge. If its rounded
        # transition does not fit, remove that depth and retry; never lower R.
        for skip in range(len(rings)):
            self.check();used=rings[skip:];outside=list(reversed(used))
            if skip and len(used)==1:
                break  # Do not disguise failed cross-layer joins as one outer pass.
            raw=[];gates=[]
            for i,ring in enumerate(outside):
                v=ordered(ring)
                if i==len(outside)-1:
                    raw.extend([v[0],v[-1]] if len(outside)>1 else [v[0],*reversed(v[1:])])
                else:
                    raw.extend([v[0],*reversed(v[1:])])
                    inner=ordered(outside[i+1])
                    gate=intersection(v[0],v[1],inner[-1],inner[0])
                    raw.append(gate);gates.append(gate)
            raw.reverse();gates.reverse()
            rounded=None
            for scale in ([1.,1.25,1.5,2.] if self.continuity=='CONTINUOUS' else [1.]):
                rounded=self._rounded_ring(raw,scale,closed=False)
                if rounded is not None:break
            if rounded is None:continue
            result,spans=rounded
            if not LineString([p[:2] for p in result]).is_simple:continue
            joins=[]
            for number,gate in enumerate(gates):
                match=next((span for span in spans if math.dist(span[0],gate)<1e-7),None)
                if match is None:break
                joins.append({'start_index':match[1],'end_index':match[2],
                    'from_layer':number,'to_layer':number+1,'model':self.curve_type})
            if len(joins)!=len(gates):continue
            layers=[]
            for i,ring in enumerate(used):
                begin=joins[i-1]['end_index'] if i else 0
                end=joins[i]['start_index'] if i<len(joins) else len(result)-1
                layers.append({k:ring[k] for k in ('level','offset_m','kind','area_m2')}|
                    {'start_index':begin,'end_index':end,'closed':False,
                     'seam_gap_m':math.dist(result[begin][:2],result[end][:2])})
            if start_pose is not None:
                approach=self.curve_between(start_pose,result[0])
                if not approach or not self.safe.covers(LineString([p[:2] for p in approach])):continue
                shift=len(approach)-1
                result=approach[:-1]+result
                for item in [*joins,*layers]:
                    item['start_index']+=shift;item['end_index']+=shift
                joins.insert(0,{'start_index':0,'end_index':shift,'from_layer':-1,
                    'to_layer':0,'model':self.curve_type})
            self.rejected.extend({'level':r['level'],'kind':r['kind'],
                'reason':'O形内侧连续转角空间不足'} for r in rings[:skip])
            return result,layers,joins,used,raw
        raise ValueError('回形O形连续转角无可行路径：半径/曲率过渡空间不足')

    def open_choices(self, ring, last=False):
        """Open a straight seam, retaining the rest of the rounded contour.

        Enter downstream and leave upstream on the same edge. The unused gap
        gives the native connection room to shift outward without closing a lap.
        """
        points=ring['points'][:-1];groups=[]
        for i in ring['anchors']:
            if (not groups or i>groups[-1][-1]+2 or
                    abs(wrap(points[i][2]-points[groups[-1][-1]][2]))>1e-6):groups.append([])
            groups[-1].append(i)
        choices=[]
        for group in groups:
            if len(group)<2:continue
            entry,gate_exit=group[-1],group[0]
            # The final outer pass stays open too; a sub-swath gap avoids an
            # exact return to its entry while finishing the outer boundary.
            exit_index=(entry-1)%len(points) if last else gate_exit
            indices=list(range(entry,len(points)))+list(range(exit_index+1)) if exit_index<entry else list(range(entry,exit_index+1))
            arc=[points[i] for i in indices]
            gap=math.dist(points[entry][:2],points[gate_exit][:2])
            if len(arc)>2:
                choices.append({'points':arc,'entry':points[entry],'gate_exit':points[gate_exit],
                    'gap_m':gap,'omitted_gap_m':math.dist(arc[0][:2],arc[-1][:2])})
        return sorted(choices,key=lambda c:-c['gap_m'])

    def compose_spiral(self, rings, choices, first, start_pose):
        result=[tuple(start_pose[:3])] if start_pose is not None else []
        layers=[];connections=[];previous=None
        # A single convex cell has a single winding with no crossings. Hole/
        # branch transitions or an external approach may need to cross earlier
        # passes, but every segment still must pass full-field clearance.
        simple=(start_pose is None and not self.field.interiors and
                math.isclose(self.field.area,self.field.convex_hull.area,rel_tol=1e-9) and
                len({r['level'] for r in rings})==len(rings))
        for number,ring in enumerate(rings):
            self.check();best=None
            options=[first] if number==0 else choices[number]
            for choice in options:
                self.check();connection=[]
                if result:
                    connection=self.curve_between(result[-1],choice['entry'])
                    if not connection:continue
                    if not self.safe.covers(LineString([p[:2] for p in connection])):continue
                    if previous is not None and abs(wrap(result[-1][2]-choice['entry'][2]))<1e-5:
                        # Restrict a parallel outward shift to the opened seam
                        # corridor; reject turn solutions that make a loop.
                        gate=MultiPoint([previous['entry'][:2],previous['gate_exit'][:2],
                            choice['entry'][:2],choice['gate_exit'][:2]]).convex_hull.buffer(1e-5)
                        if not gate.covers(LineString([p[:2] for p in connection])):continue
                trial=result+(connection[1:] if connection else [])
                arc=choice['points']
                trial+=arc[1:] if trial else arc
                if simple and not LineString([p[:2] for p in trial]).is_simple:continue
                length=sum(math.dist(a[:2],b[:2]) for a,b in zip(connection,connection[1:]))
                if best is None or length<best[0]:best=(length,choice,connection)
            if best is None:
                raise SpiralConnectionError(f'回形第{ring["level"]+1}层无可行的连续向外接缝；未退回闭环串联')
            _,choice,connection=best
            connection_start=max(0,len(result)-1)
            if connection:
                result[-1]=connection[0];result.extend(connection[1:])
                connections.append({'start_index':connection_start,'end_index':len(result)-1,
                    'from_layer':number-1,'to_layer':number,'model':self.curve_type})
            start=max(0,len(result)-1) if result else 0
            if result:result[-1]=choice['points'][0];result.extend(choice['points'][1:])
            else:result.extend(choice['points'])
            layers.append({k:ring[k] for k in ('level','offset_m','kind','area_m2')}|
                {'start_index':start,'end_index':len(result)-1,'closed':False,
                 'seam_gap_m':choice['omitted_gap_m']})
            previous=choice
            if len(result)>MAX_POINTS:raise ValueError('回形最终路径点数超限')
        return result,layers,connections

    def generate(self, max_offset=None, start_pose=None):
        # Preserve the existing manually reserved headland, then place the
        # outermost working centreline half an implement width inside it.
        rings=[];offset=self.headland+max(self.width/2,self.robot_width/2+1e-5)
        for level in range(MAX_LAYERS):
            self.check()
            if max_offset is not None and offset>max_offset+1e-6:break
            inner=self.field.buffer(-offset,join_style=2)
            if inner.is_empty:break
            for cell in sorted(pieces(inner),key=lambda p:p.bounds):
                cell=orient(cell,sign=1.)
                for kind,coordinates in [('outer',cell.exterior.coords),*[('hole',h.coords) for h in cell.interiors]]:
                    rounded=self.rounded_ring(coordinates)
                    if rounded:
                        rings.append({'level':level,'offset_m':offset,'kind':kind,'area_m2':cell.area,
                                      'points':rounded[0],'anchors':rounded[1],
                                      'vertices':list(coordinates)[:-1]})
                        if len(rings)>MAX_RINGS:raise ValueError('回形轮廓数量超限')
                    else:self.rejected.append({'level':level,'kind':kind,'reason':'半径/曲率过渡空间或净空不足'})
            offset+=self.width
        else:raise ValueError('回形层数超限，请增大作业幅宽')
        if not rings:raise ValueError('回形无可行轮廓：请检查地头宽度、幅宽、最小半径及曲率模式')
        if not any(r['level']==0 and r['kind']=='outer' for r in rings):
            raise ValueError('回形最外层不满足净空/转弯约束，不能只发布内部局部路径')
        rings.sort(key=lambda r:(-r['level'],r['kind'],r['points'][0][:2]))
        raw=[]
        # A mild concavity is common in measured farm boundaries. It does not
        # require a branched route while all offsets remain single polygons.
        # Use the O ordering there too, subject to the same simplicity/clearance
        # checks, rather than silently returning the former seam algorithm.
        reference=(not self.field.interiors and
            len({r['level'] for r in rings})==len(rings))
        if reference:
            result,layers,connections,rings,raw=self.reference_spiral(rings,start_pose)
        else:
            choices=[self.open_choices(r,last=i==len(rings)-1) for i,r in enumerate(rings)]
            if any(not c for c in choices):raise ValueError('回形缺少可打开的直线接缝，未发布闭环路径')
            for first in choices[0][:8]:
                try:
                    result,layers,connections=self.compose_spiral(rings,choices,first,start_pose)
                    break
                except SpiralConnectionError as exc:last_error=exc
            else:raise ValueError(str(last_error))
        # Omit exact consecutive duplicates while preserving layer/seam indices.
        clean=[];index_map=[]
        for p in result:
            if self.allow_reverse:p=(*p[:3],p[3] if len(p)>3 else 1)
            if not clean or math.dist(p[:2],clean[-1][:2])>1e-8:clean.append(p)
            elif self.allow_reverse:clean[-1]=p
            index_map.append(len(clean)-1)
        for layer in [*layers,*connections]:
            for key in ('start_index','end_index'):layer[key]=index_map[layer[key]]
        if not self.safe.covers(LineString([p[:2] for p in clean])):raise ValueError('回形最终路径净空校验失败')
        return {'points':clean,'metadata':{'mode':'CONTOUR','direction':'INSIDE_OUT',
            'topology':'OPEN_SPIRAL','closed_laps':False,'connections':connections,
            'ordering':'REFERENCE_O_REVERSED' if reference else 'BRANCHED_OPEN_CONTOURS',
            'raw_spiral_vertices':[list(p) for p in raw],
            'layers':layers,'layer_index_basis':'points','raw_samples':len(result),
            'rejected_rings':self.rejected,'ring_count':len(rings),'spacing_m':self.width,
            'effective_sampling_m':self.step,'headland_width_m':self.headland,
            'curve_type':self.curve_type,
            'allow_reverse':self.allow_reverse,
            'corner_model':self.curve_type,
            'connector_model':self.curve_type}}


def plan_contour(field_wkb, **parameters):
    """Serializable worker entry. All data is local metres, no origin changes."""
    return Builder(wkb.loads(field_wkb),**parameters).generate()
