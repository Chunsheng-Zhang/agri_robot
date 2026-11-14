"""Fields2Cover rows joined by straight travel and explicit stopped turns."""
import heapq
import math
from collections import OrderedDict
from shapely.geometry import LineString
from .contour import pieces, wrap, MAX_POINTS
from agri_base.path_geometry import point_turn_indices


def field_cell(field, f):
    def ring(coords):
        return f.LinearRing(f.VectorPoint([f.Point(float(x),float(y)) for x,y in coords]))
    cell=f.Cell(ring(field.exterior.coords))
    for hole in field.interiors:cell.addRing(ring(hole.coords))
    return cell


class StraightRouter:
    """Visibility paths inside the eroded field, including holes; no live avoidance."""
    def __init__(self, safe, check):
        self.safe=safe;self.cover=safe.buffer(1e-6);self.check=check
        self.vertices=[];self.graph=None;self.cache=OrderedDict()
        for cell in pieces(safe):
            for ring in [cell.exterior,*cell.interiors]:self.vertices.extend(list(ring.coords)[:-1])
        self.vertices=list(dict.fromkeys(self.vertices))

    def visible(self,a,b):
        return self.cover.covers(LineString([a,b]))

    def route(self,a,b):
        self.check();a,b=tuple(a),tuple(b)
        if math.dist(a,b)<1e-7:return [a,b]
        key=(a,b)
        if key in self.cache:return self.cache[key]
        if self.visible(a,b):result=[a,b]
        else:
            if len(self.vertices)>384:raise ValueError('地块转折/孔洞顶点超过直线连接预算')
            if self.graph is None:
                self.graph=[[] for _ in self.vertices]
                for i,u in enumerate(self.vertices):
                    self.check()
                    for j in range(i):
                        v=self.vertices[j]
                        if self.visible(u,v):
                            d=math.dist(u,v);self.graph[i].append((j,d));self.graph[j].append((i,d))
            n=len(self.vertices);nodes=self.vertices+[a,b]
            edges=[list(e) for e in self.graph]+[[],[]]
            for i,v in enumerate(self.vertices):
                if i%32==0:self.check()
                for j,p in [(n,a),(n+1,b)]:
                    if self.visible(v,p):
                        d=math.dist(v,p);edges[i].append((j,d));edges[j].append((i,d))
            distances={n:0.};previous={};queue=[(0.,n)]
            while queue:
                d,i=heapq.heappop(queue)
                if d!=distances[i]:continue
                if i==n+1:break
                for j,cost in edges[i]:
                    new=d+cost
                    if new<distances.get(j,math.inf):
                        distances[j]=new;previous[j]=i;heapq.heappush(queue,(new,j))
            if n+1 not in distances:raise ValueError('内缩后区域不连通，无法安全直线换行')
            indices=[n+1]
            while indices[-1]!=n:indices.append(previous[indices[-1]])
            result=[nodes[i] for i in reversed(indices)]
        self.cache[key]=result
        while len(self.cache)>2048:self.cache.popitem(last=False)
        return result

    def distance(self,a,b):
        p=self.route(a,b)
        return sum(math.dist(u,v) for u,v in zip(p,p[1:]))


def working_rows(field, width, angle, variant=0):
    import fields2cover as f
    swaths=f.Swaths()
    for cell in pieces(field):
        part=f.SG_BruteForce().generateSwaths(float(angle),float(width),field_cell(cell,f))
        for i in range(part.size()):swaths.push_back(part[i])
    if not swaths.size():raise ValueError('安全地头内缩后没有可用作业行')
    if swaths.size()>1500:raise ValueError('作业行数量超过1500条预算')
    ordered=f.RP_Boustrophedon().genSortedSwaths(swaths,variant)
    return [((s.startPoint().getX(),s.startPoint().getY()),
             (s.endPoint().getX(),s.endPoint().getY())) for s in ordered]


def improve_rows(rows, router, check, entry=None):
    """Bounded 2-opt over nearby rows; every working row is still visited once."""
    best=list(rows);changes=0
    for _ in range(2):
        improved=False
        for i in range(max(0,len(best)-1)):
            check()
            for j in range(i+1,min(len(best),i+5)):
                old=new=0.
                if not i and entry is not None:
                    old+=router.distance(entry,best[i][0])
                    new+=router.distance(entry,best[j][1])
                if i:
                    old+=router.distance(best[i-1][1],best[i][0])
                    new+=router.distance(best[i-1][1],best[j][1])
                if j+1<len(best):
                    old+=router.distance(best[j][1],best[j+1][0])
                    new+=router.distance(best[i][0],best[j+1][0])
                if new+1e-5<old:
                    best[i:j+1]=[(b,a) for a,b in reversed(best[i:j+1])]
                    changes+=1;improved=True;break
        if not improved:break
    return best,changes


def compose(rows, router, spacing, check):
    vertices=[]
    for a,b in rows:
        check()
        if vertices:vertices.extend(router.route(vertices[-1],a)[1:])
        else:vertices.append(a)
        vertices.append(b)
    clean=[]
    for p in vertices:
        if not clean or math.dist(clean[-1],p)>1e-6:clean.append(p)
    vertices=[]
    for p in clean:
        while len(vertices)>=2:
            a,b=vertices[-2:];u=(b[0]-a[0],b[1]-a[1]);v=(p[0]-b[0],p[1]-b[1])
            if (abs(u[0]*v[1]-u[1]*v[0])<1e-9*math.hypot(*u)*math.hypot(*v)
                    and u[0]*v[0]+u[1]*v[1]>0):vertices.pop()
            else:break
        vertices.append(p)
    points=[]
    for a,b in zip(vertices,vertices[1:]):
        check();length=math.dist(a,b);yaw=math.atan2(b[1]-a[1],b[0]-a[0])
        # Repeated XY with incoming/outgoing yaw is the stationary-turn marker.
        if not points or abs(wrap(points[-1][2]-yaw))>1e-3:points.append((*a,yaw))
        count=max(1,math.ceil(length/spacing))
        if len(points)+count>MAX_POINTS:raise ValueError('原地转向路径采样点数超限')
        for i in range(1,count+1):
            t=i/count
            points.append((*b,yaw) if i==count else
                (a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),yaw))
    if len(points)<2:raise ValueError('原地转向路径没有有效行驶段')
    return points,vertices,{'swath_count':len(rows),'point_turn_count':len(point_turn_indices(points))}
