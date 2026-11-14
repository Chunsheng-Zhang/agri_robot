"""Manual field-entry reference and validated, separate approach geometry."""
import math
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import nearest_points
from agri_base.path_geometry import rotation_envelope
from .point_turn_planner import StraightRouter


def parse_entry(value, frame):
    if value is None:
        return None
    if not isinstance(value,dict) or set(value)-{'latitude','longitude','source'}:
        raise ValueError('入口需要经纬度和输入方式')
    if value.get('source','coordinates') not in ('coordinates','map'):
        raise ValueError('入口输入方式无效')
    lat,lon=value.get('latitude'),value.get('longitude')
    if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in (lat,lon)):
        raise ValueError('入口经纬度必须为有限数值')
    x,y=frame.xy(lat,lon)
    return {'latitude':float(lat),'longitude':float(lon),'source':value.get('source','coordinates'),
            'x':x,'y':y}


class EntryConnector:
    def __init__(self,field,parameters,entry,check=lambda:None):
        self.entry=entry
        self.envelope=rotation_envelope(parameters.get('robot_length',.45),parameters['robot_width'],
            parameters.get('rotation_safety_margin',.05),parameters.get('rotation_center_x',0.),parameters.get('rotation_center_y',0.))
        self.field=field
        self.safe=field.buffer(-self.envelope['rotation_radius_m'])
        if self.safe.is_empty:raise ValueError('地块无法容纳入口连接所需的完整车体转向包络')
        point=Point(entry['x'],entry['y'])
        if any(Polygon(h).covers(point) for h in field.interiors):
            raise ValueError('作业入口不能位于禁入区域内')
        anchor=nearest_points(self.safe,point)[0]
        self.anchor=(anchor.x,anchor.y)
        self.offset=point.distance(anchor)
        self.router=StraightRouter(self.safe,check)

    def assess(self,start):
        if not self.safe.buffer(1e-5).covers(Point(start[:2])):
            raise ValueError('该覆盖起点缺少入口对齐所需的完整车体净空')
        route=self.router.route(self.anchor,start[:2])
        length=sum(math.dist(a,b) for a,b in zip(route,route[1:]))
        return {'schema_version':1,'reference':dict(self.entry),'anchor':dict(zip(('x','y'),self.anchor)),
                'coverage_start':dict(zip(('x','y'),start[:2])),
                'connection':[dict(zip(('x','y'),p)) for p in route],
                'connection_length_m':length,'reference_to_anchor_m':self.offset,
                'distance_m':length+self.offset,'access_verified':self.offset<=1e-5,
                'note':'入口到内缩安全点仅作规划距离示意，不作为执行接近路线' if self.offset>1e-5 else '仅用于起点比较的田内参考连接，已按车体包络检查'}


def validate_entry_metadata(metadata, field, parameters, expected, start):
    if expected is None:
        if metadata is not None:raise ValueError('已保存路径的入口与当前任务不一致')
        return
    if not isinstance(metadata,dict) or metadata.get('reference')!=expected:
        raise ValueError('已保存路径的入口与当前任务不一致，请重新规划')
    result=EntryConnector(field,parameters,expected).assess(start)
    for key in ('anchor','coverage_start'):
        if math.dist([metadata.get(key,{}).get(k,math.inf) for k in ('x','y')],
                     [result[key][k] for k in ('x','y')])>1e-5:
            raise ValueError('入口连接端点不一致')
    route=metadata.get('connection',[])
    xy=[(p['x'],p['y']) for p in route]
    if len(xy)<2 or math.dist(xy[0],tuple(result['anchor'].values()))>1e-5 or math.dist(xy[-1],start[:2])>1e-5:
        raise ValueError('入口连接路径端点不一致')
    if not EntryConnector(field,parameters,expected).safe.buffer(1e-5).covers(LineString(xy)):
        raise ValueError('入口连接穿越边界或禁入区')
    length=sum(math.dist(a,b) for a,b in zip(xy,xy[1:]))
    if abs(metadata.get('connection_length_m',math.inf)-length)>1e-5 or abs(metadata.get('distance_m',math.inf)-length-result['reference_to_anchor_m'])>1e-5:
        raise ValueError('入口连接距离不一致')
    if abs(metadata.get('reference_to_anchor_m',math.inf)-result['reference_to_anchor_m'])>1e-5 or metadata.get('access_verified')!=result['access_verified']:
        raise ValueError('入口通道有效性标记不一致')
