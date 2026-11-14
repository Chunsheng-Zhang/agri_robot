"""Validate archived map poses and explicit reverse permission before loading."""
import json
import math
from pathlib import Path
from .field import validate_path
from agri_base.path_geometry import directed_poses, validate_point_turn_path, motion_signature

MAX_FILE_BYTES=32*1024*1024
MAX_POINTS=150000


def read_saved_path(filename, frame, polygon, robot_width, allow_reverse=False, expected_envelope=None, expected_entry=None):
    path=Path(filename)
    if path.stat().st_size>MAX_FILE_BYTES:
        raise ValueError('已保存路径超过读取预算，请重新规划')
    data=json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data,dict):raise ValueError('已保存路径格式无效')
    if 'planning' in data and not isinstance(data['planning'],dict):raise ValueError('已保存路径元数据无效')
    if data.get('frame_id')!='map' or data.get('projection')!=frame.identity:
        raise ValueError('已保存路径的固定原点或坐标系不一致，请重新规划')
    raw=data.get('points')
    if not isinstance(raw,list) or not 2<=len(raw)<=MAX_POINTS:
        raise ValueError('已保存路径点数无效')
    points=[]
    for item in raw:
        if not isinstance(item,dict):raise ValueError('已保存路径点格式无效')
        values=tuple(item.get(k) for k in ('x','y','yaw'))
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):
            raise ValueError('已保存路径坐标或航向无效')
        if math.hypot(*values[:2])>frame.max_radius:
            raise ValueError('已保存路径超出局部坐标工作范围')
        points.append(values)
    backward=data.get('is_backward',[]);velocities=data.get('velocities',[])
    if not isinstance(backward,list) or any(not isinstance(v,bool) for v in backward):
        raise ValueError('已保存路径倒车方向格式无效')
    if any(backward) and not allow_reverse:raise ValueError('已保存路径含倒车段，尚未允许倒车')
    if not isinstance(velocities,list) or any(isinstance(v,bool) or not isinstance(v,(int,float)) or
            not math.isfinite(v) or (v<0 and not allow_reverse) for v in velocities):
        raise ValueError('已保存路径速度无效或包含倒车')
    validate_path([p[:2] for p in points],polygon,robot_width)
    planning=data.get('planning') or {}
    if planning.get('turn_mode')=='POINT_TURN':
        envelope=planning.get('rotation_envelope') or {}
        if envelope.get('robot_width_m')!=robot_width:
            raise ValueError('保存路径的原地转向车宽与任务不一致')
        if expected_envelope is not None and envelope!=expected_envelope:
            raise ValueError('保存路径的车辆尺寸/安全余量已改变，请重新规划')
        validate_point_turn_path(points,polygon,envelope)
        if data.get('motion_signature')!=motion_signature(points):
            raise ValueError('原地转向标记或航向指纹不一致')
    from .entry import validate_entry_metadata
    parameters={'robot_width':robot_width}
    if expected_envelope:
        parameters.update(robot_length=expected_envelope['robot_length_m'],
            rotation_safety_margin=expected_envelope['safety_margin_m'],
            rotation_center_x=expected_envelope.get('center_x_m',0.),rotation_center_y=expected_envelope.get('center_y_m',0.))
    validate_entry_metadata(planning.get('entry'),polygon,parameters,expected_entry,points[0])
    _,directions=directed_poses(points)
    if any(d<0 for d in directions) and not allow_reverse:
        raise ValueError('已保存路径的姿态包含倒车段，尚未允许倒车')
    return data,points
