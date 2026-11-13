"""Direction carried by body headings in a standard map-frame reference path."""
import hashlib
import json
import math


def directed_poses(poses):
    clean=[]
    for pose in poses:
        p=tuple(pose[:3])
        if len(p)!=3 or not all(math.isfinite(v) for v in p):
            raise ValueError('Path pose must contain finite x, y and body yaw')
        if clean and math.dist(clean[-1][:2],p[:2])<=1e-6:
            clean[-1]=p  # At a cusp the outgoing pose owns the next segment.
        else:clean.append(p)
    if len(clean)<2:raise ValueError('Path needs two distinct points')
    directions=[]
    for a,b in zip(clean,clean[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        dot=(dx*math.cos(a[2])+dy*math.sin(a[2]))/math.hypot(dx,dy)
        if abs(dot)<.5:
            raise ValueError('Path body heading is inconsistent with travel direction')
        directions.append(1 if dot>0 else -1)
    return clean,directions


def backward_flags(poses):
    # Keep the original sample indices, including exact duplicated cusp poses.
    result=[]
    for a,b in zip(poses,poses[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        result.append(dx*math.cos(a[2])+dy*math.sin(a[2])<0 if math.hypot(dx,dy)>1e-6 else False)
    result.append(result[-1] if result else False)
    return result


def motion_signature(poses):
    return hashlib.sha256(json.dumps([[float(v) for v in p[:3]] for p in poses],
                                    separators=(',',':'),allow_nan=False).encode()).hexdigest()


def point_turn_indices(poses):
    """Repeated XY with different body yaws marks an explicit stopped turn.

    The first yaw is incoming, the last outgoing. Indices address the cleaned
    poses returned by directed_poses; ordinary reverse cusps keep the same yaw.
    """
    stops=set();index=-1;previous=None
    for pose in poses:
        p=tuple(pose[:3])
        if previous is not None and math.dist(previous[:2],p[:2])<=1e-6:
            if abs(math.atan2(math.sin(p[2]-previous[2]),math.cos(p[2]-previous[2])))>1e-3:
                stops.add(index)
        else:index+=1
        previous=p
    if any(i<=0 or i>=index for i in stops):
        raise ValueError('原地转向标记必须位于两个行驶段之间')
    return sorted(stops)


def rotation_envelope(length, width, margin, center_x=0., center_y=0.):
    """Full-rotation disk about base_link enclosing a rectangular footprint."""
    if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v)
           for v in (length,width,margin,center_x,center_y)):
        raise ValueError('车辆尺寸和安全余量必须是有限数值')
    if not .05<=length<=20 or not .05<=width<=20 or not 0<=margin<=5:
        raise ValueError('车辆长宽或原地转向安全余量越界')
    radius=math.hypot(length/2+abs(center_x),width/2+abs(center_y))+margin
    return {'method':'MIN_SAFE_POINT_TURN','robot_length_m':length,'robot_width_m':width,
            'safety_margin_m':margin,'center_x_m':center_x,'center_y_m':center_y,
            'rotation_radius_m':radius,'headland_width_m':radius+1e-4}


def validate_point_turn_path(poses, polygon, envelope):
    """Check the same enclosing disk for all travel and stationary turns."""
    from shapely.geometry import LineString
    expected=rotation_envelope(envelope['robot_length_m'],envelope['robot_width_m'],
        envelope['safety_margin_m'],envelope.get('center_x_m',0.),envelope.get('center_y_m',0.))
    if abs(envelope.get('headland_width_m',-1)-expected['headland_width_m'])>1e-6:
        raise ValueError('原地转向包络元数据不一致')
    clean,directions=directed_poses(poses)
    stops=point_turn_indices(poses)
    if any(d<0 for d in directions):raise ValueError('原地转向智能规划仅支持前进行驶段')
    safe=polygon.buffer(-expected['rotation_radius_m'])
    if safe.is_empty or not safe.buffer(1e-5).covers(LineString([p[:2] for p in clean])):
        raise ValueError('完整路径或原地转向缺少车辆扫掠净空')
    # Every geometric corner needs an explicit stop marker. A smooth reference
    # is handled by the old planner, never silently interpreted as a point turn.
    for i in range(1,len(clean)-1):
        a,b,c=clean[i-1:i+2]
        before=math.atan2(b[1]-a[1],b[0]-a[0]);after=math.atan2(c[1]-b[1],c[0]-b[0])
        if abs(math.atan2(math.sin(after-before),math.cos(after-before)))>.01 and i not in stops:
            raise ValueError('原地转向路径拐角缺少停车标记')
    return expected
