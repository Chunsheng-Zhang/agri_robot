"""Build a local ODE world; vendor files are read-only visual sources."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT/'yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_description'


def tag(parent, element_name, value=None, **attrs):
    child = ET.SubElement(parent, element_name, {k: str(v) for k, v in attrs.items()})
    if value is not None:
        child.text = str(value)
    return child


def inertia(link, mass, dimensions):
    inertial = tag(link, 'inertial')
    tag(inertial, 'mass', mass)
    x, y, z = dimensions
    tensor = tag(inertial, 'inertia')
    for key, value in dict(ixx=mass*(y*y+z*z)/12, iyy=mass*(x*x+z*z)/12,
                           izz=mass*(x*x+y*y)/12, ixy=0, ixz=0, iyz=0).items():
        tag(tensor, key, value)


def box(parent, name, size, color=None, pose=None):
    item = tag(parent, 'visual' if color else 'collision', name=name)
    if pose:
        tag(item, 'pose', pose)
    tag(tag(tag(item, 'geometry'), 'box'), 'size', size)
    if color:
        material = tag(item, 'material')
        tag(material, 'ambient', color)
        tag(material, 'diffuse', color)
    return item


def vendor_visual(link, urdf, name, rotation='0 0 0'):
    source = urdf.find(f"link[@name='{name}']/visual/geometry/mesh")
    if source is None:
        raise ValueError('厂商URDF缺少外观：'+name)
    uri = source.attrib['filename']
    mesh = DESCRIPTION / uri.removeprefix('package://yahboomcar_description/')
    if not mesh.is_file():
        raise FileNotFoundError(mesh)
    visual = tag(link, 'visual', name=name+'_vendor_visual')
    tag(visual, 'pose', '0 0 0 '+rotation)
    tag(tag(tag(visual, 'geometry'), 'mesh'), 'uri', mesh.as_uri())


def plugin(model, name, filename):
    result = tag(model, 'plugin', name=name, filename=filename)
    tag(tag(result, 'ros'), 'namespace', '/tracking_sim')
    return result


def build_world(filename, poses, initial, config):
    v, p = config['vehicle'], config['physics']
    urdf = ET.parse(DESCRIPTION/'urdf/yahboomcar_X3.urdf').getroot()
    sdf = ET.Element('sdf', version='1.6')
    world = tag(sdf, 'world', name='tracking_world')
    tag(world, 'gravity', '0 0 -9.81')
    physics = tag(world, 'physics', name='ode', type='ode')
    tag(physics, 'max_step_size', p['step_size'])
    tag(physics, 'real_time_update_rate', p['real_time_factor']/p['step_size'])
    solver = tag(tag(physics, 'ode'), 'solver')
    tag(solver, 'type', 'quick')
    tag(solver, 'iters', 80)
    sun = tag(world, 'light', name='sun', type='directional')
    tag(sun, 'pose', '0 0 10 0 0 0')
    tag(sun, 'direction', '-0.3 -0.4 -1')
    tag(sun, 'diffuse', '0.8 0.8 0.8 1')
    ground = tag(world, 'model', name='ground')
    tag(ground, 'static', 'true')
    ground_link = tag(ground, 'link', name='ground_link')
    xs, ys = [q[0] for q in poses], [q[1] for q in poses]
    center = ((min(xs)+max(xs))/2, (min(ys)+max(ys))/2)
    ground_pose = f'{center[0]} {center[1]} -0.05 0 0 0'
    size = f'{max(xs)-min(xs)+30} {max(ys)-min(ys)+30} 0.1'
    box(ground_link, 'ground_collision', size, pose=ground_pose)
    box(ground_link, 'ground_visual', size, '0.45 0.48 0.40 1', ground_pose)
    # Reference markers are visuals only: they never collide or guide the robot.
    reference = tag(world, 'model', name='reference_path')
    tag(reference, 'static', 'true')
    reference_link = tag(reference, 'link', name='reference')
    stride = max(1, math.ceil((len(poses)-1)/1200))
    selected = poses[::stride]
    if selected[-1] != poses[-1]:
        selected.append(poses[-1])
    for i, (a, b) in enumerate(zip(selected, selected[1:])):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if length > 1e-8:
            pose = f'{(a[0]+b[0])/2} {(a[1]+b[1])/2} 0.003 0 0 {math.atan2(b[1]-a[1], b[0]-a[0])}'
            box(reference_link, f'path_{i}', f'{length} 0.025 0.004', '0 0.8 1 1', pose)
    robot = tag(world, 'model', name='tracking_robot')
    tag(robot, 'pose', f'{initial[0]} {initial[1]} {v["base_height"]} 0 0 {initial[2]}')
    base = tag(robot, 'link', name='base_link')
    inertia(base, v['body_mass'], (v['body_length'], v['body_width'], v['body_height']))
    vendor_visual(base, urdf, 'base_link')
    box(base, 'body_collision', f'{v["body_length"]} {v["body_width"]} {v["body_height"]}')
    for end, x in (('front', v['wheelbase']/2), ('back', -v['wheelbase']/2)):
        for side, y in (('left', v['wheel_separation']/2), ('right', -v['wheel_separation']/2)):
            name = end+'_'+side+'_wheel'
            wheel = tag(robot, 'link', name=name)
            tag(wheel, 'pose', f'{x} {y} {v["wheel_z"]} 0 0 0')
            inertia(wheel, v['wheel_mass'], (2*v['wheel_radius'], v['wheel_width'], 2*v['wheel_radius']))
            joint_source = next(j for j in urdf.findall('joint') if j.find('child').attrib['link'] == name)
            vendor_visual(wheel, urdf, name, joint_source.find('origin').attrib.get('rpy', '0 0 0'))
            collision = tag(wheel, 'collision', name='wheel_collision')
            tag(collision, 'pose', f'0 0 0 {math.pi/2} 0 0')
            cylinder = tag(tag(collision, 'geometry'), 'cylinder')
            tag(cylinder, 'radius', v['wheel_radius'])
            tag(cylinder, 'length', v['wheel_width'])
            ode = tag(tag(tag(collision, 'surface'), 'friction'), 'ode')
            # Cylinder local Z is the wheel axle (model Y after the collision
            # rotation). This direction stays tangent as the wheel spins;
            # local X would periodically rotate into the ground normal.
            tag(ode, 'mu', v['friction_lateral'])
            tag(ode, 'mu2', v['friction_longitudinal'])
            tag(ode, 'fdir1', '0 0 1')
            contact = tag(tag(collision.find('surface'), 'contact'), 'ode')
            tag(contact, 'kp', 1000000)
            tag(contact, 'kd', 100)
            tag(contact, 'max_vel', .1)
            tag(contact, 'min_depth', .001)
            joint = tag(robot, 'joint', name=end+'_'+side+'_joint', type='revolute')
            tag(joint, 'parent', 'base_link')
            tag(joint, 'child', name)
            axis = tag(joint, 'axis')
            tag(axis, 'xyz', '0 1 0')
            limit = tag(axis, 'limit')
            tag(limit, 'lower', -1e16)
            tag(limit, 'upper', 1e16)
    drive = plugin(robot, 'four_wheel_differential_equivalent', 'libgazebo_ros_diff_drive.so')
    for key, value in dict(num_wheel_pairs=2, update_rate=100,
                           max_wheel_torque=v['max_wheel_torque'],
                           max_wheel_acceleration=v['max_wheel_acceleration'],
                           command_topic='cmd_vel', publish_odom='false', publish_odom_tf='false',
                           publish_wheel_tf='false').items():
        tag(drive, key, value)
    for end in ('front', 'back'):
        tag(drive, 'left_joint', end+'_left_joint')
        tag(drive, 'right_joint', end+'_right_joint')
        tag(drive, 'wheel_separation', v['wheel_separation'])
        tag(drive, 'wheel_diameter', 2*v['wheel_radius'])
    truth = plugin(robot, 'physical_ground_truth', 'libgazebo_ros_p3d.so')
    tag(truth.find('ros'), 'remapping', 'odom:=ground_truth')
    for key, value in dict(body_name='base_link', frame_name='map', update_rate=100, gaussian_noise=0).items():
        tag(truth, key, value)
    gui = tag(world, 'gui')
    camera = tag(gui, 'camera', name='user_camera')
    tag(camera, 'pose', f'{center[0]-8} {center[1]-8} 15 0 0.9 0.8')
    tag(camera, 'view_controller', 'orbit')
    ET.indent(sdf)
    Path(filename).write_text(ET.tostring(sdf, encoding='unicode'), encoding='utf-8')
