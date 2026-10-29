"""RTK core plus independent vendor joystick, 4ROS and Astra Pro Plus devices."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode, chassis = value('mode'), value('chassis')
    if mode not in ('base', 'waypoint', 'coverage'):
        raise ValueError('mode must be base, waypoint or coverage')
    if chassis not in ('none', 'yahboom_x3', 'external'):
        raise ValueError('chassis must be none, yahboom_x3 or external')
    params = value('params_file')
    enabled = lambda name: value(name).lower() == 'true'
    require_joy = enabled('start_joy') if value('require_joy') == 'auto' else enabled('require_joy')
    result = [
        Node(package='agri_base', executable='localization', parameters=[params], output='screen'),
        Node(package='agri_safety', executable='safety',
             parameters=[params, {'initial_mode': mode, 'require_joy': require_joy,
                                  'event_directory': value('event_directory')}], output='screen'),
        Node(package='agri_base', executable='chassis_adapter', parameters=[params], output='screen'),
    ]
    if enabled('start_joy'):
        result += [
            Node(package='joy', executable='joy_node', name='joy_node',
                 parameters=[{'autorepeat_rate': 20.0, 'deadzone': 0.05,
                              'device_id': int(value('joy_device_id'))}], output='screen'),
            Node(package='agri_base', executable='joy_adapter', parameters=[params],
                 remappings=[('cmd_vel', '/control/manual/vendor_unused')], output='screen'),
        ]
    if enabled('publish_laser_tf'):
        result.append(Node(package='agri_base', executable='sensor_frames', parameters=[params], output='screen'))
    if enabled('start_lidar'):
        model = value('lidar_model')
        if model == '4ROS':
            share = get_package_share_directory('ydlidar_ros2_driver')
            filename = value('lidar_params_file') or os.path.join(share, 'params', 'ydlidar.yaml')
            with open(filename, encoding='utf-8') as stream:
                laser_params = yaml.safe_load(stream)['ydlidar_ros2_driver_node']['ros__parameters']
            if value('lidar_port'):
                laser_params['port'] = value('lidar_port')
            # The vendor executable is an ordinary rclcpp node, despite its launch wrapper.
            result.append(Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                               name='ydlidar_ros2_driver_node', parameters=[laser_params], output='screen'))
        elif model in ('a1', 's2'):
            filename = 'sllidar_launch.py' if model == 'a1' else 'sllidar_s2_launch.py'
            args = {'frame_id': 'laser'}
            if value('lidar_port'):
                args['serial_port'] = value('lidar_port')
            result.append(GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', filename)),
                launch_arguments=args.items())]))
        else:
            raise ValueError('lidar_model must be 4ROS, a1 or s2; calibrate laser_yaw for the chosen mount')
    if enabled('start_camera'):
        # Vendor XML declares use_uvc_camera but does not forward it as a node param.
        # Inject it in this scoped group while reusing the original camera launch.
        result.append(GroupAction(actions=[SetParameter(name='use_uvc_camera', value=True),
            IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('astra_camera'), 'launch', 'astro_pro_plus.launch.xml')),
            launch_arguments={'enable_color': 'true', 'enable_depth': 'false', 'enable_ir': 'false',
                              'product_id': '0',
                              'enable_point_cloud': 'false', 'enable_colored_point_cloud': 'false',
                              'publish_tf': 'false', 'color_width': '640', 'color_height': '480',
                              'color_fps': '30', 'uvc_retry_count': '10'}.items())]))
    if enabled('start_camera') or enabled('monitor_camera'):
        result.append(Node(package='agri_base', executable='camera_status', parameters=[params], output='screen'))
    if enabled('start_video'):
        # This bundled fork does not declare its parameter overrides. Reuse its
        # actual defaults (0.0.0.0:8080), rather than expose ineffective knobs.
        result.append(Node(package='web_video_server', executable='web_video_server', output='screen'))
    if value('start_mgx23').lower() == 'true':
        result.append(Node(package='mgx23_driver', executable='mgx23_node',
                           parameters=[params], output='screen'))
    if chassis == 'yahboom_x3':
        extra_env = {}
        if value('rosmaster_pythonpath'):
            extra_env['PYTHONPATH'] = value('rosmaster_pythonpath') + os.pathsep + os.environ.get('PYTHONPATH', '')
        result.append(Node(package='yahboomcar_bringup', executable='Mcnamu_driver_X3',
                           name='yahboom_driver',
                           additional_env=extra_env,
                           remappings=[('cmd_vel', '/base/driver_cmd_vel')], output='screen'))
    # External differential/track drivers subscribe only to /base/driver_cmd_vel.
    # Controllers stay resident. Safety alone selects which autonomous source may acquire control.
    result.append(Node(package='agri_waypoint', executable='waypoint',
                       parameters=[params, {'waypoints_file': value('waypoints_file'),
                                            'trajectory_directory': value('trajectory_directory')}], output='screen'))
    result.append(Node(package='agri_path_tracking', executable='path_tracker',
                       parameters=[params, {'trajectory_directory': value('trajectory_directory')}], output='screen'))
    if enabled('start_planner'):
        result += [
            Node(package='opennav_coverage', executable='opennav_coverage',
                 name='coverage_server', parameters=[params], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='coverage_lifecycle_manager', parameters=[{
                     'autostart': True, 'node_names': ['coverage_server'], 'use_sim_time': False}], output='screen'),
            Node(package='agri_path_planning', executable='coverage_adapter',
                 parameters=[params, {'boundary_file': value('boundary_file'),
                                      'output_directory': value('path_directory')}], output='screen'),
        ]
    return result


def generate_launch_description():
    share = get_package_share_directory('agri_bringup')
    defaults = {
        'mode': 'base', 'chassis': 'none', 'start_mgx23': 'true',
        'params_file': os.path.join(share, 'config', 'system.yaml'),
        'waypoints_file': os.path.join(share, 'config', 'waypoints.yaml'),
        'boundary_file': os.path.join(share, 'config', 'field.yaml'),
        'path_directory': '', 'trajectory_directory': '',
        'rosmaster_pythonpath': '',
        'start_joy': 'false', 'joy_device_id': '0', 'require_joy': 'auto',
        'start_lidar': 'false', 'lidar_model': '4ROS', 'lidar_port': '', 'lidar_params_file': '',
        'publish_laser_tf': 'true', 'start_camera': 'false', 'monitor_camera': 'false',
        'start_video': 'false',
        'event_directory': '',
        'start_planner': 'true',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=nodes),
    ])
