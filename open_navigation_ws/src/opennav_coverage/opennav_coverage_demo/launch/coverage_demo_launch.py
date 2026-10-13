# Copyright (c) 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    coverage_demo_dir = get_package_share_directory('opennav_coverage_demo')

    world = os.path.join(coverage_demo_dir, 'blank.world')
    param_file_path = os.path.join(coverage_demo_dir, 'demo_params.yaml')
    sdf = os.path.join(nav2_bringup_dir, 'worlds', 'waffle.model')

    # start the simulation
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[coverage_demo_dir], output='screen')

    # start_gazebo_client_cmd = ExecuteProcess(
    #     cmd=['gzclient'],
    #     cwd=[coverage_demo_dir], output='screen')

    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': robot_description}])

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'tb3',
            '-file', sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.10',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'])

    # start the visualization
    rviz_config = os.path.join(coverage_demo_dir, 'opennav_coverage_demo.rviz')
    # print("rviz_config:", rviz_config)
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '', 'rviz_config': rviz_config}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coverage_demo_dir, 'bringup_launch.py')),
        launch_arguments={'params_file': param_file_path}.items())

    # world->odom transform, no localization. For visualization & controller transform
    fake_localization_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ])

    # start the demo task
    # use_transit:=true（默认）先沿地头开到覆盖率路径的起点，再走覆盖路径。
    # 原因见 demo_coverage_via_headland.py 顶部注释：RPP 的
    # max_robot_pose_search_dist 限制的是沿路径起点算起的弧长，覆盖率路径起点离车
    # 150 m、绕 19.3 km 才经过车，直接用原版 demo_coverage 的话车一步都不会动。
    use_transit = LaunchConfiguration('use_transit')
    field_json = LaunchConfiguration('field_json')
    route_mode = LaunchConfiguration('route_mode')
    tsp_time_limit = LaunchConfiguration('tsp_time_limit')

    demo_transit_cmd = Node(
        package='opennav_coverage_demo',
        executable='demo_coverage_via_headland',
        emulate_tty=True,
        output='screen',
        arguments=['--json', field_json,
                   '--route-mode', route_mode,
                   '--tsp-time-limit', tsp_time_limit],
        condition=IfCondition(use_transit))

    # 上游原版，整条路径一次性交给 FollowPath（大地块会失败，仅作对照）
    demo_cmd = Node(
        package='opennav_coverage_demo',
        executable='demo_coverage',
        emulate_tty=True,
        output='screen',
        condition=UnlessCondition(use_transit))

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_transit', default_value='true',
        description='true: demo_coverage_via_headland（先沿地头开到规划起点）; '
                    'false: 上游原版 demo_coverage'))
    ld.add_action(DeclareLaunchArgument(
        'field_json',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'open_navigation_ws/src/navigation_ws/results/'
            'ee_field_6_utm_boundary.json'),
        description='地块边界 GeoJSON，仅 use_transit:=true 时使用'))
    ld.add_action(DeclareLaunchArgument(
        'route_mode', default_value='',
        description='路由模式: BOUSTROPHEDON / SNAKE / SPIRAL / TSP；'
                    '空 = 用服务端 default_route_type'))
    ld.add_action(DeclareLaunchArgument(
        'tsp_time_limit', default_value='0',
        description='TSP 求解时间上限（秒），0 = 用 msgs 默认值 1'))
    ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(fake_localization_cmd)
    ld.add_action(demo_transit_cmd)
    ld.add_action(demo_cmd)
    return ld
