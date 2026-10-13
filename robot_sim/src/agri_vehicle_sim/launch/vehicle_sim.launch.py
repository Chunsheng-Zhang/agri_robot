#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    nav2_dir = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('agri_vehicle_sim')

    world = os.path.expanduser(
        '~/open_navigation_ws/install/opennav_coverage_demo/'
        'share/opennav_coverage_demo/blank.world'
    )

    csv_file = os.path.expanduser(
        '~/agri_robot/robot_sim/output/stadium_path.csv'
    )

    sdf = os.path.join(
        nav2_dir, 'worlds', 'waffle.model'
    )

    urdf = os.path.join(
        nav2_dir, 'urdf', 'turtlebot3_waffle.urdf'
    )

    rviz_config = os.path.join(
        sim_dir, 'rviz', 'agri_sim.rviz'
    )

    for file_path in [world, csv_file, sdf, urdf, rviz_config]:
        if not os.path.isfile(file_path):
            raise FileNotFoundError(
                f'Simulation file not found: {file_path}'
            )

    # 读取参考路径的第一个点，作为机器人初始位置
    with open(csv_file, 'r', newline='') as f:
        reader = csv.DictReader(f)
        first_point = next(reader, None)

    if first_point is None:
        raise RuntimeError(f'No path points in {csv_file}')

    spawn_x = float(first_point['x_m'])
    spawn_y = float(first_point['y_m'])
    spawn_yaw = float(first_point['yaw_rad'])

    with open(urdf, 'r') as f:
        robot_description = f.read()

    gazebo = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3',
            '-file', sdf,
            '-x', str(spawn_x),
            '-y', str(spawn_y),
            '-z', '0.10',
            '-Y', str(spawn_yaw),
        ],
        output='screen'
    )

    # 假定底盘里程计在启动时以机器人初始位姿为局部原点。
    # 将 odom 原点放到跑道起点，使参考路径与机器人在 map 中对齐。
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '--x', str(spawn_x),
            '--y', str(spawn_y),
            '--z', '0',
            '--yaw', str(spawn_yaw),
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        map_to_odom,
        rviz,
    ])