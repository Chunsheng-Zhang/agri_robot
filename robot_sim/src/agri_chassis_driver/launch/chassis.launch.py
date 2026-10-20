from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 1. 底盘底层驱动
    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',
    )

    
    # 2. 底盘控制与运动数据处理
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        parameters=[{
            'pub_odom_tf': True,  # 发布 odom→base_link TF
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
            'angular_scale': 1.0,
        }]
    )
    

    # 3. 手柄控制节点
    yahboom_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_X3',
    )

    # 4. 手柄驱动节点
    joy_node = Node(
        package='joy',
        executable='joy_node',
    )

    return LaunchDescription([
        driver_node,
        base_node,
        yahboom_joy_node,
        joy_node,

    ])