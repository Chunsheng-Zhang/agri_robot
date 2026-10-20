多点导航代码
# ===== 终端 1：发布经纬度数据 =====
ros2 launch mgx23_driver mgx23_node

# ===== 终端 2：启动底盘驱动 =====
ros2 launch agri_chassis_driver chassis.launch.py

# ===== 终端 3：启动多点导航节点 =====
ros2 run agri_multi_point_nav multi_point_nav


路径跟踪代码
# ===== 终端 1：发布经纬度数据 =====
ros2 launch mgx23_driver mgx23_node

# ===== 终端 2：启动底盘驱动 =====
ros2 launch agri_chassis_driver chassis.launch.py

# ===== 终端 3：发布规划的路径 =====
ros2 run agri_path_planning path_publisher

# ===== 终端 4: 发布定位 =====
ros2 run agri_localization coordinate_converter

# ===== 终端 5: 发布跟踪算法 =====
ros2 run agri_path_tracking real_pure_pursuit


通过 opennav 实现全覆盖路径规划
# ===== 终端 1：发布经纬度数据 =====
ros2 launch mgx23_driver mgx23_node

# ===== 终端 2：启动底盘驱动 =====
ros2 launch agri_chassis_driver chassis.launch.py

# ===== 终端 3：启动 GPS Map TF 发布器 =====
ros2 run agri_localization gps_map_tf_publisher

# ===== 终端 4：启动导航栈 =====
ros2 launch opennav_coverage_demo bringup_launch.py \
  params_file:=/home/zhang/agri_robot/open_navigation_ws/src/opennav_coverage/opennav_coverage_demo/params/demo_params.yaml

# ===== 终端 5：启动覆盖路径规划任务 =====
ros2 run opennav_coverage_demo demo_coverage_via_headland \
  --json /home/zhang/agri_robot/open_navigation_ws/src/navigation_ws/results/1019_test_1_utm_boundary.json

# ===== 终端 6（可选）：启动 RViz 可视化 =====
ros2 launch nav2_bringup rviz_launch.py
