2. 连接真实小车后，如何测试？
先启动真实底盘：
> cd ~/agri_robot/robot_sim
> 
> source /opt/ros/humble/setup.bash
> source install/setup.bash
> 
> ros2 launch agri_chassis_driver chassis.launch.py

确认驱动节点正常运行后，另开一个终端，执行：
> source /opt/ros/humble/setup.bash
> source ~/agri_robot/robot_sim/install/setup.bash
> 
> ros2 topic info /cmd_vel

正常情况下，应能看到至少一个订阅者。
确认周围环境安全、车轮和急停状态正常后，再进行低速测试。例如，发布一次线速度为 0.1 m/s 的指令：

> ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
> "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

注意：这条命令只发布一次指令，不会持续发布。实际运动效果取决于底盘控制板对速度指令的处理方式，因此第一次测试应做好随时急停的准备。


规划路径发布代码
> zhang@humble:~/agri_robot/robot_sim$ ros2 run agri_path_planning path_publisher

路径跟踪代码
> zhang@humble:~/agri_robot/robot_sim$ ros2 run agri_path_tracking pure_pursuit

gazebo虚拟车辆仿真代码
> zhang@humble:~$ ros2 launch agri_vehicle_sim vehicle_sim.launch.py

WEB前端页面代码
> zhang@humble:~/agri_robot/agri_web$ npm run dev -- --host 0.0.0.0


fastAPI后端代码
> (.venv) zhang@humble:~/agri_robot/agri_backend$ uvicorn main:app --host 0.0.0.0 --port 8000


| 模块 | 当前情况 | 判断 |
|---|---|---|
| MGX23 驱动 `mgx23_driver` | 已经在开发，包含 `mgx23_node.py`、`mgx23_parser.py` | 需要确认户外运行时能稳定发布数据 |
| 底盘驱动 `Mcnamu_driver_X3` | 已有代码，启动时曾报找不到 `/dev/myserial` | 代码存在，真机通信尚未验证 |
| 底盘启动文件 `chassis.launch.py` | 已修改为只启动底盘驱动 | 配置已准备，但实际启动结果尚未通过验证 |
| 路径规划 `agri_path_planning` | 已有功能包和路径规划相关代码 | 需要确认能否生成符合预期的参考路径 |
| 路径跟踪 `agri_path_tracking` | 已有功能包和 `pure_pursuit.py` | 需要确认定位输入、路径输入和 `/cmd_vel` 输出是否完整 |
| 多点导航 `agri_multi_point_nav` | 已有功能包 | 本次基础户外实验不一定需要 |
| 仿真相关功能包 | 已有 `agri_vehicle_sim`、`agri_vehicle_control` 等 | 可用于仿真验证，不等于真机测试已经完成 |
| 安全控制 | 目前尚未确认完整的安全控制实现 | 户外运动前必须确认急停和异常停车方案 |
| 实验数据记录 | 尚未确认完整的数据记录流程已测试 | 建议提前验证 ROS 2 Bag 录制与回放 |


1.测组合导航精度
2.直线  转弯行驶 精度
3.多点导航

阶段 1：坐标转换验证
- 新增实车专用程序文件。
- 读取 MGX23 经纬度。
- 转换为 UTM 和局部坐标。
- 暂时不控制底盘。
- 验证车辆静止时坐标是否稳定，移动时坐标变化方向是否合理。
阶段 2：航向验证
- 确认 MGX23 航向定义。
- 完成航向角转换。
- 验证车辆朝向与计算结果是否一致。
阶段 3：离线路径跟踪验证
- 生成简单的局部米制参考路径。
- 将实际位置和航向输入控制器。
- 验证目标点选择、曲率和控制指令。
- 不连接真实底盘进行运动控制。
阶段 4：低速实车测试
- 检查串口和底盘驱动。
- 排除手柄与自动控制之间的指令冲突。
- 验证急停与数据超时保护。
- 在开阔、安全的区域进行低速测试。
阶段 5：扩展到全覆盖农田作业
- 接入农田边界。
- 生成覆盖作业路径。
- 将覆盖路径交给同一个实车跟踪控制器。
- 后续再扩展多车任务分配。
这样，后续全覆盖规划只需要生成合适的参考路径，不必重新开发一套底层路径跟踪控制器。