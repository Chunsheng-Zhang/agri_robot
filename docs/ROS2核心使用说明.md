# ROS 2 核心使用与验证

2026-11-13：一键优化采用[原地转向智能规划](原地转向智能规划.md)，按车体完整旋转包络计算地头，固定覆盖/重复/长度权重0.5/0.3/0.2。手工曲线规划保留；下文0.1 m半车宽净空说明仅针对手工规划。新路径的停车转向标记需新版规划节点和跟踪节点共同支持，部署时构建并在停车后统一重启核心与后端。

以下命令从 `/home/zhang/agri_robot` 执行。真实设备尚需完成安装方向、杆臂、航向、速度符号和底盘停止行为的实车验证；样例配置来自原有 waypoint 地块，不应直接当作任意当前位置的任务。

实际出场请使用 [室外实验指导文件](室外实验指导文件.md)：包含分阶段启动、统一数据目录、rosbag 录制、原始数据和结果验收，不要仅凭本页简化命令开展首轮实车实验。

手柄、4ROS 和 Astra Pro Plus 接入后的新用法见 [设备接入说明](手柄雷达相机接入说明.md)。雷达障碍停车默认关闭，可在 Web 勾选“启用雷达障碍停车”。勾选后要求有效 `/scan`：启动命令增加 `start_lidar:=true` 或复用外部雷达；手柄/相机按需开启。

2026-10-29 起，网页模式切换与任务操作已接入，见 [Web 任务操作与旧 GUI 功能迁移](Web任务操作与旧GUI功能迁移.md)。`mode` 现在选择初始模式，两类控制器及规划模块默认常驻；不按模式重复开另一份节点。新增 `agri_interfaces`，升级后需重新构建并正常重启旧核心和后端。

## 构建和环境

```bash
cd /home/zhang/agri_robot
bash scripts/build_core.bash
source scripts/setup_environment.bash
```

脚本依赖已有 Humble、MGX23、OpenNav、Yahboom underlay。新代码额外使用 `pyproj`、PyYAML、Shapely、pytest；当前环境已具备。部署新机器时用 `rosdep` 检查声明的依赖，再构建已固定版本的上游包。本次没有改动或重建 Yahboom 原始工作区。

## 基础待命与统一启动

```bash
bash scripts/start_core.bash mode:=base chassis:=none
```

这会启动 MGX23 TCP、坐标/TF、safety、速度适配和待命的任务/规划节点，不打开底盘串口，不开始运动。已有外部 MGX23 节点时加 `start_mgx23:=false`，并确保它是补强后的驱动，能发布 stamped 姿态话题。

运行时可在 Web 顶部选择模式，也可调用以下 Trigger 服务；切换停止并取消旧自主任务，新模式保持待命：

```bash
ros2 service call /system/mode/base std_srvs/srv/Trigger '{}'
ros2 service call /system/mode/waypoint std_srvs/srv/Trigger '{}'
ros2 service call /system/mode/coverage std_srvs/srv/Trigger '{}'
```

独立 CSV→Path 跟踪实验用 `mode:=coverage start_planner:=false`，保留一个跟踪器、一个外部参考路径发布者。跟踪器兼容原 CSV 的 volatile 周期发布和 OpenNav 的持久化发布，拒绝多个参考路径发布者。此选项关闭覆盖规划接口，正常 Web 作业保持 `start_planner:=true` 默认值。

参数源文件为 `agri_robot_ws/src/agri_bringup/config/system.yaml`。symlink-install 后默认 launch 可读该配置；也可通过 `params_file:=<绝对路径>` 指定配置副本。

2026-11-10起，用户指定默认规划车宽 `robot_width: 0.2` m，默认统一配置与 `config/jetson_system.yaml` 保持一致，圆形净空检查半径对应0.1 m。保存路径复用优先采用任务车宽，缺失时按节点覆盖值、共享配置值、节点缺省值的顺序回退。该参数与作业幅宽、地头宽度及雷达独立车体尺寸参数分开；历史运行配置快照不回写，修改后需重新规划以生成新结果。

随后增加任务级设置：Web“全覆盖规划与跟踪”→“规划与路径跟踪参数”中的“车体宽度 robot_width（m）”，默认0.2 m、范围0.05–20 m。填写后先保存任务，再生成或一键优化规划；车宽随任务及本轮`config/mission.json`归档，传给普通OpenNav规划、回形及智能规划的净空检查。保存路径复用优先采用任务车宽，缺少该字段的旧任务兼容原配置读取；修改车宽需要重新保存和规划，旧任务版本的路径不能直接作为新版本加载。页面设置不改全局YAML、雷达独立车体尺寸或历史数据。此次按用户要求只完成代码接入，未运行测试或构建、未重载服务，待用户验证。

本次加载核对：在确认本机底盘与设备关闭、急停锁定、无控制权且任务未执行后，热更新`coverage_server`和`coverage_adapter`，两节点实读`robot_width=0.2`；未重启核心，仅重载后端读取逻辑。源配置、Jetson profile及安装overlay配置均为0.2。保存路径与自动地头的25项既有回归通过；0.11 m边界间距的几何检查在车宽0.2时通过、0.35时拒绝。证据在`.diagnostics/robot_width_020/runtime.json`及`verified-checks.log`；初次普通沙箱中的本机API测试受限后终止，允许本机通信后完成复测。未开始任务、未进行实车运动；网页需重新生成路径才能得到新车宽下的几何结果。

全局原点只在上述统一 YAML 的 `/**.ros__parameters.origin_latitude/origin_longitude` 中设置，使用 WGS84 十进制度；Web 不增加原点设置。所有核心模块使用同一份参数文件。原点和局部工作半径在各坐标转换节点启动后只读，更改时先停车并取消任务，修改配置后重启整套核心，不逐节点热改原点。

规划文件 `coverage_*.json` 保存 `projection`（EPSG、原点纬度和经度）。跟踪任务配置 `trajectory_directory` 后生成一对 `track_*.csv` 与同名 `.json`；后者保存坐标基准、米/弧度单位、`base_link` 参考点和本任务实际使用的参考路径快照。离线比较时先核对 `projection` 一致，再直接叠加参考路径点与 CSV 的 x/y，不能分别减各自的第一个点。历史 CSV 没有该元数据时，应使用当轮归档 YAML 或 Bag 的 `/localization/origin` 确认基准。

检查：

```bash
source scripts/setup_environment.bash
bash scripts/check_system.bash
ros2 topic echo /localization/valid
ros2 topic echo /safety/status
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /mgx23/position
```

`/odom` 只能由 localization 发布，`/cmd_vel` 只能由 safety 发布，`/base/driver_cmd_vel` 只能由 chassis_adapter 发布。

## 直接复用原始 Yahboom 驱动

底盘原有可执行文件就是 `Mcnamu_driver_X3`，无需修改其源码。独立验证原驱动时：

```bash
ros2 run yahboomcar_bringup Mcnamu_driver_X3
```

接入本系统时输入改接安全链末端：

```bash
ros2 run yahboomcar_bringup Mcnamu_driver_X3 --ros-args -r cmd_vel:=/base/driver_cmd_vel
```

也可让新 launch 启动这同一个原始驱动：`chassis:=yahboom_x3`，二者选择一种，不能重复启动。当前运行环境未在默认 Python 路径找到 `Rosmaster_Lib`；仓库中已有 `robot_sim/src/Rosmaster_Lib`。若目标机器也缺少安装，可给仅这个原驱动进程指定：

```bash
bash scripts/start_core.bash mode:=base chassis:=yahboom_x3 \
  rosmaster_pythonpath:=/home/zhang/agri_robot/robot_sim/src
```

这只复用已有 Python 库，不会 source robot_sim overlay 或复制原厂源码。若原机器已能运行原始命令，无需该参数。未来履带驱动使用 `chassis:=external` 并订阅 `/base/driver_cmd_vel`；真实电机协议和限速由该驱动实现。

## 多目标点

```bash
bash scripts/start_core.bash mode:=waypoint chassis:=none \
  waypoints_file:=/home/zhang/agri_robot/robot_sim/src/agri_multi_point_nav/config/waypoints.yaml
```

确认定位有效后，通过另一个终端解除启动急停并开始任务：

```bash
source scripts/setup_environment.bash
ros2 service call /safety/emergency_stop std_srvs/srv/SetBool '{data: false}'
ros2 service call /waypoint/start std_srvs/srv/Trigger '{}'
ros2 topic echo /waypoint/status
```

`chassis:=none` 用于先检查数据流，真实底盘需显式选择 yahboom_x3 或 external。start 返回成功代表开始申请控制权，状态 Topic 中 `RUNNING` 才代表执行。样例任务会到点停留并切换目标，不需要覆盖规划或 Pure Pursuit。

```bash
ros2 service call /waypoint/pause std_srvs/srv/Trigger '{}'
ros2 service call /waypoint/resume std_srvs/srv/Trigger '{}'
ros2 service call /waypoint/cancel std_srvs/srv/Trigger '{}'
```

## 覆盖规划与 Pure Pursuit

```bash
bash scripts/start_core.bash mode:=coverage chassis:=none \
  boundary_file:=/home/zhang/agri_robot/agri_robot_ws/src/agri_bringup/config/field.yaml \
  path_directory:=/home/zhang/agri_robot/data/paths \
  trajectory_directory:=/home/zhang/agri_robot/data/logs
```

先规划并检查结果：

```bash
source scripts/setup_environment.bash
ros2 service call /planning/plan std_srvs/srv/Trigger '{}'
ros2 topic echo /planning/status
ros2 topic info /reference_path --verbose
```

支持输入原前端 `{"points":[{"longitude":...,"latitude":...},...]}` 格式、YAML points/rings 或单个 GeoJSON Polygon。`data/field_boundary.json` 当前与默认原点不在同一地区，直接使用会被拒绝，需换成实际地块数据或统一修改原点。

规划成功发布 `map` 参考路径，但不会自动执行。确认当前位置至覆盖起点的直达通道可通行后，再明确开始任务：

```bash
ros2 service call /safety/emergency_stop std_srvs/srv/SetBool '{data: false}'
ros2 service call /tracking/start std_srvs/srv/Trigger '{}'
ros2 topic echo /tracking/status
```

`/tracking/start` 现在包含自动接近：距起点超过 `start_tolerance` 时，先复用单点导航控制器直达首点（`phase=APPROACHING`），再自动转入 Pure Pursuit（`phase=TRACKING`）；距离较近则直接跟踪。全程使用 coverage 模式和 tracking 控制权，不切换模式、不重新规划、不自动解除急停。接近速度及角速度沿用覆盖的 `speed/max_angular`，到点阈值为 `min(0.15 m, start_tolerance/2)`。同一定位原点和局部运行半径内支持从任意有效位置起步，但没有为起点接近搜索绕障或跨地块通行路线，需先确认直达通道可通行。

暂停、继续和取消对应 `/tracking/pause`、`/tracking/resume`、`/tracking/cancel`，类型均为 Trigger，对接近及覆盖两个阶段都生效。规划取消是 `/planning/cancel`。处于运行或暂停状态时不接受中途替换路径，需要先取消再重新开始。接近期间 `/tracking/error.vector.x` 是车辆到本次接近线段的偏离，不是到覆盖起点的剩余距离；暂停期间也持续发送偏差反馈，保留原有安全门限和超时检查。

任务轨迹在 map 下发布，配置目录后先保存同名 JSON 元数据，再持续写 CSV，每 20 条刷新，关闭任务时完成写盘。自动接近另存 `approach_*.csv/.json`，列与单点导航一致；`track_*.csv` 仅记录覆盖跟踪阶段，保持原列格式，同名 JSON 的 `approach_csv` 指向接近记录。实时 `/tracked_path` 同时展示两阶段。

多点导航配置 `trajectory_directory` 后也保存 `waypoint_*.csv` 与同名 JSON，字段为 `stamp,x,y,yaw,target_index,distance,heading_error`，目标索引从 0 开始；其目标与轨迹分别发布到 `/waypoint/targets` 和 `/waypoint/trajectory`，不会向覆盖跟踪的 `/reference_path` 写入目标点。

## 急停与恢复

```bash
ros2 service call /safety/emergency_stop std_srvs/srv/SetBool '{data: true}'
```

现有后端发送 `/emergency_stop` 的 Bool 也可触发。定位、指令或误差反馈超时，控制权会被撤销；收到恢复数据后不会自动重新开始。解除急停后重新调用对应 start 服务。请通过 `/safety/status` 或 `/safety/emergency_stop` 查看软件实际状态，不能只依据 HTTP 返回“指令已发送”。

## 独立启动某个模块

基础模块已在运行时，可以分别启动任务节点；每个节点加载同一个系统参数文件：

```bash
ros2 run agri_waypoint waypoint --ros-args \
  --params-file /home/zhang/agri_robot/agri_robot_ws/src/agri_bringup/config/system.yaml \
  -p waypoints_file:=/home/zhang/agri_robot/agri_robot_ws/src/agri_bringup/config/waypoints.yaml

ros2 run agri_path_tracking path_tracker --ros-args \
  --params-file /home/zhang/agri_robot/agri_robot_ws/src/agri_bringup/config/system.yaml
```

两个节点可以同时存在，但只有一个能获得控制权。不应同时运行旧多点控制、旧 Pure Pursuit、Nav2 控制器或手柄节点向最终 `/cmd_vel` 发消息。

## 测试

算法与协议单元测试（不启动 DDS）：

```bash
/usr/bin/python3 -m pytest -q tests
```

默认跳过需要 DDS 的测试。仅模拟 GNSS/INS 和运动学的闭环测试，不启动真实 MGX23 或底盘：

```bash
source scripts/setup_environment.bash
AGRI_ROS_TESTS=1 ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/tmp/agri_robot_tests \
  /usr/bin/python3 -m pytest -q tests/test_ros_core.py
```

真实已安装 OpenNav C++ 规划服务的接口测试，同样不连接硬件：

```bash
source scripts/setup_environment.bash
AGRI_ROS_TESTS=1 ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/tmp/agri_robot_tests \
  /usr/bin/python3 -m pytest -q tests/test_ros_coverage.py
```

测试覆盖统一投影和航向、跨地块拒绝、Waypoint/Pure Pursuit 收敛、回环与跳行、控制互斥、速度限制、过期数据、超时停车、急停、原始帧校验/重组/重复时间、任务服务及真实规划 Action。覆盖集成同时验证 3 m 地头样例通过，以及不预留地头的越界样例被拒绝。它们不能代替设备航向、串口、现场路面侧滑和实际制动距离验证。
