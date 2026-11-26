# Gazebo 路径跟踪物理仿真

当前本地跟踪器见[控制器代码](../four_controler/four_controler.py)及[平面仿真说明](../README.md)，扰动结果见[噪声与执行器响应](../噪声与执行器响应.md)；迁移前物理层的建立过程见[早期验证记录](验证记录.md)。

只验证路径跟踪。复用上级 `paths/`、`config.yaml` 和四种控制器，用 Gazebo Classic 11 的 ODE 刚体与轮地接触替代理想运动积分。四种控制律均来自`four_controler/four_controler.py`，PP是所给参考脚本迁入后的机器人适配版，不再调用实车PP控制方法；不启动 MGX23、底盘驱动、规划核心、Web 或安全控制服务，不操作已有服务。

## 启动

在项目根目录运行，无需另外打开 ROS 核心终端：

```bash
cd /home/zhang/agri_robot

# 推荐先运行 PP 的 U 型路径，打开 Gazebo 三维窗口
bash tracking_simulation/gazebo/run.bash \
  --path-file tracking_simulation/paths/u_turn.json --controller pp --gui

# 不开三维窗口，同样执行真实 Gazebo 物理计算，输出图和动画
bash tracking_simulation/gazebo/run.bash \
  --path-file tracking_simulation/paths/u_turn.json --controller all

# 使用上级 config.yaml 默认的覆盖参考路径
bash tracking_simulation/gazebo/run.bash --controller pp
```

四种控制器可选 `pp / stanley / lqr / mpc / all`，默认只运行 PP，`all` 按上级配置依次运行，每个算法启动一个全新世界。`--gui` 需要能连接的图形桌面（仅有 `DISPLAY=:0` 环境变量不够）；程序在有 `xdpyinfo` 时先检查桌面连接。关闭窗口只关闭显示，终端 `Ctrl+C` 才取消当前仿真并停止其子进程。结束时自动关闭本次创建的 Gazebo，不影响其他服务。

当前`--ideal`仅支持上级平面入口`tracking_simulation/run.bash`，Gazebo入口尚不识别该参数。Gazebo运行的附加噪声/响应可在共享YAML中分别设0，但车体力矩、摩擦和惯性仍由ODE计算，不等同于纯理想运动学仿真。

可用 `--speed 0.3 --lookahead 0.5` 覆盖上级控制参数；`--duration 10` 限制模拟时长，未完成记为 `TIME_LIMIT` 并返回非零退出码，不能把短测试当作完成。`--no-animation` 关闭 GIF。可用 `--physics-config 路径` 指向另一份完整物理配置。

初始横向误差、初始航向误差和反馈周期沿用 `tracking_simulation/config.yaml`，当前为0.1 m、5°、0.1 s。本次提交保留用户当前设置：位置/航向白噪声及线/角速度附加响应时间常数四项均为0。初始扰动验证使用0.03 m、0.5°和两个0.15 s时间常数，启用时修改共享配置即可。这里的`dt`是控制周期，物理积分周期由本目录配置独立设置；即使四项均关闭，Gazebo仍计算接触、力矩与惯性。详见[噪声与执行器响应](../噪声与执行器响应.md)。

`run.bash`只 source `/opt/ros/humble/setup.bash`，使用现有后端虚拟环境；首次通过现有 CMake/C++ 编译本地日志重定向库，不安装或升级系统/ROS/Gazebo/Python。若机器缺少现有 Gazebo 11/ROS 插件或开发头文件，构建失败会指出 `.cache/gazebo_build/` 日志，不能改用理想积分冒充物理运行。

## 模型与参数

`gazebo/config.yaml` 只配置物理层：

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `physics.step_size` | 0.001 s | ODE 物理积分步长 |
| `physics.real_time_factor` | 2 | 目标加速倍率，实际取决于计算性能 |
| `physics.settle_time` | 1 s | 下落接触地面、零指令稳定时间 |
| `physics.feedback_delay` | 0 s | 模拟定位采样到交付的纯延迟，与执行惯性不同 |
| `physics.feedback_timeout` | 0.5 s | 模拟定位年龄上限，超过则停车结束 |
| `physics.wall_timeout` | 900 s | 单算法独立墙钟异常保护，不是规划预算 |
| `vehicle.body_mass` | 1.8 kg | 车体含载荷的工程估值，**未称重标定** |
| `vehicle.wheelbase` | 0.16 m | 来自 X3 URDF 的前后轮轴距 |
| `vehicle.wheel_separation` | 0.169 m | 来自 X3 URDF 的左右轮中心距 |
| `vehicle.wheel_radius` | 0.04 m | 工程轮径初值，需要测量确认 |
| `vehicle.max_wheel_torque` | 0.2 N·m | 插件轮端力矩限制，未实测 |
| `vehicle.max_wheel_acceleration` | 1 m/s² | 插件轮缘目标速度加速度限制 |
| `vehicle.friction_longitudinal / lateral` | 1 / 0.15 | ODE 接触摩擦工程初值 |

复用 `yahboomcar_description/urdf/yahboomcar_X3.urdf` 中的外观网格，新增简化车体碰撞盒、四轮圆柱碰撞体、正定惯量、重力、地面与电机限制。使用现有 `gazebo_ros_diff_drive` 的双轮对支持驱动四轮，**这是用于前进/转向路径跟踪的差速等效模型，不是精确的 X3 麦轮滚子动力学，也不是未来履带土壤模型**。当前控制器输出没有横移；低横向摩擦只是接触近似。原厂文件完全只读，没有用 `planar_move` 或不断设置车体位置来假装物理运动。

图中的轮子外观不代表所有载荷、惯量或接触参数都来自厂家实测。调参前应测量轮径、总质量、轮距，并用实车直行/转向阶跃记录标定力矩、速度响应与摩擦；再用未参与标定的实验验证模型。Gazebo 不会自动消除定位安装偏差、GNSS 漂移或模型误差，不据此承诺 2 cm 实车精度。

## 信号与隔离

```text
原有控制器(v, ω) → 一阶执行响应 → /tracking_sim/cmd_vel → 四轮电机/轮地接触/ODE
        ↑                                         ↓
周期、噪声、纯延迟 ← /tracking_sim/ground_truth ← 物理位置/速度
        ↓
/tracking_sim/localization（供查看的模拟定位 PoseStamped）
```

| 话题 | 类型 | 来源与含义 |
| --- | --- | --- |
| `/tracking_sim/cmd_vel` | `geometry_msgs/msg/Twist` | 一阶响应后的驱动输入，约100 Hz；控制请求通常20 Hz，分别归档 |
| `/tracking_sim/ground_truth` | `nav_msgs/msg/Odometry` | P3D 插件，100 Hz，`map` 世界物理真值；其速度是**世界轴分量** |
| `/tracking_sim/localization` | `geometry_msgs/msg/PoseStamped` | 周期/噪声/延迟后的模拟定位，时间戳保留采集时刻，`map` |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 模拟时钟 |

不发布 `/cmd_vel`、`/mgx23/*`、`/odom` 或 TF，不伪造 RTK FIX。保留输入路径原 `map` 米制坐标和投影；世界原点与该 map 重合，没有分别归零参考与实际轨迹。原地转向/倒车标记继续由原 `CoverageController` 状态机处理，其tracker替换为本地PP；这些路径仍仅支持 `--controller pp`。PP前视距离为基础值加`lookahead_gain×|速度反馈|`，详见上级配置的`tuning.pp`。速度反馈与定位一起采样、一起延迟交付，不将当前未延迟的物理速度泄露给延迟控制器。

程序从 201～230 中选择未被本机进程占用的 Domain，始终排除225及继承的实车 Domain，并用文件锁避免自身重复选中。ROS 发现其他节点时拒绝运行；强制 `ROS_LOCALHOST_ONLY=1`。Gazebo 另选本机空闲端口，不接入已有世界。显式 `--domain` 也须在200～230内且通过同样检查，不允许绕过检查。

## 输出位置与评价

所有产物位于 `tracking_simulation/results/gazebo_<路径名>_<时间>/`，不改 `data/experiments/`：

```text
config.yaml、summary.json、report.md       # 实际参数、输入/源码指纹、对照指标
controller_comparison.png                 # 四合一算法对照
controller_commands.png、controller_metrics.png
controller_compare.gif                   # 按配置开启的加速回放
pp/（或stanley/lqr/mpc）
  world.sdf                              # 本次生成的世界/车辆，可审查复现
  ground_truth.csv                       # 100 Hz物理记录，包含结束后1秒制动
  actuator.csv                           # 约100 Hz请求/响应输出/实际速度及故障清零记录
  gzserver.log、console/、gui.ini          # 本次Gazebo日志/桌面设置
  tracking/
    tracking.csv、metrics.json、reference.json
    tracking_simulation.png、report.md    # 轨迹、横向、航向、指令与物理响应
    actuator_response.png                 # 控制请求、一阶输出、物理实际速度三路对照
```

`tracking.csv` 中 `v_command_m_s / w_command_rad_s` 是指令；`v_actual_body_m_s / w_actual_rad_s` 是 Gazebo 实际速度，前者由世界平面速度投影到机器人朝向，后者是世界Z轴角速度（平地模型用于平面转向评价），另记录实际横向滑移速度。**两者来源不同**。指令在本行发出，实际速度是发出瞬间的状态，不能直接称为测得的纯延迟。

横向/航向误差继续用实车的当前路径段与进度匹配计算，分别保留真值与控制器观测，统计只含 `TRACKING`、含初始收敛。指标含 MAE、RMSE、标准差、P95/P99/最大绝对误差；另有指令执行偏差、反馈年龄、实际控制间隔、超周期次数、终点距离、停车后速度和 MPC 未收敛计数。Gazebo/DDS 是异步的，加速下时序可能略有不同，不宣称逐位可重复或已经公平最优整定。

全部日志、编译和绘图缓存留在 `tracking_simulation/`。Classic 控制台默认硬编码 `~/.gazebo`，本目录小型 C++ 库仅对本次 Gazebo 子进程重定向其日志初始化至结果目录，GUI 使用官方 `GAZEBO_GUI_INI_FILE`；不改 HOME、全局安装或用户配置。关闭网络模型库下载，世界无需在线资源。

## 方法来源

采用[所给论文](https://dergipark.org.tr/en/download/article-file/5922230)的“算法层验证＋ROS 2/Gazebo物理层验证”思路；未取得作者可直接复用的平台源码，没有套用其3200 kg阿克曼拖拉机参数。插件行为依据 [ROS Gazebo 差速插件源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)和 [P3D 真值插件源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_p3d.cpp)。使用本机现有 Classic 11 以避免升级环境；该版本已结束官方维护，后续迁移应另行安排。
