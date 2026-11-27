# 路径跟踪算法离线仿真

2026-11-27已用当天地面响应及0.30/0.50 m/s覆盖实验更新**X3硬质地面v2模型**。将行进/原地转向、归零响应、定位采样与交付、控制时序分开建模；另用三次未参与拟合的实验验证。推荐用它做下一轮离线算法筛选，模型边界、实车对照及完整命令见[实车校准与离线验证](2026-11-27_实车校准与离线验证.md)。它仍不能保证厘米级实车复现。

在项目根目录运行新模型，默认复用当天09:52留出实验的路径、0.30 m/s任务和第一帧跟踪位姿；之后自由闭环，不回放实测轨迹：

```bash
bash tracking_simulation/run.bash --config tracking_simulation/configs/x3_hard_ground.yaml --controller all
```

只跑LQR用`--controller lqr`；同条件理想对照增加`--ideal`；关闭GIF增加`--no-animation`。修改算法权重请编辑新配置的`tuning`；实验任务参数优先于YAML的`simulation`同名项，临时速度通过`--speed`覆盖。默认`config.yaml`、实车配置与四种控制律没有为这次校准而改动。

2026-11-26的v1模型保留作为历史基线，用法和局限见[X3室外硬地仿真与验证](X3室外硬地仿真与验证.md)。运行旧模型：

```bash
bash tracking_simulation/run.bash --controller all \
  --model-profile tracking_simulation/profiles/x3_hard_ground_v1.json
```

`--model-profile`仅适用于平面入口，显式参数与`--ideal`互斥；新配置内置的模型可由`--ideal`本轮关闭。选定模型后覆盖执行响应和反馈周期，不重复叠加一阶响应。噪声仍取YAML；未使用`--experiment`时初始偏差也取YAML。默认不加新配置或模型参数时继续使用原来的配置。输出包含模型快照、直线/转弯分区统计、CSV、PNG和GIF。

支持位置/航向高斯噪声及线/角速度一阶执行响应，统一在`config.yaml`设置，设为0分别关闭。本次提交保留用户当前配置：四项均为0，定位反馈周期为0.1 s；`--ideal`还会关闭采样保持，使每个控制周期更新定位。初始扰动验证工况为0.03 m、0.5°、两个时间常数均0.15 s，模型与结果见[噪声与执行器响应](噪声与执行器响应.md)。

2026-11-25已按用户要求迁入参考脚本PP，四种跟踪器统一在[four_controler.py](four_controler/four_controler.py)；控制律和参数见下文“控制与评价口径”，扰动验证见[噪声与执行器响应](噪声与执行器响应.md)。

在本文件夹内选择参考路径和控制器，运行独立的闭环仿真，查看轨迹、误差、速度指令和动画。四个算法是路径跟踪控制器（PP、Stanley、LQR、MPC），负责跟踪已有路径，不重新规划路径。

原来的1500 kg乘用车、5/10 m/s自行车模型案例已改成当前机器人平面速度接口。`run.bash`使用平面运动模型和可选执行响应，不启动ROS、Web、Gazebo或硬件，不向任何Topic发送指令；未选择地面模型且两项时间常数为0时为理想速度执行。实车现已有单独迁移的四控制器实现，本入口使用本地控制律；普通前进工况的一致性有软件测试，实车保护/故障流程不是完整复刻。

新增 [Gazebo物理仿真入口](gazebo/README.md)：`bash tracking_simulation/gazebo/run.bash --path-file tracking_simulation/paths/u_turn.json --controller pp --gui`（项目根目录执行）。它复用这里的四控制器、路径与控制参数，另外模拟轮地接触和执行限制，输出指令与物理实际速度；与下文的理想积分入口独立。

## 一条命令运行

不启动Gazebo，在理想条件下比较四种控制器（从项目根目录运行）：

```bash
bash tracking_simulation/run.bash --ideal --controller all
```

`--ideal`只覆盖本次运行：位置/航向噪声与两个执行响应时间常数均为0，定位在每个控制周期更新，不修改`config.yaml`。配置中的初始横向/航向偏差和控制器限速仍保留，用于观察收敛。去掉`--path-file`使用配置的默认覆盖路径；`--controller pp`只运行PP。结果目录以`ideal_`开头，实际参数保存在本轮快照中。

```bash
cd /home/jetson/agri_robot/tracking_simulation
bash run.bash
```

默认读取 `config.yaml`，当前用四种控制器跟踪 `paths/coverage_20261105_125347_567395.json`，结果自动写到 `results/<路径名>_<运行时间>/`。实际以配置的`path`为准。

`run.bash`使用项目已有的 `agri_backend/.venv/bin/python`，不需要先激活虚拟环境或source ROS环境，不安装或升级依赖。

只运行某一种，或临时改速度、前视距离：

```bash
bash run.bash --controller pp
bash run.bash --controller stanley
bash run.bash --controller lqr
bash run.bash --controller mpc

bash run.bash --controller pp --speed 0.3 --lookahead 0.5
```

更换本地路径、关闭GIF或指定模拟时长：

```bash
bash run.bash --path-file paths/coverage_20261105_141817_777878.json
bash run.bash --no-animation
bash run.bash --duration 30 --no-animation
```

`paths/u_turn.json`是新增的U型合成参考路径：先直行10 m，沿半径1 m的左转半圆掉头，再沿间距2 m的平行直线返回10 m。共464点，采样间距不超过0.05 m，长度约23.142 m，全程前进，未切换默认路径。运行：

```bash
bash run.bash --path-file paths/u_turn.json
```

`--duration 30`是最多模拟30秒，不保证完成；未完成会明确记为`TIME_LIMIT`。仿真按模拟时间计算，运行过程无需真实等待完整行驶时长。`--controller all`使用配置中的控制器列表，默认是全部四种。

原脚本名称也可以直接运行，作用与新入口相同：

```bash
../agri_backend/.venv/bin/python -B four_controler/four_controler.py --no-animation
```

## 文件组织

```text
tracking_simulation/
├── paths/                         # 已有米制参考路径，只读输入
├── four_controler/
│   └── four_controler.py          # 本地PP、Stanley角速度适配、LQR、约束MPC
├── config.yaml                    # 路径、仿真条件、算法参数、动画设置
├── configs/x3_hard_ground.yaml    # 当天实车校准工况，显式选择使用
├── run.bash                       # 推荐启动入口
├── run_tracking.py                # 统一读取/选择/运行/结果归档
├── simulate.py                    # 位置反馈—控制指令—运动学积分的公共闭环
├── ground_model.py                # 地面实验读取、等效响应、辨识与回放指标
├── calibrate_ground.py            # 辨识/独立验证入口，只读历史data
├── campaign_data.py               # 当天地面记录、串口TX及控制输入时序读取
├── campaign_model.py              # 行进/原地/归零等效响应与参数辨识
├── observation.py                 # 因果定位采样与延迟交付
├── calibrate_campaign.py          # 当天训练/留出验证、自由闭环对照
├── profiles/                      # 可入库的地面响应参数及辨识来源指纹
├── tests/                         # 地面数据语义、辨识隔离、因果响应检查
├── gazebo/                        # Gazebo ODE物理层、独立启动/参数/文档/测试
├── reports.py                     # 四算法对照图、速度图、指标图和GIF
├── results/<路径名>_<运行时间>/   # 每次独立结果，不覆盖已有目录
└── .cache/                        # 临时文件、绘图缓存和测试日志
```

`gazebo/tests/`包含共用扰动模型与Gazebo接口的离线检查。`paths/`、`results/`和`.cache/`按当前忽略规则留在本机；新克隆环境需要自行提供米制路径JSON，并修改`config.yaml`的`path`或传入`--path-file`。仿真源码仍依赖本仓库现有ROS功能包中的几何/状态工具和后端统计函数，不能只复制本文件夹就视为独立安装包。

结果目录内有：

| 文件 | 内容 |
| --- | --- |
| `controller_compare.gif` | 四算法轨迹动画，加速播放，画面标注模拟时间 |
| `controller_comparison.png` | 四合一图：参考/实际模型轨迹、横向误差、航向误差、路径进度 |
| `controller_commands.png` | 线速度、角速度指令对比；执行响应另见actuator_response.png |
| `controller_metrics.png` | 横向RMSE、航向RMSE和模拟用时对比 |
| 各算法的`actuator_response.png` | 控制请求、一阶响应输出与模型实际速度对比 |
| `summary.json` | 各算法结果、配置、输入路径及源码指纹 |
| `model_profile.json` | 仅在选择地面模型时生成，保存实际模型快照 |
| `config.yaml` | 本次实际使用的配置快照，含命令行覆盖值 |
| `report.md` | 本次中文结果报告 |
| `pp/`、`stanley/`、`lqr/`、`mpc/` | 每种算法的逐周期CSV、指标JSON、参考路径和四合一图 |

各算法子目录的`tracking.csv`记录模型位置/航向、控制器观测、横向/航向误差、速度指令、路径进度、阶段及控制计算耗时，并按参考几何记录直线/转弯区域；两区的RMSE、绝对P95和最大误差分别汇总。真值是模型积分状态，不是实车测量值。CSV距离单位m、角度rad；图中误差使用cm和°。所有输出均留在本文件夹，原路径文件读前后SHA256校验，不改动实验数据。

## 修改参数

通常只需修改`config.yaml`中的`simulation`：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `speed` | 0.5 m/s | 目标速度上限，0～1之间，不含0 |
| `lookahead` | 0.8 m | 本地PP基础前视距离；实际Ld另加车速项，其余三种控制律不使用 |
| `max_angular` | 1 rad/s | 最大角速度，0～1之间，不含0 |
| `completion_tolerance` | 0.1 m | 路径进度及终点到达阈值 |
| `search_window` | 1 m | 沿路径进度向前匹配的窗口 |
| `dt` | 0.05 s | 控制与运动积分周期 |
| `initial_lateral` | 0.1 m | 相对首点、沿首点车体左侧的初始偏移 |
| `initial_heading_deg` | 5° | 相对首点航向的初始角度偏差 |
| `feedback_period` | 0.1 s | 位置反馈周期；两帧之间保持上一次观测，0表示每控制步更新 |
| `position_noise` | 0 m | x/y观测各自独立的高斯白噪声标准差；验证工况0.03 m |
| `heading_noise_deg` | 0° | 航向高斯白噪声标准差；验证工况0.5° |
| `actuator_linear_tau` | 0 s | 线速度一阶响应时间常数；验证工况0.15 s |
| `actuator_angular_tau` | 0 s | 角速度一阶响应时间常数；验证工况0.15 s |
| `seed` | 0 | 随机种子；相同配置和种子可复现轨迹，不保证CPU耗时相同 |
| `duration` | null | 自动时限：`max(60, 5×路径长度/speed+120)`秒 |

`animation.enabled`控制GIF，`max_frames`默认90、`fps`默认10。关闭动画不影响CSV、图和指标。模拟步数上限500000，路径最多150000点，避免误设参数产生过大产物。

路径字段相对于配置文件目录解析；命令行`--path-file`相对于当前终端目录解析。配置不会自动读取或改变正在运行的ROS参数或已保存任务参数。

`tuning`集中保存控制器初始参数。Stanley的横向/航向增益、LQR的状态与控制代价、MPC的状态/控制代价及预测步数均可修改。MPC默认20步、dt=0.05 s，对应1 s预测时域；终端代价乘数为2。它们是工程初值，没有进行最佳整定。

`tuning.pp`对应从`four_controler/four_controler/four_controler.py`迁入的PP：`lookahead_gain=0.35`，实际前视距离为`lookahead + lookahead_gain × |速度反馈|`；当前基础0.8 m、速度0.5 m/s时为0.975 m。`wheelbase=0.16 m`是虚拟转角换算参数，`max_steer_deg=35°`限制虚拟转角，输出使用`ω=v×tan(δ)/wheelbase`，仍受公共角速度上限约束。`rear_offset=0 m`使控制点与base_link重合；源乘用车的1.6 m质心到后轴距离不适用于这台机器人，不照搬。若设置非零偏移，则是虚拟后轴点近似，需要单独验证。旧YAML缺少`pp`时自动补齐这组默认参数。

## 路径格式与坐标

输入为已有规划JSON，使用米制`x/y`和ROS车体航向`yaw`（rad）：

```json
{
  "frame_id": "map",
  "points": [
    {"x": 100.0, "y": 200.0, "yaw": 0.0},
    {"x": 110.0, "y": 200.0, "yaw": 0.0}
  ]
}
```

保留原`projection`及map原点，不归零或自动对齐轨迹。配置中的目标速度用于本次仿真，JSON的`velocities`不作为逐点速度曲线执行。普通前进路径可全部省略yaw，解析器会由点序补齐；不接受只有经纬度的文件。

四算法对比当前支持普通前进曲线路径。带相邻同XY、不同yaw的停车转向标记，或倒车路径，请单独选择`--controller pp`；PP继续复用项目原有的分段/对齐状态机，但内部跟踪器使用本地PP。其他算法明确拒绝这些输入，不会悄悄删除方向或转向标记。

## 控制与评价口径

公共车辆模型为：

```text
参考路径 → 控制器(v, ω) → 平面运动积分 → 模型位置/航向
                ↑                         ↓
                └──── 周期位置反馈 ───────┘
```

`x_dot=v*cos(yaw)`、`y_dot=v*sin(yaw)`、`yaw_dot=ω`；开启一阶响应时，v/ω使用响应环节输出，以不超过5 ms子步中点积分；关闭响应时保留原恒定指令精确积分。不将模型位置吸附到路径上。四种算法共用路径、初始位姿、模型、采样、噪声种子和速度上限；各自的控制律及转弯速度处理保留差异，所以实际速度曲线不一定逐点相同。正常结束记录制动过程，暂停直接清零，BRAKING样本不混入TRACKING误差统计。

- **PP**：实现位于[four_controler.py](four_controler/four_controler.py)的`PurePursuit`和`pure_pursuit_control`。迁移用户参考脚本的后轴点追踪、速度相关前视、按累计距离向前选参考点，以及`δ=atan2(2L sinα, 实际目标点距离)`和转角限幅；再适配为机器人`v/ω`。保留有界进度匹配、终点停车、机器人航向获取和分段状态机；不直接照搬1500 kg车辆模型、5/10 m/s速度或2.8 m轴距。它与实车PP是不同的控制律，未改动实车实现。
- **Stanley**：使用横向误差的arctan项、航向反馈和参考曲率前馈，适配为base_link角速度输出。它是机器人角速度适配版本，不等同于原论文前轴测点/前轮转角形式。
- **LQR**：状态为`[横向误差, 车辆航向−参考航向]`，输入为角速度修正量，使用小误差线性平面模型及曲率前馈；离散Riccati迭代保留原示例的方法。
- **MPC**：同一两状态模型，预览参考航向变化，在预测优化内部约束每一步角速度。求解有界凸二次问题，执行第一项后下一周期重新优化；不再使用旧例“无约束解最后裁剪”的形式。不包含障碍约束或复杂车辆非线性。

四种控制律由本地`build_controller()`统一返回，不以`None`回退到实车PP。公共路径匹配/进度状态继续复用实车`Polyline/PurePursuit.error`工具，但不会调用实车`PurePursuit.command()`。有界进度避免源乘用车示例的全局最近点在邻行/回环处跳段。统计以`TRACKING`样本为准，包含初始收敛，单独ALIGNING不混入。横向左正右负，输出航向误差为参考车体航向减模型航向，归一化至±π；LQR/MPC内部第二个状态取相反符号。CSV的`v_observed_m_s`为按定位采样周期保持的模型速度反馈，PP使用它计算车速前视项。

`COMPLETED`需要控制器满足终点位置和路径剩余进度条件，不能因回环起终点接近就提前结束。`TIME_LIMIT`/`ALIGNMENT_TIMEOUT`明确保存未完成状态。MPC未达到最优性容差的次数单列在`optimizer.unconverged`，非零时命令退出码为2，不假称求解通过。输入或文件错误退出1，全部完成且求解满足容差退出0。

未选择地面profile时，默认平面模型没有实车辨识的增益、限幅或延迟；v1/v2通过独立profile加入等效响应，v2另包含定位交付及横向关联。这些都没有独立分离真实轮胎滑移、导航误差或土壤作用，不能将结果称为实车精度，也不是Gazebo仿真。不同控制器的整定程度、前馈和速度策略也不同，默认结果不能用来证明某算法在实车或全部路径上更优。

方法参考：[Pure Pursuit原报告](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)、[Stanley原论文](https://robots.stanford.edu/papers/thrun.stanley05.pdf)、[线性约束MPC示例](https://www.cvxgrp.org/scs/examples/python/mpc.html)。本实现按机器人速度接口做了上述明确的工程适配。

## 历史与本轮验证

2026-11-25，本地720点覆盖路径、0.5 m/s、0.8 m前视、初始横向偏差0.1 m和航向5°，四种算法均完成。MPC调用2922次，未收敛0次；90帧GIF及三张汇总图已核对。软件测试14项、合并原控制器回归77项通过，日志在`.cache/four_controller_tests.log`。既有Matplotlib 3D导入提示不影响本工具二维图表，未为此升级系统或依赖。

该早期结果目录`results/coverage_20261105_141817_777878_20261125_102200_523511/`当前已不在本机，不恢复为新结果。本轮2026-11-27的v2校准示例：[报告](results/x3_v2_four_controllers_20261127/report.md)、[对照图](results/x3_v2_four_controllers_20261127/controller_comparison.png)、[动画](results/x3_v2_four_controllers_20261127/controller_compare.gif)。当日实车留出验证见[校准报告](results/x3_campaign_validation_final_20261127/report.md)。

2026-11-26提交前，共用扰动/Gazebo接口检查与既有导航、对齐、点转/倒车回归合计83项通过。上述14/77项是早期版本记录，当前测试源码位于`gazebo/tests/`。结果与缓存不随Git提交，历史本机报告可能已清理。

重跑当前仿真模块检查：

```bash
cd /home/zhang/agri_robot
mkdir -p tracking_simulation/.cache/tmp
TMPDIR="$PWD/tracking_simulation/.cache/tmp" \
  MPLCONFIGDIR="$PWD/tracking_simulation/.cache/matplotlib" \
  PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  agri_backend/.venv/bin/python -B -m pytest -q tracking_simulation/gazebo/tests \
  -p no:cacheprovider --basetemp="$PWD/tracking_simulation/.cache/pytest_workbench"
```
