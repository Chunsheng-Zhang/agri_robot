# Fields2Cover 原生曲线与倒车跟踪

更新：2026-11-09。本次替代原R/1.5R/2R、U/Ω分类及专门鱼尾候选。正常行进最小半径的新任务默认值为1 m，沿用用户提供的设计依据；不代表完成了实车标定，已保存任务不自动改值。

## 网页操作

在“全覆盖规划与跟踪”→“规划与路径跟踪参数”选择：

| 转弯曲率形式 | Fields2Cover 类 | 方向与平滑性 |
| --- | --- | --- |
| Dubins | `PP_DubinsCurves` | 前进，直线/圆弧，曲率可突变 |
| Dubins CC | `PP_DubinsCurvesCC` | 前进，连续曲率；默认 |
| Reeds–Shepp | `PP_ReedsSheppCurves` | 可前进/倒车，曲率可突变 |
| Reeds–Shepp HC | `PP_ReedsSheppCurvesHC` | 可前进/倒车，同向段内平滑，换向点单独处理 |

“允许倒车”默认不勾选。需要倒车时，选择Reeds–Shepp或HC，勾选、保存任务、重新生成路径，再明确开始。勾选不自动开始、解除急停或更改运动模式，也不使Dubins产生倒车。关闭时若选定原生生成器返回倒车段，会明确拒绝，不能悄悄丢掉方向或更换算法。

人工“生成覆盖路径”和“一键优化规划”使用同一曲线选择。一键规划仍比较覆盖形式、地头宽度、作业顺序等完整方案，保持原评分与覆盖评价；每条连接仅调用选定生成器和任务指定的最小半径，不再枚举半径倍数或按U/Ω/鱼尾分类。最小半径是曲率上限的约束，不代表整条平滑曲线半径恒定；仍需检查净空和禁入区。

回形的圆角与层间连接也调用选定的原生生成器。圆角保持前进绕行，允许倒车时层间连接可以反向；不是“回形每一圈都倒车”。圆角切入距离使用几何空间估计，实际轨迹由Fields2Cover生成，不能容纳时仍拒绝。

## 跟踪和换向

- 标准`/reference_path`仍为`nav_msgs/Path`、map坐标；每个Pose的yaw表示车体朝向。位移与车体朝向相反代表倒车，不能将倒车段的yaw替换为运动方向。
- 将整条路径按方向切成连续段；每段保留全局累计进度，投影搜索和前视点限制在当前段内，不能提前匹配到换向后的重叠路径。
- 倒车使用运动朝向`yaw + π`计算Pure Pursuit，线速度为负；角速度按车辆实际航向变化计算。横向误差沿路径行进方向定义，航向误差比较车体应有朝向，不人为产生180°偏差。
- 非末段距换向点和剩余段长均进入`min(0.05 m, completion_tolerance, 本段长度/4)`时，先输出零速度，再进入已有`ALIGNING`阶段。默认航向误差±10°内、新鲜定位下保持0.5 s，同时换向等待期间位置相对稳定点位移不超过2 cm；不稳定则重计时，30 s超时故障停车。位置稳定不是实测轮速为零的证明，噪声较大时可能等待或超时。
- 暂停/继续保留当前方向和进度，继续先对齐该段车体航向；重复定位消息不能完成对齐等待。急停/定位异常/任务故障仍停车且不自动恢复。
- 目标速度参数为大小，最大1 m/s，正负由路径段方向决定；角速度仍不超过任务上限及±1 rad/s，Jetson缺省目标大小保持0.1 m/s。
- 接近起点仍为原前进单点接近，接近结束后按首段车体航向对齐；不生成跨地块绕障路线。

当前雷达仍按用户确认仅保护前方180°；允许倒车不会自动获得车后方保护。实车倒车、换向停稳及机具扫掠需要另行验证，本次未行驶真实车辆。

## 参数、保存与代码

参数键`path_continuity`现在取`DUBINS / DUBINS_CC / REEDS_SHEPP / REEDS_SHEPP_HC`，保留旧键名以减少接口变动。旧任务`CONTINUOUS / DISCONTINUOUS`只读映射为DubinsCC / Dubins；旧`optimization_turns`读取时丢弃，不输出到新任务或快照，不回写历史。

`allow_reverse`为严格布尔值，网页、任务快照、coverage_adapter及path_tracker默认false；任务开始前显式下发跟踪器，运行/暂停期间不可改参。保存路径再加载也按本任务的该选项验证。

`paths/coverage_*.json`保存车体yaw和逐点`is_backward`；规划元数据记录选定曲线、倒车策略。`paths/optimization.json`中的`turn_curves`保存最多24段原生连接样例，含方向、长度、倒车长度和能源代理，不再生成鱼尾预览。开始允许倒车的任务前，后端除XY路径指纹外还核对当前ROS路径的车体航向指纹，避免相同XY、不同方向被混用。

`tracks/track_*.csv`新增`direction`列（+1前进/-1倒车），sidecar记录`segment_directions`和`allow_reverse`；换向对齐继续写入`alignment_*.csv/.json`，不混入正式跟踪误差。旧报告不回填。

代码：

- `agri_path_planning/curves.py`：四种原生名称、旧值兼容和OpenNav映射。
- `agri_path_planning/turn_library.py`：现为`NativeTurns`，固定半径的单次原生连接及净空/方向检查，保留文件位置。
- 同包`optimizer.py`、`contour.py`、`node.py`：一键、回形和人工规划接入。
- `agri_base/path_geometry.py`：车体yaw、方向推断与航向指纹。
- `agri_path_tracking/controller.py`、`coverage.py`、`node.py`：按方向分段的跟踪、换向和记录。
- `agri_web/src/components/tasks/taskParameters.ts`、`TaskEditor.vue`：四选项和倒车复选框。

## 软件验证

离线测试包含四种真实Fields2Cover生成器、固定半径、默认禁倒车、参数兼容、倒车纠偏、前进/倒车/前进重叠路径、换向等待及重复/不稳定定位。Reeds–Shepp和HC均使用原生曲线做运动学模拟闭环。

隔离ROS测试使用已审计无其他节点的Domain229和`ROS_LOCALHOST_ONLY=1`，启动真实OpenNav与模拟定位/底盘：四种人工曲线规划、未允许倒车时拒绝、带方向保存/重载、独立开始、负速度输出、完整换向并完成。另验证保存路径复用、一键规划和回形取消等兼容流程。原本机Domain225的实车入口未用于注入模拟数据。

日志：`.diagnostics/native-curves-regression.log`、`native-curves-geometry.log`、`native-curves-ros-2.log`、`native-curves-colcon.log`及`native-curves-final-build.log`。第一次ROS联调的场地空间不足和测试路由残留问题及失败日志保留在`native-curves-ros.log`，修正后四项联调通过；最终计数与加载情况见本次完成记录。

最终验证：306项Python回归通过、18项需显式启用的集成测试跳过（`native-curves-final-regression.log`）；94项前端用例通过，其中本机代理用例初次被沙箱EPERM阻止，单独允许本机通信后3项全部通过，原失败日志保留（`native-curves-all-web.log`、`native-curves-proxy-retest.log`）。前端类型检查/构建通过（`native-curves-final-build-2.log`），4个ROS包构建通过。隔离ROS完整4项通过（73.54 s），最后方向元数据修改后原生曲线/倒车执行及一键规划2项复测通过（39.51 s，`native-curves-final-ros.log`）。未进行实车运动验证。

本机加载：确认Domain225/localhost-only、无控制权、急停锁定、零输出且原启动参数为`chassis:=none`、全部设备关闭后，保留原参数重载核心和后端。初次继承旧进程环境导致构建后的跟踪包元数据路径缺失，日志保留在`native-curves-loaded.json`及`.runtime/project/native_curves_core_*.log`；随后只重载核心并重新source现有overlay，跟踪与规划均IDLE且状态新鲜，后端`/api/workspace`恢复200，新默认DubinsCC、半径1 m、允许倒车false核对通过。恢复记录`.diagnostics/native-curves-overlay-recovered.json`，核心日志`.runtime/project/native_curves_core_overlay_refreshed.log`。未开启硬件、复位急停或开始任务；刷新网页后需保存并重新规划使用新设置。
