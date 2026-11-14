# Web实时日志与串口信号流

更新：2026-11-13。参照旧`open_navigation_ws/gps_navigation/scripts/gps_field_gui.py`的日志、串口信号窗口交互与`serial_protocol.py`协议解释，新增项目级实时诊断。未修改Yahboom厂商工作区、Rosmaster库或旧GUI，不启动旧GUI的进程。

## 页面位置及使用

所有页面顶部，在“急停锁定”状态左侧增加“日志”“信号流”两个按钮。点击打开相应面板，再点同一按钮、右上关闭或Esc关闭。面板不加全屏遮罩，顶部急停入口仍可点击；窄屏保留诊断与急停控件，收起非关键标题/用户区。

- **日志**：汇总本次后端会话收到的`/rosout`、规划/多点/跟踪任务状态变化、安全事件、Web任务操作及API错误。选择WARN/ERROR等级或搜索节点、来源和错误文字，可查看本轮规划失败、定位/控制故障等原因。
- **信号流**：查看Jetson—MCU真实串口观察帧，TX蓝色、RX绿色、校验异常红色。显示采集时间（北京时间，毫秒）、方向、功能字/名称、已知字段解释和完整HEX；可筛选TX/RX、搜索功能字或数据内容。
- **暂停显示**只停止当前网页轮询，后台仍采集；继续时跳到最新记录。
- **清屏**只清除当前浏览器已显示的记录，不截断驱动数据或归档文件。
- 自动滚动可关闭，便于复制某段数据。关闭面板、网页隐藏或组件卸载时中止请求；默认关闭，不持续传输诊断数据。每路最多一个在途请求，完成后再等0.5秒刷新，2秒未应答显示接口未更新。
- 页面保留最近500条，后端各保留2000条。后台记录速度大于网页显示速度或暂停后恢复时，可能跳过部分显示记录，界面明确计数；不是车辆通信丢包率。

日志来源独立标注：ROS节点名、planning/waypoint/tracking/following状态、safety、web_task、http，以及core.stdout/backend.stdout/frontend.stdout等。当前一键启动目录中三个服务的stdout/stderr由只读后台线程补充，包含驱动初始化失败、进程异常退出和Vite错误。手工终端的stdout不会自动被读取；其ROS日志、任务状态和后端API错误仍能显示。不会扫描任意文件、用户历史终端或其他项目日志。

## 信号流的数据含义

```text
原厂Mcnamu_driver_X3节点及Rosmaster库
  ├─ 同一串口write返回的实际已写入字节 → TX观察
  └─ 原有接收线程read返回的实际字节     → RX观察
                    ↓
       /base/serial_frames 与 /base/serial_status
                    ↓
        FastAPI只读缓冲 /api/diagnostics/serial
                    ↓
           顶部“信号流”实时面板
```

串口仍由原厂驱动打开一次，由原接收线程读取一次；观察器不打开第二个串口、不抢读、不发测试指令、不生成新的速度指令。观察发生在现有Python串口调用旁路，串口读写路径不进行文件写入或网络查询；有界内存队列经原节点定时器批量发布诊断话题，传输采用best-effort，避免诊断消费者阻塞驱动。

**TX表示字节被串口write接受，不表示MCU已收到或执行。RX表示软件实际读到了协议字节，不是逻辑分析仪测得的电平，也不证明与某个TX一一对应。** `/cmd_vel`、`/base/driver_cmd_vel`和`/vel_raw`均不用于伪造原始帧。没有底盘时，信号流为空，界面显示未接入/未更新。

- 帧头沿用现有Rosmaster协议：TX `FF FC`，RX `FF FB`，长度字节包含长度/功能字/校验字节，整帧长度=长度字段+2，校验和为长度字段至数据末尾累加模256。
- 常用TX `0x12`含车型字节及小端有符号vx、vy、wz，速度值除1000；X3车型为1，不能漏掉前面的车型字节。
- RX `0x0A`含小端vx/vy/wz及电池电压，单位与现有库一致；`0x0D`解释四个32位编码器数值。它们仅用于观察，不参与项目定位估计。
- 未知功能字保留HEX，不猜测字段。坏校验和/字段不足会标注；坏帧不显示成有效测量。
- “采集节点在线”和“MCU有新鲜有效RX帧”分别判断：TX持续发送不等于MCU在线。状态/有效RX超过约1秒没有更新则变为未更新，包含来源时间和网页请求耗时；暂停、隐藏或接口过期不保持绿色在线指示。
- “采集队列丢弃”表示驱动观察队列超过1024帧；“观察序号缺口”表示后端没有观察到连续编号，原因可能包括诊断队列、DDS或后台接收。均不能直接解释为串口线缆丢包或MCU丢包率。

## 加载方式

已构建新增入口`agri_base/yahboom_driver_monitor`。统一启动中`chassis:=yahboom_x3`会通过这个包装入口实例化**原厂**`yahboomcar_bringup.Mcnamu_driver_X3.yahboomcar_driver`，沿用原节点名、参数、灯光/蜂鸣器与速度控制；`cmd_vel`仍重映射至`/base/driver_cmd_vel`。`chassis:=none`或`external`不会额外启动底盘。

现有运行中的原厂独立进程不会被自动注入，也不会被强制停止。更新后需在任务结束、车辆停稳时，构建并正常重启核心与后端，刷新网页；不要同时启动原厂独立命令和新包装入口。使用一键脚本管理的项目按原`project.bash stop/start`流程加载；手工启动时继续保留原先的params_file、ROS Domain及各设备开关，不因加载诊断而重新打开已关闭设备。

```bash
cd /home/zhang/agri_robot
source scripts/setup_environment.bash
export TMPDIR="$PWD/.runtime/tmp" ROS_LOG_DIR="$PWD/.runtime/ros"
cd agri_robot_ws
colcon build --symlink-install --packages-select agri_base agri_bringup
```

手工集成且核心使用`chassis:=external`时，可在停车并停止旧底盘后单独使用以下入口，仍确保系统仅一份底盘驱动：

```bash
ros2 run agri_base yahboom_driver_monitor --ros-args \
  -r __node:=yahboom_driver -r cmd_vel:=/base/driver_cmd_vel
```

Rosmaster库仍按当前环境/原`rosmaster_pythonpath`选择，串口设备和波特率仍由原厂库配置，本次不调整通信协议、控制参数或固件。

## 记录保存位置

每次后端启动建立独立目录：

```text
.runtime/diagnostics/YYYYMMDD_HHMMSS_微秒/
  logs.jsonl             运行日志（含原始来源）
  logs.jsonl.1 / .2       滚动日志
  serial.jsonl           实际观察帧、原始HEX、解码、驱动会话及序号
  serial.jsonl.1 / .2     滚动信号记录
```

每路单文件上限5 MiB、另保留2个滚动备份，每个后端会话两路合计约30 MiB。这是有界运行诊断，不是保证全量的科研原始串口录制；原有`data/experiments`任务采集不变，也不回填历史数据。诊断目录不纳入Git。原厂驱动独立启动且未用包装入口时，不会产生本项目串口观察数据。

归档在后台线程完成，队列最多4096条。写入错误会显示归档异常，实时内存查询仍可用；不把归档写入失败反复打印到其自身跟随的服务日志。日志正文按长度裁剪，并遮蔽常见密码、Token及Authorization字段；页面不提供向串口写入或绕过安全控制的操作。

## 接口和代码

| 接口/位置 | 用途 |
| --- | --- |
| `/base/serial_frames`，`std_msgs/msg/String` | schema_version=1的TX/RX批次；每批最多64帧，最多20Hz刷新 |
| `/base/serial_status`，`std_msgs/msg/String` | 约4Hz采集心跳、方向计数、队列丢弃和错误 |
| `/rosout`，`rcl_interfaces/msg/Log` | ROS日志 |
| `GET /api/diagnostics/logs` | 当前后端会话运行日志 |
| `GET /api/diagnostics/serial` | 串口观察帧及状态 |
| `agri_base/serial_observer.py` | 字节旁路、流式组帧、协议解码及原厂驱动包装 |
| `agri_backend/diagnostics.py` | 有界缓冲、日志归档、当前服务输出补充及只读API |
| `RuntimeDiagnostics.vue`、`useDiagnostics.ts` | 顶部入口、面板及可取消的独立轮询 |

API使用`after`游标、`session`会话和`limit`（1–300，默认200），新会话自动重置旧游标。前端暂停/清屏不请求服务端删除，未知接口和POST写操作不支持。

## 验证记录

2026-11-13：完整软件回归348项通过、20项可选/平台测试跳过；新增归档写入异常保护后，诊断/API/启动入口相关12项测试通过。前端8项诊断测试验证按需加载、单请求、暂停/关闭/隐藏取消、旧响应隔离、清屏、请求超时、来源新鲜度、有界内存及实际面板渲染。前端与base/bringup两包构建通过。

隔离Domain229、`ROS_LOCALHOST_ONLY=1`联调1项通过：合成串口观察帧和实际ROS日志进入Web接口，RX停更及观察器停止后状态变为未更新；合成`/vel_raw`不会被当作串口帧，保持唯一`/cmd_vel`发布者及零运动。原厂库字节对比测试使用内存串口，确认包装前后运动指令字节一致、无额外读取，厂商源码字节未改变。未打开真实串口、未验证实车通信时延或丢帧、未重启当前车辆服务。

证据：`.diagnostics/runtime_monitor/`中的`full2.log`、`final-api.log`、`web.log`、`frontend-build.log`、`ros-build.log`、`ros.log`。首次回归的旧启动断言未适配包装入口、测试日志传播受launch.logging影响导致2项失败，修正相应测试设置后全量复测通过，未将失败记录隐去或作为实车通过依据。
