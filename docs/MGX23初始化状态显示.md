# MGX23 初始化状态显示

Web 总览“机器人状态 → GNSS 定位信息”移除“角速度（rad/s）”，增加初始化总状态和四项条件。行驶速度、航向、经纬度和平面坐标继续使用现有数据源。

按用户给定判断规则及项目内《元控组合导航设备MGX23用户手册V1.0》§2.4：

| 标志 | 掩码 | 页面含义 |
| --- | --- | --- |
| READY | `0x00000002` | 滤波器收敛 |
| PPS_READY | `0x00040000` | 收到 PPS 信号 |
| SYNC_READY | `0x00080000` | 时间同步成功 |
| GNSS_CONNECT | `0x00100000` | 解析到 RTK 数据 |

判断为 `(status & 0x001C0002) == 0x001C0002`。其它位不参与本项判断。四项同时满足显示绿色“初始化成功”；缺少任一项显示“未初始化完成”，各项分别显示“已满足/未满足”。不使用手册另外的 FInitOk 标志替代这四个条件。

后端订阅已有 `/mgx23/status`（`std_msgs/msg/UInt32`），在 `/api/devices` 的 `mgx23` 字段返回 `status_word`、`initialized`、`flags`、`fresh` 和 `age_seconds`。没有状态字或接收超过 1 s 时，初始化结果和各项为未知；网页也累计请求传输及接收后的时间，过期显示灰色“未更新”，不会保留绿色成功标记。该显示只读，不改变导航有效性判定、安全控制、急停或任务权限；初始化成功不等于已确认 RTK 固定解或绝对定位精度。

原驱动只在位置/姿态/精度通过导航过滤后发布状态字，本次将状态字独立为：校验和通过且设备时间有限、与上一状态样本不同即可发布。位置、姿态、速度仍经过原导航过滤。这样初始化阶段精度不合格时也能看到四项状态；冻结的重复时间样本不会刷新状态。坏校验帧仍被丢弃。

代码位置：`mgx23_ws/src/mgx23_driver/mgx23_driver/mgx23_node.py`、`agri_backend/devices.py`、`agri_web/src/components/dashboard/RobotStatus.vue`。

验证命令（均不要求连接车辆，不发布模拟 ROS 数据）：

```bash
cd /home/zhang/agri_robot
source scripts/setup_environment.bash
export TMPDIR="$PWD/.runtime/tmp"
export ROS_LOG_DIR="$PWD/.runtime/ros_logs"
mkdir -p "$TMPDIR" "$ROS_LOG_DIR"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 agri_backend/.venv/bin/python -B -m pytest -q tests/test_device_api.py tests/test_mgx23_driver.py -p no:cacheprovider
node tests/web_mgx23_status.test.cjs
cd agri_web
npm run build
```

2026-11-04 软件验证：驱动、状态接口和 MGX23 速度相关测试共 43 项通过；实际 Vue 组件的离线渲染测试 3 项通过，覆盖未知、四项各自缺失、全部成功和过期清除；前端类型检查及生产构建通过，`mgx23_driver` 构建通过。HTTP 测试未启动应用生命周期或 ROS 节点；沙箱内 TestClient 线程通信等待后，使用同一项目虚拟环境在沙箱外重跑通过。未进行真实 MGX23 设备初始化或实车验证。

部署后需重新构建并在停车时重启 MGX23 驱动，使独立状态发布生效；后端需加载新版本，前端开发服务器会热更新，生产部署需使用新构建。初始化状态显示不要求启动路径规划或执行导航任务。
