# 首版 ROS 2 核心验证记录

验证环境具有 ROS 2 Humble、系统 Python 3.10、现有 OpenNav C++ 安装，以及 pyproj/PyYAML/Shapely/pytest。验证过程中没有启动真实 MGX23 TCP 接收节点或底盘串口驱动，也没有修改原始 Yahboom 工作区。

| 验证 | 结果 |
| --- | --- |
| MGX23 驱动 `colcon build --symlink-install --packages-select mgx23_driver` | 1 个包构建成功 |
| 新 `agri_robot_ws` 的 `colcon build --symlink-install` | 6 个包构建成功 |
| 算法、坐标与协议单元测试 | 32 项通过 |
| DDS 无硬件闭环集成测试 | 1 项通过，约 17 秒 |
| 真实既有 OpenNav C++ Action 集成 | 2 项通过，包括规划结果接受与边界拒绝 |
| launch 参数解析、覆盖模式节点描述生成 | 通过，不执行硬件节点 |
| Python 编译、Bash 语法、所改既有文件 diff 空白检查 | 通过 |
| 原始 Yahboom 工作区 diff | 无改动 |

共 35 项测试分别完成验证。ROS 测试运行于 localhost 的隔离 domain，使用模拟 GNSS/INS 和简化差速运动学；测试输出写到 `/tmp`。

闭环集成覆盖 waypoint 的按序到点、停留、暂停/继续和完成，Pure Pursuit 的实际位姿反馈、跟踪与轨迹发布，互斥控制权争用，定位超时停车和恢复后不自动运行，急停/解除以及重复最终速度发布者的冲突检测。单元测试另覆盖闭合路径不在起点误判完成、相邻条带不全局跳行、速度限制、旧指令、NaN、空路径、边界内孔和设备帧重组。

覆盖规划实际调用仓库中已安装的 `opennav_coverage` 服务，经 configure/activate 后发送 ComputeCoveragePath goal。3 m 地头样例通过输出净空检查并保存结果；无预留地头样例被拒绝且不发布可执行路径。调试过程中也观察到原 2 m 地头配置的连接曲线越界，保留逐次输出校验并将示例地头配置设为 3 m。

Humble 参数实验确认：当参数文件给出了节点专属值，命令行通配 `-p headland_width:=...` 不应被当作该专属值的可靠覆盖。测试用 `-p coverage_adapter:headland_width:=...`，并断言节点实际值，防止对比测试拿相同参数重复运行。

未验证的内容包括实际 MGX23 发布频率与网络延迟、设备 RTK/INS 解状态、航向安装偏角、天线杆臂、真实底盘串口依赖、驱动/固件 watchdog、真实履带侧滑与制动。当前结果证明软件模块及其通信链在上述测试条件下可运行，不等同于完成实车导航验收。

## 固定原点与轨迹对齐补充验证（2026-10-28）

保持统一 YAML 配置，不增加 Web 原点入口。使用 `LocalFrame` 的节点将原点经纬度和工作半径声明为只读，避免运行参数与已创建的转换器不一致。跟踪 CSV 列保持不变，新增同名 JSON，记录投影/原点、坐标系、单位、`base_link` 参考点与本任务收到的参考路径快照。

本次运行 `tests/test_navigation_core.py`、`tests/test_ros_core.py` 和 `tests/test_ros_coverage.py`：31 项通过，约 20 秒。后两者在独立 Domain 226 中使用模拟定位及真实既有 OpenNav C++ 规划服务器，没有启动真实设备。

闭环检查确认定位、多点导航与跟踪节点的原点完全一致，运行期更改原点参数被拒绝；保存的参考路径快照与实际接收路径逐点一致，CSV 的 x/y 与 map 实际轨迹消息逐点一致。OpenNav 检查另外确认规划 JSON 的原点/EPSG 和坐标与发布路径一致。Python 编译和 diff 空白检查通过。

```bash
source scripts/setup_environment.bash
AGRI_ROS_TESTS=1 ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/tmp/agri_origin_alignment_ros PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /tmp/agri-device-api-venv/bin/python -m pytest -q \
  tests/test_navigation_core.py tests/test_ros_core.py tests/test_ros_coverage.py
```

复现时使用能导入 ROS/pytest/Shapely/pyproj 的 Python；上述 `/tmp` 虚拟环境仅供本次验证。此检查证明软件坐标链和文件基准一致，实际定位、安装杆臂和路径跟踪误差仍需实测。旧 CSV 没有同名元数据时，需核对当轮 YAML 或 Bag 原点；外部 map 路径也须按相同原点生成。
