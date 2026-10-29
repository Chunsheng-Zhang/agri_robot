# Web 实时数据验证记录

验证日期：2026-10-28。验证对象为现有 `agri_web` Vue 总览与 `agri_backend` FastAPI 桥接；没有启动真实 MGX23、底盘、雷达、手柄或相机，没有修改原始 Yahboom 工作区。

| 检查 | 结果及边界 |
| --- | --- |
| `agri_web` 的 `npm run build` | 通过，包含 Vue/TypeScript 检查和生产构建 |
| `tests/test_device_api.py` | 4 项通过：设备过期/未知、服务不可用、参考/实际路径坐标及非法数值处理 |
| `tests/test_backend_ros_path.py` | 独立 DDS Domain 227 下 1 项通过：桥接启动前已发布的 transient-local 路径可补收；切换到原 volatile 周期路径后仍可接收 |
| `tests/web_realtime.spec.cjs` | 通过：所有 API/视频请求由隔离测试拦截；核验自动刷新经纬度/位姿、路径/任务/距离、拒绝复位后保留急停锁定、错误坐标系拒绝叠加、相机断开/视频错误、后端断连后隐藏旧值并禁用解除 |
| 实际 FastAPI 与现有 Vite 代理 | 只读 GET 验证 8000 后端及 5173 同源 `/api` 转发正常 |
| 实际页面浏览器检查 | 无 JavaScript 页面错误；ROS 桥接已启动但没有硬件数据，位姿离线、坐标未知、安全状态未知且解除急停禁用 |
| Python 编译与 `git diff --check` | 通过 |

测试环境使用项目 `.venv` 启动实际后端；HTTP 测试依赖使用 `/tmp/agri-device-api-venv`，浏览器依赖与截图位于 `/tmp`，不作为持久部署依赖或实车实验数据。

复现 API 与独立 DDS 检查：

```bash
cd /home/zhang/agri_robot
source scripts/setup_environment.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /tmp/agri-device-api-venv/bin/python \
  -m pytest -q tests/test_device_api.py
ROS_DOMAIN_ID=227 AGRI_RUN_ROS_BRIDGE_TEST=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /tmp/agri-device-api-venv/bin/python -m pytest -q tests/test_backend_ros_path.py
```

上述临时环境只用于本次验证；其他环境用已安装 FastAPI/httpx/pytest 且能导入 ROS 的 Python 替换。DDS 测试默认跳过，显式启用时只允许 200–230 的隔离 Domain，并应选择没有其他机器人进程使用的编号。

浏览器检查步骤见 [Web 实时数据显示与启动](Web实时数据显示与启动.md)。测试数据仅存在于拦截脚本和隔离 DDS 域，没有注入实际 5173 页面或正常机器人 ROS 域。

未验证：Jetson 上硬件驱动运行、MGX23 实测数据、4ROS 实物障碍停车、Astra Pro Plus 实物视频端到端延迟，以及跨物理设备的 DDS/网络连通。这些仍需按 [室外实验指导文件](室外实验指导文件.md) 完成；当前结论不能作为实车验收结果。

## 2026-10-29：三模式切换与任务工作台

已完成旧 Python GUI 功能分析，操作与参数取舍见 [Web 任务操作与旧 GUI 功能迁移](Web任务操作与旧GUI功能迁移.md)。没有运行旧 GUI，没有修改或启动原始 Yahboom 驱动。

| 验证 | 结果 |
| --- | --- |
| ROS overlay `colcon build --symlink-install` | 7 个包通过，含新增 `agri_interfaces` |
| Vue `npm run build` | 类型检查与生产构建通过 |
| 完整 pytest（全部软件集成开关开启） | 70 项通过、1 项跳过；约 40 s；跳过为 ARM 原厂视频程序与本机架构不匹配 |
| 模式/控制权竞态 | 旧模式申请、旧任务释放和旧速度不能影响新控制权；切换撤权停车，不自动恢复 |
| 独立路径源 | 原 CSV volatile 周期路径和先发布的 transient-local 路径均可接收；多个参考发布者拒绝开始/停止 |
| 真实 FastAPI→ROS 集成 | 保存/应用任务、急停约束、多点暂停/继续、模式取消、OpenNav 半径重新配置、覆盖跟踪、ZIP 和遥控过期释放通过 |
| 浏览器实时错误回归 | 过期/断连/未知、拒绝复位、错误坐标系、相机断开/视频错误通过；API 与视频请求由测试拦截 |
| 浏览器真实软件流程 | 实际 Vue→FastAPI→隔离 ROS/OpenNav，完成模式切换、编辑保存、暂停/继续、遥控按住/松开与失焦停车、GeoJSON 导入、规划跟踪/取消、下载 ZIP、设备电压与手机布局；无页面 JS 异常 |
| 历史记录 | 已完成任务不会因后续模式切换被改写成取消；配置版本与参考路径不匹配时不能开始 |
| 文档启动入口 | `cd agri_backend; uvicorn main:app` 的导入与规范包导入共享同一设备缓存 |
| Python、diff、文档链接/QoS | 通过；没有将虚拟环境、构建产物或实验数据加入源码 |

完整软件回归：

```bash
source scripts/setup_environment.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 AGRI_ROS_TESTS=1 AGRI_WEB_ROS_TESTS=1 \
  AGRI_RUN_ROS_BRIDGE_TEST=1 ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/tmp/agri_web_all_tests \
  /tmp/agri-device-api-venv/bin/python -m pytest -q tests
```

浏览器真实软件流程使用三个终端，全部在测试机器上，不能连接实车 Domain：

```bash
# 终端 A，项目根目录；只启动模拟硬件和真实软件规划器
source scripts/setup_environment.bash
ROS_DOMAIN_ID=223 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/agri_web_browser_ros \
  agri_backend/.venv/bin/python tests/web_test_runtime.py --port 18000
# 终端 B，独立前端实例
cd agri_web
AGRI_BACKEND_URL=http://127.0.0.1:18000 npm run dev -- --host 127.0.0.1 --port 15173 --strictPort
# 终端 C，项目根目录，替换自己的 Playwright 安装位置
PLAYWRIGHT_MODULE_PATH=/path/to/node_modules/playwright \
  node tests/web_workspace_e2e.cjs
```

脚本先检查仅测试系统提供的 `/api/test-environment` 标记，页面显示“软件联调 · 模拟定位”。任务、CSV 和规划结果写在自动创建的 `/tmp/agri-web-synthetic-*`，正常关闭后清理；不会注入正常 5173/8000 服务。每次重跑完整浏览器流程应先正常停止并重启终端 A，恢复空任务库及初始急停状态。测试依赖和 `/tmp` 截图/ZIP 不是持久部署依赖或实测数据。

本次是软件闭环与浏览器功能验证。MGX23 的真实 RTK/INS 状态、杆臂/航向标定、4ROS 实车制动、底盘固件 watchdog、Astra Pro Plus 实际视频和跨设备网络仍需 Jetson 外场验收。
