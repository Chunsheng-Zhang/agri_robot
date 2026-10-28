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
