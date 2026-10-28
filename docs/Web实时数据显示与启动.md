# Web 实时数据显示与启动

代码位置：前端 `agri_web/src/views/DashboardView.vue`，后端 `agri_backend/main.py` 和 `devices.py`。总览页面已接入实时数据；侧栏其他未开发页面仍为占位页。网页查询不会自动开始导航任务。

## 1. 页面现在显示什么

| 显示内容 | ROS 来源 | 页面行为 |
| --- | --- | --- |
| 经纬度 | `/mgx23/position` | 显示真实经纬度，定位原始消息过期后显示 `--` |
| 平面位置、航向、观测速度和角速度 | `/odom` | 位置为 map/odom 重合的米制坐标；ROS 航向为东 0°、逆时针正，区别于 MGX23 原始航向 |
| 路径和位置图 | `/reference_path`、`/tracked_path`、`/odom` | 绿色参考路径、橙色实际跟踪轨迹、蓝色当前位置/航向；路径必须为 map 坐标系 |
| 控制模式、定位有效性、手柄/雷达、急停及障碍锁定 | `/safety/status` | 安全状态过期显示未知；解除急停按钮禁用 |
| 全局/前方/后方最近障碍距离 | `/safety/obstacles` | 仅显示新鲜且有效的测量，实际停车判定仍在 ROS 安全模块 |
| 多点、跟踪和规划任务 | `/waypoint/status`、`/tracking/status`、`/planning/status` | 显示实际状态、已有任务进度和控制权；未启动模块显示节点未更新 |
| 相机状态/画面 | `/camera/status`、原始图像及 `web_video_server` | 图像接收状态与视频 HTTP 连接分别判断，断流显示提示 |

状态每秒查询一次，路径每 2 秒查询一次，视频为 MJPEG 连续流。ROS 安全和控制循环独立运行。页面位姿要求 odom 接收年龄小于 1 秒；后端设备状态通常 1 秒过期、原始 GNSS 0.5 秒过期；浏览器连续约 2.5 秒没有成功收到快照也会隐藏旧状态。显示的位姿时间是后端接收时间。

电池尚未接入，显示“未接入”。面积和覆盖率不再使用示例数字。当前实际轨迹来自路径跟踪节点，不是所有模式的永久行驶记录；后端只向页面提供最近最多 10000 个轨迹点，页面绘制时进一步抽样。完整研究数据仍使用 rosbag 和跟踪 CSV。实时界面及 API 内存缓存不代替实验归档。

## 2. 三个终端启动

### 终端一：ROS 核心

若核心已启动，沿用原进程，勿重复启动。实际室外实验按 [室外实验指导文件](室外实验指导文件.md) 选择模式和底盘。

仅查看设备、不接通底盘驱动的示例（在设备所在 Jetson 上运行）：

```bash
cd /home/zhang/agri_robot
bash scripts/start_core.bash mode:=base chassis:=none \
  start_joy:=true start_lidar:=true lidar_model:=4ROS \
  start_camera:=true start_video:=true
```

启动前须已构建本项目 ROS overlay，并配置现场固定原点与设备端口。原始驱动继续复用 Yahboom 工作区。若已有相机/雷达/手柄进程，只启动所需的监控或缺少的设备，不能启动两个相同驱动。`start_video:=true` 还要求原始 `web_video_server` 及依赖可执行；仓库内 ARM 设备程序不能在 x86 电脑直接运行。

### 终端二：FastAPI 后端

```bash
cd /home/zhang/agri_robot
source scripts/setup_environment.bash
source agri_backend/.venv/bin/activate
python -c 'import rclpy, fastapi, uvicorn; print("ROS/Web imports OK")'
cd agri_backend
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

以上使用项目现有 `.venv`，要求其 Python 版本兼容 ROS Humble 且能导入系统 ROS 包；如部署环境不同，使用自己的持久 Python 环境。后端和核心必须具有相同的 `ROS_DOMAIN_ID`、DDS 网络配置及时间基准。不要使用多个 Uvicorn workers，也不要同时启动两个后端桥接。

可检查 `http://127.0.0.1:8000/api/robot/status` 与 `/api/devices`。`ros_connected=true` 只说明桥接节点启动，不能证明定位和设备已在线。

### 终端三：Vue 前端

```bash
cd /home/zhang/agri_robot/agri_web
# 依赖未安装时先运行 npm ci；Node 版本须满足 package.json 的 engines。
npm run dev -- --host 0.0.0.0 --port 5173
```

Jetson 本机浏览器打开 **http://127.0.0.1:5173/**。同一局域网其他电脑/手机打开 **http://Jetson实际IP:5173/**，以终端显示的实际端口为准。默认 Vite 将 `/api` 代理至其所在机器的 `127.0.0.1:8000`，将 `/video` 代理至 `127.0.0.1:8080`，浏览器无需直接访问本地 8000/8080。

若已有 Vite 正在运行，修改配置后检查其重启日志；前后端代码更新后重新启动对应服务并刷新浏览器。

## 3. 前端在笔记本、后端在 Jetson

在笔记本的 `agri_web` 中复制 `.env.example` 为 `.env.local`，改为实际地址：

```dotenv
AGRI_BACKEND_URL=http://192.168.1.100:8000
AGRI_VIDEO_URL=http://192.168.1.100:8080
VITE_API_BASE_URL=
```

重启 Vite，在笔记本打开其 5173 页面。上述 IP 只是格式示意，不能直接作为现场地址。保持 `VITE_API_BASE_URL` 为空即可继续使用同源代理。视频默认后端返回相对 `/video/stream?...`；不要把后端 `AGRI_VIDEO_BASE_URL` 设为浏览器无法访问的 localhost。

`npm run build` 生成 `agri_web/dist`。Vite 的代理只服务于开发/preview，生产部署静态文件时还需配置 Nginx 的 `/api`、`/video` 代理，视频示例见 `deploy/nginx/agri-video.conf.example`。

## 4. 静止状态下核对实时数据

1. 对照 `ros2 topic echo /mgx23/position --once` 与网页经纬度；对照 `/odom` 与平面位置、ROS 航向。位姿不更新时先检查 ROS 核心和固定原点，不要用后台查询时间冒充传感器时间。
2. 对照 `/safety/status` 的 owner、急停和障碍状态以及 `/safety/obstacles` 的最近距离。显示有效不代表确认 RTK 固定解。
3. 通过既有 ROS 服务启动/暂停任务时，观察实时任务表更新；本次仅接入任务显示，网页尚无完整任务编辑/开始/暂停/取消面板。
4. 生成 map 参考路径后查看路径图。后端现在会检测参考路径发布者的持久性 QoS，能够补收仍在线发布者保存的最后一次规划；发布者已经退出、未保留历史或混有不兼容发布者时仍需重新发布。
5. 在底盘未启用或已确认静止条件下停止定位/设备进程：数值应变为 `--` 或未知，当前位置隐藏、视频移除、解除急停禁用。恢复数据后可重新显示，不自动开始任务。
6. 核对相机原始图像和视频服务，再观察网页。图像 Topic 新鲜不等于视频 HTTP 正常。页面会提示视频连接失败并重试；持续连接但内容冻结的端到端延迟仍需按实验指导用实际时钟画面测量。

网页急停与解除调用现有 ROS 安全服务，以服务返回和持续状态反馈为准。HTTP 409 表示拒绝、503 表示不可用、504 表示确认超时；页面不会在请求成功前自行把安全状态改成“已解除”。未知状态仍允许尝试发送急停，禁止解除急停。网页没有直接发布底盘速度的入口。

## 5. 本次代码验证与范围

`npm run build` 同时执行 Vue/TypeScript 类型检查和生产构建。`tests/test_device_api.py` 核对设备未知状态、路径接口和安全服务调用边界。`tests/web_realtime.spec.cjs` 使用隔离浏览器拦截全部 API/视频请求，验证自动更新、断连、相机异常及拒绝复位显示；测试数据不会进入正常页面或 ROS。

浏览器测试需要独立安装 Playwright 与 Chromium，并启动本地 Vite 测试服务。例如：

```bash
# 一个终端（不要占用已运行的页面）
cd agri_web
npm run dev -- --host 127.0.0.1 --port 15173 --strictPort
# 另一个终端，在项目根目录；按自己的安装位置设置环境变量
AGRI_WEB_TEST_URL=http://127.0.0.1:15173 \
PLAYWRIGHT_MODULE_PATH=/path/to/node_modules/playwright \
node tests/web_realtime.spec.cjs
```

软件验证不能代替 Jetson 上的 ROS 网络、MGX23、4ROS 和 Astra Pro Plus 实机联调。当前没有将硬件实时数据伪装成在线状态。
