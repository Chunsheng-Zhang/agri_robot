# Web跨网络遥控：持续心跳与独立应答

2026-11-06。用户批准将原180ms逐条HTTP等待改为持续心跳、独立确认应答，等待上限500ms；保留约200ms指令有效期、300ms启用门槛、急停、断联保护及相机同时观察。最新要求：偶发过期帧不弹提示，不要求松开重按，持续按住时尽量连续行驶。当前协议版本为4。

## 使用方法

项目继续使用原一键入口：

```bash
bash /home/jetson/agri_robot/scripts/project.bash start
```

加载新版后，在操作电脑按Ctrl+F5刷新一次，重新开启相机预览，在基础待命、停车且满足安全条件时点击“启用Web遥控”。按住方向按钮行驶，松开停车；键盘按住空格配合W/A/S/D。相机不因遥控自动暂停。

偶发过期帧仅丢弃，不再显示“通信波动，旧指令已丢弃”或“松开后重新按方向恢复”，也不设置恢复锁、不发送额外恢复零速度握手。只要当前仍按住方向、会话和安全状态有效，网页每80ms继续生成新序号和新期限的当前方向帧。后端只执行新鲜帧，不重放过期帧。

真正断线、500ms无应答、持续500ms没有新鲜指令、网页调度中断超过200ms、协议或时间戳异常仍退出。这些情况排除后需要重新启用，系统不自动重连或重新取得控制权；无需重启整个项目。失焦、隐藏、离页或安全状态变化也仍停止。

## 指令连续性与停车边界

| 项目 | 当前实现 | 用途 |
| --- | --- | --- |
| 建立WebSocket | 最多500ms | 取得控制权前完成，不自动重连 |
| 启用请求往返 | 不超过300ms | 用服务端时间和单调时钟建立期限锚点，保留启用门槛 |
| 发送心跳 | 80ms一条 | 新序号、新期限，不等待上一条应答 |
| 指令有效期 | 前端约200ms，后端最多保留200ms | 到期归零，不因应答或过期帧续期 |
| 应答等待 | 最多500ms，后续序号可累计确认 | 回程较慢时仍继续发送，持续无确认则退出 |
| 网页调度间断 | 超过200ms退出 | 防止网页卡住后继续旧操作 |
| 后端无新鲜指令 | 超过500ms释放会话 | 过期帧不刷新此期限 |

例如，最后一条已接受指令仍剩100ms有效期，接着收到一条晚到5.8ms的帧：仅丢弃晚到帧，不主动发零；下一个新鲜心跳在100ms内到达即可连续执行。这与“把过期帧当有效指令”不同。

如果最后一条有效指令真正到期，车端仍归零；在同一有效会话内，下一条由持续按压新生成、且未过期的心跳到达后可继续，不再要求重新按键。持续无新鲜指令超过500ms则不能这样继续，必须重新启用。不能保证网络中断时完全无停顿，也不能通过执行过期指令或无限延长旧指令保证连续。

松开立即发送更新序号的零速度；迟到应答不能改变网页的当前按压状态。所有指令仍受安全节点、急停、控制权和限速约束。上述时间是软件期限，不是机械制动时间承诺。

## 实现

- `agri_web/src/control/manualChannel.ts`：单条有序WebSocket，最多8条未确认帧、2048字节发送缓冲；过期帧成功确认只表示已处理，不弹提示或触发恢复流程。断线或积压关闭，不重连、不重发旧帧。
- `agri_web/src/components/ManualControl.vue`：先连接、申请控制权并确认初始零速度，才开放方向。使用服务端时间锚点和 `performance.now()` 推算期限，电脑壁钟跳变不会直接改变期限。保持当前按压持续发送；松开、失焦、离页和安全状态检查继续保留。
- `agri_backend/workspace.py`：通道本身不申请控制权，第一帧必须为已有Token的序号1零速度。初始探测严格检查期限；建立会话后的过期帧返回 `applied:false`、`reason:expired`，不关闭连接。异常或断线仍释放绑定Token，旧连接不能撤销新Token。
- `agri_backend/ros_workspace.py`：协议4。过期帧只消耗序号，不改变上次接受的速度、截止时间和最后新鲜指令时间，也不强制归零。定时发布仍严格检查原截止时间，到期清除旧速度并发布零，避免旧运动自行重放；后续新鲜指令可按同一有效会话继续。
- `agri_web/vite.config.ts`：API的HTTP/HTTPS keep-alive Agent及WebSocket转发保留，视频代理独立，仍使用5173同源入口。

项目虚拟环境使用 `wsproto==1.2.0`；本次未更新系统、ROS、Node、Python或其他系统依赖。旧HTTP command接口仍严格拒绝过期帧。浏览器使用协议4能力检查，加载时需同步前后端并刷新浏览器。

Firefox开发者工具中，持续指令在 WS → `/api/control/channel` → Messages 查看；arm、stop仍为HTTP请求。帧包含会话Token，不公开提交原始帧或含Token抓包。

## 验证与记录

当前前端41项通过：通道7项、遥控组件17项、相机7项、状态刷新7项、HTTP代理3项。包括按住按钮/键盘经历过期反馈仍连续生成当前方向、无警告、无需恢复握手、松开后迟到反馈不能恢复旧方向、壁钟变化、500ms无应答和调度中断。

后端85项通过，包括17项通道测试：过期帧不打断尚有效的速度、不延长运动/会话期限、过期反向帧不改变当前方向、新鲜零速度后过期运动帧不能恢复运动、真正200ms间断归零、同一有效会话可接收后续新鲜运动、超过500ms不能继续、初始探测过期严格拒绝。

真实本机Vite→WebSocket→Uvicorn/wsproto测试使用假ROS端口，模拟回包延迟250ms并同时获取假相机JPEG，验证持续心跳；这是软件联调，不是公网实测。候选前端类型检查及构建通过。

在确认Domain 229空闲后，使用 `ROS_LOCALHOST_ONLY=1` 和显式合成硬件完成1项完整Web/ROS集成，67.87秒通过。验证单条过期帧不打断仍有效的输出、真实超过200ms的间断归零、同一会话后续新鲜指令继续，以及断开释放；未使用实车Domain或设备驱动。

可复用的离线回归命令：

```bash
cd /home/jetson/agri_robot
node --test tests/web_manual_channel.test.cjs tests/web_manual_control.test.cjs \
  tests/web_camera_preview.test.cjs tests/web_connection.test.cjs tests/web_proxy_keepalive.test.cjs
TMPDIR="$PWD/.runtime/tmp" agri_backend/.venv/bin/python -B -m pytest -q \
  tests/test_manual_channel.py tests/test_web_missions.py tests/test_device_api.py \
  tests/test_project_startup.py -p no:cacheprovider
cd agri_web
TMPDIR=/home/jetson/agri_robot/.runtime/tmp npm run build
```

本轮记录位于 `.diagnostics/manual-continuous-front.log`、`manual-continuous-backend.log`、`manual-continuous-build.log`；隔离ROS记录为 `manual-continuous-ros.log`。软件验证与真实跨网驾驶效果必须分别评价；本次不启动车辆、不取得实车控制权。

此前协议2和3的诊断、回归和加载审计仍分别以 `manual-channel-*` / `manual-soft-*` 命名保留，属于历史实现。当前规则以本文件协议4说明为准。临时产物不纳入普通Git提交，不修改历史实验数据。

## 本轮加载结果

加载前及重载前再次核对：无控制权、基础待命、三个任务均IDLE、指令速度为零，并取得20条真实 `/vel_raw` 零反馈。该话题仅用于停车核对，没有接入网页行驶速度显示。只重载统一入口管理的后端并保留原环境、活动配置、日志和设备开关；核心与设备驱动未重启，急停及雷达保护选择未改变。随后加载与测试候选逐字节一致的两个前端文件。

后端重载审计为 `.runtime/project/backend_manual_continuous_reload.json`。在线检查为 `.diagnostics/manual-continuous-live.json`：5173入口WebSocket握手101，未发送Token或控制帧；前后无控制权、急停一致，相机JPEG 200（本次8879字节）。本机检查不等于真实跨网驾驶验证，实际操作效果仍待用户确认。
