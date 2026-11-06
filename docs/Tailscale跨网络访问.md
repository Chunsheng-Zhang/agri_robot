# Tailscale 跨网络访问项目网页

更新于2026-11-06。由用户自行安装和登录；Jetson上的Tailscale不随系统开机自启，运行项目一键start并确认项目就绪后再启动它。11月6日已只读确认客户端在线、服务active且开机自启disabled；用户在另一网络的电脑绕过HTTP代理后访问网页入口返回200。跨网网页直连已验证，不代表已完成遥控、视频并行及实车行驶验证。

## 现有项目是否需要改网页

目前统一入口将网页监听在 `0.0.0.0:5173`，浏览器通过同源 `/api` 和 `/video` 代理访问后端与相机。已核对相机地址为 `/video/stream?...`，没有绑定旧局域网IP。因此第一阶段使用Tailscale IPv4访问，沿用现有前后端和相机配置。

连接方式：

```text
操作电脑/手机（已连接互联网并登录Tailscale）
    → 私有Tailscale网络
    → Jetson的Tailscale IPv4:5173
    → 项目现有 /api 和 /video 代理
```

两端需安装Tailscale并加入同一tailnet，访问策略允许操作设备连接Jetson的TCP 5173。浏览器地址为 `http://<Jetson的Tailscale-IPv4>:5173/`；实际地址要在登录后用 `tailscale ip -4` 获取，不能把示例的100.x地址当作真实地址。局域网访问入口继续可用。

Vite默认接受IP地址。若以后要使用MagicDNS主机名，需要将Jetson实际域名明确加入Vite允许列表；本次优先IP访问，不预先放宽全部主机名。

## 11月5日安装前的只读检查

- Jetson为Ubuntu 22.04、aarch64，未发现Tailscale客户端或tailscaled系统服务。
- 5173网页、8000后端与8080视频服务正在监听。
- 默认路由优先经过有线连接，另有Wi-Fi默认路由；关闭终端HTTP代理后，默认路径和指定Wi-Fi路径都能连接Tailscale登录站点，当前不修改NetworkManager配置或MGX23链路。
- 官方ARM64包 `tailscale_1.104.5_arm64.deb` 已下载到 `.runtime/tailscale-install/`，37508766字节，已与官方SHA256核验一致：`7ce61e75d15c1269d0aebe3ed59524b83225121cba1b8ea682d04594195f6bc1`。包清单在同目录 `package-files.txt`，控制脚本在 `control/`，核验记录为 `verified.json`。
- `apt-get -s --no-upgrade --no-install-recommends install <本地deb>` 模拟通过：0升级、1新增（仅tailscale）、0删除。没有执行实际安装、自动清理、服务启用或登录。

## 仅首次：你在Jetson安装

安装包已经放在项目内并通过校验，无需再次下载。以下安装命令由用户本人执行，会新增Tailscale系统程序、服务、状态及APT/dpkg记录；模拟结果仅新增tailscale，不升级或删除已有软件。

```bash
cd /home/jetson/agri_robot
export TMPDIR="$PWD/.runtime/tmp"
sudo apt-get --no-upgrade --no-install-recommends install \
  "$PWD/.runtime/tailscale-install/tailscale_1.104.5_arm64.deb"
sudo systemctl disable --now tailscaled
```

最后一条关闭安装包可能自动启动的服务，并取消开机自启。安装成功后用下面的只读命令核对，应显示`disabled`（该状态的退出码通常为1，不代表安装失败）：

```bash
systemctl is-enabled tailscaled
```

`/usr/bin/tailscale`、`/usr/sbin/tailscaled`、`/lib/systemd/system/tailscaled.service`及辅助unit、`/etc/default/tailscaled`由包安装；运行时使用`/var/lib/tailscale/`和`/run/tailscale/`。我没有替用户执行这些系统变更，也没有配置sudo免密规则。

## 仅首次：运行项目，再登录Tailscale

```bash
bash /home/jetson/agri_robot/scripts/project.bash start
sudo tailscale up --accept-dns=false --accept-routes=false
sudo tailscale set --auto-update=false
```

第一条在项目就绪后启动已安装的`tailscaled`；尚未登录时会提示首次登录。第二条给出官方登录链接，由你在浏览器中使用自己的Tailscale账户完成登录；最后一条明确关闭该客户端自动更新。不要将登录链接、密码、认证Key或Token写入项目配置或Git。

`--accept-dns=false --accept-routes=false`保留Jetson现有DNS和设备路由，适合本次用Tailscale IPv4访问网页。登录完成后查询：

```bash
tailscale ip -4
tailscale status
```

操作电脑/手机通过 [Tailscale官方下载](https://tailscale.com/download) 安装客户端，登录同一tailnet并保持连接。在该设备浏览器访问 `http://<Jetson实际Tailscale IPv4>:5173/`。

## 以后每次开机：只运行原来的一键启动

```bash
bash /home/jetson/agri_robot/scripts/project.bash start
```

一键入口先使核心、后端和网页就绪，再检查是否安装Tailscale；服务停止时仅执行`systemctl start tailscaled`，不会执行`enable`、安装软件、登录账号或更改Tailscale偏好。已有登录状态保留时，启动服务后客户端会尝试重新连接；连接就绪则打印跨网络网页地址。若刚启动还在连接中，稍后查看：

```bash
bash /home/jetson/agri_robot/scripts/project.bash status
```

启动系统服务需要管理员权限，终端可能要求输入Jetson的sudo密码。交互式终端直接由sudo处理密码；无交互输入时使用`sudo -n`，无法授权会提示手动启动，不无限等待，也不回滚已经就绪的项目。没有Tailscale或VPN暂时连不上时，局域网项目服务继续运行。

如果此前手动执行过`tailscale down`，启动服务不擅自更改这一偏好；需要由你手动`sudo tailscale up --accept-dns=false --accept-routes=false`恢复连接。

`project.bash status`只查询，不启动VPN。`project.bash stop`继续只关闭项目核心/后端/网页，保留Tailscale连接，便于远程再启动项目。需要停掉VPN服务时由用户执行：

```bash
sudo systemctl stop tailscaled
```

这不清除登录状态；下次项目start会再次启动服务。以上流程不设置系统开机自启，不影响原有局域网网页入口，也不向公网发布网页。

## 验证确实跨网可用

1. 将操作电脑换到另一Wi-Fi或手机热点，或手机关闭Wi-Fi使用移动数据，同时保持Tailscale连接。
2. 使用Jetson的Tailscale IPv4加`:5173`打开网页，检查机器人状态能更新。仅打开页面不需要解除急停或开始任务。
3. 如打不开，先检查两端Tailscale在线状态、访问策略、防火墙和5173服务；不要因网页超时就放宽安全期限。
4. 相机保持原有按需低码率预览。跨网实际吞吐与往返延迟需在操作电脑端确认，Jetson本机请求不能代替跨网验证。

网页可访问与适合遥控是两个条件，当前遥控期限和停车保护保持不变，跨网实际操作效果需单独验证。

## 官方资料

软件验证：启动回归使用假服务进程、模拟Tailscale/systemctl与临时项目目录，覆盖项目成功后才启动VPN、未安装、sudo权限失败、已有连接、status/stop不启动VPN及不会enable/登录等行为。测试不启动真实Tailscale或硬件服务，不修改系统文件。日志为 `.diagnostics/tailscale-startup-tests.log`。实际在线与网页入口直连已于11月6日核对；跨网遥控与视频持续并行、启停动作及实车行驶尚未验证。

- [Linux安装、静态包与登录验证](https://tailscale.com/docs/install/linux)
- [访问tailnet内设备的IP和服务端口](https://tailscale.com/docs/how-to/connect-to-devices)
- [官方稳定包与校验和](https://pkgs.tailscale.com/stable/)
- [Vite允许主机列表](https://vite.dev/config/server-options)
