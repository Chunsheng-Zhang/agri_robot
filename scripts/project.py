"""Start/stop the three project services. Never unlocks or starts a motion task."""
import argparse
from datetime import datetime
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
from time import monotonic, sleep
from urllib.request import ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[1]
HARDWARE = {'Mcnamu_driver_X3', 'joy_node', 'ydlidar_ros2_driver_node',
            'astra_camera_node', 'web_video_server', 'localization', 'safety',
            'chassis_adapter', 'joy_adapter', 'waypoint', 'path_tracker', 'coverage_adapter',
            'coverage_server'}


def process_info(pid):
    try:
        folder = Path('/proc') / str(pid)
        fields = (folder / 'stat').read_text().split(') ', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return {'pid': int(pid), 'start_ticks': fields[19], 'parent': int(fields[1]),
                'args': [v.decode(errors='replace') for v in (folder / 'cmdline').read_bytes().split(b'\0') if v]}
    except (OSError, ValueError, IndexError):
        return None


def all_processes():
    return [info for p in Path('/proc').iterdir() if p.name.isdigit()
            if (info := process_info(p.name)) is not None]


class Project:
    def __init__(self, root=ROOT, config=None):
        self.root = Path(root)
        self.config = Path(config).resolve() if config else self.root / 'config/jetson_system.yaml'
        self.runtime = self.root / '.runtime/project'
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.state_file = self.runtime / 'state.json'
        self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.http = build_opener(ProxyHandler({}))

    def load(self):
        try:
            state = json.loads(self.state_file.read_text())
            if (not isinstance(state, dict) or not isinstance(state.get('services', {}), dict)
                    or not all(isinstance(v, dict) for v in state.get('services', {}).values())):
                return {}
            return state
        except (OSError, ValueError):
            return {}

    def save(self, state):
        temp = self.state_file.with_suffix('.tmp')
        temp.write_text(json.dumps(state, ensure_ascii=False, indent=2))
        temp.replace(self.state_file)

    def owns(self, entry):
        info = process_info(entry.get('pid', -1))
        return bool(info and entry.get('boot_id') == self.boot_id
                    and info['start_ticks'] == entry.get('start_ticks'))

    def members(self, entry):
        if entry.get('boot_id') != self.boot_id or not entry.get('token'):
            return []
        marker = ('AGRI_PROJECT_SERVICE_TOKEN=' + entry['token']).encode()
        result = []
        for info in all_processes():
            try:
                env = (Path('/proc') / str(info['pid']) / 'environ').read_bytes().split(b'\0')
                if marker in env:
                    result.append({**info, 'boot_id': self.boot_id})
            except OSError:
                pass
        if self.owns(entry) and not any(p['pid'] == entry['pid'] for p in result):
            result.append(entry)
        return result

    def active(self, entry):
        return self.owns(entry) or bool(self.members(entry))

    def api(self, path, stop=False):
        request = Request('http://127.0.0.1:8000/api/' + path, method='POST' if stop else 'GET')
        with self.http.open(request, timeout=3) as response:
            return json.load(response)

    def ready(self):
        try:
            status, devices = self.api('robot/status'), self.api('devices')
            with self.http.open('http://127.0.0.1:5173/', timeout=2) as response:
                web_ok = response.status == 200
            return web_ok and status.get('ros_connected') is True and devices['safety'].get('fresh') is True
        except (OSError, ValueError, KeyError):
            return False

    def existing(self):
        core, devices = False, set()
        for info in all_processes():
            args = info['args']
            if 'agri_bringup' in args and 'system.launch.py' in args:
                core = True
            devices.update(Path(a).name for a in args[:2] if Path(a).name in HARDWARE)
        ports = []
        for port in (8000, 5173, 8080):
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.2):
                    ports.append(port)
            except OSError:
                pass
        return core, sorted(devices), ports

    def commands(self):
        return {
            'core': (['bash', str(self.root / 'scripts/start_core.bash'),
                      'mode:=base', 'chassis:=yahboom_x3', f'params_file:={self.config}',
                      f'rosmaster_pythonpath:={self.root / "robot_sim/src"}',
                      'start_mgx23:=true', 'start_joy:=true', 'start_lidar:=true',
                      'require_lidar:=false', 'start_camera:=true', 'start_video:=true',
                      'start_planner:=true', f'event_directory:={self.root / ".runtime/safety"}'], self.root),
            'backend': ([str(self.root / 'agri_backend/.venv/bin/python'), '-B', '-m', 'uvicorn',
                         'agri_backend.main:app', '--host', '0.0.0.0', '--port', '8000'], self.root),
            'frontend': (['npm', 'run', 'dev', '--', '--host', '0.0.0.0', '--port', '5173', '--strictPort'],
                         self.root / 'agri_web'),
        }

    def preflight(self):
        for file in (self.config, self.root / 'scripts/start_core.bash',
                     self.root / 'agri_backend/.venv/bin/python', self.root / 'agri_web/node_modules/.bin/vite'):
            if not file.is_file():
                raise RuntimeError(f'缺少文件或依赖：{file}；未安装或更新任何软件。')
        if not (self.root / 'agri_robot_ws/install/setup.bash').is_file():
            raise RuntimeError('缺少已构建的 ROS 工作区，请先构建项目。')

    def show_urls(self):
        print('本机网页：http://127.0.0.1:5173/')
        try:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2)
            for address in dict.fromkeys(result.stdout.split()):
                if '.' in address and not address.startswith('127.'):
                    print(f'局域网网页：http://{address}:5173/')
        except (OSError, subprocess.TimeoutExpired):
            pass

    def show_tailscale(self):
        """Read-only status; authentication URLs and peer/user details are never printed."""
        binary = shutil.which('tailscale')
        if not binary:
            print('Tailscale：未安装；局域网访问可用，安装步骤见 docs/Tailscale跨网络访问.md。')
            return False
        try:
            result = subprocess.run([binary, 'status', '--json'], capture_output=True, text=True, timeout=3)
            if result.returncode:
                print('Tailscale：服务未运行或状态暂不可用。')
                return False
            data = json.loads(result.stdout)
            state = data.get('BackendState')
            if state in ('NeedsLogin', 'NeedsMachineAuth'):
                print('Tailscale：尚未登录或等待账户批准；首次登录请按 docs/Tailscale跨网络访问.md 操作。')
                return False
            if state != 'Running' or (data.get('Self') or {}).get('Online') is False:
                print(f'Tailscale：尚未连接（{state or "未知状态"}）；项目服务继续运行。')
                return False
            addresses = data.get('TailscaleIPs') or (data.get('Self') or {}).get('TailscaleIPs') or []
            printed = False
            for address in addresses:
                try:
                    parsed = ipaddress.ip_address(address)
                except ValueError:
                    continue
                if parsed.version == 4:
                    print(f'Tailscale 跨网络网页：http://{parsed}:5173/')
                    printed = True
            if not printed:
                print('Tailscale：连接中，尚未分配可用IPv4地址。')
            return printed
        except (OSError, subprocess.TimeoutExpired, ValueError, TypeError, AttributeError):
            print('Tailscale：状态暂不可用；项目服务继续运行。')
            return False

    def start_tailscale(self):
        """Start an already-installed daemon only; never install/enable/login/change preferences."""
        if not shutil.which('tailscale'):
            return self.show_tailscale()
        systemctl = shutil.which('systemctl')
        if not systemctl:
            print('Tailscale：未找到systemctl，请手动启动已安装的客户端；项目服务继续运行。')
            return False
        try:
            active = subprocess.run([systemctl, 'is-active', '--quiet', 'tailscaled'],
                                    capture_output=True, timeout=3)
            if active.returncode:
                command = [systemctl, 'start', 'tailscaled']
                if os.geteuid() != 0:
                    sudo = shutil.which('sudo')
                    if not sudo:
                        print('Tailscale：请手动执行 sudo systemctl start tailscaled；项目服务继续运行。')
                        return False
                    # Password entry belongs to the user's terminal; never read or store it here.
                    command = [sudo] + ([] if sys.stdin.isatty() else ['-n']) + command
                print('项目已就绪，正在启动Tailscale（仅本次启动，不设置开机自启；如提示请输入Jetson的sudo密码）。', flush=True)
                if subprocess.run(command, timeout=30).returncode:
                    print('Tailscale未启动，请手动执行 sudo systemctl start tailscaled；项目服务继续运行。')
                    return False
            return self.show_tailscale()
        except (OSError, subprocess.TimeoutExpired):
            print('Tailscale启动未确认，请查看 tailscale status；项目服务继续运行。')
            return False

    def start(self, timeout=60):
        previous = self.load()
        managed = [name for name, item in previous.get('services', {}).items() if self.active(item)]
        if managed:
            if len(managed) == 3 and self.ready():
                print('项目已经运行，未重复启动，也未改变控制权或急停状态。')
                self.show_urls()
                return
            raise RuntimeError('已有部分托管服务运行，请先执行 project.bash status，再用 stop 停止后重试。')
        core, hardware, ports = self.existing()
        if core or hardware or ports:
            if core and self.ready():
                print('项目已由现有进程运行，未重复启动；现有进程仍由原启动方式管理。')
                self.show_urls()
                return
            raise RuntimeError(f'检测到已有核心/设备或端口占用（核心={core}，设备={hardware}，端口={ports}）。'
                               '请在原终端正常停止旧服务；未启动第二套，也未终止其他进程。')
        self.preflight()
        folder = self.runtime / 'runs' / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + f'_{os.getpid()}')
        folder.mkdir(parents=True)
        env = dict(os.environ, AGRI_SYSTEM_CONFIG=str(self.config),
                   TMPDIR=str(self.root / '.runtime/tmp'), ROS_LOG_DIR=str(folder / 'ros'),
                   PYTHONDONTWRITEBYTECODE='1')
        Path(env['TMPDIR']).mkdir(parents=True, exist_ok=True)
        state = {'config': str(self.config), 'logs': str(folder), 'services': {}}
        self.save(state)
        try:
            for name, (command, cwd) in self.commands().items():
                token = f'{folder.name}:{name}'
                with (folder / f'{name}.log').open('ab') as log:
                    child = subprocess.Popen(command, cwd=cwd, env=dict(env, AGRI_PROJECT_SERVICE_TOKEN=token), stdin=subprocess.DEVNULL,
                                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                state['services'][name] = {'pid': child.pid, 'start_ticks': None,
                                           'boot_id': self.boot_id, 'token': token, 'command': command}
                info = process_info(child.pid)
                if info is not None:
                    state['services'][name]['start_ticks'] = info['start_ticks']
                self.save(state)
                if info is None:
                    raise RuntimeError(f'{name} 启动后立即退出')
                print(f'已启动 {name}，日志：{folder / (name + ".log")}', flush=True)
            deadline = monotonic() + timeout
            while monotonic() < deadline:
                if not all(self.owns(item) for item in state['services'].values()):
                    raise RuntimeError('有服务启动失败或退出')
                if self.ready():
                    safety = self.api('devices')['safety']
                    if not safety.get('emergency_stop') or safety.get('owner'):
                        raise RuntimeError('启动期间出现控制权或急停已解除，停止本次启动')
                    print('项目已就绪，车辆保持急停和待命；雷达障碍停车默认关闭。')
                    self.show_urls()
                    return
                sleep(.5)
            raise RuntimeError('等待核心、后端和网页就绪超时')
        except BaseException:
            self.shutdown(state)
            print(f'本次启动失败，已清理本次创建的进程。请查看日志：{folder}', file=sys.stderr)
            raise

    def terminate(self, entry):
        tracked = self.members(entry)
        if not tracked and not self.owns(entry):
            return
        pid = entry['pid']
        if self.owns(entry):
            try:
                if os.getpgid(pid) != pid:
                    raise RuntimeError(f'进程组不匹配，未向 PID {pid} 发送信号')
                os.killpg(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            for p in tracked:
                try:
                    os.kill(p['pid'], signal.SIGINT)
                except ProcessLookupError:
                    pass
        for sig, delay in ((signal.SIGTERM, 15), (signal.SIGKILL, 3), (None, 2)):
            deadline = monotonic() + delay
            while monotonic() < deadline and any(self.owns(p) for p in tracked):
                sleep(.1)
            alive = self.members(entry)
            if not alive:
                return
            if sig is not None:
                for p in alive:
                    try:
                        os.kill(p['pid'], sig)
                    except ProcessLookupError:
                        pass
        raise RuntimeError('仍有托管子进程未退出，请检查日志；未启动替代进程。')

    def shutdown(self, state):
        services = state.get('services', {})
        if self.owns(services.get('backend', {})):
            try:
                self.api('robot/emergency-stop', stop=True)
            except (OSError, ValueError):
                print('未能从后端确认急停，将停止本脚本启动的服务，请核对车辆停车状态。', file=sys.stderr)
        for name in ('core', 'backend', 'frontend'):
            self.terminate(services.get(name, {}))
        state['services'] = {}
        self.save(state)

    def stop(self):
        state = self.load()
        if not any(self.active(item) for item in state.get('services', {}).values()):
            if any(self.existing()):
                raise RuntimeError('当前服务不是本脚本启动的，请在原终端正常停止；未终止其他进程。')
            print('本脚本管理的服务均已停止。')
            return
        self.shutdown(state)
        print('已停止本脚本管理的核心、后端和网页。')

    def status(self):
        state = self.load()
        external_core, _, ports = self.existing()
        service_ports = {'backend': 8000, 'frontend': 5173}
        for name in ('core', 'backend', 'frontend'):
            if self.active(state.get('services', {}).get(name, {})):
                description = '运行中（本脚本管理）'
            elif name == 'core' and external_core:
                description = '运行中（其他方式启动）'
            elif name in service_ports and service_ports[name] in ports:
                description = f'端口 {service_ports[name]} 已占用（非本脚本管理）'
            else:
                description = '未运行'
            print(f'{name}: {description}')
        ready = self.ready()
        print('服务接口：' + ('已就绪' if ready else '尚未全部就绪'))
        if state.get('logs'):
            print('最近启动日志：' + state['logs'])
        if ready:
            safety = self.api('devices')['safety']
            print(f'急停={safety.get("emergency_stop")}，控制权={safety.get("owner") or "无"}，'
                  f'雷达障碍停车={safety.get("lidar_required")}')
            self.show_urls()
        self.show_tailscale()
        return ready


def main():
    parser = argparse.ArgumentParser(description='统一启动项目；不自动开始任务或解除急停。')
    parser.add_argument('action', nargs='?', choices=('start', 'stop', 'status'), default='start')
    parser.add_argument('--config', help='核心和后端共同使用的 ROS YAML，默认 config/jetson_system.yaml')
    args = parser.parse_args()
    project = Project(config=args.config)
    try:
        if args.action == 'status':
            return 0 if project.status() else 1
        with (project.runtime / 'control.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('另一个启动或停止操作正在执行，请稍后查看 status。')
            getattr(project, args.action)()
        if args.action == 'start':
            project.start_tailscale()
        return 0
    except (RuntimeError, OSError, ValueError) as exc:
        print(f'操作未完成：{exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('操作已中断。', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
