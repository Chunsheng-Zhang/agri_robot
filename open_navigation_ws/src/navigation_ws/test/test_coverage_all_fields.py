#!/usr/bin/env python3
# Copyright 2026 Chunsheng Zhang
#
# 覆盖率路径规划批量测试：跑遍 fields2Benchmark_Dataset 里的所有地块，
# 评估「目前这套规划方式」在通用地块上的可行性。
#
# ============================ 链路一致性 ============================
# 本脚本刻意复刻 coverage_demo_launch.py 的规划链路，不引入任何新算法：
#
#   coverage_demo_launch.py                   本脚本
#   ----------------------                    ------
#   wkt_to_utm.py  (EPSG:4326 -> EPSG:32635,  直接 import 同一个
#                   首点平移到原点)             gps_utm/src/wkt_to_utm.py
#   -> results/<name>_utm_boundary.json       -> 内存里的 (x, y) 列表
#      (demo_coverage_via_headland.load_boundary 读它)
#   -> demo_coverage_via_headland.compute_coverage():
#        goal.frame_id            = 'map'            同上
#        goal.polygons            = Coordinates[]    同上（axis1/axis2）
#        goal.generate_headland   = True             同上
#        goal.generate_route      = True             同上
#        goal.generate_path       = True             同上
#        goal.route_mode.mode     = launch 参数      同上（--route-mode）
#        goal.swath_mode          不设置             **不设置（关键）**
#   -> coverage_server 节点参数来自
#      demo_params.yaml 的 coverage_server 段       直接读同一个 yaml
#
# 两条被刻意保留下来的「链路特征」，是本次评估的重点：
#   1. coverage_demo_via_headland 从不设置 goal.swath_mode，所以服务端一律
#      回落到 demo_params.yaml 的 default_swath_angle_type=SET_ANGLE、
#      default_swath_angle=0.0。也就是说**所有地块都用 0°（正东）作业方向**，
#      与地块实际朝向无关。本脚本默认完全照搬；--compare-swath-angle 会额外
#      用 BRUTE_FORCE(LENGTH) 跑一遍作对照，量化这个常数带来的代价。
#   2. 地块边界经过 wkt_to_utm 平移后，第一个顶点恒为 (0,0)；gazebo 里
#      turtlebot 也正是从 (0,0) 出生。所以「车」始终站在地块第一个顶点上，
#      transit 长度就是沿边界从首点到规划起点的最短路程——本脚本照此复算。
#
# 与 launch 的唯一差异：standalone 起 coverage_server 时 use_sim_time 必须设
# False（没有 gazebo 就没有 /clock），否则 action server 的 result_timeout
# 永远不推进。该参数不影响任何几何/规划结果。
#
# ============================ 单个地块失败不影响后续 ============================
# coverage_server 是单线程执行器，整个 computeCoveragePath() 跑在 action 回调
# 里，中途不让出执行权。所以「客户端超时」只是客户端走了，服务端还在算：
#   * 还在算         -> 后面每个 goal 都石沉大海，表现为「goal 未被接受」；
#   * 被 OOM kill 掉 -> 连服务端都没有了，同样一路失败。
# 两种都不会自愈。本脚本在每个地块外面兜住异常，并对「无应答」这一类走
# recover_server()：
#   1. 先看进程还在不在（ServerHandle.died）。不在 -> 直接重启；
#   2. 还在 -> cancel（其实打不进去）+ 发一个 2 m 的小三角当探针，等它空下来；
#   3. 等不回来 -> restart() 整个进程，重新 configure+activate 再继续。
# 实测 ee_field_135（TSP）会让服务端吃到 7.9 GB 被内核 OOM kill，重启后
# 后面的地块照常跑完。
#
# ======================= 本次评估的可配置口径（默认不影响老行为） =======================
#   1. 分国分层抽样：--sample-per-country N
#      数据集是 ee(97) / lt(113) / nl(140)，全量跑一次要一个多小时。按国家分别
#      按面积分层抽 N 个（面积升序后等间隔取），大中小地块都进样本，确定性、
#      不用随机数。带洞地块另设保底名额（--min-holes-per-country，默认 3）：
#      洞在各国都是少数（ee 26、lt 16、nl 7 个），纯按面积抽很可能一个都抽不到，
#      而「带洞地块」恰恰是要评估的场景。抽样规则和最终清单写进 output/fields.txt
#      和 report.md 附录，跑过的样本可复现。
#   2. 多路由模式对比：--route-mode A,B（逗号分隔）
#      一个地块依次用每个模式各规划一次；CSV 里每个 (地块, 模式) 一行，报告里多
#      一节横向对比。不写 = 用服务端 default_route_type（BOUSTROPHEDON）。
#   3. 覆盖率指标：作业段按 operation_width/2 左右各铺半个幅宽、求并，再和地块
#      求交（--no-coverage 可关，省时间）：
#        cov_ratio_swath  作业段条带盖住的比例（作业质量）
#        cov_ratio_all    再加上转弯段条带（顺带压过去的也算）
#        missed_*         漏掉的斑块：面积、个数、最大一块
#   4. 洞（默认开启）：地块所有环一起下发（polygons[0]=外边界，polygons[1:]=每个
#      洞一个），指标里的面积和越界判定也按「扣掉洞」的真实地块算。老版本只下发
#      外边界，洞是在客户端就丢掉的，服务端根本没机会避开。见 report.md §洞的处理。
#
# ================================ 用法 ================================
#   # 先 source 工作空间
#   source /opt/ros/humble/setup.bash && source ~/open_navigation_ws/install/setup.bash
#
#   # 本次评估口径：每国抽 20 个（共 60 个），对比 BOUSTROPHEDON 与 TSP
#   python3 test_coverage_all_fields.py --sample-per-country 20 \
#       --route-mode BOUSTROPHEDON,TSP --tsp-time-limit 3
#
#   # 全量跑（脚本自己拉起 coverage_server，跑完自己关掉）
#   python3 test_coverage_all_fields.py
#
#   # 先小批量验证
#   python3 test_coverage_all_fields.py --limit 20
#
#   # 只想看某几个地块
#   python3 test_coverage_all_fields.py --fields ee_field_6,ee_field_103
#
#   # 量化「固定 0° 作业方向」的代价（每个地块多跑一次 BRUTE_FORCE）
#   python3 test_coverage_all_fields.py --compare-swath-angle --limit 50
#
#   # 用已经跑着的 coverage_server（比如 coverage_demo_launch.py 起的那个）
#   python3 test_coverage_all_fields.py --server external --limit 5
#
# 输出目录默认为 src/navigation_ws/test/results/<时间戳>_batch/：
#   per_field.csv    每个地块一行，全部指标
#   report.md        汇总报告 + 可行性结论
#   report.png       四联图
#   examples/*.png   代表性地块的路径可视化
#   server_params.yaml / server.log   实际下发的服务端参数与日志

import argparse
import csv
import datetime
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# 路径常量（全部相对本文件定位，换机器不用改）
# ---------------------------------------------------------------------------
TEST_DIR = os.path.dirname(os.path.abspath(__file__))
NAV_WS = os.path.dirname(TEST_DIR)                      # src/navigation_ws
SRC_ROOT = os.path.dirname(NAV_WS)                      # src
REPO_ROOT = os.path.dirname(SRC_ROOT)                   # open_navigation_ws

DEFAULT_DATASET = os.path.join(SRC_ROOT, 'fields2Benchmark_Dataset')
DEFAULT_DEMO_PARAMS = os.path.join(
    SRC_ROOT, 'opennav_coverage/opennav_coverage_demo/params/demo_params.yaml')
GPS_UTM_SRC = os.path.join(NAV_WS, 'gps_utm', 'src')

# 和 test_coverage_planning.py / wkt_to_utm.py 一致的坐标转换
sys.path.append(GPS_UTM_SRC)
from wkt_to_utm import wkt_to_utm_rings  # noqa: E402

COVERAGE_SERVER_NODE = 'coverage_server'
COVERAGE_ACTION = 'compute_coverage_path'

# 判定阈值（只影响「是否算异常」的计数，不影响原始指标）
OOB_TOLERANCE_M = 0.01      # 越界超过 1 cm 才计入 oob_pts
GAP_THRESHOLD_M = 0.5       # 相邻路径点间距超过 0.5 m 记为断点
RADIUS_TOL_RATIO = 0.95     # 实测转弯半径 < min_turning_radius*0.95 记为违规

# 覆盖率指标
COVER_PATCH_MIN_M2 = 0.01   # 小于这个面积的不算漏喷斑块（浮点噪声、发丝缝）
COVER_SIMPLIFY_TOL_M = 0.02  # 转弯段 buffer 前的简化容差，见 coverage_stats()

# 分国抽样：带洞地块的保底名额（每国）
DEFAULT_MIN_HOLES_PER_COUNTRY = 3

# ---------------------------------------------------------------------------
# 配色：dataviz 参考调色板（light 表面），图里最多 3 条同屏序列
# ---------------------------------------------------------------------------
C_SURFACE = '#fcfcfb'
C_INK = '#0b0b0b'
C_INK_2 = '#52514e'
C_MUTED = '#898781'
C_GRID = '#e1e0d9'
C_AXIS = '#c3c2b7'
C_S1 = '#2a78d6'            # 序列 1 blue
C_S2 = '#eb6834'            # 序列 2 orange
C_S3 = '#1baf7a'            # 序列 3 aqua
C_GOOD = '#0ca30c'
C_WARNING = '#fab219'
C_CRITICAL = '#d03b3b'


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------------------
# 服务端参数：唯一真源是 demo_params.yaml
# ---------------------------------------------------------------------------
def load_server_params(demo_params_path):
    """从 demo_params.yaml 取出 coverage_server 段的参数。

    直接把 launch 用的那份 yaml 读进来，保证测试的服务端参数不可能和
    coverage_demo_launch.py 漂移。
    """
    with open(demo_params_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if COVERAGE_SERVER_NODE not in data:
        raise KeyError(
            f'{demo_params_path} 里没有 {COVERAGE_SERVER_NODE} 段')
    section = data[COVERAGE_SERVER_NODE]
    params = section.get('ros__parameters', section)
    return dict(params)


def write_server_params(params, out_path, use_sim_time):
    """写出 standalone server 用的参数文件。"""
    payload = {COVERAGE_SERVER_NODE: {'ros__parameters': dict(params)}}
    payload[COVERAGE_SERVER_NODE]['ros__parameters']['use_sim_time'] = use_sim_time
    with open(out_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
    return out_path


# ---------------------------------------------------------------------------
# standalone coverage_server 的拉起 / 生命周期 / 关闭
# ---------------------------------------------------------------------------
class ServerHandle:
    """脚本自己拉起的 coverage_server（LifecycleNode）。

    configure/activate 用 ros2 lifecycle CLI 做，和 bringup_launch.py 里
    lifecycle_manager 的效果等价：configure 建 action server，activate 开始
    接受 goal（demo_coverage_via_headland.send_until_accepted 的重试也是为
    了兼容「已 configure 未 activate」这个窗口）。
    """

    def __init__(self, params_path, log_path):
        self.params_path = params_path
        self.log_path = log_path
        self.proc = None
        self.launched_once = False

    def start(self, timeout=60.0):
        # 已经有一个同名节点的话必须说清楚：下面的 lifecycle 调用会打到旧节点
        # 上，然后以「Unknown transition requested」这种看不懂的方式失败。
        if self._node_visible():
            raise RuntimeError(
                f'已经有一个 /{COVERAGE_SERVER_NODE} 在跑。要么先 '
                '`pkill -f opennav_coverage`，要么加 --server external 用它。')

        # 重启时接着写，否则卡住那一次的服务端日志会被下面这次覆盖掉
        self.log_file = open(self.log_path,
                             'a' if self.launched_once else 'w', encoding='utf-8')
        self.launched_once = True
        self.proc = subprocess.Popen(
            ['ros2', 'run', 'opennav_coverage', 'opennav_coverage',
             '--ros-args', '--params-file', self.params_path],
            stdout=self.log_file, stderr=subprocess.STDOUT, start_new_session=True)

        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(
                    f'coverage_server 启动即退出（code={self.proc.returncode}），'
                    f'见 {self.log_path}')
            if self._node_visible():
                break
            time.sleep(1.0)
        else:
            raise RuntimeError(f'等不到 /{COVERAGE_SERVER_NODE} 节点')

        self._lifecycle('configure', timeout=120.0)
        self._lifecycle('activate', timeout=120.0)

    def died(self):
        """进程还在不在。

        不在了 ≠ 「算得慢」：F2C 的 Graph::shortestPathsAndCosts 会把所有
        N² 条最短路径**整个物化**出来（Floyd-Warshall 之后逐对重建），
        N≈2×swath 数+边界点数，路径平均长度又是 O(N)，于是内存是 O(N³)。
        ee_field_135（826 条 swath，N≈1757）实测吃到 7.9 GB 被内核 OOM
        kill 掉。这种情况等多久都没用，只能重启。
        """
        return self.proc is not None and self.proc.poll() is not None

    def death_reason(self):
        """把退出码翻译成人话——OOM 和「被别人 SIGTERM 掉」的处置完全一样
        （重启），但原因不同，看日志时不该混在一起猜。

        coverage_server 是 rclcpp 节点，SIGTERM/SIGINT 会被 signal_handler 接住
        并正常退出，所以**退出码 0 也不是正常结束**，是外面kill了它（最常见的是
        别处跑了 `pkill -f opennav_coverage`，或者同时起了第二份同一份 launch）。
        被内核 OOM kill 是 -9，超时被我们自己的 stop() 收掉是 -15。
        """
        rc = self.proc.returncode
        if rc == -9:
            return f'exit={rc}，被 OOM kill 掉（TSP 的 O(N³) 内存）'
        if rc == 0:
            return (f'exit={rc}，收到 SIGTERM/SIGINT 正常退出——不是 OOM，'
                    '多半是别处执行了 pkill -f opennav_coverage 或起了第二份服务端')
        return f'exit={rc}'

    def _node_visible(self):
        try:
            out = subprocess.run(['ros2', 'node', 'list'],
                                 capture_output=True, text=True, timeout=15.0)
        except subprocess.TimeoutExpired:
            return False
        return f'/{COVERAGE_SERVER_NODE}' in out.stdout

    def _lifecycle(self, transition, timeout):
        try:
            out = subprocess.run(
                ['ros2', 'lifecycle', 'set', f'/{COVERAGE_SERVER_NODE}', transition],
                capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired as e:
            raise RuntimeError(f'lifecycle {transition} 超时') from e
        text = (out.stdout or '') + (out.stderr or '')
        if 'Transitioning successful' not in text:
            raise RuntimeError(
                f'lifecycle {transition} 失败: {text.strip()}\n'
                f'见 {self.log_path}')

    def restart(self):
        """算到一半卡住的 goal 只能靠重启进程甩掉。

        coverage_server 是单线程执行器，整个 computeCoveragePath() 跑在 action
        回调里，中途不让出执行权；coverage_server.cpp:168 的
        is_cancel_requested() 只在函数开头查一次，计算开始后就不再理会 cancel。
        所以「客户端超时」之后服务端还在算，只能 kill 掉重来。
        """
        self.stop()
        # 进程组收掉之后 DDS 那边还要一点时间才认节点真的没了
        deadline = time.time() + 10.0
        while time.time() < deadline and self._node_visible():
            time.sleep(0.5)
        self.start()

    def stop(self):
        """不管跑没跑完都要关掉——遗留同名节点会撞坏用户自己的 launch。

        `ros2 run` 只是个 python wrapper，真正的节点是它的子进程；直接
        terminate() 会留下孤儿节点（后面再起就撞名字，lifecycle 也会打到旧
        的那个上）。所以 start() 用 start_new_session 把整棵树放进独立进程组，
        这里对进程组发信号。
        """
        if self.proc is None:
            return
        try:
            pgid = os.getpgid(self.proc.pid)
        except ProcessLookupError:
            pgid = None
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGTERM)
            else:
                self.proc.terminate()
            try:
                self.proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    self.proc.kill()
                self.proc.wait(timeout=5.0)
        except Exception:  # noqa: BLE001
            pass
        finally:
            self.proc = None
            if getattr(self, 'log_file', None):
                self.log_file.close()
                self.log_file = None


# ---------------------------------------------------------------------------
# 几何指标（全部 numpy / shapely 2.x 向量化，几十万点也是毫秒级）
# ---------------------------------------------------------------------------
def path_xy(nav_path):
    """nav_msgs/Path -> (N, 2) float64。

    覆盖率路径动辄几十万点（0.1 m 采样），这里用一次性列表推导 + np.array，
    比逐点索引赋值快一倍以上；更大的开销在下面的 turn_stats/turns_length，
    那两处已经改成只做一次向量化调用。
    """
    poses = nav_path.poses
    if not poses:
        return np.zeros((0, 2))
    return np.array([(p.pose.position.x, p.pose.position.y) for p in poses],
                    dtype=np.float64)


def polyline_length(xy):
    if len(xy) < 2:
        return 0.0
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    return float(seg.sum())


def gap_stats(xy, threshold=GAP_THRESHOLD_M):
    """相邻点间距。断点（跳变）说明路径不连续。"""
    if len(xy) < 2:
        return 0.0, 0
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    return float(seg.max()), int((seg > threshold).sum())


def min_turn_radius(xy, window=3):
    """三点外接圆半径的最小值。

    取相隔 window 个采样点的三点算外接圆：R = a·b·c / (2·|cross|)。
    共线时 |cross| -> 0，R -> inf（直线段不算转弯）。
    只反映采样后折线的曲率，是下界的近似估计。
    """
    if len(xy) < 2 * window + 1:
        return float('inf')
    a_pt = xy[:-2 * window]
    b_pt = xy[window:-window]
    c_pt = xy[2 * window:]
    ab = b_pt - a_pt
    ac = c_pt - a_pt
    cross = np.abs(ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0])
    la = np.hypot(*(b_pt - c_pt).T)
    lb = np.hypot(*(a_pt - c_pt).T)
    lc = np.hypot(*(a_pt - b_pt).T)
    with np.errstate(divide='ignore', invalid='ignore'):
        radius = np.where(cross > 1e-12, la * lb * lc / (2.0 * cross), np.inf)
    radius = radius[np.isfinite(radius)]
    if radius.size == 0:
        return float('inf')
    return float(radius.min())


def oob_stats(xy, poly, tolerance=OOB_TOLERANCE_M):
    """越界点统计：落在多边形外的路径点数量、占比、最大越界距离。

    covers = contains or touches，与 test_coverage_planning.py 的判定一致。
    """
    if len(xy) == 0:
        return 0, 0.0, 0.0
    import shapely
    pts = shapely.points(xy)          # (N,2) -> GeometryArray
    inside = shapely.covers(poly, pts)
    if inside.all():
        return 0, 0.0, 0.0
    dist = shapely.distance(poly, pts)
    outside = dist > tolerance
    count = int(outside.sum())
    if count == 0:
        return 0, 0.0, 0.0
    return count, count / len(xy), float(dist[outside].max())


def split_nav_path(xy, turn_arrays):
    """按转弯段把 nav_path 拆成「作业段点串」和「转弯段点串」。

    为什么不用 coverage_path.swaths：服务端 utils.hpp 的
    toCoveragePathMsg(Path) 把作业段终点写成 `path[i-1].point`，可 F2C 的
    Path 里一个作业段只占**一个** PathState——point 是起点，长度在 len 里，
    终点要用 PathState::atEnd() 现算（fields2cover/types/PathState.cpp）。
    于是 swaths[i].start 恒等于 swaths[i].end：350 场地块跑下来 swath_len_m
    全是 0.000，示例 PNG 里作业段一条都画不出来，按作业段铺条带的覆盖率
    也只能算出 0。

    这里改从 nav_path 拆。toNavPathMsg() 只对作业段做 discretizeSwath()、
    转弯段的点原样搬过去，所以转弯段的坐标和 coverage_path.turns 里的
    **逐一相同**（两边都是同一个 PathState 的 float64 position），按这个把
    路径切成连续段即可。作业段是直线（地头已被 generateHeadlands 从作业区
    里挖掉），所以只取每段首尾两点就够。

    返回 (作业段点串列表, 转弯段点串列表)。
    """
    n = len(xy)
    if n == 0:
        return [], []
    turn_pts = (np.concatenate([a for a in turn_arrays if len(a)])
                if any(len(a) for a in turn_arrays) else np.zeros((0, 2)))
    if len(turn_pts):
        # 用 np.unique(axis=0) 把「同一坐标」编成同一个整数，避免 O(N*M) 广播比较
        all_pts = np.vstack((np.round(xy, 6), np.round(turn_pts, 6)))
        _uniq, inv = np.unique(all_pts, axis=0, return_inverse=True)
        is_turn = np.isin(inv[:n], np.unique(inv[n:]))
    else:
        is_turn = np.zeros(n, dtype=bool)

    bounds = np.concatenate(([0], np.flatnonzero(is_turn[1:] != is_turn[:-1]) + 1, [n]))
    swath_chunks = []
    turn_chunks = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        if is_turn[a]:
            turn_chunks.append(xy[a:b])
            continue
        # 转弯段的第一个点 = 作业段终点、最后一个点 = 下一段作业段起点，这几个
        # 坐标两边都有，判到转弯那一侧了。作业段往两边各多吃一个点，不然每段
        # 首尾各短 0.1 m（条带跟着短，行末会凭空多出一小块漏喷）。
        lo = a - 1 if a > 0 else a
        hi = b + 1 if b < n else b
        swath_chunks.append(xy[lo:hi])
    return swath_chunks, turn_chunks


def chunks_length(chunks):
    """若干点串的总长。"""
    return float(sum(polyline_length(c) for c in chunks if len(c) > 1))


def flatten_turns(coverage_path):
    """把所有转弯段的点拼成一个大数组 + 每段的长度/首尾距 + 每段自己的点数组。

    几百个转弯段、几十万个点，逐段建 numpy 数组再逐段调 shapely 会慢一个
    数量级；这里只拼一次、只调一次 vectorized API。
    返回 (xy, segment_lengths, spans, arrays)。arrays 是每个转弯段单独的
    (n, 2) 数组，给 coverage_stats 用——它要按「段」建折线，不能把相邻两段
    之间的空档也连成一条直线。
    """
    xs = []
    ys = []
    spans = []
    seg_len = []
    arrays = []
    for turn in coverage_path.turns:
        poses = turn.poses
        n = len(poses)
        if n == 0:
            seg_len.append(0.0)
            continue
        if n > 1:
            a = poses[0].pose.position
            b = poses[-1].pose.position
            spans.append(math.hypot(b.x - a.x, b.y - a.y))
        else:
            seg_len.append(0.0)
        px = [p.pose.position.x for p in poses]
        py = [p.pose.position.y for p in poses]
        xs.extend(px)
        ys.extend(py)
        arrays.append(np.column_stack((np.array(px), np.array(py))))
        if n > 1:
            dx = np.diff(px)
            dy = np.diff(py)
            seg_len.append(float(np.hypot(dx, dy).sum()))
        else:
            seg_len.append(0.0)
    if not xs:
        return np.zeros((0, 2)), seg_len, spans, arrays
    return np.column_stack((np.array(xs), np.array(ys))), seg_len, spans, arrays


def turn_stats(turn_xy, spans, poly, tolerance=OOB_TOLERANCE_M):
    """转弯段的规模与越界情况。

    这是问题的高发区：作业段一定被裁在地块内，但转弯段是 path generator
    直接在两个作业段端点之间连的 Dubins 弧——如果路由把两个相距几十米的
    作业段当成「相邻」，这条弧就会横穿地块外面的凹陷区域。
    """
    import shapely
    n_pts = len(turn_xy)
    n_oob = 0
    max_dist = 0.0
    if n_pts:
        dist = shapely.distance(poly, shapely.points(turn_xy))
        outside = dist > tolerance
        n_oob = int(outside.sum())
        if n_oob:
            max_dist = float(dist[outside].max())
    return {
        'turn_n_pts': n_pts,
        'turn_oob_pts': n_oob,
        'turn_oob_max_dist_m': max_dist,
        'turn_span_median_m': float(np.median(spans)) if spans else 0.0,
        'turn_span_max_m': float(np.max(spans)) if spans else 0.0,
    }


def swath_endpoint_oob(swath_chunks, poly, tolerance=OOB_TOLERANCE_M):
    """作业段端点越界计数（每段只取首尾两点，0.1 m 的密采样点不算）。

    作业段是车在作业的地方，理论上一定被裁在地块里；这里越界说明裁剪出了
    问题。转弯段另算（见 turn_stats）——那才是越界的高发区。
    """
    import shapely
    pts = np.array([p for c in swath_chunks if len(c) > 1 for p in (c[0], c[-1])])
    if not len(pts):
        return 0, 0
    dist = shapely.distance(poly, shapely.points(pts))
    return int((dist > tolerance).sum()), len(pts)


def coverage_stats(swath_chunks, turn_arrays, poly, width,
                   simplify_tol=COVER_SIMPLIFY_TOL_M):
    """覆盖率：把路径按幅宽铺成条带，看盖住地块多少（洞不算面积）。

    做法：
      * 每段点串先 simplify 再 buffer(operation_width/2) 成条带，
        cap_style='flat' —— 条带止于作业段两端，不向外多铺半个幅宽（机具到
        作业段端点就停了）。
      * 转弯段是 0.1 m 密采样的 Dubins 点串，直接 buffer 会炸出上百万个顶点
        （实测最坏地块单个地块 140 万点）。Douglas-Peucker 保证简化后折线
        偏离原线 <= simplify_tol，0.7 m 幅宽下 0.02 m 的偏差可以忽略，而平滑
        弧上的顶点数能少一个数量级。
      * 条带与地块求交得到「盖住的面积」，比值就是覆盖率；difference 得到漏喷
        斑块（只统计 > COVER_PATCH_MIN_M2 的，滤掉浮点噪声级别的发丝缝）。

    cov_ratio_swath 只算作业段——机具**喷到**的地；cov_ratio_all 把转弯段的
    条带也算进去，那是车**压过**的地，两者之差就是转弯顺带覆盖的贡献。

    注意分母是整个地块（扣掉洞），而 nav_path 里**没有地头那一圈**：服务端
    generate_headlands=true 时先 generateHeadlands() 把地头从作业区挖掉，作业段
    只铺内场，地头要靠 demo 的 transit 沿外边界走。所以 cov_ratio_* 天然到不了
    1，缺口主要就是地头圈的面积。
    """
    import shapely
    half = width / 2.0
    area = poly.area
    out = {
        'cov_ratio_swath': float('nan'),
        'cov_ratio_all': float('nan'),
        'missed_ha': float('nan'),
        'missed_patches': 0,
        'missed_max_patch_m2': float('nan'),
    }
    if area <= 0 or not swath_chunks:
        return out

    def strip(arr, join_style):
        line = shapely.simplify(shapely.linestrings(arr), simplify_tol,
                                preserve_topology=False)
        return shapely.buffer(line, half, quad_segs=4,
                              cap_style='flat', join_style=join_style)

    sw_bufs = [strip(arr, 'mitre') for arr in swath_chunks if len(arr) > 1]
    if not sw_bufs:
        return out
    strip_sw = shapely.union_all(sw_bufs)
    out['cov_ratio_swath'] = float(shapely.intersection(strip_sw, poly).area / area)

    missed = shapely.difference(poly, strip_sw)
    if missed.is_empty:
        parts = np.empty(0, dtype=object)
    else:
        parts = shapely.get_parts(missed)
    areas = shapely.area(parts) if len(parts) else np.empty(0)
    big = areas[areas > COVER_PATCH_MIN_M2]
    out['missed_ha'] = float(missed.area / 1e4)
    out['missed_patches'] = int(len(big))
    out['missed_max_patch_m2'] = float(big.max()) if len(big) else 0.0

    strip_all = shapely.union_all(
        sw_bufs + [strip(arr, 'round') for arr in turn_arrays if len(arr) > 1])
    out['cov_ratio_all'] = float(shapely.intersection(strip_all, poly).area / area)
    return out


def transit_distance(boundary, goal_xy, robot_xy=(0.0, 0.0)):
    """复算 demo_coverage_via_headland.build_transit 的沿边界最短距离。

    车在 (0,0)（= 平移后地块首顶点），规划起点是覆盖率路径第一个点，
    沿地块边界两个方向取短的那条。
    """
    from shapely.geometry import LineString, Point
    ring = LineString(boundary)
    total = ring.length
    if total <= 0.0:
        return 0.0
    s_robot = ring.project(Point(*robot_xy))
    s_goal = ring.project(Point(*goal_xy))
    fwd = (s_goal - s_robot) % total
    return float(min(fwd, total - fwd))


def polygon_area_ha(boundary):
    from shapely.geometry import Polygon
    return Polygon(boundary).area / 1e4


def as_rings(field):
    """把「点列表」或「环列表」统一成环列表。

    build_goal 两种都要吃：地块是 rings（外边界 + 洞），而 BatchTester.recover
    的 2 m 小三角探针是单个点列表。
    """
    if not field:
        return []
    first = field[0]
    if first and isinstance(first[0], (int, float)):
        return [field]
    return list(field)


def field_polygon(rings):
    """环列表 -> shapely 多边形（外边界 + 洞），返回 (poly, 是否本来就是有效多边形)。

    area / covers / distance / buffer 全部用它算，所以**洞是扣掉的**：下发不带洞
    时地块面积会虚高，越界判定也会把洞里的点当成合法点。
    数据集里有 2 个自交地块（nl_field_22 / nl_field_32），无效多边形上的
    covers/distance 结果不可靠，用 buffer(0) 修一下（标准手法）。修不出来
    （空几何）就退回原样，让指标带着这个问题跑，同时在 CSV 里标 poly_valid=0。
    """
    from shapely.geometry import Polygon
    poly = Polygon(rings[0], rings[1:])
    valid = bool(poly.is_valid)
    if not valid:
        fixed = poly.buffer(0)
        if not fixed.is_empty and fixed.area > 0:
            poly = fixed
    return poly, valid


def _stratified_pick(items, k):
    """按面积升序的 items 里等间隔取 k 个（确定性，不用随机数）。"""
    if k <= 0 or not items:
        return []
    if k >= len(items):
        return list(items)
    idx = np.unique(np.linspace(0, len(items) - 1, k).round().astype(int))
    return [items[i] for i in idx]


def field_country(name):
    """ee_field_6 -> ee。"""
    return name.split('_')[0]


def wkt_stratify_info(wkt_path):
    """抽样分层用的小信息：(近似面积 m², 洞数)。

    这里**不做投影**——抽样只是要按大小排序，没必要把 350 个地块全投一遍。
    用 WGS84 的「度²」面积乘度->米的换算：经度方向 111320·cos(lat) m/度、
    纬度方向 111320 m/度。同一国家内纬度差得不多，排序完全够用；
    报告里的 area_ha 是投影后 UTM 坐标算的真实面积。
    """
    from shapely import wkt as shapely_wkt
    with open(wkt_path, 'r', encoding='utf-8') as f:
        geom = shapely_wkt.loads(f.read().strip())
    lat = math.radians(geom.centroid.y)
    return geom.area * (111320.0 ** 2) * math.cos(lat), len(geom.interiors)


def sample_fields(wkt_paths, per_country, countries=None,
                  min_holes_per_country=DEFAULT_MIN_HOLES_PER_COUNTRY):
    """分国分层抽样，返回 (选中的 wkt 路径, report 用的说明行, 每国清单)。

    每国：先按面积分层抽 per_country 个；带洞地块另按面积分层保底
    min_holes_per_country 个，占掉的名额从总名额里扣（不够就按实际数量抽）。
    """
    groups = {}
    for p in wkt_paths:
        groups.setdefault(field_country(os.path.basename(p)), []).append(p)
    order = countries or sorted(groups)

    selected = []
    desc = []
    listing = {}
    for c in order:
        paths = groups.get(c)
        if not paths:
            desc.append(f'- `{c}`：数据集里没有地块，跳过')
            continue
        items = []
        for p in paths:
            area_m2, n_holes = wkt_stratify_info(p)
            items.append((area_m2, n_holes, p))
        items.sort(key=lambda it: it[0])

        holes_pool = [it for it in items if it[1] > 0]
        picked_holes = _stratified_pick(
            holes_pool, min(min_holes_per_country, len(holes_pool)))
        taken = {it[2] for it in picked_holes}
        rest = [it for it in items if it[2] not in taken]
        picked_rest = _stratified_pick(rest, per_country - len(picked_holes))

        rows = picked_holes + picked_rest
        rows.sort(key=lambda it: it[0])
        names = [os.path.splitext(os.path.basename(it[2]))[0] for it in rows]
        selected.extend(it[2] for it in rows)
        listing[c] = names
        # 保底之外，分层抽样本身也可能抽中带洞地块，所以分两个数说清楚
        n_holes_sel = sum(1 for it in rows if it[1] > 0)
        desc.append(
            f'- `{c}`：{len(paths)} 个地块中抽 {len(rows)} 个'
            f'（带洞 {n_holes_sel} 个 = 保底 {len(picked_holes)} + '
            f'分层抽中 {n_holes_sel - len(picked_holes)}；'
            f'数据集带洞地块共 {len(holes_pool)} 个）；'
            f'面积范围 {rows[0][0]/1e4:.2f}~{rows[-1][0]/1e4:.2f} ha（近似）')
    return selected, desc, listing


# ---------------------------------------------------------------------------
# 绘图（matplotlib Agg，无显示环境也能用）
# ---------------------------------------------------------------------------
def setup_font():
    """图里要写中文，得先把中文字体挂上，否则全是豆腐块。"""
    import matplotlib
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for cand in ('Noto Sans CJK JP', 'Noto Sans CJK SC', 'Droid Sans Fallback',
                 'WenQuanYi Zen Hei', 'AR PL UMing CN'):
        if cand in have:
            matplotlib.rcParams['font.sans-serif'] = [cand, 'DejaVu Sans']
            break
    matplotlib.rcParams['axes.unicode_minus'] = False


def new_axes(fig, pos, title, xlabel, ylabel):
    ax = fig.add_subplot(*pos)
    ax.set_facecolor(C_SURFACE)
    ax.grid(True, color=C_GRID, linewidth=0.6, alpha=1.0)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(C_AXIS)
    ax.tick_params(colors=C_MUTED, labelsize=9)
    ax.set_title(title, color=C_INK, fontsize=11)
    ax.set_xlabel(xlabel, color=C_INK_2, fontsize=9)
    ax.set_ylabel(ylabel, color=C_INK_2, fontsize=9)
    return ax


def scatter(ax, x, y, color=C_S1, size=14):
    ax.scatter(x, y, s=size, c=color, alpha=0.75, linewidths=0)
    if len(x):
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)


def plot_report(rows, out_path, min_turning_radius):
    """批量实验总表：每个地块一行，直接展示关键规划指标。"""
    import matplotlib
    matplotlib.use('Agg')
    setup_font()
    import matplotlib.pyplot as plt

    ok = [r for r in rows if r['ok']]

    if not ok:
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.axis('off')
        ax.text(0.5, 0.5, '没有成功样本',
                ha='center', va='center',
                fontsize=16, color=C_CRITICAL)
        fig.savefig(out_path, dpi=180,
                    bbox_inches='tight', facecolor='white')
        plt.close(fig)
        return

    # ---------------------------------------------------------
    # 排序：按照地块名称
    # ---------------------------------------------------------
    ok = sorted(ok, key=lambda r: r['field'])

    # ---------------------------------------------------------
    # 汇总统计
    # ---------------------------------------------------------
    n = len(ok)

    area = np.array([r['area_ha'] for r in ok], dtype=float)
    path = np.array([r['path_len_m'] for r in ok], dtype=float)
    plan = np.array([r['planning_time_s'] for r in ok], dtype=float)

    total_swath = sum(r.get('n_swaths', 0) for r in ok)
    total_turns = sum(r.get('n_turns', 0) for r in ok)
    oob_fields = sum(r.get('oob_pts', 0) > 0 for r in ok)
    radius_bad = sum(r.get('radius_violations', 0) > 0 for r in ok)

    # ---------------------------------------------------------
    # 图尺寸：地块越多，表格自动变高
    # ---------------------------------------------------------
    fig_height = max(7, 2.5 + 0.32 * n)

    fig, ax = plt.subplots(
        figsize=(16, fig_height),
        facecolor='white'
    )

    ax.axis('off')

    # ---------------------------------------------------------
    # 标题
    # ---------------------------------------------------------
    fig.suptitle(
        'OpenNav Coverage 批量规划实验结果',
        fontsize=18,
        fontweight='bold',
        y=0.985
    )

    ax.text(
        0.5, 0.965,
        f'成功地块：{n}    |    '
        f'总 Swath：{total_swath}    |    '
        f'总转弯：{total_turns}',
        transform=ax.transAxes,
        ha='center',
        va='center',
        fontsize=11
    )

    # ---------------------------------------------------------
    # 表格数据
    # ---------------------------------------------------------
    headers = [
        '地块',
        '面积\n(ha)',
        '作业段\n(n)',
        '转弯\n(n)',
        '总路径\n(km)',
        '作业距离\n(km)',
        '转弯距离\n(km)',
        '过渡距离\n(m)',
        '规划时间\n(s)',
        '越界点\n(n)',
        '最大越界\n(m)',
        '最小转弯半径\n(m)',
        '半径违规\n(n)',
    ]

    table_data = []

    for r in ok:
        table_data.append([
            r['field'],

            f"{r['area_ha']:.2f}",

            f"{r['n_swaths']}",

            f"{r['n_turns']}",

            f"{r['path_len_m'] / 1000:.2f}",

            f"{r['swath_len_m'] / 1000:.2f}",

            f"{r['turn_len_m'] / 1000:.2f}",

            f"{r['transit_dist_m']:.1f}",

            f"{r['planning_time_s']:.3f}",

            f"{r['oob_pts']}",

            f"{r['oob_max_dist_m']:.3f}",

            (
                f"{r['min_turn_r_m']:.3f}"
                if np.isfinite(r['min_turn_r_m'])
                else '-'
            ),

            f"{r['radius_violations']}",
        ])

    # ---------------------------------------------------------
    # 创建表格
    # ---------------------------------------------------------
    table = ax.table(
        cellText=table_data,
        colLabels=headers,
        cellLoc='center',
        colLoc='center',
        loc='upper center',
        bbox=[0.01, 0.14, 0.98, 0.80]
    )

    table.auto_set_font_size(False)
    table.set_fontsize(9)

    # ---------------------------------------------------------
    # 表头
    # ---------------------------------------------------------
    for cell in table.get_celld().values():
        cell.set_linewidth(0.5)

    for col in range(len(headers)):
        cell = table[(0, col)]
        cell.set_text_props(
            fontweight='bold',
            color='white'
        )
        cell.set_facecolor(C_S1)

    # ---------------------------------------------------------
    # 根据指标给异常值加醒目标记
    # ---------------------------------------------------------
    for i, r in enumerate(ok, start=1):

        # 越界
        if r['oob_pts'] > 0:
            table[(i, 9)].set_facecolor('#FFE0E0')
            table[(i, 10)].set_facecolor('#FFE0E0')

        # 半径违规
        if r['radius_violations'] > 0:
            table[(i, 12)].set_facecolor('#FFE0E0')

    # ---------------------------------------------------------
    # 底部统计
    # ---------------------------------------------------------
    avg_area = np.mean(area)
    avg_path = np.mean(path) / 1000
    avg_plan = np.mean(plan)

    total_path = np.sum(path) / 1000

    summary = (
        f'平均面积：{avg_area:.2f} ha    |    '
        f'平均路径：{avg_path:.2f} km    |    '
        f'总规划路径：{total_path:.2f} km    |    '
        f'平均规划时间：{avg_plan:.3f} s    |    '
        f'越界地块：{oob_fields}    |    '
        f'半径违规地块：{radius_bad}'
    )

    ax.text(
        0.5, 0.075,
        summary,
        transform=ax.transAxes,
        ha='center',
        va='center',
        fontsize=10,
        fontweight='bold'
    )

    ax.text(
        0.5, 0.035,
        f'最小转弯半径设定值：{min_turning_radius:.3f} m',
        transform=ax.transAxes,
        ha='center',
        va='center',
        fontsize=9
    )

    fig.savefig(
        out_path,
        dpi=180,
        bbox_inches='tight',
        facecolor='white'
    )

    plt.close(fig)


def plot_example(name, rings, nav_xy, swath_chunks, title, out_path):
    """单地块：边界（含洞）+ 作业段 + 完整路径。"""
    import matplotlib
    matplotlib.use('Agg')
    setup_font()
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 8), facecolor=C_SURFACE)
    ax.set_facecolor(C_SURFACE)

    boundary = rings[0]
    bx = [p[0] for p in boundary] + [boundary[0][0]]
    by = [p[1] for p in boundary] + [boundary[0][1]]
    ax.fill(bx, by, color=C_GRID, alpha=0.6, zorder=1)
    ax.plot(bx, by, '-', color=C_INK, linewidth=1.4, zorder=2)
    # 洞：用背景色挖掉，一眼能看出作业段是绕洞生成的
    for hole in rings[1:]:
        hx = [p[0] for p in hole] + [hole[0][0]]
        hy = [p[1] for p in hole] + [hole[0][1]]
        ax.fill(hx, hy, color=C_SURFACE, zorder=2)
        ax.plot(hx, hy, '-', color=C_INK, linewidth=1.0,
                linestyle='--', zorder=2)

    # 作业段（从 nav_path 拆出来的点串，见 split_nav_path）
    for chunk in swath_chunks:
        if len(chunk) < 2:
            continue
        ax.plot(chunk[:, 0], chunk[:, 1],
                '-', color=C_S3, linewidth=1.0, alpha=0.9, zorder=3)

    if len(nav_xy):
        stride = max(1, len(nav_xy) // 200000)
        ax.plot(nav_xy[::stride, 0], nav_xy[::stride, 1],
                '-', color=C_S1, linewidth=0.8, alpha=0.9, zorder=4)
        ax.plot(nav_xy[0, 0], nav_xy[0, 1], 'o', color=C_CRITICAL,
                markersize=7, zorder=5)
        ax.annotate('规划起点', (nav_xy[0, 0], nav_xy[0, 1]),
                    textcoords='offset points', xytext=(8, 8),
                    color=C_INK_2, fontsize=9)
        ax.plot(0.0, 0.0, 's', color=C_INK, markersize=6, zorder=5)
        ax.annotate('车 / 地块首顶点 (0,0)', (0.0, 0.0),
                    textcoords='offset points', xytext=(8, -14),
                    color=C_INK_2, fontsize=9)

    ax.set_aspect('equal')
    ax.grid(True, color=C_GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(C_AXIS)
    ax.tick_params(colors=C_MUTED, labelsize=9)
    ax.set_xlabel('X (m)', color=C_INK_2, fontsize=9)
    ax.set_ylabel('Y (m)', color=C_INK_2, fontsize=9)
    ax.set_title(title, color=C_INK, fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, facecolor=C_SURFACE, bbox_inches='tight')
    plt.close(fig)


# ---------------------------------------------------------------------------
# 规划客户端：goal 构造与 demo_coverage_via_headland.compute_coverage 逐字段一致
# ---------------------------------------------------------------------------
def build_goal(field, frame_id, route_mode, tsp_time_limit, swath_override):
    """构造 ComputeCoveragePath.Goal，字段与 demo_coverage_via_headland 一致。

    field 可以是「点列表」（单个外边界，探针用）或「环列表」（rings[0]=外边界，
    rings[1:]=洞），两种都支持。每个环都会独立成一个 Coordinates：
    服务端 getFieldFromGoal() 对 polygons[1:] 逐个 field.addRing()，
    也就是「第一块是外边界，后面每块挖空一个洞」。
    环必须闭合（首尾点相同），服务端会校验，否则直接 invalid_argument。
    """
    from opennav_coverage_msgs.action import ComputeCoveragePath
    from opennav_coverage_msgs.msg import Coordinate, Coordinates

    goal = ComputeCoveragePath.Goal()
    goal.frame_id = frame_id
    for ring in as_rings(field):
        poly = Coordinates()
        for coord in ring:
            pt = Coordinate()
            pt.axis1 = coord[0]
            pt.axis2 = coord[1]
            poly.coordinates.append(pt)
        goal.polygons.append(poly)
    goal.generate_headland = True
    goal.generate_route = True
    goal.generate_path = True
    # swath_mode 默认不设置 —— 这正是 demo_coverage_via_headland 的行为，
    # 服务端会回落到 demo_params.yaml 的 SET_ANGLE / 0.0
    if route_mode:
        goal.route_mode.mode = route_mode
    if tsp_time_limit > 0:
        goal.route_mode.tsp_time_limit = tsp_time_limit
    if swath_override:
        goal.swath_mode.mode = swath_override['mode']
        goal.swath_mode.objective = swath_override['objective']
    return goal


def duration_to_s(dur):
    if dur is None:
        return 0.0
    return float(dur.sec) + float(dur.nanosec) * 1e-9


class StalledError(RuntimeError):
    """服务端没在合理时间内应答——多半是上一个 goal 还在算。

    和「规划本身失败」要分开：这一类必须走 recover()/重启流程，否则会一路
    传染下去（服务端一直占着执行器，后面每个 goal 都石沉大海）。
    """


class BatchTester:
    def __init__(self, node, frame_id, route_mode, tsp_time_limit, timeout,
                 server_params, send_timeout=30.0):
        self.node = node
        self.frame_id = frame_id
        self.route_mode = route_mode
        self.tsp_time_limit = tsp_time_limit
        self.timeout = timeout
        # 发 goal 到被接受这一段单独用一个短超时：服务端卡住时 send_goal 就是
        # 没回音，用 plan 的 180 s 去等只会让每个后续地块都白等三分钟。
        self.send_timeout = send_timeout
        self.client = None
        self.pending = None
        self.min_turning_radius = float(server_params.get('min_turning_radius', 0.0))
        self.operation_width = float(server_params.get('operation_width', 0.0))
        # 地头宽度：用来估「nav_path 没盖到的那一圈」有多大（见 §4）
        self.headland_width = float(server_params.get('default_headland_width', 0.0))

    def wait_for_server(self, wait_s=120.0):
        from opennav_coverage_msgs.action import ComputeCoveragePath
        from rclpy.action import ActionClient
        if self.client is not None:
            try:
                self.client.destroy()
            except Exception:  # noqa: BLE001
                pass
        self.client = ActionClient(self.node, ComputeCoveragePath, COVERAGE_ACTION)
        self.pending = None
        deadline = time.time() + wait_s
        while time.time() < deadline:
            if self.client.wait_for_server(timeout_sec=1.0):
                return
            log('  ... 等 compute_coverage_path action server')
        raise RuntimeError('action server 一直没起来')

    def plan(self, field, mode=None, swath_override=None):
        """发一次规划请求，返回 (result, wall_time_s)。失败抛异常。

        mode=None 用 self.route_mode（主模式）；逐模式跑对比时显式传。
        """
        import rclpy

        mode = self.route_mode if mode is None else mode
        goal = build_goal(field, self.frame_id, mode,
                          self.tsp_time_limit, swath_override)
        t0 = time.time()
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future,
                                         timeout_sec=self.send_timeout)
        handle = send_future.result()
        if handle is None:
            raise StalledError(
                f'goal 在 {self.send_timeout:.0f}s 内没被应答（服务端还占着）')
        if not handle.accepted:
            raise RuntimeError('goal 被服务端拒绝')

        self.pending = handle
        result_future = handle.get_result_async()
        # 分片等待而不是一次 spin 到底：服务端要是被 OOM kill 了，结果永远不会
        # 回来，一次 spin 会把整个 --timeout（默认 180 s）白等完。
        # action server 从图上消失是进程没了的可靠信号（计算再重也不影响图发现），
        # 所以每 2 s 探一次，连续两次看不到就认定死了。
        deadline = time.time() + self.timeout
        misses = 0
        while not result_future.done():
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            rclpy.spin_until_future_complete(self.node, result_future,
                                             timeout_sec=min(2.0, remaining))
            if result_future.done():
                break
            misses = 0 if self.client.server_is_ready() else misses + 1
            if misses >= 2:
                raise StalledError('服务端在规划过程中消失了（多半被 OOM kill）')

        wall = time.time() - t0
        wrapped = result_future.result()
        if wrapped is None:
            raise StalledError(f'等待结果超时（>{self.timeout:.0f}s）')
        self.pending = None
        return wrapped.result, wall

    def recover(self, timeout=60.0):
        """超时之后把服务端拉回「能应答」的状态。

        先 cancel（聊胜于无：真正在算的 goal 停不下来，但能把请求排进队列，
        等服务端空下来时立刻终止），再发一个极小地块当探针——只要有任何应答
        （接受、拒绝、报错都算）就说明执行器空了。
        返回 True 表示可以继续用；False 表示得重启进程。
        """
        import rclpy

        if self.pending is not None:
            try:
                future = self.pending.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, future,
                                                 timeout_sec=5.0)
            except Exception:  # noqa: BLE001
                pass
            self.pending = None

        # 2 m 见方的小三角：validateGoal 放行，但整个规划毫秒级完成
        probe = [(0.0, 0.0), (2.0, 0.0), (1.0, 2.0)]
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.client.server_is_ready():
                time.sleep(1.0)
                continue
            goal = build_goal(probe, self.frame_id, self.route_mode, 0, None)
            future = self.client.send_goal_async(goal)
            # 探针的等待不能超过剩余时间，否则 --recover-timeout 形同虚设
            rclpy.spin_until_future_complete(
                self.node, future,
                timeout_sec=max(0.1, min(5.0, deadline - time.time())))
            handle = future.result()
            if handle is None:
                time.sleep(2.0)
                continue
            if handle.accepted:
                # 探针被接受了就得等它跑完，否则下一次 plan 又要撞上它
                rclpy.spin_until_future_complete(
                    self.node, handle.get_result_async(), timeout_sec=10.0)
            return True
        return False


def recover_server(server, tester, args, row):
    """一次卡死之后把服务端弄回可用状态，返回（可能换过进程的）server。

    两种情况分开处理：
      * 进程没了（被 OOM kill）——等多久都不会回来，直接重启；
      * 进程还在，只是被上一个 goal 占着——先等它空下来，等不回来再重启。
    重启是唯一可靠的手段：计算跑在 action 回调里，中途不让出执行器，
    cancel 打不进去，lifecycle 也打不进去。
    """
    if server is not None and server.died():
        note = f'服务端进程没了（{server.death_reason()}）'
        log(f'     {note}，直接重启')
        row['error_text'] = f'{row["error_text"]} | {note}'
    else:
        log(f'     服务端无应答，尝试恢复（最多等 {args.recover_timeout:.0f}s）...')
        try:
            if tester.recover(timeout=args.recover_timeout):
                log('     服务端已恢复，继续')
                return server
        except Exception as e:  # noqa: BLE001
            log(f'     探活失败: {type(e).__name__}: {e}')

        if server is None:
            log('     服务端是 --server external 拉起来的，脚本没法重启它；'
                '后续地块会快速失败而不是各等一个超时。')
            return None
        log('     等不回来，重启 coverage_server ...')

    try:
        server.restart()
        tester.wait_for_server()
    except Exception as e:  # noqa: BLE001
        log(f'     重启失败: {type(e).__name__}: {e}')
        row['error_text'] = f'{row["error_text"]} | 重启失败: {e}'
        return server
    log('     已重启并重新 activate，继续')
    return server


# ---------------------------------------------------------------------------
# 单地块：规划 + 全部指标
# ---------------------------------------------------------------------------
def new_row(name, mode):
    """一行 = 一个 (地块, 路由模式) 组合。"""
    return {
        'field': name,
        'route_mode': mode,
        'ok': False,
        'error_code': '',
        'error_text': '',
        'crs': '',
        'n_boundary_pts': 0,
        'n_holes': 0,
        'hole_area_ha': 0.0,
        'headland_ha': 0.0,
        'area_ha': 0.0,
        'concavity': 1.0,
        'poly_valid': True,
        'wall_time_s': 0.0,
        'planning_time_s': 0.0,
        'task_time_s': 0.0,
        'n_swaths': 0,
        'n_turns': 0,
        'contains_turns': False,
        'swaths_ordered': False,
        'n_path_pts': 0,
        'path_len_m': 0.0,
        'swath_len_m': 0.0,
        'turn_len_m': 0.0,
        'oob_pts': 0,
        'oob_frac': 0.0,
        'oob_max_dist_m': 0.0,
        'swath_oob_pts': 0,
        'swath_oob_total': 0,
        'turn_oob_pts': 0,
        'turn_oob_max_dist_m': 0.0,
        'turn_span_median_m': 0.0,
        'turn_span_max_m': 0.0,
        'max_gap_m': 0.0,
        'gaps_over_threshold': 0,
        'min_turn_r_m': float('inf'),
        'radius_violations': 0,
        'transit_dist_m': 0.0,
        'cov_ratio_swath': float('nan'),
        'cov_ratio_all': float('nan'),
        'missed_ha': float('nan'),
        'missed_patches': 0,
        'missed_max_patch_m2': float('nan'),
        'bf_n_swaths': '',
        'bf_path_len_m': '',
        'bf_len_ratio': '',
    }


def fill_row(tester, rings, poly, mode, row, use_coverage):
    """发一次规划、把该模式的指标写进 row。

    成功返回 (result, xy, 作业段点串列表)，失败返回 None。

    失败分两种：服务端明确报 error_code（规划失败），或者客户端侧的异常
    （goal 被拒、解析出错）。两种都只是这一行失败，不影响同一地块的其它模式。
    """
    from opennav_coverage_msgs.action import ComputeCoveragePath

    result, wall = tester.plan(rings, mode)
    row['wall_time_s'] = wall
    row['planning_time_s'] = duration_to_s(result.planning_time)
    row['task_time_s'] = float(result.task_time)
    if result.error_code != ComputeCoveragePath.Result.NONE:
        row['error_code'] = result.error_code
        row['error_text'] = f'error_code={result.error_code}'
        return None

    cp = result.coverage_path
    xy = path_xy(result.nav_path)
    row['n_swaths'] = len(cp.swaths)
    row['n_turns'] = len(cp.turns)
    row['contains_turns'] = bool(cp.contains_turns)
    row['swaths_ordered'] = bool(cp.swaths_ordered)
    row['n_path_pts'] = len(xy)
    row['path_len_m'] = polyline_length(xy)
    turn_xy, turn_seg_len, turn_spans, turn_arrays = flatten_turns(cp)
    # 作业段几何从 nav_path 拆出来（coverage_path.swaths 的 start/end 是退化的，
    # 见 split_nav_path 的说明）
    swath_chunks, _turn_chunks = split_nav_path(xy, turn_arrays)
    row['swath_len_m'] = chunks_length(swath_chunks)
    row['turn_len_m'] = float(sum(turn_seg_len))
    row['max_gap_m'], row['gaps_over_threshold'] = gap_stats(xy)
    # 越界判定按真实地块：落在洞里的点也算越界
    row['oob_pts'], row['oob_frac'], row['oob_max_dist_m'] = oob_stats(xy, poly)

    s_oob, s_tot = swath_endpoint_oob(swath_chunks, poly)
    row['swath_oob_pts'] = s_oob
    row['swath_oob_total'] = s_tot
    row.update(turn_stats(turn_xy, turn_spans, poly))

    r_min = min_turn_radius(xy)
    row['min_turn_r_m'] = r_min
    if math.isfinite(r_min) and tester.min_turning_radius > 0:
        row['radius_violations'] = int(
            r_min < tester.min_turning_radius * RADIUS_TOL_RATIO)

    if len(xy):
        row['transit_dist_m'] = transit_distance(rings[0], (xy[0, 0], xy[0, 1]))

    if use_coverage:
        row.update(coverage_stats(swath_chunks, turn_arrays, poly,
                                  tester.operation_width))

    row['ok'] = True
    return result, xy, swath_chunks


def run_field(tester, wkt_path, modes, compare_swath_angle, use_coverage,
              rows_out):
    """跑一个地块的每个 route_mode，各追加一行到 rows_out。

    返回 (rings, poly, 主模式的 result, 主模式的 xy, 主模式的作业段点串)——
    主模式 = modes[0]。
    StalledError 往上抛：那要 recover 服务端，不是这个地块自己的事；已经跑完
    的模式留在 rows_out 里，调用方按 route_mode 补上没跑的。
    """
    from shapely.geometry import Polygon
    from opennav_coverage_msgs.action import ComputeCoveragePath

    name = os.path.splitext(os.path.basename(wkt_path))[0]
    # 全部环都拿上：rings[0] 外边界，rings[1:] 洞
    rings, _offset, epsg = wkt_to_utm_rings(wkt_path)
    poly, poly_valid = field_polygon(rings)
    # 面积/凹凸性都按**扣掉洞**的真实地块算
    area_m2 = poly.area
    hole_area_ha = max(0.0, Polygon(rings[0]).area - area_m2) / 1e4
    # 地头圈面积：服务端先把它从作业区挖掉，作业段只铺内场（见 §4 的说明）
    headland_ha = 0.0
    if tester.headland_width > 0 and area_m2 > 0:
        inner = poly.buffer(-tester.headland_width, join_style='mitre')
        headland_ha = max(0.0, area_m2 - inner.area) / 1e4
    base = {
        'crs': epsg,
        'n_boundary_pts': len(rings[0]),
        'n_holes': len(rings) - 1,
        'hole_area_ha': hole_area_ha,
        'headland_ha': headland_ha,
        'area_ha': area_m2 / 1e4,
        'concavity': (poly.convex_hull.area / area_m2) if area_m2 > 0 else 1.0,
        'poly_valid': poly_valid,
    }

    primary_result = None
    primary_xy = None
    primary_swath_chunks = []
    for mode in modes:
        row = new_row(name, mode)
        row.update(base)
        try:
            got = fill_row(tester, rings, poly, mode, row, use_coverage)
        except StalledError:
            raise            # 服务端卡了，交给上层 recover
        except Exception as e:  # noqa: BLE001
            row['error_text'] = f'异常: {type(e).__name__}: {e}'
            row['error_code'] = -1
            got = None
        rows_out.append(row)
        if got is not None and primary_result is None:
            primary_result, primary_xy, primary_swath_chunks = got

    # 固定 0° 作业方向的对照只对主模式跑一次（跟路由模式无关）
    if compare_swath_angle and primary_xy is not None and len(primary_xy):
        row0 = next((r for r in rows_out if r['route_mode'] == modes[0]), None)
        if row0 is not None and row0['ok']:
            try:
                res2, _w = tester.plan(
                    rings, modes[0],
                    swath_override={'mode': 'BRUTE_FORCE',
                                    'objective': 'LENGTH'})
                if res2.error_code == ComputeCoveragePath.Result.NONE:
                    xy2 = path_xy(res2.nav_path)
                    row0['bf_n_swaths'] = len(res2.coverage_path.swaths)
                    row0['bf_path_len_m'] = polyline_length(xy2)
                    if row0['bf_path_len_m'] > 0:
                        row0['bf_len_ratio'] = (row0['path_len_m']
                                                / row0['bf_path_len_m'])
                else:
                    # 留空的话报告里只看到「没跑」，看不出是服务端报错还是没发请求
                    row0['bf_len_ratio'] = f'ERR:error_code={res2.error_code}'
            except Exception as e:  # noqa: BLE001
                row0['bf_len_ratio'] = f'ERR:{e}'

    return rings, poly, primary_result, primary_xy, primary_swath_chunks


# ---------------------------------------------------------------------------
# 汇总报告
# ---------------------------------------------------------------------------
def _fmt(v, nd=3):
    if isinstance(v, float):
        if math.isnan(v):
            return ''            # 没算的指标（比如 --no-coverage 的覆盖率）留空
        if math.isinf(v):
            return 'inf'
        return f'{v:.{nd}f}'
    return str(v)


def write_report(rows, out_path, meta):
    """汇总报告。多路由模式时：正文各节只看主模式（modes[0]），
    另外单开一节横向对比，避免同一地块被算两遍。"""
    modes = meta['route_modes']
    primary = modes[0]
    all_rows = rows
    # 主模式的行 = 每个地块一行；其它模式的行只在对比节里出现
    rows = [r for r in all_rows if r['route_mode'] == primary]
    ok = [r for r in rows if r['ok']]
    bad = [r for r in rows if not r['ok']]
    n = len(rows)
    lines = []
    add = lines.append

    if len(modes) == 1:
        mode_txt = (f'`{primary}`' if primary
                    else '（空，用服务端 default_route_type=BOUSTROPHEDON）')
    else:
        mode_txt = ('`' + '` / `'.join(m or '服务端默认' for m in modes) + '`'
                    f'（正文各节只统计主模式 `{primary or "服务端默认"}`，'
                    '横向对比见 §7）')

    add('# coverage 路径规划批量测试报告\n')
    add(f'- 生成时间：{meta["timestamp"]}')
    add(f'- 数据集：`{meta["dataset"]}`（跑 {n} 个地块）')
    add(f'- 服务端参数来源：`{meta["demo_params"]}` 的 `coverage_server` 段')
    add(f'- 服务端模式：{meta["server_mode"]}，帧 `{meta["frame_id"]}`')
    add(f'- route_mode：{mode_txt}')
    add('- 链路：wkt_to_utm_rings(按地块中心自动选 UTM 带，首点平移) -> '
        'ComputeCoveragePath(headland+route+path) -> nav_msgs/Path')
    add('- 地块环：外边界 + 全部洞一起下发（`polygons[0]` 外边界，'
        '`polygons[1:]` 每个洞一个）；面积、凹凸性、越界判定都按**扣掉洞**的'
        '真实地块算')
    add(f'- 覆盖率：作业段按 `operation_width={meta["operation_width"]} m` 铺'
        f'条带后与地块求交'
        + ('' if meta.get('coverage') else '（本次用 `--no-coverage` 关了）'))
    add('')
    if meta.get('sampling'):
        add('### 抽样口径\n')
        add(f'`--sample-per-country {meta["per_country"]}`：每国按面积分层'
            f'（升序等间隔）抽取，带洞地块每国保底 '
            f'{meta["min_holes_per_country"]} 个。逐国情况：\n')
        lines.extend(meta['sampling'])
        add('')
        add('本次实际跑的地块清单见 `fields.txt`（逐行一个地块，可复现）。\n')
    add('## 1. 规划成功率\n')
    add('| 结果 | 数量 | 占比 |')
    add('|---|---:|---:|')
    add(f'| 成功 | {len(ok)} | {len(ok)/n*100:.1f}% |')
    add(f'| 失败 | {len(bad)} | {len(bad)/n*100:.1f}% |')
    add('')
    if bad:
        codes = {}
        for r in bad:
            codes[r['error_text']] = codes.get(r['error_text'], 0) + 1
        add('失败原因分布：\n')
        add('| 错误 | 数量 | 地块 |')
        add('|---|---:|---|')
        for k, v in sorted(codes.items(), key=lambda kv: -kv[1]):
            names = [r['field'] for r in bad if r['error_text'] == k]
            shown = ', '.join(names[:8]) + ('...' if len(names) > 8 else '')
            add(f'| {k.replace("|", chr(92) + "|")} | {v} | {shown} |')
        add('')
        oom = [r for r in bad if 'OOM' in r['error_text']]
        if oom:
            add(f'其中 {len(oom)} 个是**服务端进程被 OOM killer 杀掉**，不是规划'
                '失败：F2C 的 `Graph::shortestPathsAndCosts` 会把所有 N² 条最短路'
                '**整条物化**出来（Floyd-Warshall 之后逐对重建路径），N≈2×swath'
                '数+边界点数、单条路径又是 O(N)，内存因此是 O(N³)。'
                '`tsp_time_limit` 只管 OR-Tools 的搜索，管不到这一步。'
                f'地块：{", ".join(r["field"] for r in oom[:8])}。\n')
    if not ok:
        add('\n**全部地块规划失败，无法继续评估。**\n')
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))
        return

    area = np.array([r['area_ha'] for r in ok])
    plan = np.array([r['planning_time_s'] for r in ok])
    wall = np.array([r['wall_time_s'] for r in ok])
    plen = np.array([r['path_len_m'] for r in ok])
    turns = np.array([r['n_turns'] for r in ok])
    swaths = np.array([r['n_swaths'] for r in ok])
    transit = np.array([r['transit_dist_m'] for r in ok])
    oob_pts = np.array([r['oob_pts'] for r in ok])
    oob_max = np.array([r['oob_max_dist_m'] for r in ok])
    gaps = np.array([r['gaps_over_threshold'] for r in ok])
    violations = np.array([r['radius_violations'] for r in ok])
    rad = np.array([r['min_turn_r_m'] for r in ok])

    def pct(a, q):
        return float(np.percentile(a, q))

    add('## 2. 规划耗时\n')
    add('`planning_time` 是服务端 `ComputeCoveragePath.result` 自报的纯规划耗时；'
        '`wall time` 是客户端从发 goal 到拿到结果的墙钟时间。\n')
    add('| 指标 | 服务端 planning_time | 客户端 wall time |')
    add('|---|---:|---:|')
    add(f'| 中位数 | {pct(plan, 50):.3f} s | {pct(wall, 50):.3f} s |')
    add(f'| p95 | {pct(plan, 95):.3f} s | {pct(wall, 95):.3f} s |')
    add(f'| 最大 | {plan.max():.3f} s | {wall.max():.3f} s |')
    add(f'| 合计 | {plan.sum():.1f} s | {wall.sum():.1f} s |')
    add('')
    add(f'最大地块 {area.max():.2f} ha，最小 {area.min():.2f} ha，'
        f'总面积 {area.sum():.1f} ha；全量 {n} 个地块客户端共 '
        f'{wall.sum()/60:.1f} 分钟。\n')
    add('**wall time 远大于 planning_time 的原因**：`nav_path` 按 0.1 m 采样，'
        '几公顷的地块就是几十万到上百万个 `PoseStamped`。实测单个 3.55 ha 地块'
        '（109.6 万点）服务端规划 7.0 s，而客户端收完并反序列化要 40 s —— '
        '这 40 s 是 **rclpy 客户端的开销，不是规划本身**（消费方换成 C++ 就不'
        '是这个量级），但它确实说明这条链路的**结果体量**是个真问题：'
        '0.1 m 的采样密度让「规划快、传输慢」。'
        '本节及以后的几何指标都由服务端返回的数据算出，不受该开销影响。\n')

    add('## 3. 路径规模\n')
    add('| 指标 | 中位数 | p95 | 最大 |')
    add('|---|---:|---:|---:|')
    add(f'| 作业段数 | {pct(swaths,50):.0f} | {pct(swaths,95):.0f} | {swaths.max():.0f} |')
    add(f'| 转弯段数 | {pct(turns,50):.0f} | {pct(turns,95):.0f} | {turns.max():.0f} |')
    add(f'| 路径总长 (m) | {pct(plen,50):.0f} | {pct(plen,95):.0f} | {plen.max():.0f} |')
    add('')
    theoretical = area * 1e4 / meta['operation_width']
    ratio = plen / theoretical
    add(f'这里的面积是**扣掉洞之后**的净面积（`area_ha`，洞的面积见 '
        f'`hole_area_ha`）；以作业宽度 {meta["operation_width"]} m 计算，'
        '纯覆盖的理论最短路径为 净面积/幅宽。实测路径 / 理论覆盖长度：')
    add(f'中位 **{np.median(ratio):.2f}×**，p95 {pct(ratio,95):.2f}×，'
        f'最大 {ratio.max():.2f}×。')
    add('这个比值就是「转弯 + 地头 + 掉头」带来的额外里程占比。\n')

    # ---- 覆盖率 ----
    # 定义：作业段点串按 operation_width/2 左右各铺半个幅宽、求并成条带，再与
    # 「扣掉洞」的地块求交。转弯段同样铺条带，单独再算一次。
    add('## 4. 覆盖率\n')
    if not meta.get('coverage'):
        add('本次用 `--no-coverage` 跳过了覆盖率计算。\n')
    else:
        cov = np.array([r['cov_ratio_swath'] for r in ok], dtype=float)
        cov_all = np.array([r['cov_ratio_all'] for r in ok], dtype=float)
        missed = np.array([r['missed_ha'] for r in ok], dtype=float)
        patches = np.array([r['missed_patches'] for r in ok], dtype=float)
        pmax = np.array([r['missed_max_patch_m2'] for r in ok], dtype=float)
        head = np.array([r['headland_ha'] for r in ok], dtype=float)
        f_ok = np.isfinite(cov)
        add(f'- 定义：把 nav_path 按转弯段切成作业段点串（见 `split_nav_path()`），'
            f'每段按 `operation_width={meta["operation_width"]} m` 左右各铺半个'
            '幅宽（条带止于作业段两端，不外扩），并起来与地块求交；'
            '`cov_ratio_swath` = 交面积 / 地块净面积。')
        add('- `cov_ratio_all` 再把转弯段的条带算进去。转弯是车**压过**的地，'
            '不是机具**喷到**的地；两者之差就是转弯段顺带覆盖的贡献。')
        # 分母是整块地，而 nav_path 里没有地头那一圈——这是覆盖率到不了 1 的
        # 主要原因，必须写清楚，不然读数会当成漏喷。
        if f_ok.any() and head[f_ok].sum() > 0:
            h_frac = head[f_ok].sum() / max(area[f_ok].sum(), 1e-9)
            gap = float(np.median(1.0 - cov_all[f_ok]))
            add(f'- **分母是整个地块，而 nav_path 里没有地头圈**：服务端 '
                f'`generate_headlands=true` 时先按 `default_headland_width` 把地头'
                f'从作业区挖掉，作业段只铺内场；地头靠 demo 的 transit 沿外边界走，'
                f'而且只走最短的一侧、不是整圈。抽样里地头圈合计 '
                f'{head[f_ok].sum():.3f} ha、占这些地块总面积的 {h_frac*100:.1f}%，'
                f'`1 - cov_ratio_all` 的中位数 {gap*100:.1f}%：量级相当，'
                f'说明缺口主要是这一圈，不是作业段漏喷。')
        if not f_ok.all():
            add(f'- 有 {int((~f_ok).sum())} 个地块没算出覆盖率（作业段为空）。')
        if f_ok.any():
            add('')
            add('| 指标 | 中位 | p5 | 最小 | 最大 |')
            add('|---|---:|---:|---:|---:|')
            add(f'| 作业段覆盖率 cov_ratio_swath | '
                f'{np.median(cov[f_ok]):.4f} | {pct(cov[f_ok],5):.4f} | '
                f'{cov[f_ok].min():.4f} | {cov[f_ok].max():.4f} |')
            add(f'| 含转弯覆盖率 cov_ratio_all | '
                f'{np.median(cov_all[f_ok]):.4f} | {pct(cov_all[f_ok],5):.4f} | '
                f'{cov_all[f_ok].min():.4f} | {cov_all[f_ok].max():.4f} |')
            add(f'| 覆盖率 < 0.99 的地块 | '
                f'{int((cov[f_ok] < 0.99).sum())} / {int(f_ok.sum())} | | | |')
            add('')
            add(f'- 漏喷面积合计 {missed[f_ok].sum():.2f} ha（占这些地块总面积的 '
                f'{missed[f_ok].sum() / max(area.sum(), 1e-9) * 100:.4f}%），'
                f'单地块中位 {np.median(missed[f_ok]):.4f} ha、最大 '
                f'{missed[f_ok].max():.4f} ha。')
            add(f'- 漏喷斑块（>{COVER_PATCH_MIN_M2} m² 的连通块）：地块中位 '
                f'{np.median(patches[f_ok]):.0f} 个，最大 '
                f'{patches[f_ok].max():.0f} 个；最大一块的面积中位 '
                f'{np.median(pmax[f_ok]):.3f} m²、最大 {pmax[f_ok].max():.3f} m²。')
            add('')
            worst = sorted(range(len(ok)), key=lambda i: cov[i] if f_ok[i] else 1.0)[:8]
            add('覆盖率最低的几个地块：\n')
            add('| 地块 | 净面积 (ha) | cov_swath | cov_all | 漏喷 (m²) | '
                '斑块数 | 最大斑块 (m²) |')
            add('|---|---:|---:|---:|---:|---:|---:|')
            for i in worst:
                if not f_ok[i]:
                    continue
                add(f'| {ok[i]["field"]} | {area[i]:.2f} | {cov[i]:.4f} | '
                    f'{cov_all[i]:.4f} | {missed[i]*1e4:.1f} | '
                    f'{patches[i]:.0f} | {pmax[i]:.2f} |')
            add('')
            add('读法：漏喷斑块基本都出现在作业段两端的「行末」——作业段两端被裁在'
                '地块边界上，条带以平头收尾，斜边处会剩下三角形缺口；洞的周边同理。'
                '这属于覆盖作业本身的固有损失（真实机具靠地头圈补），'
                '量级远小于转弯段越界带来的问题。\n')

    add('## 5. 几何可行性\n')
    n_oob = int((oob_pts > 0).sum())
    add(f'### 5.1 越界（路径点落在地块外 > {meta["oob_tolerance_cm"]} cm）\n')
    add(f'- 有越界点的地块：**{n_oob} / {len(ok)}**（{n_oob/len(ok)*100:.1f}%）')
    add(f'- 越界点数中位数：{pct(oob_pts, 50):.0f}，p95 {pct(oob_pts,95):.0f}')
    if n_oob:
        add(f'- 最大越界距离：中位 {np.median(oob_max[oob_pts>0]):.3f} m，'
            f'最大 {oob_max.max():.3f} m')
    add('')

    # 越界来源拆解：作业段 vs 转弯段
    sw_oob = np.array([r['swath_oob_pts'] for r in ok])
    sw_tot = np.array([r['swath_oob_total'] for r in ok])
    tn_oob = np.array([r['turn_oob_pts'] for r in ok])
    tn_tot = np.array([r['turn_n_pts'] for r in ok], dtype=float)
    span_med = np.array([r['turn_span_median_m'] for r in ok])
    span_max = np.array([r['turn_span_max_m'] for r in ok])
    add('**越界来源拆解**（作业段端点 vs 转弯段点，这两类的成因完全不同）：\n')
    add('| 来源 | 越界点数 | 总点数 | 越界比例 |')
    add('|---|---:|---:|---:|')
    add(f'| 作业段端点 | {sw_oob.sum():.0f} | {sw_tot.sum():.0f} | '
        f'{sw_oob.sum()/max(sw_tot.sum(),1)*100:.3f}% |')
    add(f'| 转弯段 | {tn_oob.sum():.0f} | {tn_tot.sum():.0f} | '
        f'{tn_oob.sum()/max(tn_tot.sum(),1)*100:.2f}% |')
    add('')
    add(f'作业段端点是**裁在地块内的**（{sw_oob.sum()/max(sw_tot.sum(),1)*100:.3f}% 越界）。'
        '越界几乎全部来自转弯段：path generator 在两个作业段端点之间直接连一条 '
        'Dubins 弧，而路由给出的「相邻作业段」有时相隔几十米。\n')
    add(f'转弯段首尾直线距离：中位 {np.median(span_med):.2f} m，'
        f'p95 {pct(span_med,95):.2f} m，最大 {span_max.max():.2f} m。'
        '正常相邻作业段应当只有 operation_width'
        f'（{meta["operation_width"]} m）。\n')

    conc = np.array([r['concavity'] for r in ok])
    # 「那根线」判据：转弯段首尾跨度。正常两条相邻作业段只隔 operation_width
    # （0.7 m），实测干净地块最大也才 5 m，中招地块最小 21 m，10 m 能干净分开。
    # 注意它比 oob_pts 敏感得多：线画在地块内部时并不越界，但看着就是错的。
    line_thr = max(10.0, 10.0 * meta['operation_width'])
    has_line = span_max >= line_thr
    add(f'**「那根明显错误的线」的触发条件**（判据：存在转弯段首尾跨度 '
        f'≥ {line_thr:.0f} m 的转弯）：\n')
    add('| 地块凹凸性 | 地块数 | 出现那根线 | 转弯首尾距最大 | 最大越界 |')
    add('|---|---:|---:|---:|---:|')
    for label, mask in (('concavity ≤ 1.05（基本凸）', conc <= 1.05),
                        ('concavity > 1.05（凹）', conc > 1.05)):
        if not mask.any():
            continue
        add(f'| {label} | {int(mask.sum())} | '
            f'{int(has_line[mask].sum())} / {int(mask.sum())} | '
            f'{span_max[mask].max():.1f} m | {oob_max[mask].max():.3f} m |')
    add('')
    add('触发需要同时满足两条：\n')
    add(f'1. **路由顺序里出现「远距离相邻对」**——地块非凸（concavity ≈ 凸包面积/'
        f'自身面积 > 1.05）+ 固定 0° 作业角 + `generate_decomp=false`。凹处让同一行'
        f'被切成几段作业段，排序时总得从一段跳到另一段。本次 {int(has_line.sum())}/'
        f'{len(ok)} 个地块中招。')
    add(f'2. **这一跳的两端点之间「沿边界绕」不比「直线」长多少**'
        f'（`path_generator.cpp:129` 的 `polyline_len < 1.3 * direct_len`）。'
        f'满足就把 router 算好的沿边界连接整段丢掉，直接画一条 Dubins；'
        f'几十米的距离配 {meta["min_turning_radius"]} m 转弯半径，Dubins 退化成直线。'
        f'两点之间若隔着深凹槽（沿边界要绕 1.3 倍以上），就不触发——路径会贴着'
        f'边界绕过去，远但不出错。')
    add('')
    add('最严重的几个地块：\n')
    add('| 地块 | concavity | 最长转弯跨度 | 最大越界 |')
    add('|---|---:|---:|---:|')
    for i in np.argsort(-span_max)[:8]:
        add(f'| {ok[i]["field"]} | {conc[i]:.3f} | {span_max[i]:.1f} m | '
            f'{oob_max[i]:.3f} m |')
    add('')
    n_holes = sum(1 for r in ok if r['n_holes'] > 0)
    if n_holes:
        add(f'另有 **{n_holes} / {len(ok)}** 个地块带内环（地块里的洞／障碍），'
            '本次全部随 `polygons[1:]` 下发给服务端，逐块对比见 §6。\n')
    add('结论：越界不是作业段的问题，是**未做分解的非凸地块 + 直接把整个地块'
        '交给 BOUSTROPHEDON 排序**造成的。链路里 `generate_decomp` 恒为 false'
        '（`demo_coverage_via_headland` 不设置，服务端默认也是 false），'
        '所以凹地块的作业段顺序会跨凹陷区跳，转弯弧自然穿到地块外面。\n')

    n_gap = int((gaps > 0).sum())
    add('### 5.2 路径连续性\n')
    add(f'- 相邻点间距 > {meta["gap_threshold_m"]} m 的地块：**{n_gap} / {len(ok)}**')
    if n_gap:
        add(f'- 最大跳变：{max(r["max_gap_m"] for r in ok):.3f} m')
    add('')
    add('### 5.3 转弯半径\n')
    add(f'- 实测最小转弯半径 < min_turning_radius'
        f'({meta["min_turning_radius"]} m)×{RADIUS_TOL_RATIO} 的地块：'
        f'**{int(violations.sum())} / {len(ok)}**')
    finite = np.isfinite(rad)
    if finite.any():
        add(f'- 有限样本的最小转弯半径：中位 {np.median(rad[finite]):.3f} m，'
            f'最小 {rad[finite].min():.3f} m')
    add('- 也就是说 path generator 本身是**尊重车辆约束的**，'
        '转弯弧都卡在 min_turning_radius 上；问题不在弧的半径，'
        '而在弧的走向（§5.1）。')
    add('')
    add('### 5.4 transit（车到规划起点的沿边界距离）\n')
    add(f'- 中位 {np.median(transit):.1f} m，p95 {pct(transit,95):.1f} m，'
        f'最大 {transit.max():.1f} m')
    add('- 这直接决定 demo_coverage_via_headland 在覆盖作业前要空跑多远，'
        '以及 RPP `max_robot_pose_search_dist`（demo_params.yaml 里 10.0 m）'
        '够不够用——只要 transit 段存在，车就落在路径 index 0，参数不用改。\n')

    # ---- 带洞地块 ----
    add('## 6. 带洞地块\n')
    hole_rows = [r for r in ok if r['n_holes'] > 0]
    plain_rows = [r for r in ok if r['n_holes'] == 0]
    add('洞的传法：`wkt_to_utm` 把所有环都取出来，`polygons[0]` 放外边界、'
        '`polygons[1:]` 每个洞各一个 `Coordinates`；服务端 '
        '`getFieldFromGoal()` 对第 1 个之后的每个环调 `field.addRing()`，'
        'F2C 生成作业段时会绕开它们。指标里的面积/凹凸性/越界判定同样按'
        '「扣掉洞」的真实地块算。\n')
    if not hole_rows:
        add('本次样本里没有带洞地块（`--min-holes-per-country` 可调保底名额）。\n')
    else:
        n_holes_tot = sum(r['n_holes'] for r in hole_rows)
        add(f'- 带洞地块：**{len(hole_rows)} / {len(ok)}**，共 {n_holes_tot} 个洞，'
            f'洞面积合计 {sum(r["hole_area_ha"] for r in hole_rows):.3f} ha')
        fails = [r for r in hole_rows if not r['ok']]
        if fails:
            add(f'- 其中 {len(fails)} 个规划失败：'
                f'{", ".join(r["field"] for r in fails[:8])}（见 §1 失败原因）')
        add('')
        add('| 分组 | 地块数 | cov_swath 中位 | 有越界的 | 最大越界 (m) | '
            '转弯首尾距最大 (m) |')
        add('|---|---:|---:|---:|---:|---:|')
        for label, group in (('带洞', hole_rows), ('无洞', plain_rows)):
            if not group:
                continue
            g_cov = [r['cov_ratio_swath'] for r in group
                     if math.isfinite(r['cov_ratio_swath'])]
            cov_txt = f'{np.median(g_cov):.4f}' if g_cov else '-'
            add(f'| {label} | {len(group)} | {cov_txt} | '
                f'{sum(1 for r in group if r["oob_pts"] > 0)} | '
                f'{max(r["oob_max_dist_m"] for r in group):.3f} | '
                f'{max(r["turn_span_max_m"] for r in group):.1f} |')
        add('')
        add('读法：和「无洞地块」比，覆盖率差多少 = 洞周边漏喷多少；越界差多少 = '
            '绕洞的转弯有没有甩到地块外。洞的形状来自真实地块数据，这里评估的是'
            '「当前链路」对它的处理。\n')
        add('逐个带洞地块：\n')
        add('| 地块 | 洞数 | 洞面积 (ha) | 净面积 (ha) | 覆盖率 | 越界点 | '
            '最大越界 (m) | 转弯首尾距最大 (m) |')
        add('|---|---:|---:|---:|---:|---:|---:|---:|')
        for r in sorted(hole_rows, key=lambda r: -r['n_holes'])[:15]:
            cov_txt = (f'{r["cov_ratio_swath"]:.4f}'
                       if math.isfinite(r['cov_ratio_swath']) else '-')
            add(f'| {r["field"]} | {r["n_holes"]} | {r["hole_area_ha"]:.3f} | '
                f'{r["area_ha"]:.2f} | {cov_txt} | {r["oob_pts"]} | '
                f'{r["oob_max_dist_m"]:.3f} | {r["turn_span_max_m"]:.1f} |')
        if len(hole_rows) > 15:
            add(f'| ... | 其余 {len(hole_rows) - 15} 个见 per_field.csv | | | | | | |')
        add('')

    # ---- 路由模式对比 ----
    add('## 7. 路由模式对比\n')
    if len(modes) < 2:
        add('本次只跑了一个路由模式。`--route-mode BOUSTROPHEDON,TSP` '
            '可以在同一批地块上依次跑多个模式做对比（每模式各占一行 CSV）。\n')
    else:
        add('同一批地块、同一份服务端参数，只换 `route_mode` 各规划一遍。'
            '`turn_len` 是转弯/连接段总长——路由模式优化的就是它：'
            'BOUSTROPHEDON 按行序进出，TSP 用 OR-Tools 在作业段端点和边界顶点'
            '之间找一条更短的访问顺序。\n')
        add('| route_mode | 成功 | planning_time 中位 (s) | 路径长中位 (km) | '
            '转弯段中位 (km) | 转弯占比中位 | 有越界的 | 最大越界 (m) | '
            'cov_swath 中位 |')
        add('|---|---:|---:|---:|---:|---:|---:|---:|---:|')
        for m in modes:
            rs = [r for r in all_rows if r['route_mode'] == m]
            good = [r for r in rs if r['ok']]
            label = f'`{m}`' if m else '服务端默认'
            if not good:
                add(f'| {label} | 0 / {len(rs)} | - | - | - | - | - | - | - |')
                continue
            g_cov = [r['cov_ratio_swath'] for r in good
                     if math.isfinite(r['cov_ratio_swath'])]
            tl = np.array([r['turn_len_m'] for r in good])
            pl = np.array([r['path_len_m'] for r in good])
            with np.errstate(invalid='ignore', divide='ignore'):
                turn_frac = np.where(pl > 0, tl / np.maximum(pl, 1e-9), np.nan)
            add(f'| {label} | {len(good)} / {len(rs)} | '
                f'{np.median([r["planning_time_s"] for r in good]):.3f} | '
                f'{np.median(pl)/1000:.2f} | {np.median(tl)/1000:.2f} | '
                f'{np.nanmedian(turn_frac)*100:.1f}% | '
                f'{sum(1 for r in good if r["oob_pts"] > 0)} | '
                f'{max(r["oob_max_dist_m"] for r in good):.3f} | '
                + (f'{np.median(g_cov):.4f} |' if g_cov else '- |'))
        add('')
        bad_modes = {m: sum(1 for r in all_rows
                            if r['route_mode'] == m and not r['ok'])
                     for m in modes}
        if any(bad_modes.values()):
            items = [f'`{m or "服务端默认"}` {c} 个'
                     for m, c in bad_modes.items() if c]
            add('各模式失败数：' + '，'.join(items) + '。\n')
        # 成对比较：同一地块在全部模式下都成功，才能消掉「地块不一样」这个变量
        by_field = {}
        for r in all_rows:
            if r['ok']:
                by_field.setdefault(r['field'], {})[r['route_mode']] = r
        common = sorted(f for f, d in by_field.items() if all(m in d for m in modes))
        if common:
            add(f'配对比较（{len(common)} 个地块在全部 {len(modes)} 个模式下都成功，'
                '同一地块自己跟自己比，消掉地块差异）：\n')
            add('| 对比 | 转弯段总长中位变化 | 转弯段减少的地块 | 路径总长中位变化 | '
                '路径变短的地块 | 覆盖率中位变化 |')
            add('|---|---:|---:|---:|---:|---:|')
            for m in modes[1:]:
                d_turn = np.array([by_field[f][m]['turn_len_m']
                                   - by_field[f][primary]['turn_len_m']
                                   for f in common])
                r_turn = np.array([(by_field[f][m]['turn_len_m']
                                    / by_field[f][primary]['turn_len_m'])
                                   for f in common
                                   if by_field[f][primary]['turn_len_m'] > 0])
                d_path = np.array([by_field[f][m]['path_len_m']
                                   - by_field[f][primary]['path_len_m']
                                   for f in common])
                c_pri = np.array([by_field[f][primary]['cov_ratio_swath']
                                  for f in common])
                c_new = np.array([by_field[f][m]['cov_ratio_swath']
                                  for f in common])
                d_cov = c_new - c_pri
                okp = np.isfinite(d_cov)
                add(f'| `{m or "服务端默认"}` vs `{primary or "服务端默认"}` | '
                    f'{np.median(d_turn):+.1f} m '
                    f'({(np.median(r_turn)-1)*100:+.1f}%) | '
                    f'{int((d_turn < 0).sum())} / {len(common)} | '
                    f'{np.median(d_path)/1000:+.2f} km | '
                    f'{int((d_path < 0).sum())} / {len(common)} | '
                    + (f'{np.median(d_cov[okp]):+.4f} |' if okp.any() else '- |'))
            add('')
            add('「转弯段总长」是路由模式直接优化/影响的东西：转弯越短，空跑越少、'
                '越界机会也越少。代价在 planning_time 一列。\n')

    # 作业方向对照
    add('## 8. 固定 0° 作业方向的代价\n')
    bf = [r for r in ok if isinstance(r.get('bf_len_ratio'), float)]
    if bf:
        br = np.array([r['bf_len_ratio'] for r in bf])
        add('链路里 `demo_coverage_via_headland` 不设置 `goal.swath_mode`，'
            '服务端一律回落到 `default_swath_angle_type=SET_ANGLE` + '
            '`default_swath_angle=0.0`，也就是**不管地块朝哪都按正东作业**。'
            '加 `--compare-swath-angle` 用 BRUTE_FORCE(LENGTH) 跑了一遍对照：\n')
        add(f'- 对照地块数：{len(bf)}')
        add(f'- 路径长度比（固定 0° / BRUTE_FORCE 最优）：'
            f'中位 **{np.median(br):.3f}×**，p95 {pct(br,95):.3f}×，'
            f'最大 {br.max():.3f}×')
        add(f'- 即固定 0° 相比最优作业方向中位多走 '
            f'**{(np.median(br)-1)*100:.1f}%** 的里程\n')
    else:
        add('未运行对照（加 `--compare-swath-angle` 可量化）。'
            '不过从链路本身就能确定：`demo_coverage_via_headland` 从不设置 '
            '`goal.swath_mode`，服务端回落到 `default_swath_angle=0.0`，'
            '对所有地块强制正东作业——而本数据集是真实地块，朝向各异，'
            '这个常数不可能是最优的。\n')

    add('## 9. 结论\n')
    add(f'1. **链路稳定**：{len(ok)}/{n} 成功。'
        if not bad else
        f'1. **链路基本稳定**：{len(ok)}/{n} 成功，{len(bad)} 个失败（见 §1）。')
    add(f'2. **算力可行**：全量端到端 {wall.sum()/60:.1f} 分钟，'
        f'单地块 p95 {pct(wall,95):.2f} s，机载规划没有压力。')
    add(f'3. **越界是真问题**：{n_oob}/{len(ok)} 个地块的路径点跑出过地块，'
        f'最大 {oob_max.max():.2f} m；越界点几乎全在转弯段'
        f'（{tn_oob.sum()/max(tn_tot.sum(),1)*100:.2f}%），'
        f'作业段端点 {sw_oob.sum()/max(sw_tot.sum(),1)*100:.3f}% 干净。'
        '根因是非凸地块没做分解（`generate_decomp` 恒 false），'
        'BOUSTROPHEDON 把跨凹陷区、相距几十米的作业段当相邻段接了起来。'
        '**凹地块要先分解，或者至少把转弯段也做边界约束。**')
    add(f'4. **转弯半径本身没问题**：实测最小半径卡在 min_turning_radius'
        f'（{meta["min_turning_radius"]} m），path generator 是守约束的——'
        '要改的是弧的走向，不是半径参数。')

    conv = np.array([r['cov_ratio_swath'] for r in ok], dtype=float)
    conv = conv[np.isfinite(conv)]
    idx = 5
    if conv.size:
        missed_tot = sum(r['missed_ha'] for r in ok
                         if math.isfinite(r['missed_ha']))
        add(f'{idx}. **覆盖率**：作业段条带盖住地块净面积的中位 '
            f'**{np.median(conv):.4f}**（最低 {conv.min():.4f}），'
            f'{int((conv < 0.99).sum())}/{conv.size} 个地块低于 0.99，'
            f'漏喷合计 {missed_tot:.2f} ha。漏喷集中在作业段两端的行末三角和'
            '洞的周边，属于覆盖作业本身的固有损失（真实机具靠地头圈补），'
            '跟转弯越界（§5.1）不是一个量级的问题。')
    else:
        add(f'{idx}. **覆盖率未计算**（本次带 `--no-coverage`）。')
    idx += 1

    if hole_rows:
        def _med_cov(group):
            v = [r['cov_ratio_swath'] for r in group
                 if math.isfinite(r['cov_ratio_swath'])]
            return f'{np.median(v):.4f}' if v else '-'

        def _max_oob(group):
            return f'{max(r["oob_max_dist_m"] for r in group):.3f}'
        add(f'{idx}. **洞是正确下发的**：{len(hole_rows)} 个带洞地块把内环放进了 '
            '`polygons[1:]`，服务端 `addRing()` 之后作业段绕洞生成，'
            '面积/越界判定也在净面积上做。带洞 vs 无洞：覆盖率中位 '
            f'{_med_cov(hole_rows)} vs {_med_cov(plain_rows)}，'
            f'最大越界 {_max_oob(hole_rows)} vs {_max_oob(plain_rows)} m。'
            '（洞带来的额外影响主要看 §6 的表。）')
        idx += 1

    if bf:
        add(f'{idx}. **作业方向是第二大短板**：`coverage_demo_via_headland` 不发 '
            'swath_mode，服务端固定用 `default_swath_angle=0.0`，'
            '对任意地块都强制正东作业，中位多走 '
            f'{(np.median(br)-1)*100:.1f}% 里程。'
            '数据集是真实地块、朝向各异，这个常数不成立。')
    else:
        add(f'{idx}. **作业方向未评估**：加 `--compare-swath-angle` 可量化'
            '固定 `default_swath_angle=0.0` 的代价。')
    idx += 1

    if len(modes) > 1:
        add(f'{idx}. **路由模式有取舍**：见 §7。转弯/连接段长度和 planning_time '
            '是主要差别——转弯段越短，空跑和越界机会都越少，代价是规划耗时。')
        idx += 1
    add('')
    add('## 10. 对 demo 链路的建议（按性价比排序）\n')
    add('1. `demo_coverage_via_headland` 在发 goal 时带上 '
        '`generate_decomp=true`（配合 `decomp_mode.mode=BOUSTROPHEDON`），'
        '凹地块不再跨凹陷区接转弯。')
    add('2. 别把 `default_swath_angle` 写死成 0.0；改成 '
        '`default_swath_angle_type=BRUTE_FORCE` + '
        '`default_swath_objective=LENGTH`，或按地块主方向算。')
    add('3. ~~洞没下发~~ **本次已改**：新增 `wkt_to_utm_rings()` 返回全部环，'
        '`demo_coverage_via_headland` / `demo_coverage` 都改成把 '
        '`polygons[1:]` 逐环下发（goal 的 `polygons` 本来就是'
        '「第一个外边界、后面每个挖空一块」的约定，服务端 '
        '`getFieldFromGoal()` 会逐环 `addRing()`）。后续只需按 §6 的结果决定'
        '要不要给洞单独加地头圈。')
    add('4. 转弯段目前没有任何边界约束，建议在 `path_mode` 上确认 '
        'Dubins/Reeds-Shepp 的连续性设置，或对越界转弯做后处理。')
    if any('OOM' in r['error_text'] for r in bad):
        add('5. **swath 数上百之后 TSP 模式会把服务端吃爆内存**（见 §1）。'
            '短期：全量跑批时给 coverage_server 加内存上限 + 用本脚本的'
            '失败隔离（单地块失败不影响后续）；中期：给 '
            '`TspRouteMethod` 加一个 swath 数上限（`max_swaths_for_global_route`'
            ' 目前是 `route_method.hpp:94` 里明确写着 unused 的死参数），'
            '超了就退回 BOUSTROPHEDON，或者干脆上 `generate_decomp=true`'
            '把大凹地块切成小块再各自 TSP。')

    # ---- 附录：地块清单 ----
    if meta.get('listing'):
        add('')
        add('## 附录 A：本次地块清单\n')
        for c in sorted(meta['listing']):
            names = meta['listing'][c]
            add(f'- `{c}`（{len(names)} 个）：' + ', '.join(f'`{x}`' for x in names))
        add('')
    if meta.get('route_modes') and len(meta['route_modes']) > 1:
        add('## 附录 B：CSV 说明\n')
        add('`per_field.csv` 每个 (地块, route_mode) 一行，'
            '`route_mode` 列为空串表示「没设置、用服务端 default_route_type」。'
            '同一地块的多行里，只有 `route_mode` 和后缀是 `bf_` 的对照列不同，'
            '几何输入（`area_ha`/`n_holes`/`crs` 等）完全一样。\n')

    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
CSV_COLUMNS = [
    'field', 'route_mode', 'ok', 'error_code', 'error_text',
    'crs', 'n_boundary_pts', 'n_holes', 'hole_area_ha', 'headland_ha', 'area_ha',
    'concavity', 'poly_valid',
    'wall_time_s', 'planning_time_s', 'task_time_s',
    'n_swaths', 'n_turns', 'contains_turns', 'swaths_ordered',
    'n_path_pts', 'path_len_m', 'swath_len_m', 'turn_len_m',
    'oob_pts', 'oob_frac', 'oob_max_dist_m',
    'swath_oob_pts', 'swath_oob_total',
    'turn_n_pts', 'turn_oob_pts', 'turn_oob_max_dist_m',
    'turn_span_median_m', 'turn_span_max_m',
    'max_gap_m', 'gaps_over_threshold', 'min_turn_r_m', 'radius_violations',
    'transit_dist_m',
    'cov_ratio_swath', 'cov_ratio_all', 'missed_ha', 'missed_patches',
    'missed_max_patch_m2',
    'bf_n_swaths', 'bf_path_len_m', 'bf_len_ratio',
]


def main():
    parser = argparse.ArgumentParser(
        description='批量跑 fields2Benchmark_Dataset 的覆盖路径规划，'
                    '链路与 coverage_demo_launch.py 一致')
    parser.add_argument('--dataset', default=DEFAULT_DATASET, help='WKT 数据集目录')
    parser.add_argument('--demo-params', default=DEFAULT_DEMO_PARAMS,
                        help='demo_params.yaml（服务端参数的唯一真源）')
    parser.add_argument('--fields', default='',
                        help='只跑这些地块，逗号分隔（不带 .wkt）')
    parser.add_argument('--limit', type=int, default=0, help='只跑前 N 个（0=全部）')
    parser.add_argument('--sample-per-country', type=int, default=0,
                        metavar='N',
                        help='每国按面积分层抽 N 个（0=不抽样，用 --limit/全部）；'
                             f'带洞地块每国保底 '
                             f'{DEFAULT_MIN_HOLES_PER_COUNTRY} 个')
    parser.add_argument('--countries', default='',
                        help='抽样时只考虑这些国家前缀，逗号分隔（默认全部）')
    parser.add_argument('--min-holes-per-country', type=int,
                        default=DEFAULT_MIN_HOLES_PER_COUNTRY, metavar='N',
                        help='抽样时每国带洞地块的保底个数（默认 %(default)s）')
    parser.add_argument('--no-coverage', action='store_true',
                        help='不算覆盖率指标（作业段铺条带 + 求交，省时间）')
    parser.add_argument('--output', default='',
                        help='输出目录（默认 test/results/<时间戳>_batch）')
    parser.add_argument('--server', choices=['spawn', 'external'], default='spawn',
                        help='spawn=自己拉起 coverage_server；'
                             'external=连已经跑着的（如 coverage_demo_launch.py）')
    parser.add_argument('--frame', default='map', help='frame_id（默认 map）')
    parser.add_argument('--route-mode', default='',
                        help='BOUSTROPHEDON / SNAKE / SPIRAL / TSP；'
                             '逗号分隔可一次跑多个做对比（如 BOUSTROPHEDON,TSP）；'
                             '空 = 服务端 default_route_type')
    parser.add_argument('--tsp-time-limit', type=int, default=0,
                        help='TSP 求解时间上限（秒），0 = 用 msgs 默认值 1')
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='单个地块等待结果的上限（秒）')
    parser.add_argument('--send-timeout', type=float, default=30.0,
                        help='发 goal 到被接受的上限（秒）；服务端卡住时就是'
                             '没回音，这个值决定后续地块白等多久')
    parser.add_argument('--recover-timeout', type=float, default=60.0,
                        help='超时后等服务端自己空下来的上限（秒），'
                             '等不回来就重启进程')
    parser.add_argument('--compare-swath-angle', action='store_true',
                        help='每个地块额外用 BRUTE_FORCE(LENGTH) 跑一遍作对照')
    parser.add_argument('--examples', type=int, default=4,
                        help='输出几个代表性地块的路径图（0=不画）')
    parser.add_argument('--no-plots', action='store_true', help='不画图')
    args = parser.parse_args()

    if not os.path.isdir(args.dataset):
        log(f'错误：数据集目录不存在 {args.dataset}')
        return 1
    if not os.path.isfile(args.demo_params):
        log(f'错误：找不到 demo_params.yaml {args.demo_params}')
        return 1

    # ---- 路由模式（逗号分隔可跑多个做对比）----
    route_modes = [s.strip() for s in args.route_mode.split(',') if s.strip()]
    if not route_modes:
        route_modes = ['']

    # ---- 地块列表 ----
    sample_desc = None
    listing = None
    if args.fields:
        names = [s.strip() for s in args.fields.split(',') if s.strip()]
        wkt_paths = []
        for nm in names:
            p = os.path.join(args.dataset, nm if nm.endswith('.wkt') else nm + '.wkt')
            if not os.path.isfile(p):
                log(f'错误：找不到 {p}')
                return 1
            wkt_paths.append(p)
        if args.sample_per_country:
            log('提示：--fields 和 --sample-per-country 同时给了，按 --fields 跑')
    else:
        wkt_paths = sorted(
            os.path.join(args.dataset, f)
            for f in os.listdir(args.dataset) if f.endswith('.wkt'))
        if args.sample_per_country > 0:
            countries = [s.strip() for s in args.countries.split(',') if s.strip()]
            wkt_paths, sample_desc, listing = sample_fields(
                wkt_paths, args.sample_per_country, countries or None,
                args.min_holes_per_country)
        elif args.limit > 0:
            wkt_paths = wkt_paths[:args.limit]
    if not wkt_paths:
        log('错误：没有找到任何 .wkt 文件')
        return 1

    # ---- 输出目录 ----
    if args.output:
        out_dir = args.output
    else:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.join(TEST_DIR, 'results', f'{stamp}_batch')
    os.makedirs(out_dir, exist_ok=True)
    examples_dir = os.path.join(out_dir, 'examples')
    if args.examples > 0 and not args.no_plots:
        os.makedirs(examples_dir, exist_ok=True)

    # ---- 服务端参数 ----
    server_params = load_server_params(args.demo_params)
    use_sim_time = (args.server == 'external')
    params_path = os.path.join(out_dir, 'server_params.yaml')
    write_server_params(server_params, params_path, use_sim_time)

    # 地块清单落盘：抽样结果可复现
    fields_txt = os.path.join(out_dir, 'fields.txt')
    with open(fields_txt, 'w', encoding='utf-8') as f:
        if sample_desc:
            f.write(f'# 每国抽样 {args.sample_per_country} 个，'
                    f'带洞保底 {args.min_holes_per_country} 个；'
                    '面积升序等间隔（分层）\n')
            for line in sample_desc:
                f.write(f'# {line.lstrip("- ")}\n')
        f.write(f'# route_mode: {", ".join(route_modes) or "(服务端默认)"}\n')
        for p in wkt_paths:
            f.write(os.path.splitext(os.path.basename(p))[0] + '\n')

    log('=' * 70)
    log('coverage 路径规划批量测试')
    log(f'  数据集      : {args.dataset}  (跑 {len(wkt_paths)} 个地块)')
    log(f'  服务端参数  : {args.demo_params} [coverage_server]')
    log(f'  输出目录    : {out_dir}')
    log(f'  服务端      : {args.server}  (use_sim_time={use_sim_time})')
    log(f'  route_mode  : {", ".join(m or "(服务端默认)" for m in route_modes)}'
        f'  （每地块每个模式各跑一次）')
    log(f'  覆盖率指标  : {"关" if args.no_coverage else "开"}'
        f'，地块环: 外边界 + 全部洞')
    if sample_desc:
        log('  抽样口径    :')
        for line in sample_desc:
            log(f'    {line.lstrip("- ")}')
    log('  关键参数    : ' + ', '.join(
        f'{k}={server_params[k]}'
        for k in ('default_headland_width', 'robot_width', 'operation_width',
                  'min_turning_radius', 'default_swath_angle_type',
                  'default_swath_angle')
        if k in server_params))
    log('=' * 70)

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    rclpy.init()
    server = None
    node = None
    rows = []
    try:
        if args.server == 'spawn':
            server = ServerHandle(params_path, os.path.join(out_dir, 'server.log'))
            log('\n启动 standalone coverage_server ...')
            server.start()
            log('  已 configure + activate')

        node = Node('coverage_batch_tester')
        if use_sim_time:
            node.set_parameters(
                [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        # route_modes[0] = 主模式（探针/recover 用它）
        tester = BatchTester(node, args.frame, route_modes[0],
                             args.tsp_time_limit, args.timeout, server_params,
                             send_timeout=args.send_timeout)
        tester.wait_for_server()
        log('action server 就绪\n')

        # per_field.csv 边跑边写：全量要好几个小时，中途也能看进度
        csv_path = os.path.join(out_dir, 'per_field.csv')
        csv_file = open(csv_path, 'w', encoding='utf-8', newline='')
        writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS,
                                extrasaction='ignore')
        writer.writeheader()
        csv_file.flush()

        t_start = time.time()
        multi = len(route_modes) > 1
        for i, wkt_path in enumerate(wkt_paths, 1):
            name = os.path.splitext(os.path.basename(wkt_path))[0]
            field_rows = []
            try:
                rings, _poly, result, xy, swath_chunks = run_field(
                    tester, wkt_path, route_modes, args.compare_swath_angle,
                    not args.no_coverage, field_rows)
            except Exception as e:  # noqa: BLE001
                # 卡死会把后面所有地块一起拖下水，必须先把服务端弄回来。
                # 已经跑完的模式留在 field_rows 里，这里只补没跑到的那些。
                stalled = isinstance(e, StalledError)
                rings = result = xy = None
                swath_chunks = []
                err = f'异常: {type(e).__name__}: {e}'
                done = {r['route_mode'] for r in field_rows}
                for m in route_modes:
                    if m in done:
                        continue
                    row = new_row(name, m)
                    row.update({'error_text': err, 'error_code': -1,
                                'area_ha': 0.0, 'min_turn_r_m': float('inf')})
                    field_rows.append(row)
                if stalled:
                    server = recover_server(server, tester, args,
                                            field_rows[-1])
            rows.extend(field_rows)
            for row in field_rows:
                writer.writerow({k: (_fmt(v) if isinstance(v, float) else v)
                                 for k, v in row.items()})
            csv_file.flush()

            done = time.time() - t_start
            eta = done / i * (len(wkt_paths) - i)
            for mi, row in enumerate(field_rows):
                tag = f"[{i:4d}/{len(wkt_paths)}]"
                mode_tag = f"{row['route_mode'] or '默认':>14s} " if multi else ''
                if not row['ok']:
                    log(f"{tag} {name:16s} {mode_tag}FAIL {row['error_text']}")
                    continue
                extra = ''
                if isinstance(row['bf_len_ratio'], float):
                    extra = f" 0°/BF={row['bf_len_ratio']:.3f}"
                cov = ''
                if math.isfinite(row['cov_ratio_swath']):
                    cov = f" cov {row['cov_ratio_swath']:.4f}"
                log(f"{tag} {name:16s} {mode_tag}"
                    f"OK  {row['area_ha']:6.2f} ha"
                    f"{'/' + str(row['n_holes']) + '洞' if row['n_holes'] else ''}  "
                    f"{row['n_swaths']:5d} 段  "
                    f"{row['path_len_m']/1000:7.2f} km  "
                    f"plan {row['planning_time_s']:6.2f}s  "
                    f"oob {row['oob_pts']:5d}({row['oob_max_dist_m']:.3f}m)  "
                    f"gap {row['max_gap_m']:.2f}m  "
                    f"Rmin {_fmt(row['min_turn_r_m'], 3)}{cov}{extra}"
                    f"  ETA {eta/60:.0f}min")

            # 每个地块出一张路径图（用主模式那次的结果）
            row0 = field_rows[0] if field_rows else None
            if (not args.no_plots
                    and row0 is not None and row0['ok']
                    and result is not None and xy is not None and len(xy)
                    and rings):

                fig_name = (
                    f"{name}_a{row0['area_ha']:.1f}ha"
                    f"{'_h' + str(row0['n_holes']) if row0['n_holes'] else ''}"
                    f"_oob{row0['oob_max_dist_m']:.2f}m.png"
                )

                plot_example(
                    name,
                    rings,
                    xy,
                    swath_chunks,
                    f"{name}  {row0['area_ha']:.2f} ha"
                    f"{'  含 ' + str(row0['n_holes']) + ' 个洞' if row0['n_holes'] else ''}"
                    f"  {row0['n_swaths']} 段  "
                    f"{row0['path_len_m']/1000:.2f} km",
                    os.path.join(examples_dir, fig_name)
                )

                row0['_example'] = fig_name

        csv_file.close()
        elapsed = time.time() - t_start
        log(f'\n全部完成，用时 {elapsed/60:.1f} 分钟')

        if not args.no_plots:
            # 总表只放主模式，否则同一地块会重复出现
            plot_report([r for r in rows if r['route_mode'] == route_modes[0]],
                        os.path.join(out_dir, 'report.png'),
                        server_params.get('min_turning_radius', 0.0))

        meta = {
            'timestamp': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'dataset': args.dataset,
            'demo_params': args.demo_params,
            'server_mode': args.server,
            'frame_id': args.frame,
            'route_modes': route_modes,
            'coverage': not args.no_coverage,
            'sampling': sample_desc,
            'listing': listing,
            'per_country': args.sample_per_country,
            'min_holes_per_country': args.min_holes_per_country,
            'operation_width': server_params.get('operation_width', 0.0),
            'min_turning_radius': server_params.get('min_turning_radius', 0.0),
            'oob_tolerance_cm': int(OOB_TOLERANCE_M * 100),
            'gap_threshold_m': GAP_THRESHOLD_M,
        }
        report_path = os.path.join(out_dir, 'report.md')
        write_report(rows, report_path, meta)

        log('\n输出：')
        log(f'  per_field.csv : {csv_path}')
        log(f'  report.md     : {report_path}')
        log(f'  fields.txt    : {fields_txt}')
        if not args.no_plots:
            log(f'  report.png    : {os.path.join(out_dir, "report.png")}')
        if args.examples > 0 and not args.no_plots:
            log(f'  examples/     : {examples_dir}')

        primary_rows = [r for r in rows if r['route_mode'] == route_modes[0]]
        ok = sum(1 for r in primary_rows if r['ok'])
        log(f'\n主模式成功 {ok}/{len(primary_rows)}'
            + (f'；全部 {len(rows)} 行中成功 '
               f'{sum(1 for r in rows if r["ok"])} 行' if len(route_modes) > 1 else ''))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if server is not None:
            log('关闭 coverage_server ...')
            server.stop()
    return 0


def _worth_example(row, rows, k):
    """挑代表性样本：面积最大 / 越界最狠 / 耗时最长，各挑一点。"""
    chosen = [r for r in rows if r.get('_example')]
    if len(chosen) >= k:
        return False
    keys = ['area_ha', 'oob_max_dist_m', 'planning_time_s']
    for key in keys:
        vals = [r.get(key, 0.0) for r in rows if r['ok']]
        if not vals:
            continue
        best = max(vals)
        if row.get(key, 0.0) >= best:
            return True
    return len(chosen) < k


if __name__ == '__main__':
    sys.exit(main())
