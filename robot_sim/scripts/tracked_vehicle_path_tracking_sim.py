#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""简化履带式农机路径跟踪仿真：Pure Pursuit + 差速履带运动学模型。"""
import csv, json, math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 可修改参数
INPUT_CSV = Path.home() / "robot_sim/outputstadium_path.csv"
OUTPUT_DIR = Path.home() / "robot_sim/output/simulation"
DT = 0.05
SIM_TIME = 460.0
SPEED = 1.0
LOOKAHEAD = 2.0
MAX_YAW_RATE = 0.8
START_X_OFFSET = -1.0
START_Y_OFFSET = 1.0
START_YAW_OFFSET_DEG = 8.0
MIN_TRAVEL_DISTANCE = 390.0
END_DISTANCE_TOLERANCE = 1.5

def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def load_path(file):
    xs, ys, yaws = [], [], []
    with file.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {"x_m", "y_m", "yaw_rad"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"CSV 必须包含列：{required}")
        for r in reader:
            xs.append(float(r["x_m"]))
            ys.append(float(r["y_m"]))
            yaws.append(float(r["yaw_rad"]))
    p = np.column_stack((xs, ys))
    yaw = np.asarray(yaws)
    if len(p) > 1 and np.linalg.norm(p[-1] - p[0]) < 1e-6:
        p, yaw = p[:-1], yaw[:-1]
    if len(p) < 10:
        raise ValueError("路径点太少")
    return p, yaw

def nearest_index(position, path, prev, window=80):
    n = len(path)
    ids = [(prev + k) % n for k in range(-3, window + 1)]
    d2 = np.sum((path[ids] - position) ** 2, axis=1)
    j = int(np.argmin(d2))
    return ids[j]

def main():
    if not INPUT_CSV.exists():
        raise FileNotFoundError(f"找不到 {INPUT_CSV}；请先运行 path_generator.py")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    path, _ = load_path(INPUT_CSV)
    n = len(path)
    x, y = path[0, 0] + START_X_OFFSET, path[0, 1] + START_Y_OFFSET

    # 初始参考切线方向
    tangent = path[1] - path[0]
    yaw = wrap_angle(math.atan2(tangent[1], tangent[0]) +
                     math.radians(START_YAW_OFFSET_DEG))

    xs, ys, yaws, ctes, hes, times = [], [], [], [], [], []
    near = 0
    travelled = 0.0
    finished = False

    for step in range(int(SIM_TIME / DT)):
        t = step * DT
        pos = np.array([x, y])
        near = nearest_index(pos, path, near)
        nxt = (near + 1) % n
        tangent = path[nxt] - path[near]
        ref_yaw = math.atan2(tangent[1], tangent[0])

        # 有符号横向误差：正值代表参考切线左侧
        dx, dy = x - path[near, 0], y - path[near, 1]
        cte = -math.sin(ref_yaw) * dx + math.cos(ref_yaw) * dy
        he = wrap_angle(ref_yaw - yaw)

        # 沿路径前视 LOOKAHEAD 米寻找目标点
        target_idx, acc = near, 0.0
        while acc < LOOKAHEAD:
            nxt = (target_idx + 1) % n
            acc += float(np.linalg.norm(path[nxt] - path[target_idx]))
            target_idx = nxt
            if target_idx == near:
                break
        target = path[target_idx]
        alpha = wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)
        curvature = 2.0 * math.sin(alpha) / max(LOOKAHEAD, 1e-3)
        omega = float(np.clip(SPEED * curvature, -MAX_YAW_RATE, MAX_YAW_RATE))

        # 简化差速履带运动学：x_dot=v*cos(yaw), y_dot=v*sin(yaw)
        x += SPEED * math.cos(yaw) * DT
        y += SPEED * math.sin(yaw) * DT
        yaw = wrap_angle(yaw + omega * DT)
        travelled += SPEED * DT

        xs.append(x); ys.append(y); yaws.append(yaw)
        ctes.append(cte); hes.append(he); times.append(t)

        if travelled >= MIN_TRAVEL_DISTANCE and math.hypot(
            x - path[0, 0], y - path[0, 1]
        ) <= END_DISTANCE_TOLERANCE:
            finished = True
            break

    xs, ys = np.asarray(xs), np.asarray(ys)
    yaws, ctes, hes, times = map(np.asarray, (yaws, ctes, hes, times))
    abs_cte, abs_he = np.abs(ctes), np.abs(hes)

    data_file = OUTPUT_DIR / "simulation_data.csv"
    with data_file.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "x_m", "y_m", "yaw_rad",
                    "cross_track_error_m", "heading_error_rad"])
        for row in zip(times, xs, ys, yaws, ctes, hes):
            w.writerow([f"{v:.6f}" for v in row])

    metrics = {
        "model": "differential_track_kinematic_model",
        "controller": "pure_pursuit",
        "dt_s": DT, "speed_mps": SPEED, "lookahead_m": LOOKAHEAD,
        "max_yaw_rate_radps": MAX_YAW_RATE,
        "simulation_duration_s": float(times[-1]),
        "travelled_distance_m": float(travelled),
        "completed_lap_near_start": bool(finished),
        "sample_count": int(len(times)),
        "cross_track_error_mean_abs_m": float(np.mean(abs_cte)),
        "cross_track_error_rms_m": float(np.sqrt(np.mean(ctes ** 2))),
        "cross_track_error_max_abs_m": float(np.max(abs_cte)),
        "heading_error_mean_abs_deg": float(np.degrees(np.mean(abs_he))),
        "heading_error_rms_deg": float(np.degrees(np.sqrt(np.mean(hes ** 2)))),
        "heading_error_max_abs_deg": float(np.degrees(np.max(abs_he))),
    }
    with (OUTPUT_DIR / "simulation_metrics.json").open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=4, ensure_ascii=False)

    fig, ax = plt.subplots(figsize=(12, 7))
    ax.plot(path[:, 0], path[:, 1], label="Reference path", linewidth=1.8)
    ax.plot(xs, ys, label="Simulated trajectory", linewidth=1.3)
    ax.scatter(path[0, 0], path[0, 1], marker="o", s=60, label="Reference start")
    ax.scatter(xs[0], ys[0], marker="x", s=60, label="Vehicle start")
    ax.set_title("Tracked Vehicle Path Tracking Simulation")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4); ax.legend()
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "trajectory_comparison.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(times, ctes, linewidth=1.0)
    axes[0].axhline(0, linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("Cross-track error (m)")
    axes[0].set_title("Tracking Errors")
    axes[0].grid(True, linestyle="--", alpha=0.4)
    axes[1].plot(times, np.degrees(hes), linewidth=1.0)
    axes[1].axhline(0, linestyle="--", linewidth=0.8)
    axes[1].set_xlabel("Time (s)"); axes[1].set_ylabel("Heading error (deg)")
    axes[1].grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "tracking_errors.png", dpi=180)
    plt.close(fig)

    print("\n========== 路径跟踪仿真完成 ==========")
    print(f"行驶距离：{travelled:.2f} m；仿真时间：{times[-1]:.2f} s")
    print(f"是否回到起点附近：{'是' if finished else '否'}")
    print(f"横向误差 MAE：{metrics['cross_track_error_mean_abs_m']:.3f} m")
    print(f"横向误差 RMS：{metrics['cross_track_error_rms_m']:.3f} m")
    print(f"横向误差最大值：{metrics['cross_track_error_max_abs_m']:.3f} m")
    print(f"航向误差 MAE：{metrics['heading_error_mean_abs_deg']:.2f}°")
    print(f"航向误差 RMS：{metrics['heading_error_rms_deg']:.2f}°")
    print(f"输出目录：{OUTPUT_DIR}")

if __name__ == "__main__":
    main()
