#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# =====================================================
# 400 m 标准跑道参考路径参数
# =====================================================

TRACK_LENGTH = 400.0       # 目标路径长度，m
STRAIGHT_LENGTH = 84.39    # 每段直道长度，m
CURVE_RADIUS = 36.8        # 测量线弯道半径，m
POINT_INTERVAL = 0.5       # 路径点间距，m

# True：逆时针；False：顺时针
COUNTERCLOCKWISE = True

OUTPUT_DIR = Path.home() / "agri_robot" / "robot_sim" / "output"


def generate_track():
    """
    生成两段直道 + 两段半圆弯道的闭合跑道。

    跑道底边位于 y=0。
    左右弯道的圆心分别为：
        (R, R)
        (L+R, R)

    返回：
        x, y：路径坐标
        yaw：路径航向角，rad
        distance：累计路径距离，m
    """

    L = STRAIGHT_LENGTH
    R = CURVE_RADIUS

    # 先构建按行驶顺序排列的稠密路径
    dense_points = []

    def add_line(x0, y0, x1, y1):
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(2, math.ceil(length / 0.05))

        for i in range(n):
            t = i / n
            dense_points.append((
                x0 + t * (x1 - x0),
                y0 + t * (y1 - y0)
            ))

    def add_arc(cx, cy, radius, start_angle, end_angle):
        arc_length = radius * abs(end_angle - start_angle)
        n = max(2, math.ceil(arc_length / 0.05))

        for i in range(n):
            angle = start_angle + (
                end_angle - start_angle
            ) * i / n

            dense_points.append((
                cx + radius * math.cos(angle),
                cy + radius * math.sin(angle)
            ))

    # 从左侧圆弧与底边的连接点出发，逆时针行驶
    # 底部直道 -> 右半圆 -> 顶部直道 -> 左半圆

    add_line(R, 0, R + L, 0)

    add_arc(
        R + L, R, R,
        -math.pi / 2, math.pi / 2
    )

    add_line(R + L, 2 * R, R, 2 * R)

    add_arc(
        R, R, R,
        math.pi / 2, 3 * math.pi / 2
    )

    # 闭合路径
    dense_points.append(dense_points[0])

    dense = np.asarray(dense_points, dtype=float)

    # 将路径起点平移到坐标原点 (0, 0)
    dense[:, 0] -= dense[0, 0]
    dense[:, 1] -= dense[0, 1]

    # 计算稠密路径的累计弧长
    segment_lengths = np.linalg.norm(
        np.diff(dense, axis=0), axis=1
    )

    arc_length = np.concatenate((
        [0.0],
        np.cumsum(segment_lengths)
    ))

    actual_length = arc_length[-1]

    # 按目标间距重采样。
    # 最后一个点与第一个点重合，形成闭合路径。
    sample_distance = np.arange(
        0.0, actual_length, POINT_INTERVAL
    )

    if (
        len(sample_distance) == 0
        or not math.isclose(
            sample_distance[-1],
            actual_length,
            abs_tol=1e-9
        )
    ):
        sample_distance = np.append(
            sample_distance, actual_length
        )

    x = np.interp(sample_distance, arc_length, dense[:, 0])
    y = np.interp(sample_distance, arc_length, dense[:, 1])

    # 确保终点与起点完全重合
    x[-1] = x[0]
    y[-1] = y[0]

    # 根据前后路径点计算航向角
    yaw = np.zeros(len(x))

    for i in range(len(x) - 1):
        dx = x[i + 1] - x[i]
        dy = y[i + 1] - y[i]

        if math.hypot(dx, dy) > 1e-9:
            yaw[i] = math.atan2(dy, dx)
        else:
            yaw[i] = yaw[i - 1] if i > 0 else 0.0

    # 终点航向角与起点保持一致
    yaw[-1] = yaw[0]

    # 根据实际输出点计算累计距离
    ds = np.hypot(np.diff(x), np.diff(y))
    distance = np.concatenate(([0.0], np.cumsum(ds)))

    if not COUNTERCLOCKWISE:
        x = x[::-1].copy()
        y = y[::-1].copy()

        # 重新计算反向路径的航向角和累计距离
        yaw = np.zeros(len(x))

        for i in range(len(x) - 1):
            dx = x[i + 1] - x[i]
            dy = y[i + 1] - y[i]

            if math.hypot(dx, dy) > 1e-9:
                yaw[i] = math.atan2(dy, dx)

        yaw[-1] = yaw[0]

        ds = np.hypot(np.diff(x), np.diff(y))
        distance = np.concatenate(([0.0], np.cumsum(ds)))

    return x, y, yaw, distance, actual_length


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    x, y, yaw, distance, geometric_length = generate_track()

    # -------------------------------------------------
    # 1. 保存路径点 CSV
    # -------------------------------------------------

    csv_path = OUTPUT_DIR / "stadium_path.csv"

    with csv_path.open(
        "w", newline="", encoding="utf-8"
    ) as f:
        writer = csv.writer(f)

        writer.writerow([
            "index",
            "x_m",
            "y_m",
            "yaw_rad",
            "distance_m"
        ])

        for i in range(len(x)):
            writer.writerow([
                i,
                f"{x[i]:.4f}",
                f"{y[i]:.4f}",
                f"{yaw[i]:.6f}",
                f"{distance[i]:.4f}"
            ])

    # -------------------------------------------------
    # 2. 保存参数 JSON
    # -------------------------------------------------

    params = {
        "path_type": "standard_400m_athletics_track",
        "coordinate_frame": "map",
        "coordinate_unit": "m",
        "target_track_length_m": TRACK_LENGTH,
        "straight_length_m": STRAIGHT_LENGTH,
        "curve_radius_m": CURVE_RADIUS,
        "point_interval_m": POINT_INTERVAL,
        "direction": (
            "counterclockwise"
            if COUNTERCLOCKWISE
            else "clockwise"
        ),
        "point_count": int(len(x)),
        "closed_path": True,
        "start_x_m": float(x[0]),
        "start_y_m": float(y[0]),
        "geometric_length_m": float(geometric_length),
        "note": (
            "Ideal stadium-shaped reference path; "
            "dimensions use an approximate measurement-line radius."
        )
    }

    json_path = OUTPUT_DIR / "stadium_params.json"

    with json_path.open("w", encoding="utf-8") as f:
        json.dump(params, f, indent=4, ensure_ascii=False)

    # -------------------------------------------------
    # 3. 绘制跑道及参考路径
    # -------------------------------------------------

    fig, ax = plt.subplots(figsize=(13, 6))

    R = CURVE_RADIUS
    L = STRAIGHT_LENGTH

    # 跑道外形示意：中心参考路径位于跑道中央
    theta_right = np.linspace(-math.pi / 2, math.pi / 2, 200)
    theta_left = np.linspace(math.pi / 2, 3 * math.pi / 2, 200)

    # 中心线轮廓
    right_x = R + L + R * np.cos(theta_right)
    right_y = R + R * np.sin(theta_right)

    left_x = R + R * np.cos(theta_left)
    left_y = R + R * np.sin(theta_left)

    boundary_x = np.concatenate([
        [R, R + L],
        right_x,
        [R,],
        left_x
    ])

    boundary_y = np.concatenate([
        [0, 0],
        right_y,
        [2 * R],
        left_y
    ])



    ax.plot(
        x, y,
        linewidth=2.2,
        label="Reference path"
    )

    ax.scatter(
        x[0], y[0],
        s=70, marker="o",
        label="Start"
    )

    # 绘制方向箭头
    arrow_step = max(1, len(x) // 24)

    for i in range(0, len(x) - 1, arrow_step):
        ax.annotate(
            "",
            xy=(x[i + 1], y[i + 1]),
            xytext=(x[i], y[i]),
            arrowprops=dict(arrowstyle="->", linewidth=1.1)
        )

    ax.set_title("400 m Athletics Track - Reference Path")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()

    png_path = OUTPUT_DIR / "stadium_path.png"
    fig.savefig(png_path, dpi=200)
    plt.close(fig)

    # -------------------------------------------------
    # 4. 打印结果
    # -------------------------------------------------

    print("\n========== 跑道路径生成完成 ==========")
    print(f"直道长度：{STRAIGHT_LENGTH:.2f} m")
    print(f"弯道半径：{CURVE_RADIUS:.2f} m")
    print(f"几何路径长度：{geometric_length:.3f} m")
    print(f"目标长度：{TRACK_LENGTH:.2f} m")
    print(f"路径点数量：{len(x)}")
    print(f"路径点间距设置：{POINT_INTERVAL:.2f} m")
    print(f"行驶方向：{'逆时针' if COUNTERCLOCKWISE else '顺时针'}")
    print(f"\n路径图片：{png_path}")
    print(f"路径点 CSV：{csv_path}")
    print(f"参数 JSON：{json_path}")


if __name__ == "__main__":
    main()