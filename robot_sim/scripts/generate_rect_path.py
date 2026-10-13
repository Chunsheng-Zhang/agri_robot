#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ==============================
# 矩形路径参数
# ==============================

WIDTH = 10.0           # 宽度，m
LENGTH = 20.0          # 长度，m
POINT_INTERVAL = 0.5   # 路径点间距，m

OUTPUT_DIR = Path.home() / "agri_robot" / "robot_sim" / "output"

OUTPUT_FILE = OUTPUT_DIR / "rect_10x20_path.csv"
IMAGE_FILE = OUTPUT_DIR / "rect_10x20_path.png"


def generate_rectangle():
    """
    生成 10 m × 20 m 的闭合矩形路径。

    起点：(0, 0)
    行驶方向：逆时针

    路径顺序：
        (0, 0)
        -> (20, 0)
        -> (20, 10)
        -> (0, 10)
        -> (0, 0)
    """

    corners = [
        (0.0, 0.0),
        (LENGTH, 0.0),
        (LENGTH, WIDTH),
        (0.0, WIDTH),
        (0.0, 0.0),
    ]

    points = []

    # 沿每条边采样
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]

        edge_length = math.hypot(
            x1 - x0,
            y1 - y0
        )

        segments = max(
            1,
            round(edge_length / POINT_INTERVAL)
        )

        for j in range(segments):
            t = j / segments

            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)

            points.append((x, y))

    # 添加闭合终点
    points.append((0.0, 0.0))

    # 计算航向角和累计距离
    path_data = []
    distance = 0.0

    for i, (x, y) in enumerate(points):

        if i < len(points) - 1:
            next_x, next_y = points[i + 1]

            dx = next_x - x
            dy = next_y - y

            yaw = math.atan2(dy, dx)

            # 当前点的累计距离先记录，
            # 再累加到下一个点的距离
        else:
            yaw = 0.0

        path_data.append(
            (i, x, y, yaw, distance)
        )

        if i < len(points) - 1:
            distance += math.hypot(
                points[i + 1][0] - x,
                points[i + 1][1] - y
            )

    return path_data


def save_csv(path_data):
    """保存路径点 CSV。"""

    with OUTPUT_FILE.open(
        "w",
        newline="",
        encoding="utf-8"
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            "index",
            "x_m",
            "y_m",
            "yaw_rad",
            "distance_m"
        ])

        for index, x, y, yaw, distance in path_data:
            writer.writerow([
                index,
                f"{x:.4f}",
                f"{y:.4f}",
                f"{yaw:.6f}",
                f"{distance:.4f}"
            ])


def save_plot(path_data):
    """绘制矩形参考路径并保存图片。"""

    x = [row[1] for row in path_data]
    y = [row[2] for row in path_data]

    fig, ax = plt.subplots(figsize=(12, 7))

    # 绘制参考路径
    ax.plot(
        x,
        y,
        linewidth=2.0,
        label="Reference path"
    )

    # 标记起点和终点
    ax.scatter(
        x[0],
        y[0],
        s=90,
        marker="o",
        label="Start / End",
        zorder=3
    )

    # 显示路径点
    ax.scatter(
        x,
        y,
        s=8,
        label="Path points",
        zorder=2
    )

    # 绘制行驶方向箭头
    arrow_step = max(
        1,
        int(2.0 / POINT_INTERVAL)
    )

    for i in range(0, len(x) - 1, arrow_step):
        dx = x[i + 1] - x[i]
        dy = y[i + 1] - y[i]

        # 跳过闭合点附近的零长度线段
        if math.hypot(dx, dy) < 1e-9:
            continue

        ax.annotate(
            "",
            xy=(x[i + 1], y[i + 1]),
            xytext=(x[i], y[i]),
            arrowprops={
                "arrowstyle": "->",
                "linewidth": 1.0
            }
        )

    # 标注矩形尺寸
    ax.text(
        LENGTH / 2,
        -0.8,
        f"Length = {LENGTH:.1f} m",
        ha="center",
        va="top"
    )

    ax.text(
        LENGTH + 0.8,
        WIDTH / 2,
        f"Width = {WIDTH:.1f} m",
        ha="left",
        va="center",
        rotation=90
    )

    # 起点坐标
    ax.annotate(
        "(0, 0)",
        xy=(0, 0),
        xytext=(0.6, 0.6),
        textcoords="data"
    )

    ax.set_title("10 m x 20 m Rectangular Reference Path")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    # 保持 X、Y 轴比例一致，避免矩形失真
    ax.set_aspect("equal", adjustable="box")

    ax.set_xlim(-2.0, LENGTH + 2.5)
    ax.set_ylim(-2.0, WIDTH + 2.0)

    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()
    fig.tight_layout()

    fig.savefig(
        IMAGE_FILE,
        dpi=200,
        bbox_inches="tight"
    )

    plt.close(fig)


def main():
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    path_data = generate_rectangle()

    # 保存 CSV
    save_csv(path_data)

    # 保存图片
    save_plot(path_data)

    print("\n========== 矩形路径生成完成 ==========")
    print(f"矩形尺寸：{LENGTH:.1f} m × {WIDTH:.1f} m")
    print(f"路径总长度：{2 * (LENGTH + WIDTH):.1f} m")
    print(f"路径点间距：{POINT_INTERVAL:.2f} m")
    print(f"路径点数量：{len(path_data)}")
    print("起点：(0.0, 0.0)")
    print("行驶方向：逆时针")
    print(f"\nCSV 文件：{OUTPUT_FILE}")
    print(f"路径图片：{IMAGE_FILE}")


if __name__ == "__main__":
    main()