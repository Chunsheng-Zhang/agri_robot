#!/usr/bin/env python3
# Copyright 2024 Chunsheng Zhang
# 覆盖路径规划测试工具（不执行路径）

import argparse
import csv
import datetime
import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from opennav_coverage_msgs.action import ComputeCoveragePath
from shapely.geometry import Point, Polygon as ShapelyPolygon
from shapely.prepared import prep

HOME = os.path.expanduser("~")
DEFAULT_WKT = os.path.join(HOME, "open_navigation_ws/src/fields2Benchmark_Dataset/ee_field_6.wkt")
sys.path.append(os.path.join(HOME, "open_navigation_ws/src/navigation_ws/gps_utm/src"))
from wkt_to_utm import wkt_to_utm

SAMPLE_STEP = 0.1


def build_field(wkt_path):
    field, offset = wkt_to_utm(wkt_path)
    return field, offset


def sample_line(p0, p1, step=SAMPLE_STEP):
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(2, int(math.ceil(dist / step)) + 1)
    return [(p0[0] + (p1[0] - p0[0]) * i / (n - 1),
             p0[1] + (p1[1] - p0[1]) * i / (n - 1)) for i in range(n)]


def classify(points, poly, prepared):
    results = []
    for pt in points:
        p = Point(pt)
        inside = prepared.contains(p) or prepared.touches(p)
        dist = 0.0 if inside else p.distance(poly)
        results.append((pt[0], pt[1], inside, dist))
    return results


def save_csv(path, header, rows):
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def save_field(output_dir, field):
    path = os.path.join(output_dir, "field.csv")
    save_csv(path, ["x", "y"], field)
    return path


def save_swaths(output_dir, coverage_path):
    path = os.path.join(output_dir, "swaths.csv")
    rows = []
    # opennav_coverage_msgs/Swath 只有 start/end 两个 Point32，不是带 poses 的路径
    for i, swath in enumerate(coverage_path.swaths):
        rows.append([i, 0, swath.start.x, swath.start.y])
        rows.append([i, 1, swath.end.x, swath.end.y])
    save_csv(path, ["swath_index", "point_index", "x", "y"], rows)
    return path


def save_turns(output_dir, coverage_path):
    path = os.path.join(output_dir, "turns.csv")
    rows = []
    # turns 是 nav_msgs/Path[]，没有 type 字段
    for i, turn in enumerate(coverage_path.turns):
        for j, pt in enumerate(turn.poses):
            rows.append([i, j, pt.pose.position.x, pt.pose.position.y])
    save_csv(path, ["turn_index", "point_index", "x", "y"], rows)
    return path


def save_nav_path(output_dir, nav_path_msg):
    path = os.path.join(output_dir, "nav_path.csv")
    rows = []
    for i, pt in enumerate(nav_path_msg.poses):
        rows.append([i, pt.pose.position.x, pt.pose.position.y])
    save_csv(path, ["point_index", "x", "y"], rows)
    return path


def save_summary(output_dir, field, coverage_path, nav_path_msg, classifications):
    path = os.path.join(output_dir, "summary.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("=== 覆盖路径规划统计 ===\n\n")
        f.write(f"地块边界点数：{len(field)}\n")
        f.write(f"作业段数量：{len(coverage_path.swaths)}\n")
        f.write(f"转弯段数量：{len(coverage_path.turns)}\n")
        f.write(f"导航路径点数：{len(nav_path_msg.poses)}\n\n")
        total_swath_len = len(coverage_path.swaths) * 2  # 每个 Swath 只有起止两点
        total_turn_len = sum(len(t.poses) for t in coverage_path.turns)
        f.write(f"作业段总点数：{total_swath_len}\n")
        f.write(f"转弯段总点数：{total_turn_len}\n\n")
        outside_points = [p for p in classifications if not p[2]]
        f.write(f"越界点数：{len(outside_points)}\n")
        if outside_points:
            max_dist = max(p[3] for p in outside_points)
            f.write(f"最大越界距离：{max_dist:.3f} m\n")
        f.write("\n=== 完成 ===\n")
    return path


def plot_plan(output_dir, field, coverage_path, nav_path_msg, classifications):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    field_x = [p[0] for p in field] + [field[0][0]]
    field_y = [p[1] for p in field] + [field[0][1]]
    ax.plot(field_x, field_y, "-", color="black", linewidth=1.5, label="Field Boundary")
    ax.fill(field_x, field_y, color="lightgray", alpha=0.3)

    poly = ShapelyPolygon(field)
    prepared = prep(poly)
    for i, swath in enumerate(coverage_path.swaths):
        pts = [(swath.start.x, swath.start.y), (swath.end.x, swath.end.y)]
        xs, ys = zip(*pts)
        all_inside = all(prepared.contains(Point(x, y)) or prepared.touches(Point(x, y))
                        for x, y in pts)
        color = "green" if all_inside else "red"
        ax.plot(xs, ys, "-", color=color, linewidth=2, alpha=0.7)

    for i, turn in enumerate(coverage_path.turns):
        pts = [(p.pose.position.x, p.pose.position.y) for p in turn.poses]
        if not pts:
            continue
        xs, ys = zip(*pts)
        ax.plot(xs, ys, "--", color="blue", linewidth=1.5, alpha=0.5)

    if nav_path_msg.poses:
        path_x = [p.pose.position.x for p in nav_path_msg.poses]
        path_y = [p.pose.position.y for p in nav_path_msg.poses]
        ax.plot(path_x, path_y, "-", color="orange", linewidth=1, alpha=0.8, label="Navigation Path")
        ax.plot(nav_path_msg.poses[0].pose.position.x, nav_path_msg.poses[0].pose.position.y,
               "o", color="red", markersize=8, label="Start")

    outside_pts = [p for p in classifications if not p[2]]
    if outside_pts:
        out_x = [p[0] for p in outside_pts]
        out_y = [p[1] for p in outside_pts]
        ax.plot(out_x, out_y, "x", color="red", markersize=4, label=f"Outside ({len(outside_pts)} pts)")

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Coverage Path Planning Result", fontsize=14)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(output_dir, "coverage_plan.png")
    plt.savefig(path, dpi=150, bbox_inches="tight")
    plt.close()
    return path


class CoveragePlannerTester(Node):
    def __init__(self):
        super().__init__(node_name="coverage_planner_tester")
        self.coverage_client = ActionClient(self, ComputeCoveragePath, "compute_coverage_path")

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def toCoordinates(self, field):
        # ComputeCoveragePath 的 goal 字段是 opennav_coverage_msgs/Coordinates[]，
        # 不是 geometry_msgs/Polygon[]（后者只在 NavigateCompleteCoverage 里用）。
        # 这里的转换与 BT 节点的 compute_complete_coverage_path.cpp 一致。
        from opennav_coverage_msgs.msg import Coordinate, Coordinates
        poly = Coordinates()
        for coord in field:
            pt = Coordinate()
            pt.axis1 = coord[0]
            pt.axis2 = coord[1]
            poly.coordinates.append(pt)
        return poly

    def compute_coverage(self, field):
        print("等待 ComputeCoveragePath action server...")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            print("action server 未就绪，等待中...")

        goal_msg = ComputeCoveragePath.Goal()
        goal_msg.frame_id = "map"
        goal_msg.polygons.append(self.toCoordinates(field))
        goal_msg.generate_headland = True
        goal_msg.generate_route = True
        goal_msg.generate_path = True
        # 注意：地头宽度、作业宽度、机器人宽度、最小转弯半径都不在 goal 里，
        # 由 coverage_server 节点参数提供（default_headland_width / operation_width /
        # robot_width / min_turning_radius），与 demo_coverage.py 经 BT 下发时一致。

        print(f"发送覆盖路径规划请求，地块边界点数：{len(field)}...")
        send_goal_future = self.coverage_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            print("规划请求被拒绝！")
            return None

        print("规划请求已接受，等待结果...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if result_future.result():
            result = result_future.result().result
            if result.error_code != ComputeCoveragePath.Result.NONE:
                print(f"路径规划失败，error_code={result.error_code}")
                return None
            print("路径规划完成！")
            return result
        else:
            print("路径规划失败！")
            return None


def main():
    parser = argparse.ArgumentParser(description="测试覆盖路径规划（不执行）")
    parser.add_argument("--wkt", type=str, default=DEFAULT_WKT, help="WKT 文件路径")
    parser.add_argument("--output", type=str, default=None, help="输出目录")
    args = parser.parse_args()

    if not os.path.exists(args.wkt):
        print(f"错误：找不到 WKT 文件 {args.wkt}")
        return

    if args.output:
        output_dir = args.output
    else:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join(os.path.dirname(__file__), "results", timestamp)

    os.makedirs(output_dir, exist_ok=True)
    print(f"输出目录：{output_dir}\n")

    rclpy.init()
    tester = CoveragePlannerTester()

    try:
        print(f"加载 WKT 文件：{args.wkt}")
        field, utm_offset = build_field(args.wkt)
        print(f"地块边界点数：{len(field)}")
        print(f"UTM 原点偏移：x={utm_offset[0]:.3f}, y={utm_offset[1]:.3f}\n")

        save_field(output_dir, field)
        result = tester.compute_coverage(field)

        if result is None:
            print("路径规划失败，退出")
            return

        print("\n保存结果...")
        coverage_path = result.coverage_path
        save_swaths(output_dir, coverage_path)
        save_turns(output_dir, coverage_path)
        save_nav_path(output_dir, result.nav_path)

        poly = ShapelyPolygon(field)
        prepared = prep(poly)
        all_points = []
        for swath in coverage_path.swaths:
            all_points.extend(sample_line((swath.start.x, swath.start.y),
                                          (swath.end.x, swath.end.y)))
        for turn in coverage_path.turns:
            pts = [(p.pose.position.x, p.pose.position.y) for p in turn.poses]
            for i in range(len(pts) - 1):
                all_points.extend(sample_line(pts[i], pts[i+1]))
        classifications = classify(all_points, poly, prepared)

        save_summary(output_dir, field, coverage_path, result.nav_path, classifications)
        plot_plan(output_dir, field, coverage_path, result.nav_path, classifications)

        print(f"\n✅ 路径规划测试完成！")
        print(f"输出文件:")
        print(f"  - field.csv: 地块边界")
        print(f"  - swaths.csv: 作业段")
        print(f"  - turns.csv: 转弯段")
        print(f"  - nav_path.csv: 导航路径")
        print(f"  - summary.txt: 统计摘要")
        print(f"  - coverage_plan.png: 可视化图")

    finally:
        tester.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
