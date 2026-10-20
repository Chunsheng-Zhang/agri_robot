#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
根据 waypoints.yaml 中的 GPS 经纬度生成地块边界参考路径。

运行后同时生成：
1. boundary_path.csv  —— 路径点 CSV 文件
2. boundary_path.png  —— 路径可视化图片
3. boundary_path.json —— 路径参数、边界点及完整路径点 JSON 文件

功能：
1. 读取 YAML 中的地块边界点。
2. 根据固定原点确定 UTM 投影分区。
3. 将经纬度转换为 UTM 平面坐标。
4. 转换到与 coordinate_converter.py 一致的局部坐标系。
5. 按照 YAML 中的点位顺序生成闭合边界路径。
6. 按指定间距采样。
7. 计算每个路径点的航向角和累计距离。
8. 检查多边形面积、重复边界点和非相邻边相交。
9. 同时保存 CSV、PNG 和 JSON。

注意：
当前生成的是地块边界路径，不是农田内部的完整覆盖作业路径。
"""

import csv
import json
import math
from pathlib import Path

import yaml
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from pyproj import CRS, Transformer


# ==========================================================
# 1. 文件路径
# ==========================================================

WAYPOINTS_FILE = (
    Path.home()
    / "agri_robot/robot_sim/src/agri_multi_point_nav/config/waypoints.yaml"
)

OUTPUT_DIR = Path.home() / "agri_robot/robot_sim/output"

OUTPUT_FILE = OUTPUT_DIR / "boundary_path.csv"
IMAGE_FILE = OUTPUT_DIR / "boundary_path.png"
JSON_FILE = OUTPUT_DIR / "boundary_path.json"


# ==========================================================
# 2. 路径参数
# ==========================================================

# 路径点之间的目标最大间距，单位 m
POINT_INTERVAL = 0.5

# 路径坐标系
COORDINATE_FRAME = "local_map"

# 坐标单位
COORDINATE_UNIT = "m"


# ==========================================================
# 3. 坐标原点
# ==========================================================

# 使用 target_1 作为局部坐标原点 (0, 0)
ORIGIN_LATITUDE = 36.6468527633281
ORIGIN_LONGITUDE = 117.021433066391


# ==========================================================
# 4. 检查经纬度
# ==========================================================

def valid_position(lat, lon):
    """检查经纬度是否合法。"""

    return (
        math.isfinite(lat)
        and math.isfinite(lon)
        and -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
    )


# ==========================================================
# 5. 确定 UTM 投影分区
# ==========================================================

def get_utm_epsg(lat, lon):
    """
    根据经纬度确定 UTM 分区和 EPSG 编号。

    北半球：EPSG = 32600 + zone
    南半球：EPSG = 32700 + zone
    """

    if not valid_position(lat, lon):
        raise ValueError(
            f"无效的经纬度：latitude={lat}, longitude={lon}"
        )

    zone = int((lon + 180.0) // 6.0) + 1
    zone = max(1, min(zone, 60))

    if lat >= 0.0:
        epsg = 32600 + zone
    else:
        epsg = 32700 + zone

    return zone, epsg


# ==========================================================
# 6. 创建坐标转换器
# ==========================================================

def create_coordinate_transformer():
    """
    创建 UTM 坐标转换器。

    输入：
        longitude, latitude

    输出：
        easting, northing
    """

    zone, epsg = get_utm_epsg(
        ORIGIN_LATITUDE,
        ORIGIN_LONGITUDE
    )

    transformer = Transformer.from_crs(
        CRS.from_epsg(4326),
        CRS.from_epsg(epsg),
        always_xy=True
    )

    origin_easting, origin_northing = transformer.transform(
        ORIGIN_LONGITUDE,
        ORIGIN_LATITUDE
    )

    print(f"UTM 分区：{zone}")
    print(f"EPSG：{epsg}")

    print(
        "局部坐标原点："
        f"latitude={ORIGIN_LATITUDE:.12f}, "
        f"longitude={ORIGIN_LONGITUDE:.12f}"
    )

    return (
        transformer,
        origin_easting,
        origin_northing,
        zone,
        epsg
    )


# ==========================================================
# 7. 读取 YAML 中的地块边界
# ==========================================================

def load_waypoints():
    """读取地块边界经纬度，并转换为局部平面坐标。"""

    if not WAYPOINTS_FILE.exists():
        raise FileNotFoundError(
            f"找不到地块边界文件：{WAYPOINTS_FILE}"
        )

    with WAYPOINTS_FILE.open(
        "r",
        encoding="utf-8"
    ) as file:
        data = yaml.safe_load(file)

    if not isinstance(data, dict):
        raise ValueError("YAML 顶层必须是字典。")

    waypoints = data.get("waypoints")

    if not isinstance(waypoints, list):
        raise ValueError(
            "YAML 中必须包含 waypoints 列表。"
        )

    if len(waypoints) < 3:
        raise ValueError(
            "地块边界至少需要 3 个点。"
        )

    (
        transformer,
        origin_e,
        origin_n,
        zone,
        epsg
    ) = create_coordinate_transformer()

    result = []

    for index, waypoint in enumerate(waypoints):

        if not isinstance(waypoint, dict):
            raise ValueError(
                f"第 {index + 1} 个边界点格式错误。"
            )

        name = str(
            waypoint.get("name", f"point_{index + 1}")
        )

        try:
            lat = float(waypoint["latitude"])
            lon = float(waypoint["longitude"])

        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"边界点 {name} 缺少有效的经纬度。"
            ) from error

        if not valid_position(lat, lon):
            raise ValueError(
                f"边界点 {name} 的经纬度无效。"
            )

        # 经纬度转换为 UTM 坐标
        easting, northing = transformer.transform(
            lon,
            lat
        )

        # 转换为以固定 GPS 原点为基准的局部坐标
        x = easting - origin_e
        y = northing - origin_n

        result.append({
            "name": name,
            "latitude": lat,
            "longitude": lon,
            "x": x,
            "y": y
        })

    print("\n========== 地块边界点 ==========")

    for point in result:
        print(
            f"{point['name']}: "
            f"latitude={point['latitude']:.12f}, "
            f"longitude={point['longitude']:.12f}, "
            f"x={point['x']:.3f} m, "
            f"y={point['y']:.3f} m"
        )

    return result, zone, epsg


# ==========================================================
# 8. 检查地块边界
# ==========================================================

def validate_polygon(boundary_points):
    """检查边界点、面积及非相邻边是否相交。"""

    points = [
        (point["x"], point["y"])
        for point in boundary_points
    ]

    n = len(points)

    # 检查相邻点是否重合
    for i in range(n):

        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]

        if math.hypot(x2 - x1, y2 - y1) < 1e-4:
            raise ValueError(
                f"边界点 {i + 1} 与下一个点重合，"
                "请检查 YAML 中的点位。"
            )

    # 鞋带公式计算多边形面积
    area_twice = 0.0

    for i in range(n):

        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]

        area_twice += x1 * y2 - x2 * y1

    signed_area = area_twice / 2.0
    area = abs(signed_area)

    if area < 1e-4:
        raise ValueError(
            "地块边界面积接近零，请检查边界点顺序。"
        )

    # 检查线段是否相交
    def orientation(a, b, c):
        return (
            (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0])
        )

    def on_segment(a, b, p):
        eps = 1e-9

        return (
            min(a[0], b[0]) - eps
            <= p[0]
            <= max(a[0], b[0]) + eps
            and
            min(a[1], b[1]) - eps
            <= p[1]
            <= max(a[1], b[1]) + eps
        )

    def segments_intersect(a, b, c, d):
        o1 = orientation(a, b, c)
        o2 = orientation(a, b, d)
        o3 = orientation(c, d, a)
        o4 = orientation(c, d, b)

        eps = 1e-9

        if (
            ((o1 > eps and o2 < -eps)
             or (o1 < -eps and o2 > eps))
            and
            ((o3 > eps and o4 < -eps)
             or (o3 < -eps and o4 > eps))
        ):
            return True

        if abs(o1) <= eps and on_segment(a, b, c):
            return True

        if abs(o2) <= eps and on_segment(a, b, d):
            return True

        if abs(o3) <= eps and on_segment(c, d, a):
            return True

        if abs(o4) <= eps and on_segment(c, d, b):
            return True

        return False

    # 检查非相邻边
    for i in range(n):

        a = points[i]
        b = points[(i + 1) % n]

        for j in range(i + 1, n):

            # 相邻边有公共端点，跳过
            if (j + 1) % n == i:
                continue

            if (i + 1) % n == j:
                continue

            c = points[j]
            d = points[(j + 1) % n]

            if segments_intersect(a, b, c, d):
                raise ValueError(
                    f"地块边界存在相交边："
                    f"第 {i + 1} 条边与第 {j + 1} 条边。"
                )

    print(f"\n地块边界面积：{area:.3f} m²")

    return area


# ==========================================================
# 9. 生成闭合边界路径
# ==========================================================

def generate_boundary_path(boundary_points):
    """
    按照 YAML 中的点位顺序生成闭合路径。

    每条边独立采样，确保相邻路径点的间距
    不大于 POINT_INTERVAL（允许浮点误差）。
    """

    if POINT_INTERVAL <= 0.0:
        raise ValueError(
            "POINT_INTERVAL 必须大于 0。"
        )

    points = [
        (point["x"], point["y"])
        for point in boundary_points
    ]

    closed_points = points + [points[0]]

    sampled_points = []

    # 每条边独立采样
    for edge_index in range(len(closed_points) - 1):

        x0, y0 = closed_points[edge_index]
        x1, y1 = closed_points[edge_index + 1]

        edge_length = math.hypot(
            x1 - x0,
            y1 - y0
        )

        segments = max(
            1,
            math.ceil(edge_length / POINT_INTERVAL)
        )

        # 不添加当前边终点，避免角点重复
        for j in range(segments):

            t = j / segments

            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)

            sampled_points.append((x, y))

    # 添加闭合终点，坐标与起点相同
    sampled_points.append(points[0])

    path_data = []
    cumulative_distance = 0.0

    for i, (x, y) in enumerate(sampled_points):

        # 计算当前点指向下一个点的航向角
        if i < len(sampled_points) - 1:

            next_x, next_y = sampled_points[i + 1]

            dx = next_x - x
            dy = next_y - y

            yaw = math.atan2(dy, dx)

        else:
            # 闭合终点使用第一段的航向角
            first_x, first_y = sampled_points[0]
            second_x, second_y = sampled_points[1]

            yaw = math.atan2(
                second_y - first_y,
                second_x - first_x
            )

        path_data.append({
            "index": i,
            "x_m": x,
            "y_m": y,
            "yaw_rad": yaw,
            "distance_m": cumulative_distance
        })

        if i < len(sampled_points) - 1:

            next_x, next_y = sampled_points[i + 1]

            cumulative_distance += math.hypot(
                next_x - x,
                next_y - y
            )

    return path_data, cumulative_distance


# ==========================================================
# 10. 保存 CSV
# ==========================================================

def save_csv(path_data):
    """保存路径点 CSV。"""

    with OUTPUT_FILE.open(
        "w",
        newline="",
        encoding="utf-8"
    ) as file:

        writer = csv.writer(file)

        writer.writerow([
            "index",
            "x_m",
            "y_m",
            "yaw_rad",
            "distance_m"
        ])

        for point in path_data:

            writer.writerow([
                point["index"],
                f"{point['x_m']:.4f}",
                f"{point['y_m']:.4f}",
                f"{point['yaw_rad']:.6f}",
                f"{point['distance_m']:.4f}"
            ])

    print(f"CSV 文件已保存：{OUTPUT_FILE}")


# ==========================================================
# 11. 保存 JSON
# ==========================================================

def save_json(
    boundary_points,
    path_data,
    total_distance,
    area,
    zone,
    epsg
):
    """
    保存路径元数据、原始边界点及完整路径点。

    JSON 包含：
    - 路径类型
    - 坐标系与单位
    - GPS 固定原点
    - UTM 投影参数
    - 路径采样间距
    - 路径点数量
    - 闭合状态
    - 起点坐标
    - 路径总长度
    - 地块面积
    - 原始边界点
    - 完整参考路径点
    """

    json_data = {
        "path_type": "gps_boundary_path",

        "coordinate_frame": COORDINATE_FRAME,
        "coordinate_unit": COORDINATE_UNIT,

        "source_file": str(WAYPOINTS_FILE),

        "projection": {
            "type": "UTM",
            "zone": zone,
            "epsg": epsg,
            "origin_latitude": ORIGIN_LATITUDE,
            "origin_longitude": ORIGIN_LONGITUDE
        },

        "point_interval_m": POINT_INTERVAL,

        "point_count": len(path_data),

        "closed_path": True,

        "start_x_m": path_data[0]["x_m"],
        "start_y_m": path_data[0]["y_m"],

        "geometric_length_m": total_distance,

        "boundary_area_m2": area,

        "boundary_waypoints": [
            {
                "name": point["name"],
                "latitude": point["latitude"],
                "longitude": point["longitude"],
                "x_m": point["x"],
                "y_m": point["y"]
            }
            for point in boundary_points
        ],

        "path_points": [
            {
                "index": point["index"],
                "x_m": point["x_m"],
                "y_m": point["y_m"],
                "yaw_rad": point["yaw_rad"],
                "distance_m": point["distance_m"]
            }
            for point in path_data
        ],

        "note": (
            "Closed field boundary path generated from GPS "
            "latitude and longitude waypoints. Coordinates "
            "are expressed relative to the configured fixed "
            "UTM origin. This is a boundary path, not an "
            "internal full-coverage path."
        )
    }

    with JSON_FILE.open(
        "w",
        encoding="utf-8"
    ) as file:

        json.dump(
            json_data,
            file,
            indent=4,
            ensure_ascii=False
        )

        file.write("\n")

    print(f"JSON 文件已保存：{JSON_FILE}")


# ==========================================================
# 12. 绘制路径
# ==========================================================

def save_plot(boundary_points, path_data, area):
    """绘制地块边界和生成的参考路径。"""

    x = [point["x_m"] for point in path_data]
    y = [point["y_m"] for point in path_data]

    bx = [point["x"] for point in boundary_points]
    by = [point["y"] for point in boundary_points]

    bx_closed = bx + [bx[0]]
    by_closed = by + [by[0]]

    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(
        bx_closed,
        by_closed,
        linestyle="--",
        linewidth=1.5,
        label="Field boundary"
    )

    ax.plot(
        x,
        y,
        linewidth=2.0,
        label="Generated path"
    )

    ax.scatter(
        bx,
        by,
        s=55,
        zorder=3,
        label="Boundary waypoints"
    )

    for point in boundary_points:

        ax.annotate(
            point["name"],
            (point["x"], point["y"]),
            xytext=(5, 5),
            textcoords="offset points"
        )

    ax.scatter(
        x[0],
        y[0],
        s=100,
        marker="o",
        zorder=4,
        label="Start / End"
    )

    # 绘制行驶方向箭头
    arrow_interval = 2.0
    next_arrow_distance = arrow_interval

    for i in range(len(path_data) - 1):

        p1 = path_data[i]
        p2 = path_data[i + 1]

        if p2["distance_m"] < next_arrow_distance:
            continue

        dx = p2["x_m"] - p1["x_m"]
        dy = p2["y_m"] - p1["y_m"]

        if math.hypot(dx, dy) > 1e-9:

            ax.annotate(
                "",
                xy=(p2["x_m"], p2["y_m"]),
                xytext=(p1["x_m"], p1["y_m"]),
                arrowprops={
                    "arrowstyle": "->",
                    "linewidth": 1.0
                }
            )

        next_arrow_distance = (
            p2["distance_m"] + arrow_interval
        )

    ax.set_title(
        f"Field Boundary Path (Area = {area:.2f} m²)"
    )

    ax.set_xlabel("Local X (m)")
    ax.set_ylabel("Local Y (m)")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()

    fig.tight_layout()

    fig.savefig(
        IMAGE_FILE,
        dpi=200,
        bbox_inches="tight"
    )

    plt.close(fig)

    print(f"路径图片已保存：{IMAGE_FILE}")


# ==========================================================
# 13. 主程序
# ==========================================================

def main():

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    print("\n========== 经纬度地块路径生成 ==========")
    print(f"边界文件：{WAYPOINTS_FILE}")

    # 读取边界点并转换为局部坐标
    boundary_points, zone, epsg = load_waypoints()

    # 检查地块边界
    area = validate_polygon(boundary_points)

    # 生成闭合路径
    path_data, total_distance = generate_boundary_path(
        boundary_points
    )

    # 保存 CSV
    save_csv(path_data)

    # 保存 PNG
    save_plot(
        boundary_points,
        path_data,
        area
    )

    # 保存 JSON
    save_json(
        boundary_points=boundary_points,
        path_data=path_data,
        total_distance=total_distance,
        area=area,
        zone=zone,
        epsg=epsg
    )

    print("\n========== 路径生成完成 ==========")

    print(f"边界点数量：{len(boundary_points)}")
    print(f"路径点数量：{len(path_data)}")
    print(f"目标点间距：{POINT_INTERVAL:.2f} m")
    print(f"地块面积：{area:.3f} m²")
    print(f"闭合路径总长度：{total_distance:.3f} m")

    print("\n生成的文件：")
    print(f"CSV ：{OUTPUT_FILE}")
    print(f"PNG ：{IMAGE_FILE}")
    print(f"JSON：{JSON_FILE}")


if __name__ == "__main__":
    main()