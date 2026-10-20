#!/usr/bin/env python3

import json
import os
from shapely import wkt
from shapely.ops import transform
from pyproj import Transformer


def utm_epsg(lon, lat):
    """按经纬度算 UTM 带号对应的 EPSG。

    WGS84 的 UTM 带号 = floor((lon + 180) / 6) + 1；北半球 EPSG = 32600 + 带号，
    南半球 = 32700 + 带号。

    原来 target_crs 写死 EPSG:32635（爱沙尼亚所在的 35N），对 lt_（立陶宛 34N）、
    nl_（荷兰 31/32N）的地块等于**投到隔壁带**：网格北和真北差一个子午线收敛角
    （荷兰约 17°），长度还会按 (1 + x²/2R²) 放大百分之几。作业方向（swath angle）
    正是覆盖测试的核心变量，不能带这个偏差。
    """
    zone = int((lon + 180.0) // 6.0) + 1
    return f"EPSG:{32600 + zone if lat >= 0 else 32700 + zone}"


def wkt_to_utm_rings(wkt_path, source_crs="EPSG:4326", target_crs=None):
    """
    读取 WKT 文件，把 Polygon（含内环）从 WGS84 转换到 UTM。

    Parameters
    ----------
    wkt_path : str
        WKT 文件路径

    source_crs : str
        输入坐标系，默认 WGS84 EPSG:4326

    target_crs : str 或 None
        输出坐标系；None = 按地块中心自动选带（推荐）

    Returns
    -------
    rings : list
        [外边界, 内环1, 内环2, ...]，每个元素是 [[x1, y1], [x2, y2], ...]，
        全部平移到以第一个外边界点为原点。这个顺序就是 action goal 里
        `polygons` 的约定：第 0 个是外边界，后面每个是挖空一块。
    offset : (float, float)
        原点在 UTM 里的绝对坐标
    epsg : str
        实际使用的目标坐标系，便于报告里记录

    注意每个环都是闭合的（shapely 的 .coords 首尾点天然相同），服务端
    getFieldFromGoal() 会校验「首元素 == 末元素」，不闭合会直接报错。
    """

    # 1. 读取 WKT
    with open(wkt_path, "r", encoding="utf-8") as f:
        data = f.read().strip()

    # 2. WKT → Shapely Polygon
    field_wgs84 = wkt.loads(data)

    # 3. 选目标带
    if target_crs is None:
        centroid = field_wgs84.centroid
        target_crs = utm_epsg(centroid.x, centroid.y)

    # 4. 建立坐标转换器 + WGS84 → UTM
    transformer = Transformer.from_crs(
        source_crs,
        target_crs,
        always_xy=True
    )
    field_utm = transform(
        transformer.transform,
        field_wgs84
    )

    # 5. 提取外边界 + 全部内环
    rings = [
        [[float(x), float(y)] for x, y in ring.coords]
        for ring in [field_utm.exterior, *field_utm.interiors]
    ]

    # 6. 平移：以第一个点为 map 原点 (0, 0)
    ox, oy = rings[0][0]
    rings = [[[p[0] - ox, p[1] - oy] for p in ring] for ring in rings]

    return rings, (ox, oy), target_crs


def wkt_to_utm(wkt_path, source_crs="EPSG:4326", target_crs=None):
    """
    老接口：只返回外边界，保持向后兼容。

    内环（地块里的洞／障碍）会被丢掉——需要洞的地方请用 wkt_to_utm_rings()。

    Returns
    -------
    field : list
        [[x1, y1], [x2, y2], ...]
    """
    rings, offset, _epsg = wkt_to_utm_rings(wkt_path, source_crs, target_crs)
    return rings[0], offset


if __name__ == '__main__':

    WKT_PATH = (
        "/home/zhang/agri_robot/data/WKT_POLYGON/1019_test_1.wkt"

    )

    rings, offset, epsg = wkt_to_utm_rings(WKT_PATH)
    field = rings[0]

    print(f"转换完成：{len(rings)} 个环（1 个外边界 + {len(rings) - 1} 个内环），"
          f"外边界 {len(field)} 个点")
    print(f"目标坐标系：{epsg}（按地块中心自动选带）")
    print(f"原点偏移量 (UTM): x={offset[0]:.3f}, y={offset[1]:.3f}")
    print(f"平移后第一个点: x={field[0][0]:.6f}, y={field[0][1]:.6f}")

    for i, point in enumerate(field):
        print(
            f"{i}: "
            f"x={point[0]:.3f}, "
            f"y={point[1]:.3f}"
        )

    # 保存结果到 results 目录
    results_dir = "/home/zhang/agri_robot/open_navigation_ws/src/navigation_ws/results"
    os.makedirs(results_dir, exist_ok=True)

    # 生成输出文件名（基于输入 WKT 文件名）
    wkt_filename = os.path.basename(WKT_PATH)
    wkt_name = os.path.splitext(wkt_filename)[0]
    output_path = os.path.join(results_dir, f"{wkt_name}_utm_boundary.json")

    # 构建 GeoJSON 格式：coordinates 是「外边界 + 内环」的列表，
    # 就是 GeoJSON Polygon 的标准写法，也是 action goal 里 polygons 的顺序。
    # demo_coverage_via_headland.load_boundary_rings() 按这个顺序逐个环下发。
    geojson = {
        "type": "Feature",
        "properties": {
            "name": wkt_name,
            "crs": epsg,
            "units": "meters",
            "description": "UTM coordinates (auto zone), origin shifted to first point",
            "source_crs": "EPSG:4326",
            "origin_utm": {"x": offset[0], "y": offset[1]},
            "point_count": len(field),
            "n_rings": len(rings),
            "n_holes": len(rings) - 1,
            "ring_point_counts": [len(r) for r in rings],
        },
        "geometry": {
            "type": "Polygon",
            "coordinates": [
                [[float(pt[0]), float(pt[1])] for pt in ring] for ring in rings
            ]
        }
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(geojson, f, indent=2)

    print(f"\n结果已保存到：{output_path}")
