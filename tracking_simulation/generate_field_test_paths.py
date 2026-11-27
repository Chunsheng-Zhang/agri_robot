#!/usr/bin/env python3
"""Generate coordinate-only field references; never starts ROS or a vehicle."""
import argparse
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / 'agri_robot_ws/src/agri_base'))
from agri_base.coordinates import LocalFrame, wrap_angle
from pyproj import Transformer


def geometry(length, radius, spacing):
    """Return forward/left coordinates and analytic tangent, with unique joins."""
    count = math.ceil(length / spacing)
    rows = [(length*i/count, 0., 0., length*i/count, 'outbound') for i in range(count+1)]
    if radius is None:
        return rows
    arc_count = math.ceil(math.pi*radius/spacing)
    for i in range(1, arc_count+1):
        theta = math.pi*i/arc_count
        rows.append((length+radius*math.sin(theta), radius*(1-math.cos(theta)),
                     theta, length+radius*theta, 'arc'))
    for i in range(1, count+1):
        distance = length*i/count
        rows.append((length-distance, 2*radius, math.pi,
                     length+math.pi*radius+distance, 'inbound'))
    return rows


def pear_geometry(length, radius, spacing, row_spacing=2.):
    """Forward R-L-R bulb turn, net left pi, with analytic circular arcs.

    For d < 2R, alpha=acos((d+2R)/(4R)) joins three radius-R arcs
    with sweeps -alpha, pi+2alpha, -alpha. The tangent is continuous;
    curvature changes sign at the joins. No reversing or point turns.
    """
    if not all(math.isfinite(v) and v > 0 for v in (length, radius, spacing, row_spacing)):
        raise ValueError('梨型长度、半径、点距和行距必须为有限正数')
    if row_spacing >= 2*radius:
        raise ValueError('此梨型要求行间距小于两倍最小半径')
    alpha = math.acos((row_spacing+2*radius)/(4*radius))
    rows = geometry(length, None, spacing)
    u, left, yaw, progress, _ = rows[-1]
    for sign, sweep, region in [(-1, alpha, 'arc_right_entry'),
                                (1, math.pi+2*alpha, 'arc_left_main'),
                                (-1, alpha, 'arc_right_exit')]:
        count = math.ceil(radius*sweep/spacing)
        start_u, start_left, start_yaw, start_s = u, left, yaw, progress
        for i in range(1, count+1):
            theta = sweep*i/count
            yaw = start_yaw+sign*theta
            u = start_u+sign*radius*(math.sin(yaw)-math.sin(start_yaw))
            left = start_left-sign*radius*(math.cos(yaw)-math.cos(start_yaw))
            progress = start_s+radius*theta
            rows.append((u, left, yaw, progress, region))
    assert math.dist((u, left), (length, row_spacing)) < 1e-9
    assert abs(yaw-math.pi) < 1e-12
    rows[-1] = (length, row_spacing, math.pi, progress, 'arc_right_exit')
    count = math.ceil(length/spacing)
    for i in range(1, count+1):
        distance = length*i/count
        rows.append((length-distance, row_spacing, math.pi, progress+distance, 'inbound'))
    return rows


def generate(output, straight_length=20., leg_length=10., spacing=.05, add_pears=False):
    for value in (straight_length, leg_length, spacing):
        if not math.isfinite(value) or value <= 0:
            raise ValueError('长度和采样间距必须为有限正数')
    output = Path(output).resolve()
    if ROOT not in output.parents:
        raise ValueError('输出必须在项目工作区内')
    previous = None
    if add_pears:
        previous = json.loads((output/'manifest.json').read_text())
        if any(output.glob('pear_turn_*')):
            raise ValueError('梨型文件已存在，请使用新目录，避免覆盖')
        for name, digest in previous['files'].items():
            if hashlib.sha256((output/name).read_bytes()).hexdigest() != digest:
                raise ValueError('原输出文件已改变，拒绝追加：'+name)
    elif output.exists() or output.with_suffix('.zip').exists():
        raise ValueError('输出目录或同名ZIP已存在，请使用新目录，避免覆盖实验输入')
    defaults_file = ROOT/'agri_web/src/components/tasks/defaultTaskPoints.ts'
    defaults = re.findall(r'latitude:\s*([\d.]+),\s*longitude:\s*([\d.]+)', defaults_file.read_text())
    if len(defaults) < 2:
        raise ValueError('无法读取网页默认前两个点')
    first, second = [tuple(map(float, row)) for row in defaults[:2]]
    frame = LocalFrame(*first)
    dx, dy = frame.xy(*second)
    if math.hypot(dx, dy) < 1e-3:
        raise ValueError('默认前两个点重合，无法确定出发方向')
    heading = math.atan2(dy, dx)
    inverse = Transformer.from_crs(frame.epsg, 4326, always_xy=True)
    datasets = []
    for name, length, radius in [('straight_20m', straight_length, None),
                                  ('u_turn_R1p5', leg_length, 1.5),
                                  ('u_turn_R2p0', leg_length, 2.),
                                  ('u_turn_R2p5', leg_length, 2.5),
                                  ('pear_turn_R1p5_d2m', leg_length, 1.5),
                                  ('pear_turn_R2p0_d2m', leg_length, 2.)]:
        if radius is None:
            name = 'straight_' + f'{length:g}'.replace('.', 'p') + 'm'
        pear = name.startswith('pear_turn_')
        raw = pear_geometry(length, radius, spacing) if pear else geometry(length, radius, spacing)
        if len(raw) > 20000:
            raise ValueError('路径超过网页导入上限20000点')
        points, wgs84 = [], []
        for u, left, tangent, progress, region in raw:
            x = u*math.cos(heading)-left*math.sin(heading)
            y = u*math.sin(heading)+left*math.cos(heading)
            if math.hypot(x, y) > frame.max_radius:
                raise ValueError('路径超出项目固定原点范围')
            yaw = wrap_angle(heading+tangent)
            longitude, latitude = inverse.transform(frame.e0+x, frame.n0+y)
            if not points:
                latitude, longitude = first  # Exact original coordinate in CSV.
            points.append(dict(x=x, y=y, yaw=yaw, s_m=progress, region=region))
            wgs84.append(dict(longitude=longitude, latitude=latitude, yaw=yaw))
        total = raw[-1][3] if pear else length if radius is None else 2*length+math.pi*radius
        document = dict(schema_version=1, name=name, frame_id='map', projection=frame.identity,
                        source='analytic field test reference; not a measured trajectory',
                        geometry=dict(straight_length_m=length if radius is None else None,
                                      leg_length_m=length if radius is not None else None,
                                      radius_m=radius, turn='left' if radius else None,
                                      row_separation_m=2. if pear else 2*radius if radius else 0.,
                                      analytic_length_m=total, max_spacing_m=spacing,
                                      initial_map_yaw_rad=heading,
                                      direction_source='web default point 1 toward point 2'), points=points)
        if pear:
            alpha = math.acos((2.+2*radius)/(4*radius))
            document['geometry'].update(shape='PEAR_RLR', min_radius_m=radius,
                arc_sweeps_rad=[-alpha, math.pi+2*alpha, -alpha],
                construction='analytic tangent-connected R-L-R arcs; engineering test, not paper digitization')
        datasets.append((name, document, wgs84))
    if previous:
        if previous['projection'] != frame.identity or abs(previous['map_yaw_deg']-math.degrees(heading)) > 1e-10:
            raise ValueError('固定原点或出发方向已改变，拒绝追加')
    # Prepare and validate every existing coordinate file before any mutation.
    contents = {}
    for name, document, wgs84 in datasets:
        contents[name+'.json'] = (json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
        stream = io.StringIO(newline='')
        writer = csv.DictWriter(stream, fieldnames=['longitude', 'latitude', 'yaw'])
        writer.writeheader()
        writer.writerows(wgs84)
        contents[name+'.csv'] = stream.getvalue().encode()
    for name, content in contents.items():
        if (output/name).exists() and (output/name).read_bytes() != content:
            raise ValueError('尺寸或生成内容与原路径不同，拒绝修改：'+name)
    output.mkdir(parents=True, exist_ok=add_pears)
    for name, content in contents.items():
        if not (output/name).exists():
            (output/name).write_bytes(content)
    for key, value in [('TMPDIR', '.cache/tmp'), ('MPLCONFIGDIR', '.cache/matplotlib')]:
        os.environ.setdefault(key, str(HERE/value))
        Path(os.environ[key]).mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 3, figsize=(12, 12), constrained_layout=True)
    all_points = [p for _, document, _ in datasets for p in document['points']]
    xmin, xmax = min(p['x'] for p in all_points), max(p['x'] for p in all_points)
    ymin, ymax = min(p['y'] for p in all_points), max(p['y'] for p in all_points)
    figure.suptitle('Green: start | Red: finish | Arrows: travel direction (same scale in all panels)')
    for ax, (name, document, _) in zip(axes.flat, datasets):
        points = document['points']
        ax.plot([p['x'] for p in points], [p['y'] for p in points], color='#27689e')
        ax.scatter(points[0]['x'], points[0]['y'], c='green', s=35, label='Start', zorder=3)
        ax.scatter(points[-1]['x'], points[-1]['y'], c='red', s=35, label='Finish', zorder=3)
        for i in [len(points)//5, len(points)*3//5]:
            a, b = points[i:i+2]
            ax.annotate('', (b['x'], b['y']), (a['x'], a['y']),
                        arrowprops=dict(arrowstyle='->', color='#27689e', mutation_scale=18))
        ax.set_title(name+f"\n{document['geometry']['analytic_length_m']:.3f} m")
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(xmin-1., xmax+1.)
        ax.set_ylim(ymin-1., ymax+1.)
        ax.grid(alpha=.3)
        ax.set_xlabel('map X / UTM grid east (m)')
        ax.set_ylabel('map Y / UTM grid north (m)')
    figure.savefig(output/'paths_overview.png', dpi=180)
    plt.close(figure)
    lines = ['# 室外直线、U型与梨型参考路径', '',
             f'起点纬度：{first[0]}；经度：{first[1]}。EPSG:{frame.epsg}，固定原点map=(0,0)。',
             f'出发方向为默认点1→点2，ROS map航向{math.degrees(heading):.6f}°；全部前进行驶，U型和梨型整体向左掉头。', '',
             '| 文件名（JSON或CSV） | 长度m | 半径m | 点数 |', '| --- | ---: | ---: | ---: |']
    for name, doc, _ in datasets:
        lines.append(f"| {name} | {doc['geometry']['analytic_length_m']:.6f} | {doc['geometry']['radius_m'] or '—'} | {len(doc['points'])} |")
    lines += ['', f'最大采样间距{spacing:g}m。JSON含固定原点map坐标与ROS航向，可用于网页及离线仿真；CSV为WGS84经纬度与相同ROS航向。',
              '', '网页使用：路径跟随 → 设置名称 → 导入对应JSON。JSON与CSV二选一，无需重复导入。新版导入保留原始点，不重采样、不平滑。',
              '选择只预览；切换路径跟随模式、加载、开始均需分别操作。起点接近须人工完成，独立路径跟随不自动从远处驶向起点。',
              '这批文件没有指定或改变实车速度、控制器及其参数，没有加载到运行中的任务。',
              '导入保留原始坐标点数、点序及已提供航向；经纬度按固定原点转换为map。已有旧版本导入记录需要重新从原始文件导入。',
              f'这是{straight_length:g}m短直线及{leg_length:g}m直线臂的实验路径，不等同论文200m直线或标准150m AB往返测试。',
              '长度、半径按本项目UTM map米制定义；轨迹是车辆控制点参考中心线，不含车体宽度及场地安全余量。',
              f'梨型两臂各{leg_length:g}m、行间距2m，最小半径分别1.5/2m；以右—左—右三段等半径圆弧相切构造，先小幅右摆再向左转大圆弧，净左转180°。',
              '梨型位置和切向连续，曲率在圆弧连接处变号；它是确定尺寸的工程实验路径，不是论文图形或原始坐标的精确复现。',
              '', '完整操作与评价说明：tracking_simulation/室外直线与U型实验路径.md。', '']
    (output/'README.md').write_text('\n'.join(lines), encoding='utf-8')
    files = sorted(p for p in output.iterdir() if p.name != 'manifest.json')
    manifest = dict(projection=frame.identity, start_wgs84=dict(latitude=first[0], longitude=first[1]),
                    map_yaw_deg=math.degrees(heading),
                    default_points_sha256=hashlib.sha256(defaults_file.read_bytes()).hexdigest(),
                    generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    files={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    (output/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    archive = output.with_suffix('.zip')
    target = archive.with_suffix('.zip.tmp') if add_pears else archive
    with zipfile.ZipFile(target, 'w' if add_pears else 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in sorted(output.iterdir()):
            bundle.write(path, arcname=output.name+'/'+path.name)
    if add_pears:
        target.replace(archive)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--straight-length', type=float, default=20.)
    parser.add_argument('--leg-length', type=float, default=10.)
    parser.add_argument('--spacing', type=float, default=.05)
    parser.add_argument('--add-pears', action='store_true', help='核对原四条路径字节不变后，在已有目录追加两条梨型，更新图/说明/清单/ZIP')
    args = parser.parse_args()
    try:
        print(generate(args.output, args.straight_length, args.leg_length, args.spacing, args.add_pears))
    except (ValueError, OSError) as exc:
        parser.exit(1, str(exc)+'\n')


if __name__ == '__main__':
    main()
