#!/usr/bin/env python3
"""从 per_field.csv 补画批量实验总图 report.png。

批量跑全量要几个小时，中途中断（Ctrl-C、机器重启、服务端卡死退出）时
per_field.csv 已经边跑边落盘，但 main() 末尾的 plot_report() 没来得及执行。
这个脚本只做一件事：把已有 CSV 读回来、还原成 main() 内存里 rows 的格式，
再调用同一个 plot_report()，出来的图和正常跑完的那张完全一样。

用法：
    python3 plot_report_from_csv.py results/20260928_185039_batch
    python3 plot_report_from_csv.py <某个 per_field.csv 路径>

输出：<批次目录>/report.png（已存在则先备份为 report.png.bak）
"""
import argparse
import csv
import os
import shutil
import sys

import yaml

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TEST_DIR)

import test_coverage_all_fields as tcf  # noqa: E402


def coerce(raw_row):
    """CSV 里全是字符串，按 new_row() 的默认值还原成 bool/int/float/str。

    不手写每列的类型表，直接拿 new_row() 的默认值当类型模板，
    以后 CSV 加列也不会漏。
    """
    types = tcf.new_row('', '')
    row = {}
    for key, default in types.items():
        val = raw_row.get(key, '')
        if isinstance(default, bool):
            row[key] = str(val).strip().lower() in ('true', '1', 'yes')
        elif isinstance(default, int):
            row[key] = int(float(val)) if str(val).strip() else 0
        elif isinstance(default, float):
            row[key] = float(val) if str(val).strip() else default
        else:
            row[key] = val
    return row


def load_rows(csv_path, mode):
    """读 CSV，返回 [row]；mode 为空时取文件里出现的第一种路由模式。"""
    with open(csv_path, encoding='utf-8', newline='') as f:
        raw = list(csv.DictReader(f))

    rows = [coerce(r) for r in raw]

    if not mode:
        # 和 main() 一致：总表只放主模式，否则同一地块会重复出现
        for r in rows:
            if r['route_mode']:
                mode = r['route_mode']
                break

    return [r for r in rows if r['route_mode'] == mode], mode


def main():
    parser = argparse.ArgumentParser(description='从 per_field.csv 补画 report.png')
    parser.add_argument('path', help='批次目录，或直接给 per_field.csv 路径')
    parser.add_argument('--mode', default='', help='只统计这个路由模式（默认：文件里第一个）')
    parser.add_argument('--out', default='', help='输出 png 路径（默认 report.png）')
    parser.add_argument('--min-turning-radius', type=float, default=None,
                        help='覆盖图里标注的最小转弯半径（默认读同目录 server_params.yaml）')
    args = parser.parse_args()

    if os.path.isdir(args.path):
        batch_dir = args.path
        csv_path = os.path.join(batch_dir, 'per_field.csv')
    else:
        csv_path = args.path
        batch_dir = os.path.dirname(os.path.abspath(csv_path))

    if not os.path.isfile(csv_path):
        sys.exit(f'找不到 {csv_path}')

    rows, mode = load_rows(csv_path, args.mode)
    n_ok = sum(1 for r in rows if r['ok'])
    print(f'{csv_path}\n  读入 {len(rows)} 行（路由模式 {mode or "-"}），'
          f'其中成功 {n_ok} 行')

    # 转弯半径设定值：批次目录里那份 server_params.yaml 就是这次用的真源
    radius = args.min_turning_radius
    if radius is None:
        radius = 0.0
        params_path = os.path.join(batch_dir, 'server_params.yaml')
        if os.path.isfile(params_path):
            with open(params_path, encoding='utf-8') as f:
                params = (yaml.safe_load(f) or {}).get(
                    tcf.COVERAGE_SERVER_NODE, {}).get('ros__parameters', {})
            radius = float(params.get('min_turning_radius', 0.0))
            print(f'  min_turning_radius = {radius}（取自 {os.path.basename(params_path)}）')

    out_path = args.out or os.path.join(batch_dir, 'report.png')
    if os.path.exists(out_path):
        shutil.copy2(out_path, out_path + '.bak')
        print(f'  已存在，备份为 {os.path.basename(out_path)}.bak')

    tcf.plot_report(rows, out_path, radius)

    print(f'  写出 {out_path}  ({os.path.getsize(out_path) / 1e6:.2f} MB)')


if __name__ == '__main__':
    main()
