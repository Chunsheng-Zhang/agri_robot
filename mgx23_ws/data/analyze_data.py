#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import statistics

def analyze_csv(filepath, session_name):
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    
    total_frames = len(rows)
    elapsed_times = [float(r['elapsed_sec']) for r in rows]
    duration = elapsed_times[-1] - elapsed_times[0]
    
    latitudes = [float(r['latitude']) for r in rows]
    longitudes = [float(r['longitude']) for r in rows]
    altitudes = [float(r['altitude']) for r in rows]
    
    vel_north = [float(r['vel_north']) for r in rows]
    vel_east = [float(r['vel_east']) for r in rows]
    vel_down = [float(r['vel_down']) for r in rows]
    
    rolls = [float(r['roll']) for r in rows]
    pitches = [float(r['pitch']) for r in rows]
    yaws = [float(r['yaw']) for r in rows]
    
    temps = [float(r['temperature']) for r in rows]
    accuracy = [float(r['accuracy']) for r in rows]
    checksum_ok = sum(1 for r in rows if r['checksum_ok'] == 'True')
    
    acc_bias_x = [float(r['acc_bias_x']) for r in rows]
    acc_bias_y = [float(r['acc_bias_y']) for r in rows]
    acc_bias_z = [float(r['acc_bias_z']) for r in rows]
    gyro_bias_x = [float(r['gyro_bias_x']) for r in rows]
    gyro_bias_y = [float(r['gyro_bias_y']) for r in rows]
    gyro_bias_z = [float(r['gyro_bias_z']) for r in rows]
    
    speeds = [(n**2 + e**2 + d**2)**0.5 for n, e, d in zip(vel_north, vel_east, vel_down)]
    
    print(f"\n{'='*60}")
    print(f"会话：{session_name}")
    print(f"{'='*60}")
    print(f"总帧数：{total_frames}")
    print(f"采集时长：{duration:.3f} 秒")
    print(f"平均帧率：{total_frames/duration:.2f} Hz")
    print(f"\n--- 位置信息 ---")
    print(f"纬度范围：{min(latitudes):.10f} ~ {max(latitudes):.10f}")
    lat_diff_m = (max(latitudes) - min(latitudes)) * 111000
    print(f"  纬度差值：{lat_diff_m:.3f} 米")
    print(f"经度范围：{min(longitudes):.10f} ~ {max(longitudes):.10f}")
    lon_diff_m = (max(longitudes) - min(longitudes)) * 111000 * 0.75
    print(f"  经度差值：{lon_diff_m:.3f} 米")
    print(f"高度范围：{min(altitudes):.3f} ~ {max(altitudes):.3f} 米 (差值：{max(altitudes)-min(altitudes):.3f} 米)")
    print(f"\n--- 速度信息 (m/s) ---")
    print(f"北向速度：min={min(vel_north):.6f}, max={max(vel_north):.6f}, mean={statistics.mean(vel_north):.6f}, std={statistics.stdev(vel_north):.6f}")
    print(f"东向速度：min={min(vel_east):.6f}, max={max(vel_east):.6f}, mean={statistics.mean(vel_east):.6f}, std={statistics.stdev(vel_east):.6f}")
    print(f"垂向速度：min={min(vel_down):.6f}, max={max(vel_down):.6f}, mean={statistics.mean(vel_down):.6f}, std={statistics.stdev(vel_down):.6f}")
    print(f"合速度：mean={statistics.mean(speeds):.6f}, std={statistics.stdev(speeds):.6f}")
    print(f"\n--- 姿态信息 (度) ---")
    print(f"Roll:  min={min(rolls):.6f}, max={max(rolls):.6f}, mean={statistics.mean(rolls):.6f}, std={statistics.stdev(rolls):.6f}")
    print(f"Pitch: min={min(pitches):.6f}, max={max(pitches):.6f}, mean={statistics.mean(pitches):.6f}, std={statistics.stdev(pitches):.6f}")
    print(f"Yaw:   min={min(yaws):.6f}, max={max(yaws):.6f}, mean={statistics.mean(yaws):.6f}, std={statistics.stdev(yaws):.6f}")
    print(f"\n--- 传感器偏差 ---")
    print(f"加速度计偏差 (m/s²):")
    print(f"  X: mean={statistics.mean(acc_bias_x):.6f}, std={statistics.stdev(acc_bias_x):.6f}")
    print(f"  Y: mean={statistics.mean(acc_bias_y):.6f}, std={statistics.stdev(acc_bias_y):.6f}")
    print(f"  Z: mean={statistics.mean(acc_bias_z):.6f}, std={statistics.stdev(acc_bias_z):.6f}")
    print(f"陀螺仪偏差 (rad/s):")
    print(f"  X: mean={statistics.mean(gyro_bias_x):.6f}, std={statistics.stdev(gyro_bias_x):.6f}")
    print(f"  Y: mean={statistics.mean(gyro_bias_y):.6f}, std={statistics.stdev(gyro_bias_y):.6f}")
    print(f"  Z: mean={statistics.mean(gyro_bias_z):.6f}, std={statistics.stdev(gyro_bias_z):.6f}")
    print(f"\n--- 温度 (°C) ---")
    print(f"温度范围：{min(temps):.3f} ~ {max(temps):.3f}, 平均：{statistics.mean(temps):.3f}")
    print(f"\n--- 定位精度 ---")
    unique_acc = sorted(set(accuracy))
    print(f"唯一精度值：{unique_acc}")
    print(f"(1=单点解，2=伪距，4=浮点解，5=RTK 浮点解，16=固定解)")
    print(f"\n--- 数据质量 ---")
    print(f"校验和正确率：{checksum_ok}/{total_frames} = {checksum_ok/total_frames*100:.2f}%")
    
    return {
        'session': session_name,
        'total_frames': total_frames,
        'duration': duration,
        'lat_range': lat_diff_m,
        'lon_range': lon_diff_m,
        'alt_range': max(altitudes) - min(altitudes),
        'mean_speed': statistics.mean(speeds),
        'std_speed': statistics.stdev(speeds),
        'mean_roll': statistics.mean(rolls),
        'std_roll': statistics.stdev(rolls),
        'mean_pitch': statistics.mean(pitches),
        'std_pitch': statistics.stdev(pitches),
        'mean_yaw': statistics.mean(yaws),
        'std_yaw': statistics.stdev(yaws),
        'mean_acc_bias_x': statistics.mean(acc_bias_x),
        'mean_acc_bias_y': statistics.mean(acc_bias_y),
        'mean_acc_bias_z': statistics.mean(acc_bias_z),
        'mean_gyro_bias_x': statistics.mean(gyro_bias_x),
        'mean_gyro_bias_y': statistics.mean(gyro_bias_y),
        'mean_gyro_bias_z': statistics.mean(gyro_bias_z),
        'mean_temp': statistics.mean(temps),
        'checksum_rate': checksum_ok/total_frames*100,
    }


if __name__ == '__main__':
    stats1 = analyze_csv(
        '/home/zhang/agri_robot/mgx23_ws/data/mgx23_logs/mgx23_20261014_094721.csv',
        '第一套 (09:47:21 开始)'
    )
    stats2 = analyze_csv(
        '/home/zhang/agri_robot/mgx23_ws/data/mgx23_logs/mgx23_20261014_102345.csv',
        '第二套 (10:23:45 开始)'
    )
    
    print(f"\n{'='*60}")
    print(f"两套数据对比分析")
    print(f"{'='*60}")
    print(f"{'指标':<30} {'第一套':<20} {'第二套':<20}")
    print(f"{'-'*70}")
    print(f"{'采集时间':<30} {'09:47:21':<20} {'10:23:45':<20}")
    print(f"{'采集时长 (秒)':<30} {stats1['duration']:<20.3f} {stats2['duration']:<20.3f}")
    print(f"{'总帧数':<30} {stats1['total_frames']:<20} {stats2['total_frames']:<20}")
    print(f"{'平均帧率 (Hz)':<30} {stats1['total_frames']/stats1['duration']:<20.2f} {stats2['total_frames']/stats2['duration']:<20.2f}")
    print(f"{'纬度变化 (米)':<30} {stats1['lat_range']:<20.3f} {stats2['lat_range']:<20.3f}")
    print(f"{'经度变化 (米)':<30} {stats1['lon_range']:<20.3f} {stats2['lon_range']:<20.3f}")
    print(f"{'高度变化 (米)':<30} {stats1['alt_range']:<20.3f} {stats2['alt_range']:<20.3f}")
    print(f"{'平均合速度 (m/s)':<30} {stats1['mean_speed']:<20.6f} {stats2['mean_speed']:<20.6f}")
    print(f"{'速度标准差':<30} {stats1['std_speed']:<20.6f} {stats2['std_speed']:<20.6f}")
    print(f"{'平均 Roll (度)':<30} {stats1['mean_roll']:<20.6f} {stats2['mean_roll']:<20.6f}")
    print(f"{'平均 Pitch (度)':<30} {stats1['mean_pitch']:<20.6f} {stats2['mean_pitch']:<20.6f}")
    print(f"{'平均 Yaw (度)':<30} {stats1['mean_yaw']:<20.6f} {stats2['mean_yaw']:<20.6f}")
    print(f"{'平均温度 (°C)':<30} {stats1['mean_temp']:<20.3f} {stats2['mean_temp']:<20.3f}")
    print(f"{'校验和正确率 (%)':<30} {stats1['checksum_rate']:<20.2f} {stats2['checksum_rate']:<20.2f}")
    print(f"\n--- 加速度计偏差对比 (m/s²) ---")
    print(f"  Acc Bias X: 第一套={stats1['mean_acc_bias_x']:.6f}, 第二套={stats2['mean_acc_bias_x']:.6f}")
    print(f"  Acc Bias Y: 第一套={stats1['mean_acc_bias_y']:.6f}, 第二套={stats2['mean_acc_bias_y']:.6f}")
    print(f"  Acc Bias Z: 第一套={stats1['mean_acc_bias_z']:.6f}, 第二套={stats2['mean_acc_bias_z']:.6f}")
    print(f"\n--- 陀螺仪偏差对比 (rad/s) ---")
    print(f"  Gyro Bias X: 第一套={stats1['mean_gyro_bias_x']:.6f}, 第二套={stats2['mean_gyro_bias_x']:.6f}")
    print(f"  Gyro Bias Y: 第一套={stats1['mean_gyro_bias_y']:.6f}, 第二套={stats2['mean_gyro_bias_y']:.6f}")
    print(f"  Gyro Bias Z: 第一套={stats1['mean_gyro_bias_z']:.6f}, 第二套={stats2['mean_gyro_bias_z']:.6f}")