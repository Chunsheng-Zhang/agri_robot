#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MGX23 IMU Data Visualization Analysis"""

import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import statistics

def load_csv(fp):
    with open(fp, 'r', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))

def pf(v):
    try:
        return float(v)
    except:
        return 0.0

def main():
    d1 = load_csv('/home/zhang/agri_robot/mgx23_ws/data/mgx23_logs/mgx23_20261014_094721.csv')
    d2 = load_csv('/home/zhang/agri_robot/mgx23_ws/data/mgx23_logs/mgx23_20261014_102345.csv')
    print(f"Loaded: {len(d1)} frames (static), {len(d2)} frames (dynamic)")

    t1 = [pf(r['elapsed_sec']) for r in d1]
    t2 = [pf(r['elapsed_sec']) for r in d2]

    lat1 = [(pf(r['latitude'])-pf(d1[0]['latitude']))*111000 for r in d1]
    lon1 = [(pf(r['longitude'])-pf(d1[0]['longitude']))*111000*0.75 for r in d1]
    alt1 = [pf(r['altitude'])-pf(d1[0]['altitude']) for r in d1]
    lat2 = [(pf(r['latitude'])-pf(d2[0]['latitude']))*111000 for r in d2]
    lon2 = [(pf(r['longitude'])-pf(d2[0]['longitude']))*111000*0.75 for r in d2]
    alt2 = [pf(r['altitude'])-pf(d2[0]['altitude']) for r in d2]

    roll1=[pf(r['roll']) for r in d1]; pitch1=[pf(r['pitch']) for r in d1]; yaw1=[pf(r['yaw']) for r in d1]
    roll2=[pf(r['roll']) for r in d2]; pitch2=[pf(r['pitch']) for r in d2]; yaw2=[pf(r['yaw']) for r in d2]

    vn1=[pf(r['vel_north']) for r in d1]; ve1=[pf(r['vel_east']) for r in d1]; vd1=[pf(r['vel_down']) for r in d1]
    vn2=[pf(r['vel_north']) for r in d2]; ve2=[pf(r['vel_east']) for r in d2]; vd2=[pf(r['vel_down']) for r in d2]
    sp1=[(n**2+e**2+d**2)**0.5 for n,e,d in zip(vn1,ve1,vd1)]
    sp2=[(n**2+e**2+d**2)**0.5 for n,e,d in zip(vn2,ve2,vd2)]

    fig, ax = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('MGX23 IMU Data Analysis\n(Red=Static, Blue=Dynamic)', fontsize=14, fontweight='bold')

    # Row 1: Position
    ax[0,0].plot(t1,lat1,'r-',linewidth=0.8,alpha=0.7,label='Static-North')
    ax[0,0].plot(t1,lon1,'g-',linewidth=0.8,alpha=0.7,label='Static-East')
    ax[0,0].plot(t1,alt1,'b-',linewidth=0.8,alpha=0.7,label='Static-Alt')
    ax[0,0].plot(t2,lat2,'r--',linewidth=1.0,label='Dynamic-North')
    ax[0,0].plot(t2,lon2,'g--',linewidth=1.0,label='Dynamic-East')
    ax[0,0].plot(t2,alt2,'b--',linewidth=1.0,label='Dynamic-Alt')
    ax[0,0].set_xlabel('Time (s)'); ax[0,0].set_ylabel('Relative Position (m)')
    ax[0,0].set_title('Position (Solid=Static, Dashed=Dynamic)')
    ax[0,0].legend(fontsize=8); ax[0,0].grid(True, alpha=0.3)

    ax[0,1].plot(lon1,lat1,'r-',linewidth=0.5,alpha=0.5,label='Static')
    ax[0,1].plot(lon2,lat2,'b-',linewidth=0.8,label='Dynamic')
    ax[0,1].plot(lon2[0],lat2[0],'go',markersize=8,label='Dyn Start')
    ax[0,1].plot(lon2[-1],lat2[-1],'mo',markersize=8,label='Dyn End')
    ax[0,1].set_xlabel('East (m)'); ax[0,1].set_ylabel('North (m)')
    ax[0,1].set_title('2D Trajectory'); ax[0,1].legend(fontsize=8)
    ax[0,1].grid(True,alpha=0.3); ax[0,1].set_aspect('equal')

    # Row 2: Attitude
    ax[1,0].plot(t1,roll1,'r-',linewidth=0.8,alpha=0.7,label='Static-Roll')
    ax[1,0].plot(t1,pitch1,'g-',linewidth=0.8,alpha=0.7,label='Static-Pitch')
    ax[1,0].plot(t1,yaw1,'b-',linewidth=0.8,alpha=0.7,label='Static-Yaw')
    ax[1,0].plot(t2,roll2,'r--',linewidth=1.0,label='Dynamic-Roll')
    ax[1,0].plot(t2,pitch2,'g--',linewidth=1.0,label='Dynamic-Pitch')
    ax[1,0].plot(t2,yaw2,'b--',linewidth=1.0,label='Dynamic-Yaw')
    ax[1,0].set_xlabel('Time (s)'); ax[1,0].set_ylabel('Angle (deg)')
    ax[1,0].set_title('Attitude Angles'); ax[1,0].legend(fontsize=8); ax[1,0].grid(True,alpha=0.3)

    ax[1,1].hist(roll1,bins=50,alpha=0.4,color='r',label=f'Roll S(std={statistics.stdev(roll1)*1000:.1f}mdeg)')
    ax[1,1].hist(pitch1,bins=50,alpha=0.4,color='g',label=f'Pitch S(std={statistics.stdev(pitch1)*1000:.1f}mdeg)')
    ax[1,1].hist(yaw1,bins=50,alpha=0.4,color='b',label=f'Yaw S(std={statistics.stdev(yaw1):.3f}deg)')
    ax[1,1].hist(roll2,bins=50,alpha=0.3,color='red',label=f'Roll D(std={statistics.stdev(roll2)*1000:.1f}mdeg)')
    ax[1,1].hist(pitch2,bins=50,alpha=0.3,color='lime',label=f'Pitch D(std={statistics.stdev(pitch2)*1000:.1f}mdeg)')
    ax[1,1].hist(yaw2,bins=50,alpha=0.3,color='blue',label=f'Yaw D(std={statistics.stdev(yaw2):.3f}deg)')
    ax[1,1].set_xlabel('Angle (deg)'); ax[1,1].set_ylabel('Count')
    ax[1,1].set_title('Attitude Distribution'); ax[1,1].legend(fontsize=7); ax[1,1].grid(True,alpha=0.3)

    # Row 3: Velocity
    ax[2,0].plot(t1,vn1,'r-',linewidth=0.8,alpha=0.7,label='Static-N')
    ax[2,0].plot(t1,ve1,'g-',linewidth=0.8,alpha=0.7,label='Static-E')
    ax[2,0].plot(t1,vd1,'b-',linewidth=0.8,alpha=0.7,label='Static-D')
    ax[2,0].plot(t2,vn2,'r--',linewidth=1.0,label='Dynamic-N')
    ax[2,0].plot(t2,ve2,'g--',linewidth=1.0,label='Dynamic-E')
    ax[2,0].plot(t2,vd2,'b--',linewidth=1.0,label='Dynamic-D')
    ax[2,0].set_xlabel('Time (s)'); ax[2,0].set_ylabel('Velocity (m/s)')
    ax[2,0].set_title('3-Axis Velocity'); ax[2,0].legend(fontsize=8); ax[2,0].grid(True,alpha=0.3)

    ax[2,1].hist(sp1,bins=50,alpha=0.5,color='orange',label=f'Static(mean={statistics.mean(sp1)*1000:.1f}mm/s)')
    ax[2,1].hist(sp2,bins=50,alpha=0.5,color='cyan',label=f'Dynamic(mean={statistics.mean(sp2)*1000:.1f}mm/s)')
    ax[2,1].set_xlabel('Speed (m/s)'); ax[2,1].set_ylabel('Count')
    ax[2,1].set_title('Speed Distribution'); ax[2,1].legend(fontsize=9); ax[2,1].grid(True,alpha=0.3)

    plt.tight_layout()
    out = '/home/zhang/agri_robot/mgx23_ws/data/mgx23_analysis.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")

if __name__ == '__main__':
    main()