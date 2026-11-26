"""Plots and animation for results produced by the common closed loop."""
import math

import numpy as np

from four_controler.four_controler import LABELS


def save_actuator_response(output, rows):
    if not rows or 'v_actuator_m_s' not in rows[0]:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
    try:
        t = [row['time_s'] for row in rows]
        for ax, fields, unit in (
                (axes[0], ('v_command_m_s', 'v_actuator_m_s', 'v_actual_body_m_s'), 'v (m/s)'),
                (axes[1], ('w_command_rad_s', 'w_actuator_rad_s', 'w_actual_rad_s'), 'w (rad/s)')):
            for field, style, label in zip(fields, ('--', ':', '-'),
                    ('Controller request', 'Actuator lag output', 'Model actual velocity')):
                ax.plot(t, [row[field] for row in rows], style, label=label, linewidth=1)
            ax.set_ylabel(unit)
            ax.grid(alpha=.25)
            ax.legend(fontsize=8)
        axes[1].set_xlabel('Simulation time (s)')
        fig.suptitle('Controller request / added first-order response / model velocity')
        fig.savefig(output/'actuator_response.png', dpi=160)
    finally:
        plt.close(fig)


def save_comparison(output, poses, results, animation):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    physical = all(m.get('kind') == 'GAZEBO_ODE_TRACKING' for _, m in results.values())
    colors=dict(zip(results,('tab:blue','tab:orange','tab:green','tab:red')))
    fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
    try:
        axes[0,0].plot([p[0] for p in poses],[p[1] for p in poses],'k--',lw=1,label='Reference')
        for name,(rows,metrics) in results.items():
            color=colors[name];label=LABELS[name]
            axes[0,0].plot([r['x_true_m'] for r in rows],[r['y_true_m'] for r in rows],color=color,label=label,lw=1)
            for ax,column,factor in ((axes[0,1],'cross_track_true_m',100),
                                     (axes[1,0],'heading_error_true_rad',180/math.pi)):
                ax.plot([r['progress_m'] for r in rows],[factor*r[column] for r in rows],color=color,label=label,lw=.9)
            axes[1,1].plot([r['time_s'] for r in rows],[r['progress_m'] for r in rows],color=color,label=label)
        axes[0,0].set(title='Reference and simulated robot trajectories',xlabel='map x (m)',ylabel='map y (m)')
        axes[0,0].set_aspect('equal',adjustable='datalim')
        axes[0,1].set(title='Lateral error',xlabel='Route progress (m)',ylabel='Error (cm)')
        axes[1,0].set(title='Body-heading error',xlabel='Route progress (m)',ylabel='Error (deg)')
        axes[1,1].set(title='Route progress',xlabel='Simulation time (s)',ylabel='Progress (m)')
        for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=8)
        fig.suptitle('SIMULATION | '+('Gazebo ODE physics' if physical else 'shared planar robot model')+' | no real-vehicle calibration')
        fig.savefig(output/'controller_comparison.png',dpi=150)
    finally:plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True,constrained_layout=True)
    try:
        for name,(rows,_) in results.items():
            for ax,column in zip(axes,('v_command_m_s','w_command_rad_s')):
                ax.plot([r['time_s'] for r in rows],[r[column] for r in rows],color=colors[name],label=LABELS[name],lw=.8)
        axes[0].set_ylabel('Linear speed command (m/s)')
        axes[1].set(ylabel='Angular speed command (rad/s)',xlabel='Simulation time (s)')
        for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
        lag = any(m['parameters'].get('actuator_linear_tau', 0) or m['parameters'].get('actuator_angular_tau', 0) for _,m in results.values())
        fig.suptitle('Body speed requests | '+('responses are in each actuator_response.png' if physical or lag else 'ideal model executes them directly'))
        fig.savefig(output/'controller_commands.png',dpi=150)
    finally:plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13,4),constrained_layout=True)
    try:
        for ax,key,factor,title in ((axes[0],'cross_track_true_m',100,'Lateral RMSE (cm)'),
                                    (axes[1],'heading_error_true_deg',1,'Heading RMSE (deg)'),
                                    (axes[2],None,1,'Simulation duration (s)')):
            vals=[(m[key]['rmse']*factor if m[key]['rmse'] is not None else math.nan) if key else m['elapsed_simulation_s'] for _,m in results.values()]
            ax.bar(list(results),vals,color=[colors[n] for n in results])
            ax.set_title(title);ax.grid(axis='y',alpha=.25)
        fig.suptitle('Includes initial convergence | engineering defaults, not an optimal-controller ranking')
        fig.savefig(output/'controller_metrics.png',dpi=150)
    finally:plt.close(fig)
    if not animation['enabled']:return
    from matplotlib.animation import FuncAnimation, PillowWriter
    fig,ax=plt.subplots(figsize=(9,5),constrained_layout=True)
    try:
        ax.plot([p[0] for p in poses],[p[1] for p in poses],'k--',lw=1,label='Reference')
        xs=[p[0] for p in poses];ys=[p[1] for p in poses]
        for rows,_ in results.values():xs.extend(r['x_true_m'] for r in rows);ys.extend(r['y_true_m'] for r in rows)
        margin=max(.5,.04*max(max(xs)-min(xs),max(ys)-min(ys)))
        ax.set(xlim=(min(xs)-margin,max(xs)+margin),ylim=(min(ys)-margin,max(ys)+margin),xlabel='map x (m)',ylabel='map y (m)')
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.25)
        artists={name:(ax.plot([],[],color=colors[name],lw=1,label=LABELS[name])[0],
                       ax.plot([],[],'o',color=colors[name],ms=5)[0]) for name in results}
        ax.legend(fontsize=7);title=ax.set_title('')
        times=np.linspace(0,max(m['elapsed_simulation_s'] for _,m in results.values()),animation['max_frames'])
        def update(t):
            changed=[]
            for name,(rows,_) in results.items():
                index=min(len(rows)-1,max(0,int(np.searchsorted([r['time_s'] for r in rows],t,side='right')-1)))
                line,point=artists[name]
                line.set_data([r['x_true_m'] for r in rows[:index+1]],[r['y_true_m'] for r in rows[:index+1]])
                point.set_data([rows[index]['x_true_m']],[rows[index]['y_true_m']]);changed.extend((line,point))
            title.set_text(f'SIMULATION | t={t:.1f} s | accelerated playback; dots are base_link')
            return changed+[title]
        movie=FuncAnimation(fig,update,frames=times,interval=1000/animation['fps'],blit=False)
        movie.save(output/'controller_compare.gif',writer=PillowWriter(fps=animation['fps']),dpi=90)
    finally:plt.close(fig)
