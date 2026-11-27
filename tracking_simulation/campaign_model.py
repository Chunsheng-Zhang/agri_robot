"""Navigation-equivalent moving/in-place response, fitted offline only.

This is an empirical planar model, not a mechanical tyre/MCU model. Its time
constants include the navigation instrument response. Raw device delays cannot
be separated without independent ground truth.
"""
import heapq
import math
import numpy as np
from scipy.signal import lfilter

from disturbances import FirstOrderActuator


def validate_campaign_profile(p):
    if type(p.get('schema_version')) is not int or p.get('schema_version')!=2 or p.get('kind')!='X3_GROUND_EQUIVALENT_RESPONSE':
        raise ValueError('需要schema2的X3地面模型')
    for name in ('linear','angular','in_place_angular'):
        c=p[name]
        if set(c)!={'gain','limit','tau_s','delay_s'}:raise ValueError('响应参数字段不完整')
        for key,v in c.items():
            if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or v<0:
                raise ValueError('响应参数必须为有限非负数')
            if key in ('gain','limit') and not 0<v<=5:raise ValueError('增益/限幅超出模型范围')
            if key=='tau_s' and not 0<v<=5 or key=='delay_s' and v>2:raise ValueError('响应时间超出模型范围')
    for v in p['stop_tau_s']:
        if isinstance(v,bool) or not math.isfinite(v) or not 0<=v<=1:raise ValueError('归零时间常数无效')
    if len(p['stop_tau_s'])!=2:raise ValueError('需要两个归零响应参数')
    if not .005<=p['feedback_period_s']<=1:raise ValueError('反馈采样周期无效')
    if type(p['feedback_period_s']) not in (int,float) or isinstance(p['feedback_period_s'],bool):raise ValueError('反馈周期类型无效')
    if not math.isfinite(p.get('feedback_timeout_s',.5)) or not 0<p.get('feedback_timeout_s',.5)<=2:raise ValueError('反馈超时无效')
    if not math.isfinite(p['lateral_gain_s']) or abs(p['lateral_gain_s'])>2:raise ValueError('横向等效参数无效')
    if not 0<=p.get('controller_start_phase_s',0.)<=.1:raise ValueError('控制器采样相位无效')
    if p.get('in_place_order',1) not in (1,2):raise ValueError('原地响应阶数只支持1/2')
    points=p.get('moving_angular_points',[])
    if points:
        if len(points)>10 or any(set(c)!={'speed','gain','tau_s'} for c in points):raise ValueError('行进转向工况字段无效')
        if any(not math.isfinite(c[k]) or isinstance(c[k],bool) for c in points for k in c):raise ValueError('工况参数无效')
        if any(not 0<c['speed']<=1 or not .1<c['gain']<=2 or not 0<c['tau_s']<=3 for c in points):raise ValueError('工况参数超出范围')
        if any(b['speed']<=a['speed'] for a,b in zip(points,points[1:])):raise ValueError('工况速度必须严格递增')
    for name in ('observation_delay_samples_s','control_interval_samples_s'):
        a=p[name]
        if not isinstance(a,list) or not 1<=len(a)<=1000:raise ValueError('时序样本数量无效')
        for v in a:
            if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or not 0<=v<=.5:
                raise ValueError('时序样本超出0～0.5秒')
        if name.startswith('control') and min(a)<.005:raise ValueError('控制周期过小')


def estimate_delivery_distribution(timings, grid_s=.001):
    """Discrete likelihood p(interval)=sum(p_j in interval), training data only.

EM assigns each interval's mass among admissible grid bins; equal weight per
run. This estimates effective input availability, not physical network delay.
"""
    grid=np.arange(0.,.5001,grid_s)
    matrices=[];weights=[]
    for item in timings:
        bounds=np.asarray(item['delivery_intervals_s'])
        m=(grid[None,:]>=bounds[:,0,None]-1e-8)&(grid[None,:]<=bounds[:,1,None]+1e-8)
        # A sub-grid first-use interval remains represented by its midpoint.
        for i in np.flatnonzero(~m.any(axis=1)):
            m[i,int(np.argmin(abs(grid-np.mean(bounds[i]))))]=True
        matrices.append(m);weights.extend([1/(len(timings)*len(bounds))]*len(bounds))
    m=np.vstack(matrices).astype(float);weights=np.asarray(weights)
    p=m.sum(axis=0);p/=p.sum()
    converged=False
    for iteration in range(500):
        likelihood=np.maximum(m@p,1e-300)
        new=p*(m.T@(weights/likelihood));new/=new.sum()
        if np.max(abs(new-p))<1e-8:p=new;converged=True;break
        p=new
    quantiles=np.linspace(.005,.995,101)
    samples=grid[np.minimum(len(grid)-1,np.searchsorted(np.cumsum(p),quantiles))]
    return samples.tolist(),dict(method='interval-censored discrete likelihood; EM, equal weight per training run',
        grid_s=grid_s,iterations=iteration+1,converged=converged,weighted_log_likelihood=float(weights@np.log(np.maximum(m@p,1e-300))),
        mean_s=float(grid@p),note='Effective controller input availability inferred from unique LQR output matches; not recorder arrival or MCU delay')


class CampaignActuator:
    """Ordered causal targets with a faster explicit full-stop response.

Changing between moving/in-place/stop cancels pending targets from the old
mode. A normal stop decays the model velocity; fault reset is a separate hard
zero abstraction. No plant feedback is copied from a recorded trajectory.
"""
    def __init__(self,p):
        validate_campaign_profile(p)
        self.p=p
        self.base=FirstOrderActuator(p['linear']['tau_s'],p['angular']['tau_s'])
        self.pending=[];self.seq=0;self.mode=None
        self.in_place_state=0.
        self.taus=(p['linear']['tau_s'],max([p['angular']['tau_s'],p['in_place_angular']['tau_s']]+[c['tau_s'] for c in p.get('moving_angular_points',[])]))
        self.delays=(p['linear']['delay_s'],max(p['angular']['delay_s'],p['in_place_angular']['delay_s']))

    @property
    def output(self):return self.base.output

    @property
    def lateral_output(self):return self.p['lateral_gain_s']*self.output[0]*self.output[1]

    def _advance_response(self,t):
        dt=0. if self.base.stamp is None else max(0.,t-self.base.stamp)
        old=self.base.output[1];z=self.in_place_state;target=self.base.target[1]
        self.base.advance(t)
        if self.mode=='in_place' and self.p.get('in_place_order',1)==2:
            tau=self.base.taus[1]
            a=math.exp(-dt/max(tau,1e-12));b=dt/max(tau,1e-12)
            self.in_place_state=target+(z-target)*a
            omega=target+(old-target)*a+(z-target)*b*a
            self.base.output=(self.base.output[0],omega)
        else:self.in_place_state=self.base.output[1]
        return self.base.output

    def advance(self,t):
        if not math.isfinite(t) or self.base.stamp is not None and t<self.base.stamp-1e-9:
            raise ValueError('执行器时间无效或倒退')
        while self.pending and self.pending[0][0]<=t:
            stamp,_,axis,value=heapq.heappop(self.pending)
            self._advance_response(stamp)
            target=list(self.base.target);target[axis]=value
            self.base.target=tuple(target)
        return self._advance_response(t)

    def command(self,v,w,t):
        if not all(math.isfinite(x) for x in (v,w,t)):raise ValueError('执行器输入无效')
        self.advance(t)
        mode='moving' if abs(v)>.02 else 'in_place' if abs(w)>1e-9 else 'stop'
        if mode!=self.mode:
            self.pending.clear();self.base.target=(0.,0.);self.mode=mode
            self.in_place_state=self.base.output[1]
        if mode=='stop':
            self.pending.clear();self.base.taus=tuple(self.p['stop_tau_s'])
            self.base.target=(0.,0.)
            return self._advance_response(t)
        angular=dict(self.p['angular' if mode=='moving' else 'in_place_angular'])
        points=self.p.get('moving_angular_points',[])
        if mode=='moving' and points:
            for key in ('gain','tau_s'):
                angular[key]=float(np.interp(abs(v),[c['speed'] for c in points],[c[key] for c in points]))
        self.base.taus=(self.p['linear']['tau_s'] if abs(v)>1e-9 else self.p['stop_tau_s'][0],angular['tau_s'])
        for axis,(u,c) in enumerate(zip((v,w),(self.p['linear'],angular))):
            self.seq+=1
            target=max(-c['limit'],min(c['limit'],c['gain']*u))
            heapq.heappush(self.pending,(t+c['delay_s'],self.seq,axis,target))
        return self.advance(t)

    def reset(self,t):
        self.advance(t);self.pending.clear();self.mode=None
        self.in_place_state=0.
        return self.base.reset(t)


def replay_response(run,p,dt=.01,*,observed=True):
    """Vectorised causal ZOH response, with no measured-state resets."""
    grid=np.arange(min(0.,run.command_time[0]),run.time[-1]+dt,dt)
    idx=np.searchsorted(run.command_time,grid,side='right')-1
    u=run.commands[np.maximum(idx,0)].copy();u[idx<0]=0
    mode=np.where(abs(u[:,0])>.02,1,np.where(abs(u[:,1])>1e-9,2,0))
    epochs=np.cumsum(np.r_[True,np.diff(mode)!=0])
    results=[]
    for channel in range(2):
        c=p['linear'] if channel==0 else p['angular'];ip=p['linear'] if channel==0 else p['in_place_angular']
        delay=np.where(mode==2,ip['delay_s'],c['delay_s'])
        delayed=np.searchsorted(grid,grid-delay+1e-9,side='right')-1
        valid=(delayed>=0)&(epochs[np.maximum(delayed,0)]==epochs)
        gain=np.where(mode==2,ip['gain'],c['gain'])
        tau=np.where(mode==2,ip['tau_s'],c['tau_s'])
        points=p.get('moving_angular_points',[]) if channel==1 else []
        if points:
            for key,field in (('gain',gain),('tau_s',tau)):
                field[mode==1]=np.interp(abs(u[mode==1,0]),[a['speed'] for a in points],[a[key] for a in points])
        target=gain[np.maximum(delayed,0)]*u[np.maximum(delayed,0),channel]
        target[~valid]=0;target[mode==0]=0
        target=np.clip(target,-c['limit'],c['limit'])
        tau[mode==0]=p['stop_tau_s'][channel]
        if channel==0:
            target[abs(u[:,0])<=1e-9]=0
            tau[abs(u[:,0])<=1e-9]=p['stop_tau_s'][0]
        out=np.zeros(len(grid))
        breaks=np.unique(np.r_[0,np.flatnonzero((np.diff(tau)!=0)|(np.diff(mode)!=0))+1,len(grid)-1])
        # On each step y[n+1] uses u[n], including a mode's new tau.
        for a,b in zip(breaks[:-1],breaks[1:]):
            if b<=a:continue
            decay=math.exp(-dt/max(float(tau[a]),1e-12))
            if channel==1 and mode[a]==2 and p.get('in_place_order',1)==2:
                z=out[a];ratio=dt/max(float(tau[a]),1e-12)
                c=1-decay-decay*ratio
                d=decay*ratio*(1-decay)-c*decay
                values,_=lfilter([c,d],[1.,-2*decay,decay*decay],target[a:b],
                    zi=[decay*out[a]+decay*ratio*z,-decay*decay*out[a]])
            else:values,_=lfilter([1-decay],[1.,-decay],target[a:b],zi=[decay*out[a]])
            out[a+1:b+1]=values
        sampled=np.interp(run.time,grid,out)
        if channel==1 and observed:
            # /odom angular_z is a backward heading difference, not an
            # instantaneous gyro sample at the interval's right endpoint.
            integral=np.r_[0.,np.cumsum((out[:-1]+out[1:])*.5*dt)]
            headings=np.interp(run.time,grid,integral)
            sampled[1:]=np.diff(headings)/np.diff(run.time)
        results.append(sampled)
    return np.column_stack(results)


def fit_response(runs,base,*,in_place=False):
    """Fit only supplied training runs; structural form is fixed in advance."""
    from copy import deepcopy
    from scipy.optimize import least_squares
    result=deepcopy(base)
    for channel,name in enumerate(('linear','angular')):
        if in_place and channel==0:continue
        key='in_place_angular' if in_place else name
        best=None
        delays=np.arange(0.,1.801,.1) if in_place else np.arange(0.,.301,.025)
        for delay in delays:
            def residual(x):
                p=deepcopy(result)
                if in_place:
                    p[key].update(tau_s=float(x[0]),delay_s=float(delay));p['stop_tau_s'][channel]=float(x[1])
                else:
                    p[key].update(gain=float(x[0]),tau_s=float(x[1]),delay_s=float(delay))
                    p['stop_tau_s'][channel]=float(x[2])
                output=[]
                for r in runs:
                    predicted=replay_response(r,p)[:,channel]
                    mask=r.valid.copy()
                    latest=np.maximum(0,np.searchsorted(r.command_time,r.time,side='right')-1)
                    if in_place:mask&=abs(r.commands[latest,0])<=.02
                    elif channel==1:mask&=abs(r.commands[latest,0])>.05
                    output.append((predicted[mask]-r.measured[mask,channel])/math.sqrt(max(1,int(mask.sum()))))
                return np.concatenate(output)
            candidate=least_squares(residual,[.7,.04] if in_place else [.92 if channel==0 else .9,.4 if channel==0 else .8,.04],
                bounds=([.01,.005],[3.,.4]) if in_place else ([.3,.01,.005],[1.5,3.,.4]),max_nfev=60)
            score=float(candidate.fun@candidate.fun)
            if candidate.success and (best is None or score<best[0]):best=(score,candidate.x,delay)
        if best is None:raise ValueError('响应辨识未收敛')
        _,x,delay=best
        if in_place:result[key].update(tau_s=float(x[0]),delay_s=float(delay))
        else:result[key].update(gain=float(x[0]),tau_s=float(x[1]),delay_s=float(delay))
        # In-place fit adjusts angular stop response with its own clean steps.
        result['stop_tau_s'][channel]=float(x[1] if in_place else x[2])
    return result


def fit_moving_points(runs,base):
    """Per-speed gain/tau; fixed common transport delay, tracking-only loss."""
    from copy import deepcopy
    from scipy.optimize import least_squares
    points=[]
    for r in runs:
        latest=np.maximum(0,np.searchsorted(r.command_time,r.time,side='right')-1)
        mask=r.valid&(abs(r.commands[latest,0])>.05)
        if r.tracking_window:
            mask&=(r.time>=r.tracking_window[0])&(r.time<=r.tracking_window[1])
        speed=float(np.median(abs(r.commands[latest[mask],0])))
        def residual(x):
            p=deepcopy(base);p.pop('moving_angular_points',None)
            p['angular'].update(gain=float(x[0]),tau_s=float(x[1]))
            return (replay_response(r,p)[mask,1]-r.measured[mask,1])/math.sqrt(int(mask.sum()))
        fit=least_squares(residual,[base['angular']['gain'],base['angular']['tau_s']],bounds=([.3,.05],[1.5,3.]),max_nfev=80)
        if not fit.success:raise ValueError('速度工况辨识未收敛')
        points.append(dict(speed=speed,gain=float(fit.x[0]),tau_s=float(fit.x[1])))
    return sorted(points,key=lambda p:p['speed'])
