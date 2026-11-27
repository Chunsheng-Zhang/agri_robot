"""Causal sampled navigation delivery for the planar offline model."""
import heapq
import math
import random


class DelayedPoseSensor:
    def __init__(self, period, delay_samples, noise, seed=0):
        if not math.isfinite(period) or period<=0:raise ValueError('采样周期必须为正')
        if not delay_samples or any(not math.isfinite(x) or x<0 or x>.5 for x in delay_samples):
            raise ValueError('交付延迟样本无效')
        self.period=period;self.delays=tuple(delay_samples);self.noise=noise
        self.rng=random.Random(seed+81421)
        self.pending=[];self.sequence=0;self.latest=None
        self.next_sample=period

    def sample(self,t,pose,speed,*,initial=False):
        if not math.isfinite(t) or any(not math.isfinite(x) for x in (*pose,speed)):
            raise ValueError('传感器样本无效')
        if t<0:raise ValueError('采样时间不能为负')
        self.sequence+=1
        observation=(*self.noise.sample(*pose),float(speed))
        delivery=t+(0. if initial else self.rng.choice(self.delays))
        heapq.heappush(self.pending,(delivery,self.sequence,t,observation))
        if not initial:self.next_sample=t+self.period

    def deliver(self,t):
        while self.pending and self.pending[0][0]<=t+1e-10:
            delivery,seq,stamp,value=heapq.heappop(self.pending)
            # A late old frame must not overwrite a newer navigation sample.
            if self.latest is None or stamp>self.latest['source_time_s']:
                self.latest=dict(source_time_s=stamp,delivery_time_s=delivery,sequence=seq,value=value)
        return self.latest
