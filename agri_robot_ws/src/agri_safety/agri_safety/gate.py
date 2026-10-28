"""ROS-independent exclusive command gate; all clocks supplied by caller."""

import math


class CommandGate:
    def __init__(self, max_linear=0.3, max_angular=0.6, command_timeout=0.25,
                 max_path_error=2.0, max_lateral=0.3):
        for value in (max_linear, max_angular, command_timeout, max_path_error, max_lateral):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Gate limits must be finite and positive')
        self.max_linear, self.max_angular = max_linear, max_angular
        self.command_timeout, self.max_path_error = command_timeout, max_path_error
        self.owner, self.reason = '', 'Emergency stop at startup'
        self.emergency_stop = True
        self.max_lateral, self.lateral_output = max_lateral, 0.0
        self.samples = {}
        self.sample_stamps = {}
        self.granted_at = -math.inf
        self.granted_stamp = -math.inf
        self.error, self.error_time = None, -math.inf

    def revoke(self, reason):
        self.owner, self.reason = '', reason
        self.lateral_output = 0.0
        self.samples.clear()
        self.sample_stamps.clear()

    def stop(self, active, healthy):
        if not active and not healthy:
            return False
        self.emergency_stop = active
        self.revoke('Emergency stop' if active else 'Ready')
        return True

    def enable(self, source, active, healthy, now, stamp):
        if source not in ('waypoint', 'tracking', 'manual', 'web'):
            return False
        if not active:
            if self.owner and self.owner != source:
                return False
            self.revoke('Task released')
            return True
        if self.owner or self.emergency_stop or not healthy:
            return False
        self.owner, self.reason = source, 'Acquired'
        self.granted_at, self.granted_stamp = now, stamp
        self.samples.clear()
        self.sample_stamps.clear()
        self.error, self.error_time = None, -math.inf
        return True

    def receive(self, source, linear, angular, now, stamp, lateral=0.0):
        if source != self.owner:
            return
        if not all(math.isfinite(v) for v in (linear, angular, stamp, lateral)):
            self.revoke('Non-finite command')
            return
        if stamp < self.granted_stamp or stamp < self.sample_stamps.get(source, -math.inf):
            return
        self.samples[source] = (linear, angular, now, lateral)
        self.sample_stamps[source] = stamp

    def output(self, now, healthy):
        self.lateral_output = 0.0
        if self.emergency_stop:
            return 0.0, 0.0
        if not healthy:
            self.revoke('Localization invalid, stale, or conflicting publishers')
        if self.emergency_stop or not self.owner:
            return 0.0, 0.0
        if self.owner == 'tracking':
            if self.error is not None and abs(self.error) > self.max_path_error:
                self.revoke('Path deviation exceeded')
                return 0.0, 0.0
            if now - self.error_time > self.command_timeout:
                if now - self.granted_at > 0.5:
                    self.revoke('Tracking error feedback expired')
                return 0.0, 0.0
        sample = self.samples.get(self.owner)
        if sample is None or now - sample[2] > self.command_timeout:
            if now - self.granted_at > 0.5:
                self.revoke('Controller command expired')
            return 0.0, 0.0
        v, w, _, lateral = sample
        self.lateral_output = max(-self.max_lateral, min(self.max_lateral, lateral))
        return (max(-self.max_linear, min(self.max_linear, v)),
                max(-self.max_angular, min(self.max_angular, w)))
