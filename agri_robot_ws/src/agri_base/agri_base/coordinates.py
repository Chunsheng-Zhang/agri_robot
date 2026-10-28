"""One fixed UTM origin for robot, waypoints and field geometry (metres)."""

import math
from pyproj import Geod, Transformer


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def finite_positive(value, name, allow_zero=False):
    value = float(value)
    if not math.isfinite(value) or value < 0 or (not allow_zero and value == 0):
        raise ValueError(f'{name} must be finite and positive')
    return value


class LocalFrame:
    def __init__(self, latitude, longitude, max_radius=5000.0):
        self.validate(latitude, longitude)
        self.latitude, self.longitude = float(latitude), float(longitude)
        self.max_radius = finite_positive(max_radius, 'max_local_radius')
        zone = min(60, int((longitude + 180) // 6) + 1)
        self.epsg = (32600 if latitude >= 0 else 32700) + zone
        self.transformer = Transformer.from_crs(4326, self.epsg, always_xy=True)
        self.e0, self.n0 = self.transformer.transform(longitude, latitude)
        self.geod = Geod(ellps='WGS84')

    @staticmethod
    def validate(latitude, longitude):
        if not (math.isfinite(latitude) and math.isfinite(longitude)
                and -80 <= latitude <= 84 and -180 <= longitude <= 180):
            raise ValueError('Invalid coordinate or outside UTM latitude range')

    @property
    def identity(self):
        return {'epsg': self.epsg, 'latitude': self.latitude, 'longitude': self.longitude}

    def xy(self, latitude, longitude):
        self.validate(latitude, longitude)
        e, n = self.transformer.transform(longitude, latitude)
        x, y = e - self.e0, n - self.n0
        if not math.isfinite(x + y) or math.hypot(x, y) > self.max_radius:
            raise ValueError('Coordinate outside configured local operating radius')
        return x, y

    def yaw(self, latitude, longitude, north_clockwise_radians, offset=0.0):
        """Project a 1 m heading vector, including UTM grid convergence.

        On the central meridian this reduces to pi/2 - heading + offset.
        Offset aligns the INS heading axis with base_link's forward axis.
        """
        if not math.isfinite(north_clockwise_radians + offset):
            raise ValueError('Invalid heading')
        lon2, lat2, _ = self.geod.fwd(longitude, latitude,
                                    math.degrees(north_clockwise_radians), 1.0)
        e1, n1 = self.transformer.transform(longitude, latitude)
        e2, n2 = self.transformer.transform(lon2, lat2)
        return wrap_angle(math.atan2(n2 - n1, e2 - e1) + offset)


def frame_from_node(node):
    from rcl_interfaces.msg import ParameterDescriptor

    defaults = {'origin_latitude': 36.6468527633281,
                'origin_longitude': 117.021433066391, 'max_local_radius': 5000.0}
    for name, value in defaults.items():
        node.declare_parameter(name, value, ParameterDescriptor(
            read_only=True,
            description='Fixed local coordinate configuration; change YAML and restart the whole core.'))
    return LocalFrame(*(node.get_parameter(name).value for name in defaults))


def track_speeds(linear, angular, separation):
    """Differential track linear speeds, m/s; not a hardware command."""
    finite_positive(separation, 'track_separation')
    return linear - angular * separation / 2, linear + angular * separation / 2
