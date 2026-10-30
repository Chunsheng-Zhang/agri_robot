"""Geographic display of the same base_link pose used by the task maps."""
from functools import lru_cache
import math

from pyproj import Geod, Transformer


@lru_cache(maxsize=4)
def projection_tools(latitude, longitude, epsg):
    forward = Transformer.from_crs(4326, epsg, always_xy=True)
    inverse = Transformer.from_crs(epsg, 4326, always_xy=True)
    e0, n0 = forward.transform(longitude, latitude)
    return inverse, e0, n0, Geod(ellps='WGS84')


def geographic_pose(data, projection):
    """Convert fixed-origin UTM metres and grid yaw, without changing ROS data."""
    try:
        lat, lon = float(projection['latitude']), float(projection['longitude'])
        epsg = int(projection['epsg'])
        x, y = float(data['position_x']), float(data['position_y'])
        if not all(math.isfinite(value) for value in (lat, lon, x, y)):
            return None
        if not (-80 <= lat <= 84 and -180 <= lon <= 180):
            return None
        zone = min(60, int((lon + 180) // 6) + 1)
        if epsg != (32600 if lat >= 0 else 32700) + zone:
            return None
        inverse, e0, n0, geod = projection_tools(lat, lon, epsg)
        longitude, latitude = inverse.transform(e0 + x, n0 + y)
        if not math.isfinite(latitude + longitude) or not (-85 <= latitude <= 85 and -180 <= longitude <= 180):
            return None
        bearing = None
        heading = data.get('heading')
        if heading is not None and math.isfinite(float(heading)):
            yaw = math.radians(float(heading))
            lon2, lat2 = inverse.transform(e0 + x + math.cos(yaw), n0 + y + math.sin(yaw))
            bearing = geod.inv(longitude, latitude, lon2, lat2)[0] % 360.0
        return {'latitude': latitude, 'longitude': longitude, 'bearing': bearing,
                'reference': 'base_link', 'source': 'mgx23_localization', 'crs': 'EPSG:4326'}
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
