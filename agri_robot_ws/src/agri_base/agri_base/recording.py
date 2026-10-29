"""CSV recordings always carry a matching fixed-origin/reference JSON sidecar."""
import csv
import json
from datetime import datetime
from pathlib import Path


def open_recording(folder, prefix, frame, points, columns, **extra):
    directory = Path(folder).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / datetime.now().strftime(f'{prefix}_%Y%m%d_%H%M%S_%f.csv')
    stream = path.open('x', newline='', encoding='utf-8')
    try:
        metadata = {
            'schema_version': 1, 'trajectory_csv': path.name,
            'frame_id': 'map', 'position_reference': 'base_link', 'projection': frame.identity,
            'axes': {'x': 'UTM grid east', 'y': 'UTM grid north'},
            'units': {'position': 'm', 'angle': 'rad', 'time': 's'},
            'time_source': '/odom header.stamp (ROS time)',
            'reference_path': {'frame_id': 'map', 'points': points}, 'columns': columns, **extra,
        }
        with path.with_suffix('.json').open('x', encoding='utf-8') as output:
            json.dump(metadata, output, ensure_ascii=False, allow_nan=False, indent=2)
        writer = csv.writer(stream)
        writer.writerow(columns)
        return stream, writer
    except (ValueError, OSError):
        stream.close()
        raise
