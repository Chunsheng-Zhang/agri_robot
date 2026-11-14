"""Four work modes and their command authorities; no ROS runtime dependency."""
from typing import Literal

WorkMode = Literal['base', 'waypoint', 'coverage', 'following']
MODE_LABELS = {'base': '基础待命', 'waypoint': '多点导航',
               'coverage': '全覆盖规划与跟踪', 'following': '路径跟随'}
WORK_MODES = tuple(MODE_LABELS)
MODE_SOURCE = {'waypoint': 'waypoint', 'coverage': 'tracking', 'following': 'following'}
SOURCE_MODE = {source: mode for mode, source in MODE_SOURCE.items()}
AUTONOMOUS_SOURCES = tuple(SOURCE_MODE)
COMMAND_SOURCES = (*AUTONOMOUS_SOURCES, 'manual', 'web')


def source_allowed(mode, source):
    if mode not in WORK_MODES or source not in COMMAND_SOURCES:
        return False
    # Explicit joystick takeover is available from every selected work mode.
    return source == 'manual' or (source == 'web' and mode == 'base') or SOURCE_MODE.get(source) == mode
