"""Compatibility import: ROS planning and Web reports share one geometry model."""
import sys
from agri_path_planning import coverage_geometry as _implementation
sys.modules[__name__] = _implementation
