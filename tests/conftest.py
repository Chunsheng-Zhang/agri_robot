import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for package in (ROOT / 'agri_robot_ws' / 'src').iterdir():
    if package.is_dir():
        sys.path.insert(0, str(package))
sys.path.insert(0, str(ROOT / 'mgx23_ws' / 'src' / 'mgx23_driver'))
