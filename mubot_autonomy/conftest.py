"""Lets `python -m pytest src/*/test` import every package without a ROS build."""
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for pkg in sorted(glob.glob(os.path.join(HERE, 'src', 'mubot_*'))):
    if os.path.isfile(os.path.join(pkg, 'setup.py')) and pkg not in sys.path:
        sys.path.insert(0, pkg)
