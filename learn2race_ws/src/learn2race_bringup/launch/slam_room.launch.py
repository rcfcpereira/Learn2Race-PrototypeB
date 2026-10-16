"""Compatibility alias for automatic room SLAM.

This file keeps the user-facing command obvious while the implementation stays
in `mapping_room.launch.py`.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R47] Macenski & Jambrecic (2021) SLAM Toolbox: SLAM Toolbox
#   [R79] LDROBOT STL-06P lidar, ldlidar_stl_ros2: lidar
# More details: README.md of this package

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


def generate_launch_description():
    # Load the sibling launch file directly so all declared launch arguments are
    # exposed exactly as they are in `mapping_room.launch.py`.
    target = Path(__file__).with_name("mapping_room.launch.py")
    spec = spec_from_file_location("learn2race_mapping_room_launch", target)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load launch file: {target}")

    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
