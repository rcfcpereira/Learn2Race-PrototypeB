"""Visual SLAM for Prototype B with the calibrated stereo pair (RTAB-Map).

Pure visual: stereo visual odometry replaces the wheels as odom -> base_link,
so start the robot without its wheel-odometry transform:

    ros2 launch learn2race_bringup prototype_b.launch.py teleop:=false wheel_odom_tf:=false
    ros2 launch learn2race_bringup stereo_capture.launch.py
    ros2 launch learn2race_bringup rtabmap_stereo_b.launch.py

Outputs: /odom_visual (visual odometry), /map (2D occupancy grid projected from
stereo depth), /rtabmap/cloud_map, and the database ~/maps/rtabmap_b.db, which
keeps the map for later sessions (delete_db:=true starts a new one).

Visual odometry needs texture and moderate motion: it loses track on plain
walls and fast turns, and then waits until it recognises the scene again.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R48] Labbe & Michaud (2019) RTAB-Map: RTAB-Map
#   [R35] Hartley & Zisserman, Multiple View Geometry: stereo
# More details: README.md of this package

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Raw images: RTAB-Map rectifies them itself from camera_info (see
# Rtabmap/ImagesAlreadyRectified). The separate image_proc rectifiers only
# reached ~5 Hz on the Pi 5 even at 43% idle CPU, starving visual odometry.
STEREO = [
    ('left/image_rect', '/stereo/left/image_raw'),
    ('left/camera_info', '/stereo/left/camera_info'),
    ('right/image_rect', '/stereo/right/image_raw'),
    ('right/camera_info', '/stereo/right/camera_info'),
]

COMMON = {
    'frame_id': 'base_link',
    # The two IMX219 run free (no shared trigger): pair by nearest timestamp.
    'approx_sync': True,
    # The cameras run at slightly different rates (~17 and ~18 fps), so the
    # nearest pair can be over 30 ms apart; 40 ms keeps them without pairing
    # frames from different moments on a slow robot.
    'approx_sync_max_interval': 0.04,
    # Odometry arrives ~0.7 s after its images (0.25-0.3 s per frame on the
    # Pi 5), and rtabmap pairs it with those same images. The old queue of 10
    # (0.5 s at 20 fps) had already dropped them, so rtabmap never received a
    # complete set and never published a map. 30 frames is 1.5 s.
    'topic_queue_size': 10,
    'sync_queue_size': 30,
    'wait_for_transform': 0.3,
    # A ground robot moves in the plane: solve x, y, yaw only.
    'Reg/Force3DoF': 'true',
    'Rtabmap/ImagesAlreadyRectified': 'false',
}


def generate_launch_description():
    database = os.path.expanduser('~/maps/rtabmap_b.db')
    os.makedirs(os.path.dirname(database), exist_ok=True)
    slam_params = dict(COMMON, **{
        'subscribe_stereo': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'database_path': database,
        'Mem/IncrementalMemory': 'true',
        'RGBD/LinearUpdate': '0.05',         # add a node every 5 cm ...
        'RGBD/AngularUpdate': '0.05',        # ... or 3 degrees of turn
        'Grid/FromDepth': 'true',            # occupancy grid from stereo depth
        'Grid/3D': 'false',
        'Grid/RangeMax': '3.0',              # stereo depth is noisy far away
        # Forward-looking cameras see little floor, so free space came only
        # from floor points and the grid held obstacles alone. Ray tracing
        # marks the cells between the camera and each obstacle as free.
        'Grid/RayTracing': 'true',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '0.6',
        'Stereo/MaxDisparity': '128',
    })
    return LaunchDescription([
        DeclareLaunchArgument('delete_db', default_value='false'),
        Node(package='rtabmap_odom', executable='stereo_odometry', name='stereo_odometry',
             output='screen',
             parameters=[dict(COMMON, **{
                 'odom_frame_id': 'odom',
                 'publish_tf': True,
                 'Odom/ResetCountdown': '1',  # restart tracking after a loss
                 'Vis/MinInliers': '12',
                 # With raw input each update takes ~0.05 s, so keep the full
                 # 1000 features: fewer (600) cut quality to ~25 inliers.
                 'Vis/MaxFeatures': '1000',
             })],
             remappings=STEREO + [('odom', '/odom_visual')]),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
             namespace='rtabmap', parameters=[slam_params],
             remappings=STEREO + [('odom', '/odom_visual'), ('map', '/map')],
             arguments=['--delete_db_on_start'],
             condition=IfCondition(LaunchConfiguration('delete_db'))),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
             namespace='rtabmap', parameters=[slam_params],
             remappings=STEREO + [('odom', '/odom_visual'), ('map', '/map')],
             condition=UnlessCondition(LaunchConfiguration('delete_db'))),
    ])
