"""Stereo IMX219 pair on the Raspberry Pi 5, without the rest of the robot.

    ros2 launch learn2race_bringup stereo_capture.launch.py

Topics: /stereo/{left,right}/image_raw and camera_info, 640x480 at 20 fps.
For visual SLAM: rectify:=true format:=R8 frame_us:=100000 (grey, 10 fps).

The ROS apt libcamera (/opt/ros/jazzy/lib) has no Pi 5 (rpi/pisp) pipeline,
so camera_ros reports "no cameras available". The Raspberry Pi fork built into
/usr/local has it; LD_LIBRARY_PATH makes camera_ros load that one instead.

With rectify:=true, only image_proc's rectifiers run, adding
/stereo/{left,right}/image_rect (what visual SLAM needs) at a fraction of the
CPU of the full depth pipeline.

With depth:=true, stereo_image_proc adds /stereo/{left,right}/image_rect,
/stereo/disparity and /stereo/points2 (needs the calibration below).

camera_info comes from ~/learn2race/config/stereo/{left,right}.yaml, written by
`ros2 run learn2race_evaluation calibrate_stereo`. Until then the cameras run
uncalibrated. Calibrate in the same resolution and sensor mode used here.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R17] camera_ros (libcamera node): camera_ros
# More details: README.md of this package

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PI_LIBCAMERA = '/usr/local/lib/aarch64-linux-gnu'
CALIBRATION_DIR = os.path.expanduser('~/learn2race/config/stereo')


def camera(side, index, orientation):
    info = os.path.join(CALIBRATION_DIR, f'{side}.yaml')
    return Node(
        package='camera_ros', executable='camera_node', name='camera',
        namespace=f'stereo/{side}', output='screen',
        additional_env={'LD_LIBRARY_PATH': PI_LIBCAMERA + ':' + os.environ.get('LD_LIBRARY_PATH', '')},
        parameters=[{
            'camera': ParameterValue(LaunchConfiguration(index), value_type=int),
            'role': 'viewfinder',
            # XRGB8888 arrives as bgra8 (1.2 MB per frame). For visual SLAM
            # use format:=R8 (grey, 0.3 MB): at bgra8 x 2 cameras x 20 fps the
            # rectifiers only reached ~5 Hz on the Pi 5 and tracking was lost.
            'format': LaunchConfiguration('format'),
            'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
            'sensor_mode': LaunchConfiguration('sensor_mode'),
            'frame_id': f'{side}_camera_optical_frame',
            # Rotation applied in the sensor (IMX219 flips, no CPU cost).
            'orientation': ParameterValue(LaunchConfiguration(orientation), value_type=int),
            'camera_info_url': f'file://{info}' if os.path.exists(info) else '',
            'FrameDurationLimits': ParameterValue(
                ['[', LaunchConfiguration('frame_us'), ', ', LaunchConfiguration('frame_us'), ']']),
        }],
        remappings=[
            (f'/stereo/{side}/camera/image_raw', f'/stereo/{side}/image_raw'),
            (f'/stereo/{side}/camera/camera_info', f'/stereo/{side}/camera_info'),
        ],
    )


def rectifier(side):
    only_rectify = PythonExpression(["'", LaunchConfiguration('rectify'), "' == 'true' and '",
                                     LaunchConfiguration('depth'), "' != 'true'"])
    return Node(package='image_proc', executable='rectify_node', name='rectify',
                namespace=f'stereo/{side}', condition=IfCondition(only_rectify),
                remappings=[('image', 'image_raw')])


def generate_launch_description():
    return LaunchDescription([
        # libcamera indices: which physical camera is "left" is checked by
        # calibrate_stereo (it warns if the pair is swapped).
        DeclareLaunchArgument('left_camera', default_value='0'),
        DeclareLaunchArgument('right_camera', default_value='1'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('sensor_mode', default_value='3280:2464'),
        # Image rotation per camera (0/90/180/270), applied in the sensor. Both
        # of Robot B's cameras deliver upright images at 0. Calibrate with the
        # same values used at run time.
        DeclareLaunchArgument('left_orientation', default_value='0'),
        DeclareLaunchArgument('right_orientation', default_value='0'),
        DeclareLaunchArgument('format', default_value='XRGB8888'),
        # Frame period in microseconds: 50000 = 20 fps, 100000 = 10 fps.
        DeclareLaunchArgument('frame_us', default_value='50000'),
        DeclareLaunchArgument('depth', default_value='false'),
        DeclareLaunchArgument('rectify', default_value='false'),
        # 128 px of disparity: with the ~62 mm baseline and ~700 px rectified
        # focal length the nearest measurable range is about 0.35 m.
        DeclareLaunchArgument('disparity_range', default_value='128'),
        camera('left', 'left_camera', 'left_orientation'),
        camera('right', 'right_camera', 'right_orientation'),
        rectifier('left'),
        rectifier('right'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('stereo_image_proc'), 'launch', 'stereo_image_proc.launch.py'])),
            condition=IfCondition(LaunchConfiguration('depth')),
            launch_arguments={
                'namespace': 'stereo',
                'left_namespace': 'left',
                'right_namespace': 'right',
                # The two sensors free-run; they are not hardware-triggered.
                'approximate_sync': 'True',
                'stereo_algorithm': '1',          # semi-global matching
                'disparity_range': LaunchConfiguration('disparity_range'),
                'correlation_window_size': '7',
                'uniqueness_ratio': '10.0',
                'speckle_size': '100',
                'speckle_range': '4',
            }.items(),
        ),
    ])
