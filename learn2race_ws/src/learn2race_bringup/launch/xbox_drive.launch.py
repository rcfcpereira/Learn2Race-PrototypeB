"""Main Learn2Race robot launch.

Every hardware group has a `start_*` argument so the same file can power the
full robot, sensor-only modes, drive-only mode, and calibration wrappers.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R19] joy / teleop_twist_joy: joy, teleop_twist_joy
#   [R79] LDROBOT STL-06P lidar, ldlidar_stl_ros2: lidar
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # LaunchConfiguration objects are symbolic values resolved at launch time.
    serial_dev = LaunchConfiguration("serial_dev")
    agent_baud = LaunchConfiguration("agent_baud")
    agent_verbose = LaunchConfiguration("agent_verbose")
    joy_dev = LaunchConfiguration("joy_dev")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    start_agent = LaunchConfiguration("start_agent")
    start_joy = LaunchConfiguration("start_joy")
    start_teleop = LaunchConfiguration("start_teleop")
    start_leds = LaunchConfiguration("start_leds")
    led_gpio = LaunchConfiguration("led_gpio")
    led_count = LaunchConfiguration("led_count")
    start_display = LaunchConfiguration("start_display")
    display_i2c_bus = LaunchConfiguration("display_i2c_bus")
    display_i2c_address = LaunchConfiguration("display_i2c_address")
    display_width = LaunchConfiguration("display_width")
    display_height = LaunchConfiguration("display_height")
    start_imu = LaunchConfiguration("start_imu")
    imu_i2c_bus = LaunchConfiguration("imu_i2c_bus")
    imu_i2c_address = LaunchConfiguration("imu_i2c_address")
    imu_publish_rate_hz = LaunchConfiguration("imu_publish_rate_hz")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_base_frame = LaunchConfiguration("imu_base_frame")
    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_roll = LaunchConfiguration("imu_roll")
    imu_pitch = LaunchConfiguration("imu_pitch")
    imu_yaw = LaunchConfiguration("imu_yaw")
    start_tof = LaunchConfiguration("start_tof")
    tof_i2c_bus = LaunchConfiguration("tof_i2c_bus")
    tof_mux_address = LaunchConfiguration("tof_mux_address")
    tof_publish_rate_hz = LaunchConfiguration("tof_publish_rate_hz")
    tof_base_frame = LaunchConfiguration("tof_base_frame")
    start_lidar = LaunchConfiguration("start_lidar")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    lidar_scan_topic = LaunchConfiguration("lidar_scan_topic")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    lidar_base_frame = LaunchConfiguration("lidar_base_frame")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_roll = LaunchConfiguration("lidar_roll")
    lidar_pitch = LaunchConfiguration("lidar_pitch")
    lidar_yaw = LaunchConfiguration("lidar_yaw")
    start_stereo_cameras = LaunchConfiguration("start_stereo_cameras")
    start_stereo_processing = LaunchConfiguration("start_stereo_processing")
    left_camera = LaunchConfiguration("left_camera")
    right_camera = LaunchConfiguration("right_camera")
    stereo_width = LaunchConfiguration("stereo_width")
    stereo_height = LaunchConfiguration("stereo_height")
    stereo_sensor_mode = LaunchConfiguration("stereo_sensor_mode")
    stereo_base_frame = LaunchConfiguration("stereo_base_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("agent_baud", default_value="921600"),
            DeclareLaunchArgument("agent_verbose", default_value="6"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("start_agent", default_value="true"),
            DeclareLaunchArgument("start_joy", default_value="true"),
            DeclareLaunchArgument("start_teleop", default_value="true"),
            DeclareLaunchArgument("start_leds", default_value="true"),
            DeclareLaunchArgument("led_gpio", default_value="10"),
            DeclareLaunchArgument("led_count", default_value="10"),
            DeclareLaunchArgument("start_display", default_value="true"),
            DeclareLaunchArgument("display_i2c_bus", default_value="1"),
            DeclareLaunchArgument("display_i2c_address", default_value="60"),
            DeclareLaunchArgument("display_width", default_value="128"),
            DeclareLaunchArgument("display_height", default_value="32"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("imu_i2c_bus", default_value="1"),
            DeclareLaunchArgument("imu_i2c_address", default_value="106"),
            DeclareLaunchArgument("imu_publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("imu_base_frame", default_value="base_link"),
            DeclareLaunchArgument("imu_x", default_value="0"),
            DeclareLaunchArgument("imu_y", default_value="0"),
            DeclareLaunchArgument("imu_z", default_value="0.05"),
            DeclareLaunchArgument("imu_roll", default_value="0"),
            DeclareLaunchArgument("imu_pitch", default_value="0"),
            DeclareLaunchArgument("imu_yaw", default_value="0"),
            DeclareLaunchArgument("start_tof", default_value="true"),
            DeclareLaunchArgument("tof_i2c_bus", default_value="1"),
            DeclareLaunchArgument("tof_mux_address", default_value="112"),
            DeclareLaunchArgument("tof_publish_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("tof_base_frame", default_value="base_link"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("lidar_baud", default_value="230400"),
            DeclareLaunchArgument("lidar_scan_topic", default_value="scan"),
            DeclareLaunchArgument("lidar_frame_id", default_value="base_laser"),
            DeclareLaunchArgument("lidar_base_frame", default_value="base_link"),
            DeclareLaunchArgument("lidar_x", default_value="0"),
            DeclareLaunchArgument("lidar_y", default_value="0"),
            DeclareLaunchArgument("lidar_z", default_value="0.18"),
            DeclareLaunchArgument("lidar_roll", default_value="0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0"),
            # The LD06 0 deg mark points to the robot's right (wall tests, 13-15 Oct 2026): yaw -90 deg.
            DeclareLaunchArgument("lidar_yaw", default_value="-1.5708"),
            DeclareLaunchArgument("start_stereo_cameras", default_value="true"),
            DeclareLaunchArgument("start_stereo_processing", default_value="true"),
            DeclareLaunchArgument("left_camera", default_value="0"),
            DeclareLaunchArgument("right_camera", default_value="1"),
            DeclareLaunchArgument("stereo_width", default_value="640"),
            DeclareLaunchArgument("stereo_height", default_value="480"),
            DeclareLaunchArgument("stereo_sensor_mode", default_value="3280:2464"),
            DeclareLaunchArgument("stereo_base_frame", default_value="base_link"),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent",
                output="screen",
                condition=IfCondition(start_agent),
                arguments=[
                    "serial",
                    "--dev",
                    serial_dev,
                    "-b",
                    agent_baud,
                    "-v",
                    agent_verbose,
                ],
            ),
            # These two nodes convert Xbox input into geometry_msgs/Twist commands.
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                condition=IfCondition(start_joy),
                parameters=[
                    {
                        "dev": joy_dev,
                        "deadzone": 0.10,
                        "autorepeat_rate": 20.0,
                    },
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                condition=IfCondition(start_teleop),
                parameters=[
                    {
                        "require_enable_button": True,
                        "enable_button": 4,
                        "enable_turbo_button": 5,
                        "axis_linear": {"x": 1},
                        "scale_linear": {"x": 0.25},
                        "scale_linear_turbo": {"x": 0.60},
                        "axis_angular": {"yaw": 3},
                        "scale_angular": {"yaw": 1.20},
                        "scale_angular_turbo": {"yaw": 2.50},
                        "publish_stamped_twist": False,
                    }
                ],
                remappings=[
                    ("cmd_vel", cmd_vel_topic),
                ],
            ),
            # Visual feedback and local robot status display.
            Node(
                package="learn2race_hardware",
                executable="grove_rgb_stick_node",
                name="grove_rgb_stick_node",
                output="screen",
                condition=IfCondition(start_leds),
                parameters=[{
                        "gpio_pin": ParameterValue(led_gpio, value_type=int),
                        "led_count": ParameterValue(led_count, value_type=int),
                    }],
            ),
            # Blinks the RGB stick red while the ESP32's micro-ROS session is
            # not established (no recent /odom), solid green once it is.
            Node(
                package="learn2race_hardware",
                executable="micro_ros_link_led_node",
                name="micro_ros_link_led_node",
                output="screen",
                condition=IfCondition(start_leds),
            ),
            Node(
                package="learn2race_hardware",
                executable="i2c_display_node",
                name="i2c_display_node",
                output="screen",
                condition=IfCondition(start_display),
                parameters=[{
                        "i2c_bus": ParameterValue(display_i2c_bus, value_type=int),
                        "i2c_address": ParameterValue(display_i2c_address, value_type=int),
                        "width": ParameterValue(display_width, value_type=int),
                        "height": ParameterValue(display_height, value_type=int),
                    }],
            ),
            # I2C motion/range sensors plus their static frames.
            Node(
                package="learn2race_hardware",
                executable="grove_lsm6ds3_imu_node",
                name="grove_lsm6ds3_imu_node",
                output="screen",
                condition=IfCondition(start_imu),
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("learn2race_bringup"), "config", "grove_lsm6ds3_imu.yaml"]
                    ),
                    {
                        "i2c_bus": ParameterValue(imu_i2c_bus, value_type=int),
                        "i2c_address": ParameterValue(imu_i2c_address, value_type=int),
                        "publish_rate_hz": ParameterValue(imu_publish_rate_hz, value_type=float),
                        "frame_id": imu_frame_id,
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_imu_link",
                output="screen",
                condition=IfCondition(start_imu),
                arguments=[
                    "--x",
                    imu_x,
                    "--y",
                    imu_y,
                    "--z",
                    imu_z,
                    "--roll",
                    imu_roll,
                    "--pitch",
                    imu_pitch,
                    "--yaw",
                    imu_yaw,
                    "--frame-id",
                    imu_base_frame,
                    "--child-frame-id",
                    imu_frame_id,
                ],
            ),
            Node(
                package="learn2race_hardware",
                executable="pca9548_vl6180_tof_node",
                name="pca9548_vl6180_tof_node",
                output="screen",
                condition=IfCondition(start_tof),
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("learn2race_bringup"),
                            "config",
                            "pca9548_vl6180_tof.yaml",
                        ]
                    ),
                    {
                        "i2c_bus": ParameterValue(tof_i2c_bus, value_type=int),
                        "mux_address": ParameterValue(tof_mux_address, value_type=int),
                        "publish_rate_hz": ParameterValue(
                            tof_publish_rate_hz, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_tof_front",
                output="screen",
                condition=IfCondition(start_tof),
                arguments=[
                    "--x",
                    "0.12",
                    "--y",
                    "0",
                    "--z",
                    "0.08",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    tof_base_frame,
                    "--child-frame-id",
                    "tof_front_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_tof_left",
                output="screen",
                condition=IfCondition(start_tof),
                arguments=[
                    "--x",
                    "0.05",
                    "--y",
                    "0.08",
                    "--z",
                    "0.08",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "1.5708",
                    "--frame-id",
                    tof_base_frame,
                    "--child-frame-id",
                    "tof_left_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_tof_right",
                output="screen",
                condition=IfCondition(start_tof),
                arguments=[
                    "--x",
                    "0.05",
                    "--y",
                    "-0.08",
                    "--z",
                    "0.08",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "-1.5708",
                    "--frame-id",
                    tof_base_frame,
                    "--child-frame-id",
                    "tof_right_link",
                ],
            ),
            # USB LD06 LiDAR and its frame relative to the robot base.
            Node(
                package="ldlidar_stl_ros2",
                executable="ldlidar_stl_ros2_node",
                name="ld06_lidar",
                output="screen",
                condition=IfCondition(start_lidar),
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("learn2race_bringup"), "config", "ld06_lidar.yaml"]
                    ),
                    {
                        "port_name": lidar_port,
                        "port_baudrate": ParameterValue(lidar_baud, value_type=int),
                        "topic_name": lidar_scan_topic,
                        "frame_id": lidar_frame_id,
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_base_laser_ld06",
                output="screen",
                condition=IfCondition(start_lidar),
                arguments=[
                    "--x",
                    lidar_x,
                    "--y",
                    lidar_y,
                    "--z",
                    lidar_z,
                    "--roll",
                    lidar_roll,
                    "--pitch",
                    lidar_pitch,
                    "--yaw",
                    lidar_yaw,
                    "--frame-id",
                    lidar_base_frame,
                    "--child-frame-id",
                    lidar_frame_id,
                ],
            ),
            # Raspberry Pi Camera v2 stereo pair and post-processing.
            Node(
                package="camera_ros",
                executable="camera_node",
                namespace="stereo/left",
                name="camera",
                output="screen",
                condition=IfCondition(start_stereo_cameras),
                parameters=[
                    {
                        "camera": ParameterValue(left_camera, value_type=int),
                        "role": "viewfinder",
                        "width": ParameterValue(stereo_width, value_type=int),
                        "height": ParameterValue(stereo_height, value_type=int),
                        "sensor_mode": stereo_sensor_mode,
                        "frame_id": "left_camera_optical_frame",
                        "FrameDurationLimits": [50000, 50000],
                    }
                ],
                remappings=[
                    ("/stereo/left/camera/image_raw", "/stereo/left/image_raw"),
                    ("/stereo/left/camera/camera_info", "/stereo/left/camera_info"),
                ],
            ),
            Node(
                package="camera_ros",
                executable="camera_node",
                namespace="stereo/right",
                name="camera",
                output="screen",
                condition=IfCondition(start_stereo_cameras),
                parameters=[
                    {
                        "camera": ParameterValue(right_camera, value_type=int),
                        "role": "viewfinder",
                        "width": ParameterValue(stereo_width, value_type=int),
                        "height": ParameterValue(stereo_height, value_type=int),
                        "sensor_mode": stereo_sensor_mode,
                        "frame_id": "right_camera_optical_frame",
                        "FrameDurationLimits": [50000, 50000],
                    }
                ],
                remappings=[
                    ("/stereo/right/camera/image_raw", "/stereo/right/image_raw"),
                    ("/stereo/right/camera/camera_info", "/stereo/right/camera_info"),
                ],
            ),
            Node(
                package="image_proc",
                executable="rectify_node",
                namespace="stereo/left",
                name="rectify",
                output="screen",
                condition=IfCondition(start_stereo_processing),
                remappings=[
                    ("image", "image_raw"),
                ],
            ),
            Node(
                package="image_proc",
                executable="rectify_node",
                namespace="stereo/right",
                name="rectify",
                output="screen",
                condition=IfCondition(start_stereo_processing),
                remappings=[
                    ("image", "image_raw"),
                ],
            ),
            Node(
                package="stereo_image_proc",
                executable="disparity_node",
                namespace="stereo",
                name="disparity",
                output="screen",
                condition=IfCondition(start_stereo_processing),
                parameters=[
                    {
                        "approximate_sync": True,
                        "approximate_sync_tolerance_seconds": 0.02,
                        "queue_size": 8,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_left_camera_link",
                output="screen",
                condition=IfCondition(start_stereo_cameras),
                arguments=[
                    "--x",
                    "0.14",
                    "--y",
                    "0.03",
                    "--z",
                    "0.12",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    stereo_base_frame,
                    "--child-frame-id",
                    "left_camera_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_right_camera_link",
                output="screen",
                condition=IfCondition(start_stereo_cameras),
                arguments=[
                    "--x",
                    "0.14",
                    "--y",
                    "-0.03",
                    "--z",
                    "0.12",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    stereo_base_frame,
                    "--child-frame-id",
                    "right_camera_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="left_camera_link_to_optical",
                output="screen",
                condition=IfCondition(start_stereo_cameras),
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "-1.5708",
                    "--pitch",
                    "0",
                    "--yaw",
                    "-1.5708",
                    "--frame-id",
                    "left_camera_link",
                    "--child-frame-id",
                    "left_camera_optical_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="right_camera_link_to_optical",
                output="screen",
                condition=IfCondition(start_stereo_cameras),
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "-1.5708",
                    "--pitch",
                    "0",
                    "--yaw",
                    "-1.5708",
                    "--frame-id",
                    "right_camera_link",
                    "--child-frame-id",
                    "right_camera_optical_frame",
                ],
            ),
        ]
    )
