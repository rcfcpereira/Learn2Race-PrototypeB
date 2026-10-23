from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'learn2race_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ruben Pereira',
    maintainer_email='1181915@isep.ipp.pt',
    description='Evaluation and benchmarking tools for Learn2Race.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_calibration = learn2race_evaluation.odom_calibration_node:main',
            'stop_test = learn2race_evaluation.stop_test_node:main',
            'endurance_logger = learn2race_evaluation.endurance_logger_node:main',
            'analyze_timing = learn2race_evaluation.analyze_timing:main',
            'analyze_odometry = learn2race_evaluation.analyze_odometry:main',
            'analyze_tracking = learn2race_evaluation.analyze_tracking:main',
            'make_report = learn2race_evaluation.report_generator:main',
            'microros_benchmark = learn2race_evaluation.microros_benchmark:main',
            'startup_probe = learn2race_evaluation.startup_probe:main',
            'rmw_report = learn2race_evaluation.rmw_report:main',
            'sensor_check = learn2race_evaluation.sensor_check_node:main',
            'ranging_characterisation = learn2race_evaluation.ranging_characterisation_node:main',
            'analyze_sensors = learn2race_evaluation.analyze_sensors:main',
            'calibrate_imu = learn2race_evaluation.calibrate_imu_node:main',
            'calibrate_imu_axes = learn2race_evaluation.calibrate_imu_axes_node:main',
            'calibrate_stereo = learn2race_evaluation.calibrate_stereo_node:main',
            'stereo_depth_check = learn2race_evaluation.stereo_depth_check_node:main',
            'calibrate_magnetometer = learn2race_evaluation.calibrate_magnetometer_node:main',
            'wheel_direction_test = learn2race_evaluation.wheel_direction_test_node:main',
            'calibrate_odometry = learn2race_evaluation.calibrate_odometry_node:main',
            'calibrate_line_array = learn2race_evaluation.calibrate_line_array_node:main',
            'hardware_check = learn2race_evaluation.hardware_check_node:main',
        ],
    },
)
