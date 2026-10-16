from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'learn2race_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ruben Pereira',
    maintainer_email='1181915@isep.ipp.pt',
    description='Sensor and actuator drivers for the Learn2Race platform.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vl6180_tof_node = learn2race_hardware.pca9548_vl6180_tof_node:main',
            'lsm6ds3_imu_node = learn2race_hardware.grove_lsm6ds3_imu_node:main',
            'rgb_stick_node = learn2race_hardware.grove_rgb_stick_node:main',
            'chainable_led_node = learn2race_hardware.chainable_led_node:main',
            'i2c_display_node = learn2race_hardware.i2c_display_node:main',
            'link_led_node = learn2race_hardware.micro_ros_link_led_node:main',
            'line_array_node = learn2race_hardware.line_array_node:main',
            'ltr303_node = learn2race_hardware.ltr303_node:main',
            'tof050c_node = learn2race_hardware.tof050c_node:main',
            'ultrasonic_node = learn2race_hardware.ultrasonic_node:main',
            'sharp_ir_node = learn2race_hardware.sharp_ir_node:main',
            'battery_monitor = learn2race_hardware.battery_monitor_node:main',
        ],
    },
)
