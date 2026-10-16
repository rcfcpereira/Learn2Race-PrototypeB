from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'learn2race_safety'

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
    description='Command arbitration, arming lock and safety supervision.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmd_vel_mux = learn2race_safety.cmd_vel_mux_node:main',
            'safety_monitor = learn2race_safety.safety_monitor_node:main',
        ],
    },
)
