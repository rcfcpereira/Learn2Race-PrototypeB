from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'learn2race_bringup'

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
    description='Per-prototype bringup launch files and parameter sets.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'room_mapping_explorer_node = learn2race_bringup.room_mapping_explorer_node:main',
            'scan_fixed_node = learn2race_bringup.scan_fixed_node:main',
        ],
    },
)
