"""Package installation metadata for Learn2Race control helpers."""

from glob import glob
from setuptools import setup

package_name = "learn2race_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer='Ruben Pereira',
    maintainer_email='1181915@isep.ipp.pt',
    description="PID velocity control helpers for Learn2Race.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "twist_pid_controller_node = learn2race_control.twist_pid_controller_node:main",
        ]
    },
)
