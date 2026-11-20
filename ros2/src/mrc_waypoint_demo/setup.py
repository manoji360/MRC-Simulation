"""Python package setup for mrc_waypoint_demo (ament_python): installs the node, launch file and package index marker."""

from glob import glob

from setuptools import setup

PACKAGE = "mrc_waypoint_demo"
"""ROS package name."""

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Minimal ROS 2 waypoint follower for the Unity rover simulation.",
    license="MIT",
    entry_points={"console_scripts": [f"waypoint_follower = {PACKAGE}.waypoint_follower:main"]},
)
