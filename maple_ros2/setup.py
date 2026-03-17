import os
from glob import glob
from setuptools import setup, find_packages

package_name = "maple_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Register with ament index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Package manifest
        ("share/" + package_name, ["package.xml"]),
        # Launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # Config files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
        "requests",
        "numpy",
        "Pillow",
    ],
    zip_safe=True,
    maintainer="Shaswat Garg",
    maintainer_email="shaswat@maple-robotics.dev",
    description="ROS 2 wrapper for MAPLE VLA evaluation framework",
    license="MIT",
    entry_points={
        "console_scripts": [
            "daemon_bridge_node = maple_ros2.daemon_bridge_node:main",
            "policy_server_node = maple_ros2.policy_server_node:main",
            "env_bridge_node = maple_ros2.env_bridge_node:main",
            "eval_node = maple_ros2.eval_node:main",
            "diagnostics_node = maple_ros2.diagnostics_node:main",
        ],
    },
)