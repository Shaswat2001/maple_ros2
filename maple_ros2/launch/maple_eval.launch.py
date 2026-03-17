"""
maple_eval.launch.py
=====================
Lightweight launch for batch evaluation only.
Brings up daemon bridge + eval action server.

Usage:
  ros2 launch maple_ros2 maple_eval.launch.py

Then trigger evaluations via:
  ros2 action send_goal /maple_eval/run_eval maple_ros2_msgs/action/RunEval \
      '{policy_id: "openvla-7b-xxx", env_id: "libero-yyy", \
        tasks: ["libero_spatial_task_0"], seeds: [0,1,2], max_steps: 300}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("daemon_host", default_value="localhost"),
        DeclareLaunchArgument("daemon_port", default_value="8000"),
    ]

    daemon_bridge = Node(
        package="maple_ros2",
        executable="daemon_bridge_node.py",
        name="maple_daemon_bridge",
        output="screen",
        parameters=[{
            "daemon_host": LaunchConfiguration("daemon_host"),
            "daemon_port": LaunchConfiguration("daemon_port"),
        }],
    )

    eval_node = Node(
        package="maple_ros2",
        executable="eval_node.py",
        name="maple_eval",
        output="screen",
        parameters=[{
            "daemon_host": LaunchConfiguration("daemon_host"),
            "daemon_port": LaunchConfiguration("daemon_port"),
        }],
    )

    diagnostics = Node(
        package="maple_ros2",
        executable="diagnostics_node.py",
        name="maple_diagnostics",
        output="screen",
        parameters=[{
            "daemon_host": LaunchConfiguration("daemon_host"),
            "daemon_port": LaunchConfiguration("daemon_port"),
        }],
    )

    return LaunchDescription(args + [daemon_bridge, eval_node, diagnostics])
