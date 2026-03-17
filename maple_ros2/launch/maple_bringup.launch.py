"""
maple_bringup.launch.py
========================
Brings up the full MAPLE ROS 2 wrapper stack:
  1. Daemon bridge  — services for managing policies/envs
  2. Diagnostics    — /diagnostics health monitoring
  3. (optional) Policy server — image→action inference loop
  4. (optional) Env bridge     — env obs/action ROS topics
  5. (optional) Eval node      — action servers for batch eval

Usage:
  # Minimal (just daemon bridge + diagnostics):
  ros2 launch maple_ros2 maple_bringup.launch.py

  # With policy + env for closed-loop:
  ros2 launch maple_ros2 maple_bringup.launch.py \
      policy_id:=openvla-7b-a1b2c3d4 \
      env_id:=libero-x1y2z3w4 \
      model_kwargs:='{"unnorm_key":"libero_spatial"}'

  # Full stack including eval node:
  ros2 launch maple_ros2 maple_bringup.launch.py \
      policy_id:=openvla-7b-a1b2c3d4 \
      env_id:=libero-x1y2z3w4 \
      enable_eval:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("maple_ros2")
    config_file = os.path.join(pkg_share, "config", "maple_ros2.yaml")

    # ── Launch arguments ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("daemon_host", default_value="localhost"),
        DeclareLaunchArgument("daemon_port", default_value="8000"),
        DeclareLaunchArgument("policy_id", default_value="",
                              description="MAPLE policy container ID"),
        DeclareLaunchArgument("env_id", default_value="",
                              description="MAPLE env container ID"),
        DeclareLaunchArgument("default_instruction", default_value=""),
        DeclareLaunchArgument("model_kwargs", default_value="{}"),
        DeclareLaunchArgument("image_key", default_value="agentview_image"),
        DeclareLaunchArgument("proprio_key", default_value="robot0_proprio-state"),
        DeclareLaunchArgument("image_encoding", default_value="rgb8"),
        DeclareLaunchArgument("enable_eval", default_value="false",
                              description="Launch eval action server node"),
        DeclareLaunchArgument("enable_env_bridge", default_value="true",
                              description="Launch env bridge (disable if using external sim)"),
    ]

    # ── Always-on nodes ───────────────────────────────────────────────
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

    # ── Policy server (launched when policy_id is non-empty) ──────────
    policy_server = Node(
        package="maple_ros2",
        executable="policy_server_node.py",
        name="maple_policy_server",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("policy_id"), "' != ''"])
        ),
        parameters=[{
            "daemon_host": LaunchConfiguration("daemon_host"),
            "daemon_port": LaunchConfiguration("daemon_port"),
            "policy_id": LaunchConfiguration("policy_id"),
            "default_instruction": LaunchConfiguration("default_instruction"),
            "image_encoding": LaunchConfiguration("image_encoding"),
            "model_kwargs": LaunchConfiguration("model_kwargs"),
        }],
        remappings=[
            # Wire policy image input to env bridge image output
            ("~/image_raw", "/maple_env_bridge/image_raw"),
            ("~/instruction", "/maple_env_bridge/instruction"),
        ],
    )

    # ── Env bridge (launched when env_id is non-empty) ────────────────
    env_bridge = Node(
        package="maple_ros2",
        executable="env_bridge_node.py",
        name="maple_env_bridge",
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("env_id"), "' != '' and '",
                LaunchConfiguration("enable_env_bridge"), "' == 'true'",
            ])
        ),
        parameters=[{
            "daemon_host": LaunchConfiguration("daemon_host"),
            "daemon_port": LaunchConfiguration("daemon_port"),
            "env_id": LaunchConfiguration("env_id"),
            "image_key": LaunchConfiguration("image_key"),
            "proprio_key": LaunchConfiguration("proprio_key"),
        }],
        remappings=[
            # Wire env action input from policy server output
            ("~/action_in", "/maple_policy_server/action"),
        ],
    )

    # ── Eval node (opt-in) ────────────────────────────────────────────
    eval_node = Node(
        package="maple_ros2",
        executable="eval_node.py",
        name="maple_eval",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_eval")),
        parameters=[{
            "daemon_host": LaunchConfiguration("daemon_host"),
            "daemon_port": LaunchConfiguration("daemon_port"),
        }],
    )

    return LaunchDescription(args + [
        daemon_bridge,
        diagnostics,
        policy_server,
        env_bridge,
        eval_node,
    ])
