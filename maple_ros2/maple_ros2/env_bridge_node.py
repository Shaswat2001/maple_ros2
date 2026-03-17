#!/usr/bin/env python3
"""
Environment Bridge Node
=======================
Wraps a MAPLE simulation environment container into the ROS 2 topic graph.

The environment's observations are published as standard ROS messages
(sensor_msgs/Image, sensor_msgs/JointState) so that any ROS 2 node —
including the PolicyServerNode or a classical controller — can consume them.

Actions received on a subscription topic are forwarded to the MAPLE env
container's /step endpoint.

Publications:
  ~/image_raw      (sensor_msgs/Image)        — camera observation
  ~/joint_states   (sensor_msgs/JointState)   — proprioceptive state
  ~/reward         (std_msgs/Float64)          — step reward
  ~/status         (maple_ros2/msg/EnvStatus)

Subscriptions:
  ~/action_in      (maple_ros2/msg/ActionArray) — actions to execute

Services:
  ~/setup          (std_srvs-like, via custom logic)
  ~/reset          (std_srvs-like)

Parameters:
  daemon_host, daemon_port, env_id, auto_reset, step_on_action
"""

import time
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Float64, Header, String
from sensor_msgs.msg import Image, JointState

from maple_ros2_msgs.msg import ActionArray, EnvStatus
from maple_ros2.maple_client import MapleDaemonClient, b64_to_numpy


class EnvBridgeNode(Node):
    """Wraps a MAPLE env container as a ROS 2 topic publisher/subscriber."""

    def __init__(self):
        super().__init__("maple_env_bridge")

        # Parameters
        self.declare_parameter("daemon_host", "localhost")
        self.declare_parameter("daemon_port", 8000)
        self.declare_parameter("env_id", "")
        self.declare_parameter("auto_reset", True)
        self.declare_parameter("image_key", "agentview_image")
        self.declare_parameter("proprio_key", "robot0_proprio-state")

        host = self.get_parameter("daemon_host").value
        port = self.get_parameter("daemon_port").value
        self._env_id = self.get_parameter("env_id").value
        self._auto_reset = self.get_parameter("auto_reset").value
        self._image_key = self.get_parameter("image_key").value
        self._proprio_key = self.get_parameter("proprio_key").value

        self._client = MapleDaemonClient(host=host, port=port)

        # State tracking
        self._current_task = ""
        self._current_instruction = ""
        self._episode_step = 0
        self._episode_reward = 0.0
        self._episode_running = False

        # Callback groups
        self._action_group = MutuallyExclusiveCallbackGroup()
        self._svc_group = ReentrantCallbackGroup()

        # Publishers
        self._image_pub = self.create_publisher(Image, "~/image_raw", 10)
        self._joint_pub = self.create_publisher(JointState, "~/joint_states", 10)
        self._reward_pub = self.create_publisher(Float64, "~/reward", 10)
        self._done_pub = self.create_publisher(String, "~/done", 10)
        self._status_pub = self.create_publisher(EnvStatus, "~/status", 10)
        self._instruction_pub = self.create_publisher(String, "~/instruction", 10)

        # Subscribers
        self.create_subscription(
            ActionArray, "~/action_in", self._action_cb, 1,
            callback_group=self._action_group,
        )

        # Status timer
        self.create_timer(1.0, self._publish_status)

        if not self._env_id:
            self.get_logger().warn(
                "No env_id set. Use param or launch arg."
            )
        else:
            self.get_logger().info(f"EnvBridge ready — env={self._env_id}")

    # ------------------------------------------------------------------
    # Observation publishing helpers
    # ------------------------------------------------------------------

    def _publish_observation(self, observation: dict):
        """Extract images and state from MAPLE observation dict and publish."""
        stamp = self.get_clock().now().to_msg()

        # Publish camera image
        if self._image_key in observation:
            img_data = observation[self._image_key]
            try:
                if isinstance(img_data, str):
                    # base64 encoded
                    img_np = b64_to_numpy(img_data)
                elif isinstance(img_data, dict) and img_data.get("type") == "image":
                    img_np = b64_to_numpy(img_data["data"])
                else:
                    img_np = np.array(img_data, dtype=np.uint8)

                img_msg = Image()
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = "camera_link"
                img_msg.height, img_msg.width = img_np.shape[:2]
                channels = img_np.shape[2] if img_np.ndim == 3 else 1
                img_msg.encoding = "rgb8" if channels == 3 else "mono8"
                img_msg.step = img_msg.width * channels
                img_msg.data = img_np.tobytes()
                self._image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish image: {e}")

        # Publish proprioceptive state
        if self._proprio_key in observation:
            try:
                state_data = observation[self._proprio_key]
                if isinstance(state_data, str):
                    state_values = json.loads(state_data)
                elif isinstance(state_data, list):
                    state_values = state_data
                else:
                    state_values = list(state_data)

                js = JointState()
                js.header.stamp = stamp
                js.header.frame_id = "base_link"
                # Generate generic joint names if we don't know them
                js.name = [f"joint_{i}" for i in range(len(state_values))]
                js.position = [float(v) for v in state_values]
                self._joint_pub.publish(js)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish joint state: {e}")

    # ------------------------------------------------------------------
    # Action callback — the main step loop
    # ------------------------------------------------------------------

    def _action_cb(self, msg: ActionArray):
        """Receive an action from the policy server, step the env."""
        if not self._env_id:
            return
        if not self._episode_running:
            self.get_logger().warn("Received action but no episode is running")
            return

        action = list(msg.transformed_action) if msg.transformed_action else list(msg.action)

        try:
            result = self._client.step_env(self._env_id, action)
        except Exception as e:
            self.get_logger().error(f"step_env failed: {e}")
            return

        observation = result.get("observation", {})
        reward = result.get("reward", 0.0)
        terminated = result.get("terminated", False)
        truncated = result.get("truncated", False)

        self._episode_step += 1
        self._episode_reward += reward

        # Publish observation
        self._publish_observation(observation)

        # Publish reward
        reward_msg = Float64()
        reward_msg.data = reward
        self._reward_pub.publish(reward_msg)

        # Check done
        if terminated or truncated:
            done_msg = String()
            done_msg.data = json.dumps({
                "terminated": terminated,
                "truncated": truncated,
                "total_reward": self._episode_reward,
                "total_steps": self._episode_step,
                "task": self._current_task,
            })
            self._done_pub.publish(done_msg)
            self.get_logger().info(
                f"Episode done: steps={self._episode_step} "
                f"reward={self._episode_reward:.3f} success={terminated}"
            )
            self._episode_running = False

            if self._auto_reset:
                self._do_reset()

    # ------------------------------------------------------------------
    # Setup / Reset (callable via parameter updates or programmatically)
    # ------------------------------------------------------------------

    def setup_task(self, task: str, seed: int | None = None):
        """Setup the environment with a task, then reset."""
        if not self._env_id:
            self.get_logger().error("No env_id configured")
            return

        try:
            result = self._client.setup_env(self._env_id, task, seed=seed)
            self._current_task = result.get("task", task)
            self._current_instruction = result.get("instruction", "")
            self.get_logger().info(
                f"Task setup: {self._current_task} — {self._current_instruction}"
            )

            # Publish instruction so PolicyServerNode picks it up
            instr_msg = String()
            instr_msg.data = self._current_instruction
            self._instruction_pub.publish(instr_msg)

            self._do_reset(seed=seed)
        except Exception as e:
            self.get_logger().error(f"setup_task failed: {e}")

    def _do_reset(self, seed: int | None = None):
        """Reset the environment and publish initial observation."""
        try:
            result = self._client.reset_env(self._env_id, seed=seed)
            observation = result.get("observation", {})
            self._episode_step = 0
            self._episode_reward = 0.0
            self._episode_running = True
            self._publish_observation(observation)
            self.get_logger().info("Environment reset — episode started")
        except Exception as e:
            self.get_logger().error(f"reset failed: {e}")

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _publish_status(self):
        msg = EnvStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.env_id = self._env_id or ""
        msg.status = "running" if self._episode_running else "idle"
        msg.current_task = self._current_task
        msg.current_instruction = self._current_instruction
        msg.episode_step = self._episode_step
        msg.episode_reward = self._episode_reward
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnvBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
