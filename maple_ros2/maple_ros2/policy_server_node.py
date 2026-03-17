#!/usr/bin/env python3
"""
Policy Server Node
==================
Subscribes to camera image topics and a language instruction topic,
forwards them to a MAPLE policy container for inference, and publishes
the resulting action vector on ROS 2 topics.

This node turns any MAPLE-served VLA policy into a ROS 2-native
controller that can be wired into ros2_control or any downstream
action consumer.

Subscriptions:
  ~/image_raw       (sensor_msgs/Image)       — camera observation
  ~/instruction     (std_msgs/String)         — language instruction

Publications:
  ~/action          (maple_ros2/msg/ActionArray) — raw + transformed action
  ~/joint_command   (trajectory_msgs/JointTrajectory) — optional ros2_control compat
  ~/status          (maple_ros2/msg/PolicyStatus)

Parameters:
  daemon_host      (str)   MAPLE daemon hostname
  daemon_port      (int)   MAPLE daemon port
  policy_id        (str)   ID of a serving policy (from `maple serve policy`)
  default_instruction (str) fallback instruction if no /instruction msg received
  publish_rate     (float) max Hz for inference loop (0 = as fast as possible)
  image_encoding   (str)   expected encoding of incoming images ("rgb8", "bgr8")
  joint_names      (str[]) joint names for JointTrajectory output
  model_kwargs     (str)   JSON string of model_kwargs (e.g. '{"unnorm_key":"libero_spatial"}')
"""

import json
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from maple_ros2_msgs.msg import ActionArray, PolicyStatus
from maple_ros2.maple_client import MapleDaemonClient, numpy_to_b64


class PolicyServerNode(Node):
    """Bridges a MAPLE policy container into the ROS 2 topic graph."""

    def __init__(self):
        super().__init__("maple_policy_server")

        # ---- Parameters ----
        self.declare_parameter("daemon_host", "localhost")
        self.declare_parameter("daemon_port", 8000)
        self.declare_parameter("policy_id", "")
        self.declare_parameter("default_instruction", "")
        self.declare_parameter("publish_rate", 0.0)
        self.declare_parameter("image_encoding", "rgb8")
        self.declare_parameter("joint_names", [""])
        self.declare_parameter("model_kwargs", "{}")

        host = self.get_parameter("daemon_host").value
        port = self.get_parameter("daemon_port").value
        self._policy_id = self.get_parameter("policy_id").value
        self._default_instruction = self.get_parameter("default_instruction").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._image_encoding = self.get_parameter("image_encoding").value
        self._joint_names = self.get_parameter("joint_names").value
        model_kwargs_str = self.get_parameter("model_kwargs").value

        try:
            self._model_kwargs = json.loads(model_kwargs_str)
        except json.JSONDecodeError:
            self._model_kwargs = {}
            self.get_logger().warn(f"Could not parse model_kwargs: {model_kwargs_str}")

        self._client = MapleDaemonClient(host=host, port=port)

        # ---- State ----
        self._latest_image: np.ndarray | None = None
        self._current_instruction: str = self._default_instruction
        self._step_index: int = 0
        self._total_inference_ms: float = 0.0
        self._total_inferences: int = 0
        self._last_inference_ms: float = 0.0
        self._running = True

        # ---- Callback groups ----
        self._sub_group = MutuallyExclusiveCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()

        # ---- Subscribers ----
        self.create_subscription(
            Image, "~/image_raw", self._image_cb, 1,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            String, "~/instruction", self._instruction_cb, 1,
            callback_group=self._sub_group,
        )

        # ---- Publishers ----
        self._action_pub = self.create_publisher(ActionArray, "~/action", 10)
        self._joint_pub = self.create_publisher(JointTrajectory, "~/joint_command", 10)
        self._status_pub = self.create_publisher(PolicyStatus, "~/status", 10)

        # ---- Inference timer ----
        if self._publish_rate > 0:
            period = 1.0 / self._publish_rate
        else:
            period = 0.01  # 100 Hz poll, gated by image arrival
        self.create_timer(period, self._inference_tick, callback_group=self._timer_group)

        # Status timer
        self.create_timer(1.0, self._publish_status)

        if not self._policy_id:
            self.get_logger().warn(
                "No policy_id set. Use: ros2 param set /maple_policy_server policy_id <id>"
            )
        else:
            self.get_logger().info(
                f"PolicyServer ready — policy={self._policy_id} "
                f"daemon={host}:{port}"
            )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image):
        """Convert sensor_msgs/Image to numpy and store."""
        h, w = msg.height, msg.width
        encoding = msg.encoding or self._image_encoding

        if encoding in ("rgb8",):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
        elif encoding in ("bgr8",):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            arr = arr[:, :, ::-1].copy()  # BGR -> RGB
        elif encoding in ("rgba8",):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))[:, :, :3]
        elif encoding in ("mono8",):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
            arr = np.stack([gray, gray, gray], axis=-1)
        else:
            self.get_logger().warn(f"Unsupported encoding: {encoding}, treating as rgb8")
            try:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            except ValueError:
                return

        self._latest_image = arr

    def _instruction_cb(self, msg: String):
        """Update the current language instruction."""
        if msg.data:
            self._current_instruction = msg.data

    # ------------------------------------------------------------------
    # Inference loop
    # ------------------------------------------------------------------

    def _inference_tick(self):
        """Called periodically: if we have a new image, run inference."""
        if self._latest_image is None:
            return
        if not self._policy_id:
            return
        if not self._current_instruction:
            return

        image = self._latest_image
        self._latest_image = None  # consume

        # Encode image for MAPLE daemon
        image_b64 = numpy_to_b64(image)

        t0 = time.monotonic()
        try:
            raw_action = self._client.act(
                policy_id=self._policy_id,
                image_b64=image_b64,
                instruction=self._current_instruction,
                model_kwargs=self._model_kwargs if self._model_kwargs else None,
            )
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            return

        dt_ms = (time.monotonic() - t0) * 1000.0
        self._last_inference_ms = dt_ms
        self._total_inference_ms += dt_ms
        self._total_inferences += 1
        self._step_index += 1

        # Publish ActionArray
        action_msg = ActionArray()
        action_msg.header = Header()
        action_msg.header.stamp = self.get_clock().now().to_msg()
        action_msg.action = [float(a) for a in raw_action]
        action_msg.transformed_action = action_msg.action  # adapter runs daemon-side
        action_msg.policy_id = self._policy_id
        action_msg.instruction = self._current_instruction
        action_msg.inference_time_ms = dt_ms
        action_msg.step_index = self._step_index
        self._action_pub.publish(action_msg)

        # Publish JointTrajectory for ros2_control compatibility
        if self._joint_names and self._joint_names != [""]:
            jt = JointTrajectory()
            jt.header.stamp = self.get_clock().now().to_msg()
            jt.joint_names = list(self._joint_names)
            pt = JointTrajectoryPoint()
            # Map action values to joint positions (truncate/pad to match joint count)
            n_joints = len(self._joint_names)
            positions = raw_action[:n_joints] if len(raw_action) >= n_joints else (
                raw_action + [0.0] * (n_joints - len(raw_action))
            )
            pt.positions = [float(p) for p in positions]
            pt.time_from_start = Duration(sec=0, nanosec=100_000_000)  # 100ms
            jt.points.append(pt)
            self._joint_pub.publish(jt)

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _publish_status(self):
        msg = PolicyStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.policy_id = self._policy_id or ""
        msg.status = "serving" if self._policy_id else "idle"
        msg.last_inference_time_ms = self._last_inference_ms
        msg.avg_inference_time_ms = (
            self._total_inference_ms / self._total_inferences
            if self._total_inferences > 0 else 0.0
        )
        msg.total_inferences = self._total_inferences
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
