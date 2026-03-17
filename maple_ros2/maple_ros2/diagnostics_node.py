#!/usr/bin/env python3
"""
Diagnostics Node
================
Monitors all MAPLE containers and publishes health on the standard
/diagnostics topic, compatible with rqt_robot_monitor and diagnostic_aggregator.

Publications:
  /diagnostics (diagnostic_msgs/DiagnosticArray)
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from maple_ros2.maple_client import MapleDaemonClient


class DiagnosticsNode(Node):
    """Publishes MAPLE container health as ROS 2 diagnostics."""

    def __init__(self):
        super().__init__("maple_diagnostics")

        self.declare_parameter("daemon_host", "localhost")
        self.declare_parameter("daemon_port", 8000)
        self.declare_parameter("rate", 0.5)  # Hz

        host = self.get_parameter("daemon_host").value
        port = self.get_parameter("daemon_port").value
        rate = self.get_parameter("rate").value

        self._client = MapleDaemonClient(host=host, port=port)
        self._pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(f"DiagnosticsNode ready — polling {host}:{port}")

    def _tick(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Daemon-level status
        daemon_ds = DiagnosticStatus()
        daemon_ds.name = "maple/daemon"
        daemon_ds.hardware_id = self._client.base_url

        if not self._client.ping():
            daemon_ds.level = DiagnosticStatus.ERROR
            daemon_ds.message = "Daemon unreachable"
            msg.status.append(daemon_ds)
            self._pub.publish(msg)
            return

        try:
            st = self._client.status()
        except Exception as e:
            daemon_ds.level = DiagnosticStatus.WARN
            daemon_ds.message = f"Status query failed: {e}"
            msg.status.append(daemon_ds)
            self._pub.publish(msg)
            return

        daemon_ds.level = DiagnosticStatus.OK
        daemon_ds.message = "Running"
        n_policies = len(st.get("serving", {}).get("policies", []))
        n_envs = len(st.get("serving", {}).get("envs", []))
        daemon_ds.values = [
            KeyValue(key="policies_serving", value=str(n_policies)),
            KeyValue(key="envs_serving", value=str(n_envs)),
        ]
        msg.status.append(daemon_ds)

        # Per-container health from health monitor
        health_data = st.get("health_monitor", {}).get("containers", {})
        for cid, cinfo in health_data.items():
            cs = DiagnosticStatus()
            cs.name = f"maple/container/{cid[:12]}"
            cs.hardware_id = cid
            status_str = cinfo if isinstance(cinfo, str) else str(cinfo)
            if "healthy" in status_str.lower():
                cs.level = DiagnosticStatus.OK
                cs.message = "Healthy"
            elif "unhealthy" in status_str.lower():
                cs.level = DiagnosticStatus.ERROR
                cs.message = "Unhealthy"
            else:
                cs.level = DiagnosticStatus.WARN
                cs.message = status_str
            msg.status.append(cs)

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
