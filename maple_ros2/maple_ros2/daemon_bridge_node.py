#!/usr/bin/env python3
"""
Daemon Bridge Node
==================
Central bridge between the MAPLE daemon and the ROS 2 graph.

Exposes MAPLE daemon operations as ROS 2 services:
  ~/serve_policy   (maple_ros2/srv/ServePolicy)
  ~/serve_env      (maple_ros2/srv/ServeEnv)
  ~/list_policies  (maple_ros2/srv/ListPolicies)
  ~/list_envs      (maple_ros2/srv/ListEnvs)
  ~/list_tasks     (maple_ros2/srv/ListTasks)

Publishes daemon health on:
  ~/daemon_status  (diagnostic_msgs/DiagnosticArray)

Parameters:
  daemon_host  (str, default "localhost")
  daemon_port  (int, default 8000)
  health_rate  (float, default 1.0 Hz)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from maple_ros2_msgs.srv import ServePolicy, ServeEnv, ListPolicies, ListEnvs, ListTasks
from maple_ros2.maple_client import MapleDaemonClient


class DaemonBridgeNode(Node):
    """Bridges the MAPLE HTTP daemon into the ROS 2 service graph."""

    def __init__(self):
        super().__init__("maple_daemon_bridge")

        # Parameters
        self.declare_parameter("daemon_host", "localhost")
        self.declare_parameter("daemon_port", 8000)
        self.declare_parameter("health_rate", 1.0)

        host = self.get_parameter("daemon_host").value
        port = self.get_parameter("daemon_port").value
        health_rate = self.get_parameter("health_rate").value

        self._client = MapleDaemonClient(host=host, port=port)
        self._cb_group = ReentrantCallbackGroup()

        # Services
        self.create_service(
            ServePolicy, "~/serve_policy", self._serve_policy_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            ServeEnv, "~/serve_env", self._serve_env_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            ListPolicies, "~/list_policies", self._list_policies_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            ListEnvs, "~/list_envs", self._list_envs_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            ListTasks, "~/list_tasks", self._list_tasks_cb,
            callback_group=self._cb_group,
        )

        # Health publisher
        self._diag_pub = self.create_publisher(DiagnosticArray, "~/daemon_status", 10)
        self.create_timer(1.0 / health_rate, self._publish_health)

        self.get_logger().info(
            f"DaemonBridge ready — daemon at {host}:{port}"
        )

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _serve_policy_cb(self, request, response):
        try:
            result = self._client.serve_policy(
                spec=request.spec, device=request.device,
            )
            response.success = True
            response.policy_id = result.get("policy_id", "")
            response.message = f"Policy serving: {response.policy_id}"
        except Exception as e:
            response.success = False
            response.policy_id = ""
            response.message = str(e)
            self.get_logger().error(f"serve_policy failed: {e}")
        return response

    def _serve_env_cb(self, request, response):
        try:
            result = self._client.serve_env(
                name=request.name, device=request.device,
            )
            # Daemon returns env_ids as a list
            env_ids = result.get("env_ids", [])
            response.success = True
            response.env_id = env_ids[0] if env_ids else result.get("env_id", "")
            response.message = f"Env serving: {response.env_id}"
        except Exception as e:
            response.success = False
            response.env_id = ""
            response.message = str(e)
            self.get_logger().error(f"serve_env failed: {e}")
        return response

    def _list_policies_cb(self, request, response):
        try:
            status = self._client.status()
            # Available backends are the keys of POLICY_BACKENDS in MAPLE
            response.available_backends = list(
                status.get("pulled", {}).get("policies", {}).keys()
            )
            serving = status.get("serving", {}).get("policies", [])
            response.serving_ids = serving
            # We don't have backend/version per ID in status, fill empty
            response.serving_backends = [""] * len(serving)
            response.serving_versions = [""] * len(serving)
        except Exception as e:
            self.get_logger().error(f"list_policies failed: {e}")
            response.available_backends = []
            response.serving_ids = []
            response.serving_backends = []
            response.serving_versions = []
        return response

    def _list_envs_cb(self, request, response):
        try:
            status = self._client.status()
            response.available_backends = list(
                status.get("pulled", {}).get("envs", {}).keys()
            )
            serving = status.get("serving", {}).get("envs", [])
            response.serving_ids = serving
            response.serving_backends = [""] * len(serving)
        except Exception as e:
            self.get_logger().error(f"list_envs failed: {e}")
            response.available_backends = []
            response.serving_ids = []
            response.serving_backends = []
        return response

    def _list_tasks_cb(self, request, response):
        try:
            status = self._client.status()
            serving_envs = status.get("serving", {}).get("envs", [])
            # Find an env_id matching the requested name
            target_id = None
            for eid in serving_envs:
                if request.env_name in eid:
                    target_id = eid
                    break
            if target_id is None:
                response.success = False
                response.message = f"No serving env matching '{request.env_name}'"
                response.task_names = []
                response.task_instructions = []
                return response

            info = self._client.env_info(target_id)
            tasks = info.get("tasks", {})
            names = []
            instructions = []
            for suite_name, task_list in tasks.items():
                if request.suite and suite_name != request.suite:
                    continue
                for t in task_list:
                    names.append(t.get("name", ""))
                    instructions.append(t.get("instruction", ""))
            response.success = True
            response.task_names = names
            response.task_instructions = instructions
            response.message = f"Found {len(names)} tasks"
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.task_names = []
            response.task_instructions = []
            self.get_logger().error(f"list_tasks failed: {e}")
        return response

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def _publish_health(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        ds = DiagnosticStatus()
        ds.name = "maple_daemon"
        ds.hardware_id = self._client.base_url

        if self._client.ping():
            ds.level = DiagnosticStatus.OK
            ds.message = "Daemon reachable"
            try:
                st = self._client.status()
                ds.values = [
                    KeyValue(key="policies_serving",
                             value=str(len(st.get("serving", {}).get("policies", [])))),
                    KeyValue(key="envs_serving",
                             value=str(len(st.get("serving", {}).get("envs", [])))),
                ]
            except Exception:
                pass
        else:
            ds.level = DiagnosticStatus.ERROR
            ds.message = "Daemon unreachable"

        msg.status.append(ds)
        self._diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DaemonBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
