#!/usr/bin/env python3
"""
Evaluation Node
===============
Provides ROS 2 action servers for running MAPLE evaluations.

Action Servers:
  ~/run_eval     (maple_ros2/action/RunEval)    — batch eval across tasks × seeds
  ~/run_episode  (maple_ros2/action/RunEpisode) — single episode

Publications:
  ~/eval_progress (maple_ros2/msg/EvalProgress) — live progress updates

This node calls the MAPLE daemon's /run endpoint for each episode and
streams feedback through the ROS 2 action protocol, letting clients
monitor progress, cancel evaluations, and receive structured results.
"""

import time
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from maple_ros2_msgs.action import RunEval, RunEpisode
from maple_ros2_msgs.msg import EvalProgress
from maple_ros2.maple_client import MapleDaemonClient


class EvalNode(Node):
    """ROS 2 action server for MAPLE batch evaluation."""

    def __init__(self):
        super().__init__("maple_eval")

        self.declare_parameter("daemon_host", "localhost")
        self.declare_parameter("daemon_port", 8000)

        host = self.get_parameter("daemon_host").value
        port = self.get_parameter("daemon_port").value
        self._client = MapleDaemonClient(host=host, port=port)

        self._cb_group = ReentrantCallbackGroup()

        # Action servers
        self._eval_server = ActionServer(
            self, RunEval, "~/run_eval",
            execute_callback=self._execute_eval,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self._episode_server = ActionServer(
            self, RunEpisode, "~/run_episode",
            execute_callback=self._execute_episode,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        # Progress publisher
        self._progress_pub = self.create_publisher(EvalProgress, "~/eval_progress", 10)

        self.get_logger().info(f"EvalNode ready — daemon at {host}:{port}")

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("Eval cancel requested")
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # RunEval action
    # ------------------------------------------------------------------

    async def _execute_eval(self, goal_handle):
        """Execute a batch evaluation: tasks × seeds."""
        request = goal_handle.request
        tasks = list(request.tasks)
        seeds = list(request.seeds) if request.seeds else [0]
        max_steps = request.max_steps if request.max_steps > 0 else 300

        total = len(tasks) * len(seeds)
        completed = 0
        successes = 0
        all_results = []
        batch_id = f"ros-eval-{int(time.time())}"
        t_start = time.monotonic()

        self.get_logger().info(
            f"Starting batch eval: {len(tasks)} tasks × {len(seeds)} seeds = {total} episodes"
        )

        for task in tasks:
            for seed in seeds:
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Eval cancelled")
                    result = RunEval.Result()
                    result.batch_id = batch_id
                    result.total_episodes = completed
                    result.successful_episodes = successes
                    result.success_rate = successes / completed if completed > 0 else 0.0
                    return result

                # Run episode
                try:
                    ep_result = self._client.run_episode(
                        policy_id=request.policy_id,
                        env_id=request.env_id,
                        task=task,
                        seed=seed,
                        max_steps=max_steps,
                        save_video=request.save_video,
                    )
                    ep_success = ep_result.get("success", False)
                    ep_reward = ep_result.get("total_reward", 0.0)
                    ep_steps = ep_result.get("steps", 0)
                except Exception as e:
                    self.get_logger().error(f"Episode failed: {task} seed={seed}: {e}")
                    ep_success = False
                    ep_reward = 0.0
                    ep_steps = 0
                    ep_result = {"error": str(e)}

                completed += 1
                if ep_success:
                    successes += 1
                all_results.append(ep_result)

                current_rate = successes / completed

                # Publish feedback
                feedback = RunEval.Feedback()
                feedback.completed = completed
                feedback.total = total
                feedback.last_task = task
                feedback.last_seed = seed
                feedback.last_success = ep_success
                feedback.last_reward = ep_reward
                feedback.current_success_rate = current_rate
                goal_handle.publish_feedback(feedback)

                # Publish progress topic
                prog = EvalProgress()
                prog.stamp = self.get_clock().now().to_msg()
                prog.batch_id = batch_id
                prog.completed_episodes = completed
                prog.total_episodes = total
                prog.current_success_rate = current_rate
                prog.last_task = task
                prog.last_success = ep_success
                prog.last_reward = ep_reward
                prog.last_steps = ep_steps
                prog.elapsed_seconds = time.monotonic() - t_start
                self._progress_pub.publish(prog)

                status_icon = "✓" if ep_success else "✗"
                self.get_logger().info(
                    f"[{completed}/{total}] {status_icon} {task} s={seed} "
                    f"r={ep_reward:.3f} rate={current_rate:.1%}"
                )

        # Build result
        elapsed = time.monotonic() - t_start
        goal_handle.succeed()

        result = RunEval.Result()
        result.batch_id = batch_id
        result.total_episodes = total
        result.successful_episodes = successes
        result.failed_episodes = total - successes
        result.success_rate = successes / total if total > 0 else 0.0
        result.avg_reward = (
            sum(r.get("total_reward", 0.0) for r in all_results) / total
            if total > 0 else 0.0
        )
        result.avg_steps = (
            sum(r.get("steps", 0) for r in all_results) / total
            if total > 0 else 0.0
        )
        result.total_duration_seconds = elapsed
        result.results_json = json.dumps(all_results, default=str)

        self.get_logger().info(
            f"Batch complete: {result.success_rate:.1%} success in {elapsed:.1f}s"
        )
        return result

    # ------------------------------------------------------------------
    # RunEpisode action
    # ------------------------------------------------------------------

    async def _execute_episode(self, goal_handle):
        """Execute a single evaluation episode."""
        request = goal_handle.request
        max_steps = request.max_steps if request.max_steps > 0 else 300

        self.get_logger().info(
            f"Running episode: {request.task} seed={request.seed} on "
            f"{request.policy_id} / {request.env_id}"
        )

        t_start = time.monotonic()
        try:
            ep_result = self._client.run_episode(
                policy_id=request.policy_id,
                env_id=request.env_id,
                task=request.task,
                instruction=request.instruction if request.instruction else None,
                seed=request.seed,
                max_steps=max_steps,
                save_video=request.save_video,
            )
        except Exception as e:
            self.get_logger().error(f"Episode failed: {e}")
            goal_handle.abort()
            result = RunEpisode.Result()
            result.success = False
            return result

        elapsed = time.monotonic() - t_start

        goal_handle.succeed()
        result = RunEpisode.Result()
        result.run_id = ep_result.get("run_id", "")
        result.success = ep_result.get("success", False)
        result.steps = ep_result.get("steps", 0)
        result.total_reward = ep_result.get("total_reward", 0.0)
        result.terminated = ep_result.get("terminated", False)
        result.truncated = ep_result.get("truncated", False)
        result.video_path = ep_result.get("video_path", "") or ""
        result.duration_seconds = elapsed

        icon = "✓" if result.success else "✗"
        self.get_logger().info(
            f"Episode {icon}: steps={result.steps} reward={result.total_reward:.3f} "
            f"in {elapsed:.1f}s"
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = EvalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
