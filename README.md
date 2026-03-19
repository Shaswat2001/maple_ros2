# MAPLE ROS2
<p align="center">
  <img src="docs/maple_ros2_banner.svg" alt="MAPLE ⬥ ROS 2" width="700"/>
</p>

<p align="center">
  <a href="https://github.com/Shaswat2001/maple_ros2/actions"><img src="https://img.shields.io/github/actions/workflow/status/Shaswat2001/maple_ros2/ci.yml?branch=main&style=flat-square&label=CI" alt="CI"></a>
  <a href="https://opensource.org/licenses/MIT"><img src="https://img.shields.io/badge/License-MIT-green.svg?style=flat-square" alt="License: MIT"></a>
  <a href="https://github.com/Shaswat2001/maple-robotics"><img src="https://img.shields.io/badge/MAPLE-v0.1-orange?style=flat-square&logo=github" alt="MAPLE"></a>
  <a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS_2-Jazzy%20|%20Kilted-blue?style=flat-square&logo=ros" alt="ROS 2"></a>
</p>

---

**maple_ros2** bridges [MAPLE](https://github.com/Shaswat2001/maple-robotics)'s containerized VLA policy serving and simulation environments into the ROS 2 ecosystem — exposing inference as standard topics, evaluation as action servers, and container health as diagnostics.

> **One launch command.** Fully `rosbag`-recordable. `rqt`-monitorable. `ros2_control`-compatible.

---

## Why?

VLA models (OpenVLA, SmolVLA, π₀, GR00T N1.5) are exploding in number. Evaluating them is a mess:

| Without maple_ros2 | With maple_ros2 |
|---|---|
| Custom glue code per policy × env pair | One adapter registry handles all combos |
| Conflicting CUDA / Python deps | Docker containerizes everything |
| Bespoke eval scripts, no reproducibility | `ros2 action send_goal` with structured results |
| No integration with ROS tooling | `rosbag record`, `rqt_robot_monitor`, `ros2_control` out of the box |
| Results stuck in JSON files | Stream via topics, visualize in `rqt`, record with `rosbag` |

---

## Quick Start

### Prerequisites

- **ROS 2** Jazzy or Kilted (Ubuntu 24.04)
- **[MAPLE](https://github.com/Shaswat2001/maple-robotics)** installed (`pip install maple-robotics`)
- Docker with NVIDIA GPU support
- Python 3.10+

### Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/Shaswat2001/maple_ros2.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select maple_ros2_msgs maple_ros2
source install/setup.bash
```

### Run

```bash
# Terminal 1 — Start MAPLE daemon + containers
maple serve --detach
maple pull policy openvla:7b && maple serve policy openvla:7b
maple pull env libero       && maple serve env libero
```

```bash
# Terminal 2 — Launch the ROS 2 bridge
ros2 launch maple_ros2 maple_bringup.launch.py \
    policy_id:=openvla-7b-a1b2c3d4 \
    env_id:=libero-x1y2z3w4 \
    model_kwargs:='{"unnorm_key":"libero_spatial"}'
```

```bash
# Terminal 3 — Run a batch evaluation
ros2 launch maple_ros2 maple_eval.launch.py

ros2 action send_goal /maple_eval/run_eval maple_ros2_msgs/action/RunEval \
    "{policy_id: 'openvla-7b-a1b2c3d4', env_id: 'libero-x1y2z3w4',
      tasks: ['libero_spatial_task_0', 'libero_spatial_task_1'],
      seeds: [0, 1, 2], max_steps: 300}" --feedback
```

```bash
# Terminal 4 — Monitor with standard ROS tools
ros2 topic echo /maple_policy_server/status
ros2 topic echo /maple_eval/eval_progress
ros2 run rqt_robot_monitor rqt_robot_monitor
ros2 bag record -a -o maple_eval_bag
```

---

## Packages

This workspace follows ROS 2 best practices with two packages:

| Package | Build Type | Purpose |
|---|---|---|
| `maple_ros2_msgs` | `ament_cmake` | Custom `.msg`, `.srv`, `.action` definitions |
| `maple_ros2` | `ament_python` | Nodes, launch files, config |

### Nodes

| Node | What it does |
|---|---|
| **DaemonBridgeNode** | Exposes MAPLE daemon as ROS 2 services (`serve_policy`, `serve_env`, `list_*`) + diagnostics |
| **PolicyServerNode** | Subscribes to `sensor_msgs/Image` + instruction, runs VLA inference, publishes `ActionArray` + `JointTrajectory` |
| **EnvBridgeNode** | Publishes env observations as `Image` + `JointState`, consumes actions, handles episode lifecycle |
| **EvalNode** | Action servers for batch eval (`RunEval`) and single episodes (`RunEpisode`) with streaming feedback |
| **DiagnosticsNode** | Publishes MAPLE container health on `/diagnostics` — compatible with `rqt_robot_monitor` |

### Custom Interfaces

**Messages**

| Message | Description |
|---|---|
| `ActionArray` | Policy action output with raw + transformed actions, inference timing |
| `PolicyStatus` | Container status, inference count, avg latency |
| `EnvStatus` | Current task, episode step/reward, container status |
| `EvalProgress` | Batch eval progress: completed/total, running success rate |

**Services**

| Service | Description |
|---|---|
| `ServePolicy` / `ServeEnv` | Start a MAPLE container |
| `ListPolicies` / `ListEnvs` | Query available and running containers |
| `ListTasks` | List tasks for a serving environment |

**Actions**

| Action | Description |
|---|---|
| `RunEval` | Batch evaluation (tasks × seeds) with per-episode feedback |
| `RunEpisode` | Single episode with per-step feedback |

---

## Supported Policies & Environments

Inherits everything MAPLE supports:

| Policies | Environments |
|---|---|
| OpenVLA (7B) | LIBERO |
| SmolVLA | RoboCasa |
| OpenPI (π₀) | Bridge |
| GR00T N1.5 | Fractal |
| | AlohaSim |

Full list: [MAPLE Policies & Environments Reference](https://maple-robotics.readthedocs.io/en/latest/guides/policy_envs.html)

---

## ros2_control Integration

The `PolicyServerNode` publishes `trajectory_msgs/JointTrajectory` on `~/joint_command`. Wire it into your controller:

```bash
ros2 launch maple_ros2 maple_bringup.launch.py \
    policy_id:=openvla-7b-xxx \
    --ros-args \
    -p maple_policy_server.joint_names:="['j1','j2','j3','j4','j5','j6','gripper']" \
    -r /maple_policy_server/joint_command:=/joint_trajectory_controller/joint_trajectory
```

---

## Configuration

All parameters live in [`config/maple_ros2.yaml`](config/maple_ros2.yaml). Override at launch:

```bash
ros2 launch maple_ros2 maple_bringup.launch.py \
    daemon_host:=192.168.1.100 \
    daemon_port:=8000 \
    policy_id:=smolvla-base-abcd1234 \
    image_key:=agentview_image \
    enable_eval:=true
```

| Parameter | Default | Description |
|---|---|---|
| `daemon_host` | `localhost` | MAPLE daemon hostname |
| `daemon_port` | `8000` | MAPLE daemon port |
| `policy_id` | `""` | ID of serving policy container |
| `env_id` | `""` | ID of serving env container |
| `model_kwargs` | `"{}"` | JSON string (e.g. `'{"unnorm_key":"libero_spatial"}'`) |
| `image_key` | `agentview_image` | Observation dict key for camera image |
| `publish_rate` | `0.0` | Max inference Hz (0 = fast as images arrive) |

---

## Testing

```bash
colcon build --packages-select maple_ros2_msgs maple_ros2
colcon test --packages-select maple_ros2
colcon test-result --verbose
```

---

## Roadmap

- [ ] Gazebo environment backend (native ROS 2 sim alongside MAPLE Docker envs)
- [ ] `ros2_tracing` integration for VLA inference latency profiling
- [ ] World-model-based runtime safety monitor node
- [ ] Latency benchmark suite across policies and hardware
- [ ] Real-robot deployment examples (SO-100, Franka)

---

## Related Projects

| Project | Relationship |
|---|---|
| [MAPLE](https://github.com/Shaswat2001/maple-robotics) | Underlying evaluation framework |
| [OpenVLA](https://github.com/openvla/openvla) | Supported VLA model |
| [LeRobot](https://github.com/huggingface/lerobot) | Robot learning framework |
| [ros2_control](https://github.com/ros-controls/ros2_control) | Control framework we integrate with |

---

## Citation

```bibtex
@software{maple_ros2,
  title={maple\_ros2: Containerized VLA Policy Evaluation for the ROS 2 Ecosystem},
  author={Garg, Shaswat},
  year={2026},
  url={https://github.com/Shaswat2001/maple_ros2}
}
```

---

<p align="center">
  <b>MIT License</b> · Built for <a href="https://roscon.ros.org/2026/">ROSCon Global 2026</a> in Toronto
</p>
