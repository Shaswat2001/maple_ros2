"""
MAPLE daemon HTTP client for ROS 2 nodes.

Provides a thin Python wrapper around the MAPLE daemon's REST API,
used by all maple_ros2 nodes to communicate with the daemon process.
"""

import io
import base64
import time
import requests
import numpy as np
from typing import Any, Dict, List, Optional
from PIL import Image


class MapleDaemonClient:
    """HTTP client for the MAPLE daemon REST API."""

    def __init__(self, host: str = "localhost", port: int = 8000, timeout: int = 30):
        self._base_url = f"http://{host}:{port}"
        self._timeout = timeout

    @property
    def base_url(self) -> str:
        return self._base_url

    # ------------------------------------------------------------------
    # Health / status
    # ------------------------------------------------------------------

    def ping(self) -> bool:
        """Return True if daemon is reachable."""
        try:
            resp = requests.get(f"{self._base_url}/status", timeout=5)
            return resp.status_code == 200
        except requests.exceptions.RequestException:
            return False

    def status(self) -> Dict[str, Any]:
        """Get full daemon status."""
        resp = requests.get(f"{self._base_url}/status", timeout=self._timeout)
        resp.raise_for_status()
        return resp.json()

    # ------------------------------------------------------------------
    # Policy management
    # ------------------------------------------------------------------

    def pull_policy(self, spec: str) -> Dict[str, Any]:
        """Pull a policy (image + weights). spec e.g. 'openvla:7b'."""
        resp = requests.post(
            f"{self._base_url}/pull_policy",
            json={"spec": spec},
            timeout=600,  # model download can be very slow
        )
        resp.raise_for_status()
        return resp.json()

    def serve_policy(
        self, spec: str, device: str = "cuda:0", host_port: Optional[int] = None,
        model_load_kwargs: Optional[Dict] = None,
    ) -> Dict[str, Any]:
        """Start serving a policy container."""
        payload = {"spec": spec, "device": device}
        if host_port is not None:
            payload["host_port"] = host_port
        if model_load_kwargs:
            payload["model_load_kwargs"] = model_load_kwargs
        resp = requests.post(
            f"{self._base_url}/serve_policy",
            json=payload,
            timeout=600,
        )
        resp.raise_for_status()
        return resp.json()

    def stop_policy(self, policy_id: str) -> Dict[str, Any]:
        """Stop a serving policy container."""
        resp = requests.post(
            f"{self._base_url}/stop_policy",
            json={"policy_id": policy_id},
            timeout=self._timeout,
        )
        resp.raise_for_status()
        return resp.json()

    def act(
        self,
        policy_id: str,
        image_b64: str,
        instruction: str,
        model_kwargs: Optional[Dict] = None,
    ) -> List[float]:
        """Single policy inference via the daemon /act endpoint."""
        payload = {
            "policy_id": policy_id,
            "image": image_b64,
            "instruction": instruction,
        }
        if model_kwargs:
            payload["model_kwargs"] = model_kwargs
        resp = requests.post(
            f"{self._base_url}/act",
            json=payload,
            timeout=300,
        )
        resp.raise_for_status()
        return resp.json().get("action", [])

    # ------------------------------------------------------------------
    # Environment management
    # ------------------------------------------------------------------

    def serve_env(
        self, name: str, device: str = "cpu", num_envs: int = 1,
    ) -> Dict[str, Any]:
        """Start serving environment container(s)."""
        resp = requests.post(
            f"{self._base_url}/serve_env",
            json={"name": name, "device": device, "num_envs": num_envs},
            timeout=300,
        )
        resp.raise_for_status()
        return resp.json()

    def stop_env(self, env_id: str) -> Dict[str, Any]:
        """Stop a serving environment container."""
        resp = requests.post(
            f"{self._base_url}/stop_env",
            json={"env_id": env_id},
            timeout=self._timeout,
        )
        resp.raise_for_status()
        return resp.json()

    def setup_env(
        self, env_id: str, task: str, seed: Optional[int] = None,
        env_kwargs: Optional[Dict] = None,
    ) -> Dict[str, Any]:
        """Setup environment with a task."""
        payload: Dict[str, Any] = {"env_id": env_id, "task": task}
        if seed is not None:
            payload["seed"] = seed
        if env_kwargs:
            payload["env_kwargs"] = env_kwargs
        resp = requests.post(
            f"{self._base_url}/setup_env",
            json=payload,
            timeout=60,
        )
        resp.raise_for_status()
        return resp.json()

    def reset_env(self, env_id: str, seed: Optional[int] = None) -> Dict[str, Any]:
        """Reset environment, returns initial observation."""
        payload: Dict[str, Any] = {"env_id": env_id}
        if seed is not None:
            payload["seed"] = seed
        resp = requests.post(
            f"{self._base_url}/reset_env",
            json=payload,
            timeout=30,
        )
        resp.raise_for_status()
        return resp.json()

    def step_env(self, env_id: str, action: List[float]) -> Dict[str, Any]:
        """Step environment, returns observation, reward, terminated, truncated."""
        resp = requests.post(
            f"{self._base_url}/step_env",
            json={"env_id": env_id, "action": action},
            timeout=30,
        )
        resp.raise_for_status()
        return resp.json()

    def env_info(self, env_id: str) -> Dict[str, Any]:
        """Get environment metadata."""
        resp = requests.post(
            f"{self._base_url}/env_info",
            json={"env_id": env_id},
            timeout=10,
        )
        resp.raise_for_status()
        return resp.json()

    # ------------------------------------------------------------------
    # Evaluation (run)
    # ------------------------------------------------------------------

    def run_episode(
        self,
        policy_id: str,
        env_id: str,
        task: str,
        instruction: Optional[str] = None,
        max_steps: int = 300,
        seed: Optional[int] = None,
        model_kwargs: Optional[Dict] = None,
        env_kwargs: Optional[Dict] = None,
        save_video: bool = False,
        video_dir: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Run a single evaluation episode via the daemon /run endpoint."""
        payload: Dict[str, Any] = {
            "policy_id": policy_id,
            "env_id": env_id,
            "task": task,
            "max_steps": max_steps,
            "save_video": save_video,
        }
        if instruction:
            payload["instruction"] = instruction
        if seed is not None:
            payload["seed"] = seed
        if model_kwargs:
            payload["model_kwargs"] = model_kwargs
        if env_kwargs:
            payload["env_kwargs"] = env_kwargs
        if video_dir:
            payload["video_dir"] = video_dir

        resp = requests.post(
            f"{self._base_url}/run",
            json=payload,
            timeout=max_steps * 5,  # generous timeout
        )
        resp.raise_for_status()
        return resp.json()

    # ------------------------------------------------------------------
    # Daemon lifecycle
    # ------------------------------------------------------------------

    def stop_daemon(self) -> Dict[str, Any]:
        """Gracefully stop the daemon."""
        resp = requests.post(f"{self._base_url}/stop", timeout=10)
        resp.raise_for_status()
        return resp.json()


# ------------------------------------------------------------------
# Image encoding utilities
# ------------------------------------------------------------------

def numpy_to_b64(img_array: np.ndarray, fmt: str = "PNG") -> str:
    """Encode a numpy HxWxC uint8 image to base64 string."""
    pil_img = Image.fromarray(img_array.astype(np.uint8))
    buf = io.BytesIO()
    pil_img.save(buf, format=fmt)
    return base64.b64encode(buf.getvalue()).decode("utf-8")


def b64_to_numpy(b64_str: str) -> np.ndarray:
    """Decode a base64 image string to numpy HxWxC uint8 array."""
    img_bytes = base64.b64decode(b64_str)
    pil_img = Image.open(io.BytesIO(img_bytes))
    return np.array(pil_img)
