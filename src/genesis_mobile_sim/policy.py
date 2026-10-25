"""Inference adapter for the published Go2 policy; no gait is trained here.

The upstream MIT notice and the observation contract accompany the checkpoint.
"""

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

MAX_LINEAR, MAX_LATERAL, MAX_ANGULAR = 0.35, 0.25, 1.0
COMMAND_TIMEOUT = 0.5
POLICY_SHA256 = "e62b93ba457f593ea4c6895fe29c575f63b7185723e1579827202437cfb8b5bb"
# Training groups joints by motor family; Genesis 1.3.2 stores them by leg.
JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint" for joint in ("hip", "thigh", "calf") for leg in ("FL", "FR", "RL", "RR")
)
DEFAULT_ANGLES = tuple(
    math.radians(value)
    for value in (
        0,
        0,
        0,
        0,
        50,
        50,
        55,
        55,
        -90,
        -90,
        -90,
        -90,
    )
)


@dataclass
class VelocityCommand:
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    received_at: float = -math.inf

    def request(self, x: float, y: float, yaw: float, now: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, yaw, now)):
            self.velocity = (0.0, 0.0, 0.0)
            self.received_at = -math.inf
            raise ValueError("Velocity commands must contain finite numbers")
        self.velocity = tuple(
            max(-limit, min(limit, value))
            for value, limit in (
                (x, MAX_LINEAR),
                (y, MAX_LATERAL),
                (yaw, MAX_ANGULAR),
            )
        )
        self.received_at = now

    def current(self, now: float) -> tuple[float, float, float]:
        return self.velocity if now - self.received_at < COMMAND_TIMEOUT else (0.0, 0.0, 0.0)


def actor_network(torch, observations, device):
    return (
        torch.nn.Sequential(
            torch.nn.Linear(observations, 512),
            torch.nn.ELU(),
            torch.nn.Linear(512, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 12),
        )
        .to(device)
        .eval()
    )


def projected_gravity(torch, quaternion):
    w, x, y, z = quaternion.unbind()
    return torch.stack((2 * (w * y - x * z), -2 * (w * x + y * z), 2 * (x * x + y * y) - 1))


class Go2Policy:
    def __init__(self, checkpoint: Path, device) -> None:
        import torch

        self.torch = torch
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != POLICY_SHA256:
            raise ValueError("Go2 policy integrity check failed; restore the pinned checkpoint")
        # Never load upstream cfgs.pkl or execute a pickled controller.
        state = torch.load(checkpoint, map_location=device, weights_only=True)["actor_state_dict"]
        self.network = actor_network(torch, 47, device)
        self.network.load_state_dict(
            {
                key.removeprefix("mlp."): value
                for key, value in state.items()
                if key.startswith("mlp.")
            },
            strict=True,
        )
        self.mean = state["obs_normalizer._mean"]
        self.std = state["obs_normalizer._std"]
        if self.mean.shape != (1, 47) or self.std.shape != (1, 47):
            raise ValueError("Go2 policy has an incompatible observation normalizer")
        self.default = torch.tensor(DEFAULT_ANGLES, dtype=torch.float32, device=device)
        self.actions = torch.zeros(12, dtype=torch.float32, device=device)

    def reset(self):
        self.actions = self.torch.zeros_like(self.actions)

    def targets(
        self,
        angular_velocity,
        quaternion,
        joint_position,
        joint_velocity,
        command: tuple[float, float, float],
        elapsed: float,
    ):
        torch = self.torch
        gravity = projected_gravity(torch, quaternion)
        phase = 2 * math.pi * elapsed / 0.5
        observation = torch.cat(
            (
                angular_velocity * 0.5,
                gravity,
                joint_position - self.default,
                joint_velocity * 0.05,
                self.actions,
                torch.tensor(
                    (*command, math.sin(phase), math.cos(phase)),
                    dtype=torch.float32,
                    device=quaternion.device,
                ),
            )
        )
        if observation.shape != (47,) or not torch.isfinite(observation).all():
            raise ValueError("Go2 state is invalid; stopping the simulation")
        with torch.inference_mode():
            self.actions = self.network((observation.unsqueeze(0) - self.mean) / (self.std + 0.01))[
                0
            ]
        if not torch.isfinite(self.actions).all():
            raise ValueError("Go2 policy returned invalid actions; stopping the simulation")
        return self.default + 0.25 * self.actions
