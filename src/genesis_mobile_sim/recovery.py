"""Tensor-only inference for FR-Net's existing Go2 fall-recovery checkpoint."""

import hashlib
from pathlib import Path

from .policy import actor_network, projected_gravity

SHA256 = "e19fb55a201b99c104018763d1354bf1a52d6d4e11752a75118f360b71faf505"


class RecoveryPolicy:
    def __init__(self, checkpoint: Path, device):
        import torch

        self.torch = torch
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != SHA256:
            raise ValueError("Recovery policy integrity check failed")
        state = torch.load(checkpoint, map_location=device, weights_only=True)["model_state_dict"]
        self.network = actor_network(torch, 45, device)
        self.network.load_state_dict(
            {k.removeprefix("actor."): v for k, v in state.items() if k.startswith("actor.")},
            strict=True,
        )
        # FR-Net uses FL / FR / RL / RR, each hip / thigh / calf.
        self.order = torch.tensor([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], device=device)
        self.inverse = torch.argsort(self.order)
        self.default = torch.tensor([0.0, 1.5, -2.4] * 4, device=device)
        self.actions = torch.zeros(12, device=device)
        self.command = torch.zeros(3, device=device)

    def reset(self):
        self.actions = self.torch.zeros_like(self.actions)

    def targets(self, angular_velocity, quaternion, position, velocity, command, elapsed):
        torch = self.torch
        gravity = projected_gravity(torch, quaternion)
        observation = torch.cat(
            (
                angular_velocity * 0.25,
                gravity,
                self.command,
                position[self.order] - self.default,
                velocity[self.order] * 0.05,
                self.actions,
            )
        )
        if not torch.isfinite(observation).all():
            raise ValueError("Invalid recovery observations")
        with torch.inference_mode():
            self.actions = self.network(observation.clamp(-100, 100)).clamp(-10, 10)
        if not torch.isfinite(self.actions).all():
            raise ValueError("Invalid recovery actions")
        return (self.default + 0.25 * self.actions)[self.inverse]
