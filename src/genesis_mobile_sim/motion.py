"""Walking/recovery handoff without teleporting the robot or resetting SLAM."""

from dataclasses import dataclass, field

from .policy import COMMAND_TIMEOUT, VelocityCommand


@dataclass
class Motion:
    mode: str = "stand"
    reason: str = "Standing with the recovery policy"
    since: int = 0
    stable_steps: int = 0
    commands: VelocityCommand = field(default_factory=VelocityCommand)

    def transition(self, mode: str, steps: int, reason: str) -> None:
        self.mode, self.since, self.reason = mode, steps, reason
        self.stable_steps = 0

    def request(self, mode: str, steps: int, upright: bool) -> None:
        if mode not in {"stand", "recover"}:
            raise ValueError("Go2 motion must be 'stand' or 'recover'")
        if mode == "stand" and not upright:
            raise ValueError("Go2 is not upright; request recovery first")
        self.commands = VelocityCommand()
        self.transition(
            mode, steps, "Recovery requested" if mode == "recover" else "Stand requested"
        )

    def command(self, x: float, y: float, yaw: float, now: float, steps: int, upright: bool):
        self.commands.request(x, y, yaw, now)
        if any(self.commands.current(now)):
            if self.mode in {"recover", "fallen"} or not upright:
                self.commands = VelocityCommand()
                raise ValueError("Walking is unavailable until Go2 stands upright")
            if self.mode == "stand":
                self.transition("walk", steps, "Tracking velocity commands")

    def update(self, steps: int, now: float, upright: float, height: float) -> None:
        if self.mode == "recover":
            self.stable_steps = self.stable_steps + 1 if upright > 0.92 and height > 0.26 else 0
            if steps - self.since >= 50 and self.stable_steps >= 25:
                self.transition("stand", steps, "Recovery complete")
            elif steps - self.since >= 600:
                self.transition(
                    "fallen", steps, "Recovery timed out; clear nearby obstacles and retry"
                )
        elif self.mode != "fallen":
            if steps > 50 and (upright < 0.5 or height < 0.12):
                self.commands = VelocityCommand()
                self.transition("fallen", steps, "Fall detected; request recovery")
            elif self.mode == "walk" and now - self.commands.received_at >= COMMAND_TIMEOUT:
                self.transition("stand", steps, "Command expired; standing")
