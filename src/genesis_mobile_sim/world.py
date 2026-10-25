"""The existing Unitree Go2 model and published walking policy in Genesis."""

from pathlib import Path
from xml.etree import ElementTree

from .environment import warehouse
from .motion import Motion
from .policy import JOINT_NAMES, Go2Policy
from .recovery import RecoveryPolicy
from .robot_model import BASE_FRAME, CAMERA_OFFSET, LASER_OFFSET, model_path

ROOT = Path(__file__).resolve().parents[2]
DT = 0.02
CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV = 320, 240, 60.0


class World:
    def __init__(self, device: str = "cpu") -> None:
        import genesis as gs
        import numpy as np

        self.gs, self.np = gs, np
        gs.init(
            backend={"cpu": gs.cpu, "amdgpu": gs.amdgpu, "cuda": gs.cuda}[device],
            logging_level="warning",
        )
        self.policy = Go2Policy(ROOT / "assets/policies/go2/model_10000.pt", gs.device)
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=DT, substeps=10),
            rigid_options=gs.options.RigidOptions(
                enable_self_collision=True, tolerance=1e-5, max_collision_pairs=200
            ),
            show_viewer=False,
            vis_options=gs.options.VisOptions(
                shadow=False, plane_reflection=False, env_separate_rigid=False
            ),
        )
        warehouse(self.scene, gs)
        self.robot = self.scene.add_entity(gs.morphs.URDF(file=str(model_path()), pos=(0, 0, 0.35)))
        self.base = self.robot.get_link(BASE_FRAME)
        self.lidar = self.scene.add_sensor(
            gs.sensors.Lidar(
                entity_idx=self.robot.idx,
                link_idx_local=self.base.idx_local,
                pos_offset=LASER_OFFSET,
                pattern=gs.sensors.SphericalPattern(angles=(list(range(-180, 180)), [0.0])),
                min_range=0.08,
                max_range=10.0,
                return_points=False,
            )
        )
        self.imu = self.scene.add_sensor(
            gs.sensors.IMU(
                entity_idx=self.robot.idx,
                link_idx_local=self.base.idx_local,
            )
        )
        self.camera = self.scene.add_sensor(
            gs.sensors.RasterizerCameraOptions(
                entity_idx=self.robot.idx,
                link_idx_local=self.base.idx_local,
                pos=CAMERA_OFFSET,
                lookat=(1.34, 0.0, 0.04),
                up=(0, 0, 1),
                res=(CAMERA_WIDTH, CAMERA_HEIGHT),
                fov=CAMERA_FOV,
            )
        )
        self.scene.build()
        self.joint_names = JOINT_NAMES
        self.motors = [self.robot.get_joint(name).dofs_idx_local[0] for name in JOINT_NAMES]
        description = ElementTree.parse(model_path())
        limits = [description.find(f"joint[@name='{name}']/limit").attrib for name in JOINT_NAMES]
        torch = self.policy.torch
        self.lower = torch.tensor([float(item["lower"]) for item in limits], device=gs.device)
        self.upper = torch.tensor([float(item["upper"]) for item in limits], device=gs.device)
        effort = np.array([float(item["effort"]) for item in limits])
        self.robot.set_dofs_force_range(-effort, effort, self.motors)
        self.robot.set_dofs_kp(np.full(12, 40.0), self.motors)
        self.robot.set_dofs_kv(np.full(12, 1.0), self.motors)
        self.robot.set_dofs_position(self.policy.default, self.motors)
        self.robot.control_dofs_position(self.policy.default, self.motors)
        self.recovery = RecoveryPolicy(
            ROOT / "assets/policies/go2/recovery/model_2000.pt", gs.device
        )
        self.motion = Motion()
        self.applied_mode = None
        self.steps = 0

    @property
    def commands(self):
        return self.motion.commands

    @property
    def upright(self):
        quaternion = self.array(self.base.get_quat())
        return float(1 - 2 * (quaternion[1] ** 2 + quaternion[2] ** 2))

    @property
    def scan_valid(self):
        return self.motion.mode not in {"recover", "fallen"} and self.upright > 0.95

    def command(self, x, y, yaw, now):
        self.motion.command(x, y, yaw, now, self.steps, self.upright > 0.92)

    def request_motion(self, mode):
        self.motion.request(mode, self.steps, self.upright > 0.92)

    def step(self, now: float) -> None:
        mode = self.motion.mode
        if mode != self.applied_mode:
            kp, kd = (
                (40.0, 1.0) if mode == "walk" else (0.0, 2.0) if mode == "fallen" else (30.0, 0.8)
            )
            self.robot.set_dofs_kp(self.np.full(12, kp), self.motors)
            self.robot.set_dofs_kv(self.np.full(12, kd), self.motors)
            if mode == "walk":
                self.policy.reset()
            elif self.applied_mode not in {"recover", "stand"} or mode == "recover":
                self.recovery.reset()
            self.applied_mode = mode
        if mode != "fallen":
            position = self.robot.get_dofs_position(self.motors)
            velocity = self.robot.get_dofs_velocity(self.motors)
            policy = self.policy if mode == "walk" else self.recovery
            targets = policy.targets(
                self.imu.read().ang_vel,
                self.base.get_quat(),
                position,
                velocity,
                self.commands.current(now),
                max(0, self.steps - self.motion.since - 1) * DT,
            )
            self.robot.control_dofs_position(targets.clamp(self.lower, self.upper), self.motors)
        self.scene.step()
        self.steps += 1
        self.motion.update(self.steps, now, self.upright, float(self.base.get_pos()[2]))

    def array(self, tensor):
        return tensor.detach().cpu().numpy()

    @property
    def sim_time(self) -> float:
        return self.steps * DT

    def close(self) -> None:
        self.gs.destroy()
