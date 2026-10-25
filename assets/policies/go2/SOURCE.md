# Existing Go2 walking policy

The checkpoint is copied unchanged from
[`GaganBhusal/unitree-go2-sim2real`](https://github.com/GaganBhusal/unitree-go2-sim2real),
commit `79bc6d27152ec9aba09032fae7203ce14159ba26`,
`logs/go2_walk_seven/model_10000.pt` (iteration 10000).
Its SHA-256 is `e62b93ba457f593ea4c6895fe29c575f63b7185723e1579827202437cfb8b5bb`.
The upstream MIT notice is retained in [LICENSE](LICENSE).

The inference adapter follows the same commit's `configs/robot_config.py`,
`configs/sim_config.py`, `configs/task_config.py`, `configs/train_params.yaml`,
`src/env/base_env.py`, and `src/env/env_walk_feet_rewards.py`:

- 47 actor observations: body angular velocity × 0.5, projected gravity, joint
  offsets from the default pose, joint velocity × 0.05, previous actions,
  commanded x/y/yaw velocity, and sine/cosine gait phase (0.5 s period)
- Saved observation mean and standard deviation, with the upstream rsl-rl
  empirical-normalization epsilon of 0.01
- ELU actor network with layers 47 → 512 → 256 → 128 → 12
- Joint position targets: standing pose + 0.25 × actor output
- PD gains 40 / 1; 50 Hz control with ten physics substeps
- Motor-family order: FL/FR/RL/RR hips, then thighs, then calves

The adapter explicitly maps joint names because Genesis now stores them by leg.
It loads tensor data with `weights_only=True`, verifies the checkpoint hash,
and does not import the training framework or load upstream pickle configs.
No training, hand-written gait, or root-position animation is used. The optional CUDA
backend accelerates the same inference adapter.

The robot is the existing `urdf/go2/urdf/go2.urdf` and mesh set bundled with the
pinned Genesis 1.3.2 package, which is also the model named by the upstream policy
configuration. Physics loads it unchanged. The ROS description adds only virtual
lidar and camera attachment frames and resource URLs for the existing meshes.
