# Existing Go2 recovery policy

Copied unchanged from [FR-Net](https://github.com/lu-yidan/FR-Net), commit
`85c872d9bef0a74716d8d3fb1e89606c8a42101e`,
`logs/GO2_recovery/pretrained_recovery_v3/model_2000.pt`.
SHA-256: `e19fb55a201b99c104018763d1354bf1a52d6d4e11752a75118f360b71faf505`.
The upstream BSD-3-Clause notice is retained in [LICENSE](LICENSE).

The inference contract follows that commit's `go2_recovery_config.py`,
`go2_recovery_env.py`, `go2.py`, `go2_config.py`, `legged_robot_config.py`,
and `actor_critic.py`. The [upstream recovery guide](https://github.com/lu-yidan/FR-Net/blob/85c872d9bef0a74716d8d3fb1e89606c8a42101e/docs/go2_recovery.md)
describes the bundled checkpoint.

- ELU actor: 45 → 512 → 256 → 128 → 12, no empirical normalization
- Observations: body angular velocity × 0.25, projected gravity, zero velocity
  command, joint offsets, joint velocity × 0.05, previous action
- Leg-major FL / FR / RL / RR order, mapped explicitly to Genesis joint names
- Reference pose per leg: hip 0, thigh 1.5, calf −2.4 rad
- Actions clipped to ±10, observations to ±100; targets = reference + 0.25 × action
- PD gains 30 / 0.8, 50 Hz inference

Training used Isaac Gym. Inference here uses only verified tensor weights in
PyTorch and the existing Genesis Go2 model; Isaac Gym is not a runtime dependency.
No source framework code or pickle configuration is executed. CPU and GPU use
the same adapter. Simulation transfer is verified independently; this integration
is not a hardware controller.
