# Genesis Go2 simulation

Walk the existing Unitree Go2 robot dog, build a map from its lidar, and navigate through a warehouse inspection course.
Genesis runs the physics and sensors. ROS 2 Jazzy, SLAM Toolbox, and Nav2 provide the same
mapping and navigation interfaces you can use on a physical robot. The robot model comes
with Genesis; the [published walking policy](assets/policies/go2/SOURCE.md) is reused
unchanged. A second [published recovery policy](assets/policies/go2/recovery/SOURCE.md)
stands it up after a fall and maintains its resting stance. This project does not
design a robot or train a gait.

The default image uses CPU physics and Mesa software rendering. Optional NVIDIA and
AMD overlays select CUDA or ROCm physics and hardware EGL rendering. Both policies
remain portable; proprietary simulation software is not required.

## Start

Clone this repository alongside [Robo-Boy](https://github.com/tessel-la/robo-boy).
The start commands below read ROS discovery settings from `../robo-boy/.env`.

Build the CPU image once:

```bash
docker compose build
```

Start it with the same ROS discovery settings as Robo-Boy:

```bash
docker compose --env-file ../robo-boy/.env up -d --no-build
docker compose ps
```

**Genesis Go2 Sim** in the dashboard controls this same Compose project. Start never
builds an image. The first run compiles Genesis kernels and can take several minutes;
the named cache volume preserves them for subsequent starts. The health check waits for
live lidar and camera data, a SLAM map, and active navigation nodes.

Run one simulator per ROS domain. This environment owns `/clock`, `/tf`, `/cmd_vel`, and
the standard navigation node names. Stop other simulators in that domain before starting
it. Use an isolated domain for tests; do not share it with a live vehicle.

## NVIDIA host

The NVIDIA image uses PyTorch CUDA 12.8, Genesis CUDA physics, and the OpenGL
rasterizer through the NVIDIA container runtime. Install a compatible driver and
NVIDIA Container Toolkit on the host, then build and start:

```bash
docker compose -f docker-compose.yml -f docker-compose.nvidia.yml build
docker compose --env-file ../robo-boy/.env \
  -f docker-compose.yml -f docker-compose.nvidia.yml up -d --no-build
```

Set `GENESIS_MOBILE_GPU_COMPOSE=docker-compose.nvidia.yml` in the dashboard service
environment and restart that service to use NVIDIA for future dashboard launches.
Use exactly one GPU overlay. Leave the variable empty for CPU, or select the AMD
overlay on an AMD host.

## Warehouse course

The 18 × 12 m course reuses the original textured shelves, pallet jacks, and boxed
clutter from [AWS RoboMaker's warehouse world](assets/environments/warehouse/SOURCE.md).
Six shelf units define a 2.7 m center aisle with routes around both ends. Loading
props and asymmetric clutter provide landmarks and occlusions for mapping.
Yellow floor paint is visual only and does not become a lidar obstacle.

Start at `(0, 0)` in the center aisle. Useful map goals after observing the route
are `(5, 0)` for receiving, `(7.4, 3.5)` for the end route, and `(-3, 4)` for inspection.
The room has no ceiling so an operator can inspect it from above.

## Learned motion

Walking and standing/recovery use separate existing checkpoints. `/cmd_vel`
automatically selects walking when Go2 is upright. Releasing control or command
expiry returns to the recovery policy's stable standing behavior.

```bash
ros2 topic pub --once /go2/motion_cmd std_msgs/msg/String '{data: recover}'
ros2 topic pub --once /go2/motion_cmd std_msgs/msg/String '{data: stand}'
```

`/go2/status` reports the current motion, backend, environment, and reason. Recovery
runs with zero velocity commands, then hands off to standing after a stable stance.
Walking commands are rejected during recovery or while fallen. A 12 s simulated
timeout returns to damping if recovery cannot complete; it never teleports the base
or resets SLAM. Tilted lidar is withheld during recovery because it is not a valid
planar floor scan. Odometry, joints, status, and the RGB camera remain live.

The [Go2 Control external panel](https://github.com/tessel-la/robo-boy-go2-control-panel)
exposes **Stand** and **Recover / stand up**.
Cancel navigation before switching motion or recovering. This is a simulation
integration; it does not send commands to physical Go2 hardware.

## AMD host

Install a ROCm-compatible driver for your GPU, then build and start the AMD image:

```bash
docker compose -f docker-compose.yml -f docker-compose.amd.yml build
docker compose --env-file ../robo-boy/.env \
  -f docker-compose.yml -f docker-compose.amd.yml up -d --no-build
```

Set `GENESIS_MOBILE_GPU_COMPOSE=docker-compose.amd.yml` in the dashboard service's
environment and restart the dashboard to select this image. Leave the setting empty
for the CPU image. The AMD image uses the ROCm 6.3 PyTorch wheel channel; select a
matching channel through the Docker build argument `PYTORCH_INDEX_URL` if your GPU
requires another supported ROCm version. No GPU architecture override is forced.

AMD support depends on the GPU and driver. The CPU path remains available on AMD
machines, including GPUs outside ROCm's supported list. This project uses Genesis's
OpenGL rasterizer, not its CUDA-only batch renderer.

## Sensors and navigation

| Interface | Purpose |
| --- | --- |
| `/go2/motion_cmd`, `/go2/status` | Motion requests and execution status |
| `/cmd_vel` | Body x/y/yaw `geometry_msgs/msg/Twist` commands |
| `/scan` | 360-ray planar lidar, 10 m range, 5 Hz in simulation time |
| `/camera/image_raw`, `/camera/camera_info` | 320 × 240 RGB camera and calibration, 5 Hz |
| `/imu/data` | Body acceleration and angular velocity, 25 Hz |
| `/odom` | Ideal simulator pose and body velocity, 25 Hz |
| `/joint_states`, `/robot_description`, `/tf`, `/tf_static` | Robot model and transforms |
| `/clock` | Simulation time; mapping and navigation use this clock |
| `/map` | Occupancy grid built from lidar by SLAM Toolbox |
| `/local_costmap/costmap` | Rolling obstacle map centered on the robot |
| `/global_costmap/costmap` | SLAM map plus obstacle and inflation layers |
| `/navigate_to_pose` | Nav2 navigation action |

The transform chain is `map → odom → base → sensors`. SLAM Toolbox owns
`map → odom`; the simulator publishes its physical base pose as `odom → base`.
This is ideal ground-truth odometry, not a leg state estimator. Lidar measurements
come from Genesis raycasts, and the camera renders the scene through OpenGL.
The original Go2 meshes are served on port 8000 through Robo-Boy's existing mesh proxy.

Connect Robo-Boy through the shared ROS stack. Add a **Camera** using
`/camera/image_raw`, and a **3D view** with the robot description, `/scan`, and fixed
frame `map`. Occupancy grids and costmaps can be viewed in RViz on a ROS workstation
with `use_sim_time` enabled; Robo-Boy currently has no occupancy-grid renderer.

For navigation actions in Robo-Boy, start this simulator once to populate
`genesis-mobile-interfaces-jazzy`. Select **Mobile navigation** in the dashboard's
Robo-Boy interface overlays, then stop/start Robo-Boy's ROS stack. The overlay supplies
the standard `nav2_msgs` action types to rosbridge. It is built from the upstream
Jazzy source package with colcon; no custom message definitions are introduced.

To drive from a ROS workstation, publish continuously while holding a control:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Commands expire after 0.5 s of wall time. Speeds are bounded to 0.35 m/s forward, 0.25 m/s lateral, and 1.0 rad/s yaw.
The walking policy continues balancing when commands expire. Nav2 uses forward/yaw motion.
Cancel an active navigation goal before teleoperation. To navigate within the observed map:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}'
```

Save the occupancy grid in the persistent map volume:

```bash
docker compose exec mobile_sim bash -lc \
  'source /opt/ros/jazzy/setup.bash; ros2 run nav2_map_server map_saver_cli -f /maps/room --ros-args -p use_sim_time:=true'
```

Stop preserves maps, kernels, and container logs:

```bash
docker compose stop
```

## Verification

The command and configuration tests do not import Genesis or ROS:

```bash
PYTHONPATH=src python -m pytest
```

With Genesis and PyTorch installed, verify real CPU physics, camera pixels, lidar
geometry, walking, turning, and standing after command expiry:

```bash
PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1 PYTHONPATH=src \
  python tests/smoke_physics.py
```

`tests/smoke_ros.py` tests the running ROS stack, including live maps, a moving local
costmap, command expiry, and a navigation goal. It commands motion: run it only in an
isolated ROS domain, never against a physical robot.

CPU and NVIDIA verification passed on the development host. The warehouse CUDA run
covered 0.77 m of forward travel, a 1.26 rad turn, 0.004 m of standing drift, lidar and
RGB data, SLAM, a rolling local costmap, a successful Nav2 goal, and map saving.
Recovery from the back and side passed on both CPU and CUDA in 1.96 s and 1.56 s of
simulation time. NVIDIA physics, inference, and OpenGL rendering used an RTX 4080 SUPER.
AMD acceleration has not been hardware-tested.

## Limitations

- CPU simulation may run slower than wall time. Sensor rates are in simulation time.
- The camera is RGB. Mapping uses planar lidar; this setup does not implement visual SLAM or a 3D map.
- Odometry is ideal simulator data. The IMU is published but is not fused into a leg state estimator.
- The existing Go2 policies are simulation baselines, not controllers validated for physical hardware.
- Virtual lidar and camera mounts are for integration tests, not calibrated Go2 sensor hardware.
- A fall switches to damping. Recovery is explicit, may fail near obstacles, and has a 12 s simulated timeout.
- Each start creates a fresh SLAM session. Saved map files persist; automatic pose-graph restoration is not configured.
- AMD acceleration requires validation on the intended AMD hardware. CPU compatibility is tested independently.

## Upstream components

This integration is MIT licensed; see [LICENSE](LICENSE).

[Genesis](https://github.com/Genesis-Embodied-AI/Genesis) is Apache-2.0 licensed and
documents [CPU and AMD backends](https://genesis-world.readthedocs.io/en/latest/user_guide/configuration/initialization.html).
[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) is LGPL-2.1 licensed.
[Nav2](https://github.com/ros-navigation/navigation2) supplies navigation, costmaps,
and lifecycle management under its upstream package licenses. They are installed
as dependencies; their implementations and license notices remain upstream.

The [Go2 policy source and license](assets/policies/go2/SOURCE.md) are pinned locally.
The robot model and meshes remain in the installed Genesis package with their upstream notices.
