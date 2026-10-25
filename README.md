# Genesis Go2 simulation for Robo-Boy

Walk Unitree's existing Go2 robot dog through a warehouse, build a map from lidar,
and navigate with ROS 2 Jazzy, SLAM Toolbox, and Nav2. Genesis runs physics and sensors;
existing trained walking and recovery policies run unchanged. You do not need a robot
or proprietary simulation software.

## Run it

Use Linux x86-64, Docker Engine, and Docker Compose v2. The default runs real physics
on the CPU with software rendering. NVIDIA and AMD paths are optional; no GPU is
required for this quickstart. Docker Desktop DDS and native macOS/Windows deployments
are not validated.

Clone Robo-Boy alongside this repository. Prepare and start its app and ROS stack using
its README, then use the same ROS domain and CycloneDDS configuration for the simulator.
Run only one simulator in a domain; use a separate domain from any physical robot.

```bash
git clone https://github.com/tessel-la/robo-boy.git
git clone https://github.com/tessel-la/genesis-mobile-sim.git
cd genesis-mobile-sim
docker compose build
docker compose --env-file ../robo-boy/.env up -d --no-build
docker compose ps
```

The first build downloads large scientific Python wheels; allow several GB of disk
space. Genesis compiles kernels on first start, which can take several minutes. The
cache volume preserves them for later starts. The container becomes healthy after
camera, lidar, map, and navigation data are live. Inspect progress with:

```bash
docker compose logs --tail 80 mobile_sim
```

Connect Robo-Boy to its ROS stack. Add **Camera** with `/camera/image_raw` and a **3D view**
with fixed frame `map`. Install [Go2 Control](https://github.com/tessel-la/robo-boy-go2-control-panel)
through **Manage installations…**, add its tile, and request control before driving.
Its README also covers local installation before public releases are available.

```bash
docker compose stop
```

Stopping preserves maps, kernels, containers, and logs. Restart with the same Compose
files and discovery settings. See [operation](docs/operation.md) for GPU overlays,
policy controls, navigation goals, saved maps, and ROS interfaces.

## How it fits

This simulator is ROS-native. The simulator container runs physics, sensors, SLAM, and
navigation; Robo-Boy owns rosbridge, control arbitration, and the UI. The dashboard is
optional. Teleoperation uses standard ROS messages. Only navigation action examples
need the generated `genesis-mobile-interfaces-jazzy` overlay mounted into Robo-Boy;
see [operation](docs/operation.md). Installing the control panel does not start a sim.

## Develop

Use Python 3.12 and [uv](https://docs.astral.sh/uv/). The declared runtime range is
Python 3.10–3.13. `uv.lock` owns project and development versions; Docker consumes
its hash-checked export and installs the hardware-specific pinned PyTorch wheel.

```bash
uv sync --locked --extra dev --python 3.12
uv run --frozen --extra dev pytest
uv run --frozen --extra dev ruff check .
```

For native CPU physics, install the same CPU PyTorch version as Docker, then run the
sensor/motion smoke test from the repository root:

```bash
uv pip install 'torch==2.14.1' --index-url https://download.pytorch.org/whl/cpu
PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1 uv run --no-sync python tests/smoke_physics.py
```

EGL/Mesa system libraries are also required for native rendering; the Dockerfile lists
them. See [CONTRIBUTING.md](CONTRIBUTING.md), [AGENTS.md](AGENTS.md), and [guides](docs/README.md).

## Configuration

[docker-compose.yml](docker-compose.yml) owns deployment defaults. Share Robo-Boy's DDS
settings through `--env-file`; use an existing host file when overriding the CycloneDDS
config path. CPU, NVIDIA, and AMD are mutually exclusive Compose profiles, not simultaneous
simulators. GPU drivers and container access must already be configured on the host.

The read-only mesh server binds to loopback by default. Robo-Boy's same-host mesh proxy
serves its assets to remote clients. Only override `MESH_RESOURCES_HOST` to a trusted VPN
address when a deployment needs direct mesh access. Keep command traffic on the ROS path.

## Limitations

- CPU simulation can run slower than wall time; sensor rates use simulation time.
- The policies are simulation baselines, not hardware controllers. Stop sends zero velocity and is not a hardware emergency stop. Cancel navigation before teleoperation or recovery.
- Odometry is ideal ground truth. Mapping is planar lidar SLAM; the RGB camera does not implement visual SLAM. Robo-Boy currently has no occupancy-grid renderer.
- A fall requires explicit recovery, which can fail near obstacles. Each startup creates a new SLAM session; saved maps persist, but pose-graph restoration is not automatic.
- AMD hardware support is unverified. Its ROCm 6.3 overlay pins PyTorch 2.7.1; CPU/NVIDIA pin 2.14.1. Match the documented wheel channel to the host driver.
- Python versions and policy bytes are pinned. System ROS repositories, base images, and host drivers retain their own update lifecycle.

## License

[MIT](LICENSE) for the integration. Walking, recovery, and warehouse assets retain their
[pinned source notices](docs/operation.md#upstream-components). Genesis, the robot model,
ROS, Nav2, and SLAM Toolbox retain their upstream licenses. No new policy training is required.
