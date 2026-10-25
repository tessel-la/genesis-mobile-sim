FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1 \
    PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1 GENESIS_DEVICE=cpu
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-venv libegl1 libegl-mesa0 libgl1-mesa-dri libgl1 libglib2.0-0 libgomp1 \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-robot-state-publisher ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# CPU wheels keep the default runtime independent of NVIDIA hardware. An AMD
# deployment builds the same application with the matching ROCm PyTorch wheels.
ARG PYTORCH_VERSION=2.14.1
ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cpu
RUN python3 -m venv --system-site-packages /opt/sim \
    && /opt/sim/bin/pip install --no-cache-dir "torch==${PYTORCH_VERSION}" --index-url "${PYTORCH_INDEX_URL}"
WORKDIR /app
COPY pyproject.toml README.md requirements.lock ./
RUN /opt/sim/bin/pip install --no-cache-dir --require-hashes -r requirements.lock \
    && mkdir -p src/genesis_mobile_sim && touch src/genesis_mobile_sim/__init__.py \
    && /opt/sim/bin/pip install --no-cache-dir --no-deps -e .

# Build the upstream message package as a relocatable overlay for rosbridge.
# Robo-Boy sources /overlay_ws/*, so no application-specific action code is needed.
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions ros-jazzy-rosidl-default-generators \
    && mkdir -p /interfaces/src && cd /interfaces/src \
    && apt-get source ros-jazzy-nav2-msgs \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --merge-install \
        --base-paths /interfaces/src --build-base /interfaces/build \
        --install-base /opt/mobile_interfaces --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf /interfaces /var/lib/apt/lists/*
COPY src ./src
COPY assets ./assets
COPY docker ./docker
COPY LICENSE README.md ./
ENTRYPOINT ["bash", "/app/docker/entrypoint.sh"]
