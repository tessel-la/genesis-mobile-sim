#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/sim/bin/activate
mkdir -p /overlay_ws/mobile
cp -a /opt/mobile_interfaces/. /overlay_ws/mobile/
exec python -m genesis_mobile_sim.bringup
