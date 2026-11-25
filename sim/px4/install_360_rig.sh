#!/bin/bash
# Install the 5-camera 360 depth rig (x500_depth_360, airframe 4099) into a PX4-Autopilot checkout.
#   bash sim/px4/install_360_rig.sh [PX4_DIR]     (default: ~/PX4-Autopilot)
set -e
PX4=${1:-$HOME/PX4-Autopilot}
HERE=$(cd "$(dirname "$0")" && pwd)
cp -r "$HERE/models/OakD-360" "$HERE/models/x500_depth_360" "$PX4/Tools/simulation/gz/models/"
A="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
cp "$HERE/airframes/4099_gz_x500_depth_360" "$A/"
grep -q 4099_gz_x500_depth_360 "$A/CMakeLists.txt" || sed -i "/4001_gz_x500/a\\	4099_gz_x500_depth_360" "$A/CMakeLists.txt"
echo "Installed. Build and run with: make px4_sitl gz_x500_depth_360"
