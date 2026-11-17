#!/bin/bash
# =============================================================================
#  360 DEPTH RIG LAUNCHER  —  x500_depth_360 (5 cameras) + cloud merger
#
#    ./launch_sim_360.sh          manual (no navigator)
#    ./launch_sim_360.sh --auto   also start the frontier navigator
#
#  Leaves launch_sim.sh (single camera) untouched.
# =============================================================================
set -u

REPO="$HOME/Desktop/Drone_IP"
PX4_DIR="$HOME/PX4-Autopilot"
ROS2_WS="$HOME/px4_ros_ws"
GZ_WS="$HOME/gz_ws"
WORLD="house_2floor"
MODEL="gz_x500_depth_360"

AUTO=false
[[ "${1:-}" == "--auto" ]] && AUTO=true

echo "Cleaning up previous run..."
pkill -f navigator_ 2>/dev/null
pkill -f cloud_merger.py 2>/dev/null
pkill -f tf_broadcaster 2>/dev/null
pkill -f octomap_server_node 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
pkill -x rviz2 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -x px4 2>/dev/null
pkill -x ruby 2>/dev/null
pkill -x MicroXRCEAgent 2>/dev/null
sleep 3

cp "$REPO/worlds/$WORLD.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/" 2>/dev/null

TERM_CMD=""
for t in gnome-terminal xfce4-terminal konsole xterm; do
    command -v $t &>/dev/null && TERM_CMD=$t && break
done
[[ -z "$TERM_CMD" ]] && { echo "no terminal emulator found"; exit 1; }

open_term() {
    local title="$1" cmd="$2"
    case "$TERM_CMD" in
        gnome-terminal)  gnome-terminal --title="$title" -- bash -c "$cmd; exec bash" & ;;
        xfce4-terminal)  xfce4-terminal --title="$title" -e "bash -c '$cmd; exec bash'" & ;;
        konsole)         konsole --new-tab -p tabtitle="$title" -e bash -c "$cmd; exec bash" & ;;
        xterm)           xterm -T "$title" -e bash -c "$cmd; exec bash" & ;;
    esac
}

SRC_ROS="source /opt/ros/humble/setup.bash"
SRC_GZ="$SRC_ROS && source $GZ_WS/install/setup.bash"
SRC_PX4="$SRC_ROS && source $ROS2_WS/install/setup.bash"

echo "=========================================="
echo "  360 Rig — 5 depth cameras + fusion"
echo "  world: $WORLD   model: $MODEL"
echo "=========================================="

echo "[T1] DDS agent"
open_term "T1 - DDS Agent" "MicroXRCEAgent udp4 -p 8888"
sleep 2

echo "[T2] PX4 + Gazebo (headless)"
open_term "T2 - PX4 + Gazebo" \
  "cd $PX4_DIR && HEADLESS=1 PX4_GZ_WORLD=$WORLD make px4_sitl $MODEL"

echo "     waiting 150s for PX4 + Gazebo..."
sleep 150

echo "[T3] GZ->ROS bridge (5 clouds)"
open_term "T3 - Bridge" \
  "$SRC_GZ && ros2 run ros_gz_bridge parameter_bridge \
   /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
   /depth_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
   /depth_right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
   /depth_rear/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
   /depth_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
   /depth_up/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
   /world/$WORLD/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
sleep 5

echo "[T4] TF broadcaster (360)"
open_term "T4 - TF 360" \
  "$SRC_PX4 && python3 $REPO/core/tf_broadcaster_360.py --ros-args -p use_sim_time:=true"
sleep 4

echo "[T5] Cloud merger"
open_term "T5 - Cloud Merger" \
  "$SRC_PX4 && python3 $REPO/core/cloud_merger.py --ros-args -p use_sim_time:=true"
sleep 5

echo "[T6] OctoMap (cloud_in=/fused_points)"
open_term "T6 - OctoMap" \
  "$SRC_ROS && ros2 run octomap_server octomap_server_node --ros-args \
   -p use_sim_time:=true -r cloud_in:=/fused_points \
   -p resolution:=0.10 -p frame_id:=map \
   -p sensor_model.max_range:=5.0 -p sensor_model.min_range:=0.3 \
   -p sensor_model.hit:=0.7 -p sensor_model.miss:=0.4 \
   -p occupancy_min_z:=0.15 -p occupancy_max_z:=7.5"
sleep 5

echo "[T7] RViz2"
open_term "T7 - RViz2" \
  "$SRC_ROS && rviz2 -d $REPO/config/drone_rviz.rviz --ros-args -p use_sim_time:=true"
sleep 3

if [[ "$AUTO" == "true" ]]; then
    echo "[T8] Frontier navigator"
    open_term "T8 - Navigator" \
      "$SRC_PX4 && python3 $REPO/planners/navigator_astar.py"
fi

cat <<EOF

==========================================
  Launched. In PX4 (T2), if arming fails:
      param set NAV_DLL_ACT 0
      param save
  Verify:
      ros2 topic hz /fused_points     (~5 Hz)
      ros2 topic hz /projected_map
  Merger should print "5/5 clouds".
==========================================
EOF
