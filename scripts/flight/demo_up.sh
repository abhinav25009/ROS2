#!/bin/bash
# GUI (demo) 360-rig flight stack in tmux session "flight"; every window logs to $LOG.
set -u
REPO=$HOME/Indoor-Drone-Navigation
PX4_DIR=$HOME/PX4-Autopilot
WORLD=${WORLD:-house_2floor}
MODEL=gz_x500_depth_360
LOG=$HOME/flight_logs/$(date +%Y-%m-%d_%H-%M)
mkdir -p "$LOG"; ln -sfn "$LOG" $HOME/flight_logs/latest
SRC_ROS="source /opt/ros/humble/setup.bash"
SRC_GZ="$SRC_ROS && source $HOME/gz_ws/install/setup.bash"
SRC_PX4="$SRC_ROS && source $HOME/px4_ros_ws/install/setup.bash"

tmux kill-session -t flight 2>/dev/null
pkill -f navigator_; pkill -f cloud_merger.py; pkill -f tf_broadcaster; pkill -f octomap_server_node
pkill -f parameter_bridge; pkill -f "gz sim"; pkill -x px4; pkill -x ruby; pkill -x MicroXRCEAgent
sleep 3
cp "$REPO/worlds/$WORLD.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/"

win() {  # name, command
  tmux new-window -t flight -n "$1"
  tmux pipe-pane -t "flight:$1" "cat >> $LOG/$1.log"
  tmux send-keys -t "flight:$1" "$2" Enter
}
tmux new-session -d -s flight -n shell
win dds  "MicroXRCEAgent udp4 -p 8888"
sleep 2
win px4  "$GUI && cd $PX4_DIR && PX4_GZ_WORLD=$WORLD make px4_sitl $MODEL"
echo "$LOG"
