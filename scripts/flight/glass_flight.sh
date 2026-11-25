#!/bin/bash
# glass_flight.sh LABEL "ENV=1 ..." [GX GY GZ TIMEOUT]
#   One headless PX4 SITL flight in worlds/glass_test.sdf: bring up the stack
#   (flight_up.sh + flight_up2.sh), start the navigator with the given AHPPA
#   flags, log the pose, take off, send one goal, wait for an outcome or
#   TIMEOUT seconds, land. Logs: ~/flight_logs/<stamp>/ (label.txt says which).
set -u
LABEL=$1; ENVS=$2; GX=${3:-8.0}; GY=${4:-0.0}; GZ=${5:-1.8}; TO=${6:-150}
export WORLD=glass_test
REPO=$HOME/Indoor-Drone-Navigation
SRC="source /opt/ros/humble/setup.bash && source $HOME/px4_ros_ws/install/setup.bash"
LOGDIR=$(bash $HOME/flight_up.sh | tail -1)
echo "$LABEL | $ENVS | goal $GX $GY $GZ" > "$LOGDIR/label.txt"
echo "log: $LOGDIR"
timeout 300 bash -c "until grep -q 'home set' $LOGDIR/px4.log 2>/dev/null; do sleep 2; done" \
  || { echo "PX4 not ready (no home)"; exit 1; }
bash $HOME/flight_up2.sh                      # also sets NAV_DLL_ACT 0
timeout 120 bash -c "until grep -q 'Ready for takeoff' $LOGDIR/px4.log 2>/dev/null; do sleep 2; done" \
  || { echo "PX4 not ready for takeoff"; exit 1; }
sleep 12
tmux new-window -t flight -n pose
tmux send-keys -t flight:pose "$SRC && python3 $REPO/benchmark/flight_pose_logger.py $LOGDIR/pose.csv --ros-args -p use_sim_time:=true -p world:=$WORLD" Enter
tmux new-window -t flight -n nav
tmux pipe-pane -t flight:nav "cat >> $LOGDIR/nav.log"
tmux send-keys -t flight:nav "$SRC && cd $REPO/planners && $ENVS AHPPA_PORTFOLIO=1 PYTHONUNBUFFERED=1 AHPPA_LOG_DIR=$LOGDIR python3 -W ignore navigator_astar.py --ros-args -p use_sim_time:=true -p world:=$WORLD 2>/dev/null" Enter
timeout 240 bash -c "until grep -q 'Airborne' $LOGDIR/nav.log 2>/dev/null; do sleep 2; done" \
  || { echo "no takeoff"; exit 1; }
sleep 10                                   # let the map fill in around the start
date +%s > "$LOGDIR/goal_wall.txt"
$HOME/fly2.sh "$GX" "$GY" "$GZ" "$TO"
sleep 2
tmux send-keys -t flight:px4 "commander land" Enter
sleep 12
echo "done: $LOGDIR"
