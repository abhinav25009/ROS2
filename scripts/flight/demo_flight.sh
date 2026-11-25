#!/bin/bash
# demo_flight.sh glass|house : one AHPPA v3.1 flight with the Gazebo GUI, RViz and
# a navigator log window on the VM desktop, for screen recording.
#   1. brings the stack up with GUIs, takes off, maps the start for 10 s
#   2. prints DEMO_READY and waits for ~/demo_go (arrange windows, start recording)
#   3. flies the goals, prints DEMO_DONE, lands
set -u
SCENE=${1:-glass}
REPO=$HOME/Indoor-Drone-Navigation
SRC="source /opt/ros/humble/setup.bash && source $HOME/px4_ros_ws/install/setup.bash"
XA=$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)
GUI="export DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus XAUTHORITY=$XA QT_QPA_PLATFORM=xcb"
if [ "$SCENE" = glass ]; then
  export WORLD=glass_test; RVIZ=demo_glass.rviz; GOALS="8.0,0.0,1.8"
  CAM='pose: {position: {x: -2.1, y: -1.9, z: 2.5}, orientation: {w: 0.9652, x: -0.0224, y: 0.0883, z: 0.2452}}'
  SHOW=glass_pane
else
  export WORLD=house_2floor; RVIZ=demo_house.rviz; GOALS="4.6,-3.0,1.8 3.0,0.0,1.8 3.1,0.1,3.6 3.0,3.5,3.6"
  CAM='pose: {position: {x: -14.0, y: -13.0, z: 12.0}, orientation: {w: 0.9142, x: -0.0812, y: 0.2286, z: 0.3246}}'
  SHOW=
fi
rm -f ~/demo_go
LOGDIR=$(GUI="$GUI" bash $HOME/demo_up.sh | tail -1)
echo "demo $SCENE ($WORLD) | AHPPA_V3=1 AHPPA_VIZ=1 | goals $GOALS" > "$LOGDIR/label.txt"
echo "log: $LOGDIR"
timeout 600 bash -c "until grep -q 'home set' $LOGDIR/px4.log 2>/dev/null; do sleep 2; done" || { echo "PX4 not ready (no home)"; exit 1; }
bash $HOME/flight_up2.sh
tmux send-keys -t flight:px4 "param set COM_LOW_BAT_ACT 0" Enter
timeout 300 bash -c "until grep -q 'Ready for takeoff' $LOGDIR/px4.log 2>/dev/null; do sleep 2; done" || { echo "PX4 not ready for takeoff"; exit 1; }
sleep 12
tmux new-window -t flight -n mapviz
tmux send-keys -t flight:mapviz "$SRC && python3 $REPO/planners/demo_mapviz.py --ros-args -p use_sim_time:=true" Enter
tmux new-window -t flight -n rviz
tmux send-keys -t flight:rviz "$GUI && $SRC && rviz2 -d $REPO/config/$RVIZ --ros-args -p use_sim_time:=true" Enter
tmux new-window -t flight -n pose
tmux send-keys -t flight:pose "$SRC && python3 $REPO/benchmark/flight_pose_logger.py $LOGDIR/pose.csv --ros-args -p use_sim_time:=true -p world:=$WORLD" Enter
tmux new-window -t flight -n nav
tmux pipe-pane -t flight:nav "cat >> $LOGDIR/nav.log"
tmux send-keys -t flight:nav "$SRC && cd $REPO/planners && ${DEMO_NAVENV:-AHPPA_V3=1} AHPPA_VIZ=1 AHPPA_PORTFOLIO=1 PYTHONUNBUFFERED=1 AHPPA_LOG_DIR=$LOGDIR python3 -W ignore navigator_astar.py --ros-args -p use_sim_time:=true -p world:=$WORLD 2>/dev/null" Enter
( eval "$GUI"; GDK_BACKEND=x11 gnome-terminal --title="AHPPA navigator log" --geometry=100x9+761+748 -- bash -c \
  "tail -n +1 -f $LOGDIR/nav.log | grep --line-buffered -vE 'WP [0-9]+/|Layer [0-9]|blocked cells'" ) >/dev/null 2>&1 &
timeout 300 bash -c "until grep -q 'Airborne' $LOGDIR/nav.log 2>/dev/null; do sleep 2; done" || { echo "no takeoff"; exit 1; }
sleep 10
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req "$CAM" >/dev/null
[ -n "$SHOW" ] && gz service -s /gui/view/collisions --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req "data: \"$SHOW\"" >/dev/null
echo DEMO_READY
until [ -f ~/demo_go ]; do sleep 1; done
for g in $GOALS; do
  IFS=, read GX GY GZ <<< "$g"
  $HOME/fly2.sh "$GX" "$GY" "$GZ" 1000
  grep -q 'CRASH:' $LOGDIR/nav.log && break
  sleep 3
done
echo DEMO_DONE
sleep 8
tmux send-keys -t flight:px4 "commander land" Enter
echo "done: $LOGDIR"
