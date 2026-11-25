#!/bin/bash
# goal.sh X Y Z  -> publish a 3D goal like RViz "Publish Point"; no args -> print drone pose
source /opt/ros/humble/setup.bash
if [ $# -eq 3 ]; then
  ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
    "{header: {frame_id: map}, point: {x: $1, y: $2, z: $3}}" >/dev/null && echo "goal sent ($1, $2, $3)"
fi
timeout 5 ros2 topic echo --once /world/${WORLD:-house_2floor}/dynamic_pose/info 2>/dev/null | \
  awk '/child_frame_id: x500/{f=1} f&&/x:/{x=$2} f&&/y:/{y=$2} f&&/z:/{print "drone at x="x" y="y" z="$2; exit}'
