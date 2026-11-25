#!/bin/bash
set -u
REPO=$HOME/Indoor-Drone-Navigation; WORLD=${WORLD:-house_2floor}; LOG=$HOME/flight_logs/latest
SRC_ROS="source /opt/ros/humble/setup.bash"
SRC_GZ="$SRC_ROS && source $HOME/gz_ws/install/setup.bash"
SRC_PX4="$SRC_ROS && source $HOME/px4_ros_ws/install/setup.bash"
win() { tmux new-window -t flight -n "$1"; tmux pipe-pane -t "flight:$1" "cat >> $LOG/$1.log"; tmux send-keys -t "flight:$1" "$2" Enter; }
tmux send-keys -t flight:px4 "param set NAV_DLL_ACT 0" Enter
tmux send-keys -t flight:px4 "param set COM_LOW_BAT_ACT 0" Enter
win bridge "$SRC_GZ && ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /depth_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked /depth_right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked /depth_rear/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked /depth_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked /depth_up/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked /world/$WORLD/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
sleep 5
win tf "$SRC_PX4 && python3 $REPO/core/tf_broadcaster_360.py --ros-args -p use_sim_time:=true -p world:=$WORLD"
sleep 4
win merger "$SRC_PX4 && python3 $REPO/core/cloud_merger.py --ros-args -p use_sim_time:=true"
sleep 5
win octomap "$SRC_ROS && ros2 run octomap_server octomap_server_node --ros-args -p use_sim_time:=true -r cloud_in:=/fused_points -p resolution:=0.10 -p frame_id:=map -p sensor_model.max_range:=5.0 -p sensor_model.min_range:=0.3 -p sensor_model.hit:=0.7 -p sensor_model.miss:=0.4 -p occupancy_min_z:=0.15 -p occupancy_max_z:=7.5"
