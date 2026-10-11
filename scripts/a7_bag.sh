#!/bin/bash
# Records the A7 evaluation bag inside MentorPi (as root). Usage:
#   docker cp scripts/a7_bag.sh MentorPi:/tmp/a7_bag.sh
#   docker exec MentorPi bash /tmp/a7_bag.sh <name>    # e.g. stage1
# Stop it with SIGINT (kill -INT) so the bag is closed cleanly.
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
mkdir -p /home/ubuntu/shared/a7_bags /tmp/a7_logs
cd /home/ubuntu/shared/a7_bags
nohup ros2 bag record -o "${1:-a7}_$(date +%Y%m%d_%H%M%S)" \
  /scan_raw /odom_raw /odom_rf2o /odom /imu /ros_robot_controller/imu_raw \
  /a7/odom_rf2o /a7/odom_raw /a7/imu \
  /tf /tf_static /map /controller/cmd_vel > /tmp/a7_logs/bag.log 2>&1 &
