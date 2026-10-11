#!/bin/bash
# Launches A7 (a7_slam.launch.py) inside the MentorPi container, as root like
# the drivers started by tools/robot_reset.sh (FastDDS shared memory does not
# deliver data across users). Usage from the host:
#   docker cp scripts/a7_up.sh MentorPi:/tmp/a7_up.sh
#   docker exec MentorPi bash /tmp/a7_up.sh
mkdir -p /tmp/a7_logs
export MACHINE_TYPE=LanderPi_Mecanum LIDAR_TYPE=MS200 DEPTH_CAMERA_TYPE=aurora need_compile=False
source /opt/ros/humble/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
nohup ros2 launch landerpi_web_nav a7_slam.launch.py > /tmp/a7_logs/a7_slam.log 2>&1 &
