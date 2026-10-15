#!/bin/bash
# Records a navigation test bag inside MentorPi (as root): the planned
# path, both costmaps, the command at every link of the chain (Nav2 ->
# collision_monitor -> velocity_gate -> wheels), the laser, TF and the logs.
#   docker cp scripts/a7_nav_bag.sh MentorPi:/tmp/a7_nav_bag.sh
#   docker exec MentorPi bash /tmp/a7_nav_bag.sh <name>
# Stop it with SIGINT so the bag is closed cleanly.
source /opt/ros/humble/setup.bash
mkdir -p /home/ubuntu/shared/a7_bags /tmp/a7_logs
cd /home/ubuntu/shared/a7_bags
nohup ros2 bag record -o "${1:-nav}_$(date +%Y%m%d_%H%M%S)" \
  /plan /local_costmap/costmap /global_costmap/costmap /local_costmap/published_footprint \
  /a7/nav/cmd_vel_raw /a7/nav/cmd_vel_checked /a7/cmd_vel_nav /controller/cmd_vel \
  /a7/gate_status /a7/depth_obstacles /a7/posture /a7/stand_check /a7/crouch_status \
  /a7/crouch_cells /a7/robot_height /scan /tf /tf_static /rosout \
  > /tmp/a7_logs/nav_bag.log 2>&1 &
