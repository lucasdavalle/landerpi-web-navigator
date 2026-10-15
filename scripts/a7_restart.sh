#!/bin/bash
# Restarts the A7 stack and Nav2 inside MentorPi (as root), e.g. after
# rebuilding. Kills by PID with the [x] grep trick: a plain pkill -f inside
# bash -c would match its own command line.
# Usage: docker cp scripts/a7_restart.sh MentorPi:/tmp/ && docker exec MentorPi bash /tmp/a7_restart.sh [nav]
#   nav: restart only Nav2 (and crouch_coordinator), keep SLAM running.
if [ "$1" = "nav" ]; then
  PAT='[a]7_nav.launch|[n]av2_|[c]rouch_coordinator_node'
else
  PAT='[a]7_nav.launch|[a]7_slam.launch|[n]av2_|[l]anderpi_web_nav/|[l]anderpi_posture/|[s]lam_toolbox|[e]kf_node|[r]f2o'
fi
pids=$(ps -eo pid,args | grep -E "$PAT" | awk '{print $1}')
[ -n "$pids" ] && kill -INT $pids 2>/dev/null
sleep 10
pids=$(ps -eo pid,args | grep -E "$PAT" | awk '{print $1}')
[ -n "$pids" ] && kill -9 $pids 2>/dev/null
sleep 2
mkdir -p /tmp/a7_logs
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
if [ "$1" != "nav" ]; then
  bash /tmp/a7_up.sh
  sleep 25
fi
nohup ros2 launch landerpi_web_nav a7_nav.launch.py > /tmp/a7_logs/a7_nav.log 2>&1 &
sleep 35
echo "nav2 active: $(grep -a -c 'Managed nodes are active' /tmp/a7_logs/a7_nav.log)"
