source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
mkdir -p /home/ubuntu/shared/a7_bags
cd /home/ubuntu/shared/a7_bags
nohup ros2 bag record -o stage0_$(date +%Y%m%d_%H%M%S) \
  /scan_raw /odom_raw /odom_rf2o /odom /imu /ros_robot_controller/imu_raw \
  /tf /tf_static /map /controller/cmd_vel > /tmp/a7_logs/bag.log 2>&1 &
