#!/usr/bin/env python3
# Offline EKF evaluation against a recorded bag (stage 1).
#
# Replays /odom_raw, /odom_rf2o and /imu from a bag in an isolated
# ROS_DOMAIN_ID with sim time, runs robot_state_publisher + covariance_stamper
# + robot_localization's ekf_node with the given config (TF publishing off,
# output on /eval/odom), and reports the closure error: the bag starts and
# ends with the robot on the same floor mark, so any start->end displacement
# of the filtered pose is pure error.
#
# Usage (inside MentorPi, as root, with the workspaces sourced):
#   python3 eval_ekf.py <bag_dir> <ekf_yaml> [stamper_yaml] [--rate 3]
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

DOMAIN = '77'
URDF = '/home/ubuntu/ros2_ws/src/simulations/landerpi_description/urdf/landerpi.xacro'


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):

    def __init__(self):
        super().__init__('eval_probe', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.first = None
        self.last = None
        self.count = 0
        self.create_subscription(Odometry, '/eval/odom', self.cb, 50)

    def cb(self, m):
        p = m.pose.pose
        cur = (p.position.x, p.position.y, yaw(p.orientation))
        self.first = self.first or cur
        self.last = cur
        self.count += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('ekf_yaml')
    ap.add_argument('stamper_yaml', nargs='?')
    ap.add_argument('--rate', default='3')
    a = ap.parse_args()

    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN)
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    procs = []

    def start(cmd):
        procs.append(subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL, start_new_session=True))

    urdf = subprocess.run(['xacro', URDF], capture_output=True, text=True, check=True).stdout
    start(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args',
           '-p', f'robot_description:={urdf}', '-p', 'use_sim_time:=true'])
    if a.stamper_yaml:
        start(['ros2', 'run', 'landerpi_web_nav', 'covariance_stamper_node', '--ros-args',
               '--params-file', a.stamper_yaml, '-p', 'use_sim_time:=true'])
    start(['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
           '-r', '__node:=ekf_filter_node', '--params-file', a.ekf_yaml,
           '-p', 'use_sim_time:=true', '-p', 'publish_tf:=false',
           '-r', 'odometry/filtered:=/eval/odom'])

    rclpy.init()
    probe = Probe()
    time.sleep(4)  # let the nodes come up before the first message
    play = subprocess.Popen(
        ['ros2', 'bag', 'play', a.bag, '--clock', '--rate', a.rate,
         '--topics', '/odom_raw', '/odom_rf2o', '/imu'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    while play.poll() is None:
        rclpy.spin_once(probe, timeout_sec=0.1)
    end = time.time() + 1.5
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)

    for p in procs:
        os.killpg(p.pid, signal.SIGINT)
    for p in procs:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)

    if not probe.first:
        print('SIN SALIDA del EKF (/eval/odom vacío)')
        return 1
    dx = probe.last[0] - probe.first[0]
    dy = probe.last[1] - probe.first[1]
    dyaw = (probe.last[2] - probe.first[2] + math.pi) % (2 * math.pi) - math.pi
    print(f'{os.path.basename(a.ekf_yaml):24s} {os.path.basename(a.stamper_yaml or "-"):28s} '
          f'pos {math.hypot(dx, dy) * 100:6.1f} cm  yaw {math.degrees(dyaw):+6.1f} deg  '
          f'({probe.count} msgs)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
