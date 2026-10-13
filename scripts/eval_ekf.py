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
#   python3 eval_ekf.py <bag_dir> <ekf_yaml> [stamper_yaml] [--rate 3] [--domain 77]
#       [--rf2o bag|raw|centered] [--probe /eval/odom] [--no-ekf] [--out traj.csv]
import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

URDF = '/home/ubuntu/ros2_ws/src/simulations/landerpi_description/urdf/landerpi.xacro'


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):

    def __init__(self, topic):
        super().__init__('eval_probe', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.first = None
        self.last = None
        self.count = 0
        self.rows = []  # (stamp_s, x, y, yaw) for rpe.py
        self.create_subscription(Odometry, topic, self.cb, 50)

    def cb(self, m):
        p = m.pose.pose
        cur = (p.position.x, p.position.y, yaw(p.orientation))
        self.first = self.first or cur
        self.last = cur
        self.count += 1
        st = m.header.stamp
        self.rows.append((st.sec + st.nanosec * 1e-9,) + cur)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('ekf_yaml')
    ap.add_argument('stamper_yaml', nargs='?')
    ap.add_argument('--rate', default='3')
    ap.add_argument('--domain', default='77', help='isolated ROS_DOMAIN_ID')
    ap.add_argument('--out', help='CSV with the full filtered trajectory (for rpe.py)')
    ap.add_argument('--rf2o', choices=['bag', 'raw', 'centered'], default='bag',
                    help='bag: replay the recorded /odom_rf2o; raw/centered: recompute rf2o '
                         'from the bag /scan_raw, as recorded or through scan_recenter')
    ap.add_argument('--probe', default='/eval/odom', help='Odometry topic to evaluate')
    ap.add_argument('--no-ekf', action='store_true', help='do not run the EKF')
    a = ap.parse_args()

    env = dict(os.environ, ROS_DOMAIN_ID=a.domain)
    os.environ['ROS_DOMAIN_ID'] = a.domain
    procs = []
    log_dir = f'/tmp/a7_eval_logs/{a.domain}'
    os.makedirs(log_dir, exist_ok=True)

    def start(cmd):
        # Each process logs to its own file: errors used to vanish silently.
        log = open(os.path.join(log_dir, cmd[3] + '.log'), 'w')
        procs.append(subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True))

    urdf = subprocess.run(['xacro', URDF], capture_output=True, text=True, check=True).stdout
    start(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args',
           '-p', f'robot_description:={urdf}', '-p', 'use_sim_time:=true'])
    if a.stamper_yaml:
        start(['ros2', 'run', 'landerpi_web_nav', 'covariance_stamper_node', '--ros-args',
               '--params-file', a.stamper_yaml, '-p', 'use_sim_time:=true'])
    if a.rf2o != 'bag':
        scan_topic = '/scan_raw'
        if a.rf2o == 'centered':
            scan_topic = '/a7/scan_centered'
            start(['ros2', 'run', 'landerpi_web_nav', 'scan_recenter_node', '--ros-args',
                   '-p', 'use_sim_time:=true'])
        # Via a params file: an empty value ('-p init_pose_from_topic:=') is
        # rejected on the command line and rf2o aborts at startup.
        rf2o_params = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        rf2o_params.write(
            'rf2o_laser_odometry:\n  ros__parameters:\n'
            f'    laser_scan_topic: {scan_topic}\n    odom_topic: /odom_rf2o\n'
            '    publish_tf: false\n    base_frame_id: base_footprint\n'
            "    odom_frame_id: odom\n    init_pose_from_topic: ''\n"
            '    freq: 10.0\n    use_sim_time: true\n')
        rf2o_params.close()
        start(['ros2', 'run', 'rf2o_laser_odometry', 'rf2o_laser_odometry_node', '--ros-args',
               '-r', '__node:=rf2o_laser_odometry', '--params-file', rf2o_params.name])
    if not a.no_ekf:
        start(['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
               '-r', '__node:=ekf_filter_node', '--params-file', a.ekf_yaml,
               '-p', 'use_sim_time:=true', '-p', 'publish_tf:=false',
               '-r', 'odometry/filtered:=/eval/odom'])

    rclpy.init()
    probe = Probe(a.probe)
    time.sleep(4)  # let the nodes come up before the first message
    play = subprocess.Popen(
        ['ros2', 'bag', 'play', a.bag, '--clock', '--rate', a.rate,
         '--topics', '/odom_raw', '/imu',
         *(['/odom_rf2o'] if a.rf2o == 'bag' else ['/scan_raw'])],
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

    if a.out and probe.rows:
        with open(a.out, 'w') as f:
            f.writelines(f'{t:.6f},{x:.6f},{y:.6f},{th:.6f}\n' for t, x, y, th in probe.rows)
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
