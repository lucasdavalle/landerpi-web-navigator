# Copyright 2026 ldavalle
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
TF + odometry base for A7: URDF, IMU filter, rf2o and the EKF.

Builds the chain odom -> base_footprint -> lidar_frame that robot_reset.sh
does not bring up. Assumes the chassis driver (ros_robot_controller +
odom_publisher, publishing /odom_raw) and the lidar (/scan_raw) are already
running, as left by tools/robot_reset.sh.

rf2o always runs and publishes /odom_rf2o (no TF) so it can be recorded and
compared; whether the EKF fuses it depends on ekf_config. covariance_stamper
republishes /odom_rf2o, /odom_raw and /imu under /a7/ with realistic
covariances (the drivers publish zeros); ekf_rf2o.yaml consumes those.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

STOCK_SRC = '/home/ubuntu/ros2_ws/src'


def generate_launch_description():
    pkg_share = get_package_share_directory('landerpi_web_nav')

    ekf_config = LaunchConfiguration('ekf_config')
    stamper_config = LaunchConfiguration('stamper_config')
    urdf_path = LaunchConfiguration('urdf_path')
    imu_calib_file = LaunchConfiguration('imu_calib_file')

    args = [
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(pkg_share, 'config', 'ekf_rf2o.yaml'),
            description='EKF parameters (default: rf2o + IMU, stage 1 winner; ekf_baseline.yaml = stock-like)'),
        DeclareLaunchArgument(
            'stamper_config',
            default_value=os.path.join(pkg_share, 'config', 'covariance_stamper_zupt.yaml'),
            description='covariance_stamper parameters'),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=os.path.join(
                STOCK_SRC, 'simulations/landerpi_description/urdf/landerpi.xacro'),
            description='Stock LanderPi xacro'),
        DeclareLaunchArgument(
            'imu_calib_file',
            default_value=os.path.join(STOCK_SRC, 'calibration/config/imu_calib.yaml'),
            description='Stock IMU calibration (imu_calib apply_calib)'),
    ]

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Arm joints come from the stock servo controller (commanded positions,
    # not real feedback); needed for the camera TF, which sits on the arm.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['/controller_manager/joint_states'],
                     'rate': 20.0}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
    )

    imu_calib = Node(
        package='imu_calib',
        executable='apply_calib',
        name='imu_calib',
        parameters=[{'calib_file': imu_calib_file}],
        remappings=[('raw', '/ros_robot_controller/imu_raw'),
                    ('corrected', '/imu_corrected')],
    )

    imu_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        parameters=[{'use_mag': False,
                     'do_bias_estimation': True,
                     'do_adaptive_gain': True,
                     'publish_tf': False}],
        remappings=[('/imu/data_raw', '/imu_corrected'),
                    ('imu/data', '/imu')],
    )

    # rf2o ignores angle_min and assumes a [-pi, pi) scan; this lidar
    # publishes 0..2pi, which made rf2o report translation reversed.
    scan_recenter = Node(
        package='landerpi_web_nav',
        executable='scan_recenter_node',
        name='scan_recenter',
        parameters=[{'input_topic': '/scan_raw',
                     'output_topic': '/a7/scan_centered'}],
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{'laser_scan_topic': '/a7/scan_centered',
                     'odom_topic': '/odom_rf2o',
                     'publish_tf': False,
                     'base_frame_id': 'base_footprint',
                     'odom_frame_id': 'odom',
                     'init_pose_from_topic': '',
                     'freq': 10.0}],
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    covariance_stamper = Node(
        package='landerpi_web_nav',
        executable='covariance_stamper_node',
        name='covariance_stamper',
        parameters=[stamper_config],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom')],
    )

    return LaunchDescription(args + [
        joint_state_publisher,
        robot_state_publisher,
        imu_calib,
        imu_filter,
        scan_recenter,
        rf2o,
        covariance_stamper,
        ekf,
    ])
