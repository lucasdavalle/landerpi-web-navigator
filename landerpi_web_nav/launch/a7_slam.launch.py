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

"""a7_base (TF + odometry) + slam_toolbox (async) + wall_vectorizer + web_gateway."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('landerpi_web_nav')

    slam_config = LaunchConfiguration('slam_config')
    ekf_config = LaunchConfiguration('ekf_config')
    stamper_config = LaunchConfiguration('stamper_config')

    args = [
        DeclareLaunchArgument(
            'slam_config',
            default_value=os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(pkg_share, 'config', 'ekf_rf2o.yaml')),
        DeclareLaunchArgument(
            'stamper_config',
            default_value=os.path.join(pkg_share, 'config', 'covariance_stamper_zupt.yaml')),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'a7_base.launch.py')),
        launch_arguments={'ekf_config': ekf_config,
                          'stamper_config': stamper_config}.items(),
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config, {'use_sim_time': False}],
    )

    # Live map -> wall segments as RViz markers on /a7/walls (stage 2).
    walls = Node(
        package='landerpi_web_nav',
        executable='wall_vectorizer_node',
        name='wall_vectorizer',
        parameters=[os.path.join(pkg_share, 'config', 'wall_vectorizer.yaml')],
    )

    # Read-only web gateway (stage 3). Needs ~/.a7/password (a7_set_password);
    # without it the node logs the reason and exits, the rest keeps running.
    gateway = Node(
        package='landerpi_web_nav',
        executable='web_gateway_node',
        name='web_gateway',
        parameters=[os.path.join(pkg_share, 'config', 'web_gateway.yaml')],
    )

    return LaunchDescription(args + [base, slam, walls, gateway])
