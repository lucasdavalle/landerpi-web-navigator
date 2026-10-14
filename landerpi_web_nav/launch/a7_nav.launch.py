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
Stage 5: Nav2 on top of the running A7 stack (a7_slam.launch.py).

No AMCL or map_server: slam_toolbox already provides /map and map -> odom,
so goals can be set in space discovered a moment ago.

SAFETY: every velocity Nav2 produces goes to /a7/cmd_vel_nav, which only
velocity_gate reads. Nothing here may publish on /cmd_vel or
/controller/cmd_vel: the stock odom_publisher listens on /cmd_vel and would
drive the wheels bypassing the gate. That includes behavior_server, whose
recoveries (spin, back up) publish their own cmd_vel in Humble.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

NAV_NODES = [
    'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother',
]


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('landerpi_web_nav'), 'config', 'nav2.yaml')
    raw = '/a7/nav/cmd_vel_raw'   # controller -> smoother
    out = '/a7/cmd_vel_nav'       # -> velocity_gate

    def nav_node(package, executable, name, remappings=()):
        return Node(
            package=package, executable=executable, name=name, output='screen',
            parameters=[params], remappings=list(remappings))

    return LaunchDescription([
        nav_node('nav2_controller', 'controller_server', 'controller_server',
                 [('cmd_vel', raw)]),
        nav_node('nav2_planner', 'planner_server', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', 'behavior_server',
                 [('cmd_vel', out)]),
        nav_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                 [('cmd_vel', raw), ('cmd_vel_smoothed', out)]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': NAV_NODES}]),
    ])
