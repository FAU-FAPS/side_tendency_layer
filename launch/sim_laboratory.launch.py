# Copyright 2025 Matthias Kalenberg, Song Li
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Scenario laboratory: long corridor of a laboratory, 2.9 m wide.

    ros2 launch side_tendency_layer sim_laboratory.launch.py
    ros2 launch side_tendency_layer sim_laboratory.launch.py side_tendency:=false

The second line is the baseline without the layer. The demo route goes to the east end of the
corridor. The map is in the world frame. Every value below can be overridden from the command
line, see README.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SCENARIO = {
    'world': 'laboratory.world',  # Gazebo world in worlds/
    'map': 'laboratory.yaml',  # map for AMCL in maps/
    'x_pose': '-17.0',  # spawn pose in the Gazebo world
    'y_pose': '-3.45',
    'yaw': '0.0',
    'goals': '7.0 -3.45 0.0',  # demo route in the map frame, x y yaw; ...
}


def generate_launch_description():
    simulation = os.path.join(get_package_share_directory('side_tendency_layer'),
                              'launch', 'simulation.launch.py')
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in SCENARIO.items()]
        + [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={name: LaunchConfiguration(name) for name in SCENARIO}.items())])
