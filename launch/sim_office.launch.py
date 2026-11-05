# Copyright 2025 Matthias Kalenberg, Song Li
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Scenario office: the simulation of Figure 6a in the paper.

A loop around an inner block with corridors of different widths, doors, a crossing,
pedestrians and cluttered areas.

    ros2 launch side_tendency_layer sim_office.launch.py
    ros2 launch side_tendency_layer sim_office.launch.py side_tendency:=false

The second line is the baseline without the layer. The demo route goes to the four corners of
the loop, taken from the global plan recorded in the paper's run. The map is in the world
frame. Every value below can be overridden from the command line, see README.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SCENARIO = {
    'world': 'office.world',  # Gazebo world in worlds/
    'map': 'office.yaml',  # map for AMCL in maps/
    'x_pose': '4.5',  # spawn pose in the Gazebo world
    'y_pose': '2.0',
    'yaw': '1.57',
    # demo route in the map frame, x y yaw; ...
    'goals': '5.3 8.7 0.0; 15.4 7.4 -1.57; 13.6 0.9 3.14; 1.0 0.0 3.14',
}


def generate_launch_description():
    simulation = os.path.join(get_package_share_directory('side_tendency_layer'),
                              'launch', 'simulation.launch.py')
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in SCENARIO.items()]
        + [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={name: LaunchConfiguration(name) for name in SCENARIO}.items())])
