# Copyright 2025 Matthias Kalenberg, Song Li
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Scenario straight_corridor: a straight 20 m corridor, 2.85 m wide.

    ros2 launch side_tendency_layer sim_straight_corridor.launch.py
    ros2 launch side_tendency_layer sim_straight_corridor.launch.py side_tendency:=false

The second line is the baseline without the layer. The demo route goes to the end of the
corridor. The map was recorded with SLAM from the spawn pose, so AMCL starts at the map origin.
Every value below can be overridden from the command line, see README.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SCENARIO = {
    'world': 'straight_corridor.world',  # Gazebo world in worlds/
    'map': 'straight_corridor.yaml',  # map for AMCL in maps/
    'x_pose': '-9.0',  # spawn pose in the Gazebo world
    'y_pose': '0.0',
    'yaw': '0.0',
    'initial_x': '0.0',  # AMCL start pose in the map frame
    'initial_y': '0.0',
    'initial_yaw': '0.0',
    'goals': '17.0 0.0 0.0',  # demo route in the map frame, x y yaw; ...
}


def generate_launch_description():
    simulation = os.path.join(get_package_share_directory('side_tendency_layer'),
                              'launch', 'simulation.launch.py')
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in SCENARIO.items()]
        + [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={name: LaunchConfiguration(name) for name in SCENARIO}.items())])
