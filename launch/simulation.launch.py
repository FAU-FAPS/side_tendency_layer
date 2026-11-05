# Copyright 2025 Matthias Kalenberg, Song Li
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Simulation with the side tendency layer, included by the sim_*.launch.py files.

    ros2 launch side_tendency_layer simulation.launch.py world:=a.world map:=a.yaml goals:="5 0 0"

Starts nav2_bringup's tb3_simulation_launch.py (Gazebo, TurtleBot3 waffle, AMCL on a static
map, Nav2 with DWB, RViz) with config/nav2_tb3_side_tendency.yaml. With side_tendency:=false
the layer is removed from the costmaps and everything else stays identical, so the two runs
can be compared directly. Once Nav2 is up, demo_goal.py drives the demo route given in goals
(demo_goal:=false switches it off).
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

PACKAGE = 'side_tendency_layer'
LAYER = 'side_tendency_layer'


def _params_file(source, side_tendency, initial):
    """Return the params file with the AMCL initial pose set and, if wanted, the layer removed."""
    with open(source) as f:
        params = yaml.safe_load(f)
    for costmap in ('local_costmap', 'global_costmap'):
        ros_params = params[costmap][costmap]['ros__parameters']
        if not side_tendency:
            ros_params['plugins'] = [p for p in ros_params['plugins'] if p != LAYER]
    amcl = params['amcl']['ros__parameters']
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': initial[0], 'y': initial[1], 'z': 0.0, 'yaw': initial[2]}
    suffix = '_st.yaml' if side_tendency else '_baseline.yaml'
    out = tempfile.NamedTemporaryFile('w', prefix='nav2_tb3_', suffix=suffix, delete=False)
    with out:
        yaml.safe_dump(params, out, sort_keys=False)
    return out.name


def _setup(context):
    pkg_dir = get_package_share_directory(PACKAGE)

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def is_true(name):
        return arg(name).lower() in ('true', '1', 'yes')

    def package_file(name, folder):
        return name if os.path.isabs(name) else os.path.join(pkg_dir, folder, name)

    spawn = tuple(float(arg(n)) for n in ('x_pose', 'y_pose', 'yaw'))
    # The AMCL initial pose defaults to the spawn pose, which fits maps in the world frame.
    initial = tuple(float(arg(n)) if arg(n) else s
                    for n, s in zip(('initial_x', 'initial_y', 'initial_yaw'), spawn))
    goals = [tuple(float(v) for v in goal.split()) for goal in arg('goals').split(';')
             if goal.strip()]
    if any(len(goal) != 3 for goal in goals):
        raise RuntimeError(f'goals must be "x y yaw; x y yaw; ...", got {arg("goals")!r}')
    params_file = _params_file(arg('params_file'), is_true('side_tendency'), initial)

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'world': package_file(arg('world'), 'worlds'),
            'map': package_file(arg('map'), 'maps'),
            'params_file': params_file,
            'x_pose': str(spawn[0]),
            'y_pose': str(spawn[1]),
            'yaw': str(spawn[2]),
            'headless': arg('headless'),
            'use_rviz': arg('use_rviz'),
            'use_sim_time': 'True',
        }.items(),
    )]
    if is_true('demo_goal') and goals:
        # Waits by itself until Nav2 is active, then drives the route once.
        goal_args = []
        for goal in goals:
            goal_args += ['--goal', *map(str, goal)]
        actions.append(Node(
            package=PACKAGE,
            executable='demo_goal.py',
            name='demo_goal',
            output='screen',
            arguments=['--initial', *map(str, initial), *goal_args],
            parameters=[{'use_sim_time': True}],
        ))
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory(PACKAGE)

    return LaunchDescription([
        DeclareLaunchArgument('world', description='Gazebo world, a file in worlds/ or a path'),
        DeclareLaunchArgument('map', description='Map for AMCL, a file in maps/ or a path'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Spawn x in the world'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Spawn y in the world'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw in the world'),
        DeclareLaunchArgument('initial_x', default_value='',
                              description='AMCL initial x in the map; empty: spawn x'),
        DeclareLaunchArgument('initial_y', default_value='',
                              description='AMCL initial y in the map; empty: spawn y'),
        DeclareLaunchArgument('initial_yaw', default_value='',
                              description='AMCL initial yaw in the map; empty: spawn yaw'),
        DeclareLaunchArgument('goals', default_value='',
                              description='Demo route in the map frame, "x y yaw; x y yaw; ..."'),
        DeclareLaunchArgument('side_tendency', default_value='true',
                              description='false removes the side tendency layer (baseline)'),
        DeclareLaunchArgument('demo_goal', default_value='true',
                              description='Drive the demo route once Nav2 is up'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_tb3_side_tendency.yaml'),
            description='Nav2 parameters including the side tendency layer'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='True: no Gazebo GUI'),
        DeclareLaunchArgument('use_rviz', default_value='True', description='Start RViz'),

        # Models of the office world come with this package, the waffle's meshes with
        # turtlebot3_gazebo, sun and ground plane with Gazebo itself.
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models'),
        AppendEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_dir, 'models')),

        OpaqueFunction(function=_setup),
    ])
