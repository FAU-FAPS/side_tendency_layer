#!/usr/bin/env python3
# Copyright 2025 Matthias Kalenberg, Song Li
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Drive the demo route of a scenario once Nav2 is up.

Started by the sim_*.launch.py files (demo_goal:=true). Waits until AMCL and the BT
navigator are active, publishes the initial pose and sends the goal. Several goals are driven
through in order. Logs the remaining distance until the end, then exits; the simulation keeps
running and further goals can be set in RViz.

    ros2 run side_tendency_layer demo_goal.py --initial 0 0 0 --goal 17 0 0

Poses are x y yaw in the map frame.
"""

import argparse
import math
import time

from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.utilities import remove_ros_args


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def wait_until_nav2_active(nav, initial_pose):
    """
    Wait until AMCL and the BT navigator are active and AMCL has the initial pose.

    Like BasicNavigator.waitUntilNav2Active(), but a lost service response cannot block it.
    """
    for node in ('amcl', 'bt_navigator'):
        client = nav.create_client(GetState, f'{node}/get_state')
        while not client.wait_for_service(timeout_sec=1.0):
            nav.info(f'{node}/get_state not available, waiting...')
        while True:
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, future, timeout_sec=2.0)
            result = future.result() if future.done() else None
            if result is not None and result.current_state.label == 'active':
                break
            time.sleep(1.0)
        if node == 'amcl':
            while not nav.initial_pose_received:
                nav.setInitialPose(initial_pose)
                rclpy.spin_once(nav, timeout_sec=1.0)
    nav.info('Nav2 is ready')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--initial', nargs=3, type=float, required=True,
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--goal', nargs=3, type=float, required=True, action='append',
                        metavar=('X', 'Y', 'YAW'), help='repeat for a route')
    parser.add_argument('--delay', type=float, default=3.0,
                        help='seconds to wait after Nav2 is active, so the costmaps are filled')
    rclpy.init()
    args = parser.parse_args(remove_ros_args()[1:])

    nav = BasicNavigator('demo_goal')
    # The initial pose equals the AMCL start pose from the params, so publishing it is harmless.
    wait_until_nav2_active(nav, pose(nav, *args.initial))
    time.sleep(args.delay)

    goals = [pose(nav, *goal) for goal in args.goal]
    route = ', '.join(f'({x:.2f}, {y:.2f})' for x, y, _ in args.goal)
    nav.info(f'Demo route (map frame): {route}')
    accepted = nav.goToPose(goals[0]) if len(goals) == 1 else nav.goThroughPoses(goals)
    if not accepted:
        nav.error('Demo goal was rejected')
        rclpy.shutdown()
        return

    last_log = 0.0
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback and time.monotonic() - last_log > 5.0:
            nav.info(f'Distance remaining: {feedback.distance_remaining:.1f} m')
            last_log = time.monotonic()
        time.sleep(0.1)

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.info('Demo goal reached')
    else:
        nav.warn(f'Demo goal not reached: {result}')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
