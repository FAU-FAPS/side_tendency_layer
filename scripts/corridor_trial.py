#!/usr/bin/env python3
# Copyright 2025 Matthias Kalenberg, Song Li
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Drive one corridor trial and record the path.

    ros2 run side_tendency_layer corridor_trial.py --start 0 0 0 --goal 17 0 0 --out st.csv

Launch the simulation with demo_goal:=false. Sets the start as initial pose, sends a
NavigateToPose goal and samples map -> base_footprint at 10 Hz until the goal is reached.
Writes t, x, y and the signed lateral offset to the straight line from start to goal
(positive = left of the travel direction) and prints the offset statistics, so runs with and
without the layer can be compared. Poses are x y yaw in the map frame.
"""

import argparse
import csv
import math
import time

from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from tf2_ros import Buffer, TransformException, TransformListener


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
    parser.add_argument('--start', nargs=3, type=float, default=[-17.0, -3.45, 0.0],
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--goal', nargs=3, type=float, default=[7.0, -3.45, 0.0],
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--out', default='corridor_trial.csv',
                        help='CSV file for the recorded path')
    parser.add_argument('--timeout', type=float, default=300.0,
                        help='seconds (sim time) before cancelling')
    args, _ = parser.parse_known_args()

    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters(
        [rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    tf_buffer = Buffer()
    TransformListener(tf_buffer, nav)

    # The start is where AMCL starts, so publishing it as initial pose is always correct.
    wait_until_nav2_active(nav, pose(nav, *args.start))

    sx, sy = args.start[:2]
    gx, gy = args.goal[:2]
    length = math.hypot(gx - sx, gy - sy)
    ux, uy = (gx - sx) / length, (gy - sy) / length

    nav.goToPose(pose(nav, *args.goal))
    rows, t0 = [], None
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.1)
        try:
            tf = tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            continue
        now = nav.get_clock().now().nanoseconds * 1e-9
        t0 = now if t0 is None else t0
        if rows and now - rows[-1][0] - t0 < 0.1:
            continue
        x, y = tf.transform.translation.x, tf.transform.translation.y
        along = (x - sx) * ux + (y - sy) * uy
        lateral = -(x - sx) * uy + (y - sy) * ux
        rows.append((now - t0, x, y, along, lateral))
        if now - t0 > args.timeout:
            nav.cancelTask()
        time.sleep(0.02)

    result = nav.getResult()
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'along', 'lateral'])
        w.writerows(rows)

    inner = [r[4] for r in rows if 0.1 * length < r[3] < 0.9 * length] or [0.0]
    path_len = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(rows, rows[1:]))
    outcome = 'SUCCEEDED' if result == TaskResult.SUCCEEDED else str(result)
    print(f'result:            {outcome}')
    print(f'duration [s]:      {rows[-1][0] if rows else 0.0:.1f}')
    print(f'path length [m]:   {path_len:.2f}  (straight line {length:.2f})')
    print('lateral offset [m], middle 80 % of the way, + = left of travel direction:')
    print(f'  mean {sum(inner) / len(inner):+.3f}   min {min(inner):+.3f}   max {max(inner):+.3f}')
    print(f'path written to {args.out}')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
