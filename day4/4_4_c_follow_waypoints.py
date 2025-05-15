#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []
    # goal_pose.append(navigator.getPoseStamped([-3.3, 5.9], TurtleBot4Directions.NORTH))
    # goal_pose.append(navigator.getPoseStamped([2.1, 6.3], TurtleBot4Directions.EAST))
    # goal_pose.append(navigator.getPoseStamped([2.0, 1.0], TurtleBot4Directions.SOUTH))
    # goal_pose.append(navigator.getPoseStamped([-1.0, 0.0], TurtleBot4Directions.NORTH))

    goal_pose.append(navigator.getPoseStamped([-1.55069, 0.0668084], -2.5896))
    goal_pose.append(navigator.getPoseStamped([-0.761671, -0.852567], -1.02024))
    goal_pose.append(navigator.getPoseStamped([0.0343325, -1.96793], 0.21976))
    goal_pose.append(navigator.getPoseStamped([-0.711899, -0.0612125], 0.606873))

    # Undock
    navigator.undock()

    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
