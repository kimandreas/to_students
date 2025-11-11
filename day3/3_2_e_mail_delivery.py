#!/usr/bin/env python3

# Copyright 2023 Clearpath Robotics, Inc.
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
# @author Hilary Luo (hluo@clearpathrobotics.com)

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main(args=None):
    rclpy.init(args=args)

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

    # Undock
    navigator.undock()

    # Prepare goal pose options
    goal_options = [
        {'name': 'Home',
         'pose': navigator.getPoseStamped([-0.6, 0.0], TurtleBot4Directions.NORTH)},

        # {'name': 'Position 1',
        #  'pose': navigator.getPoseStamped([-1.7, -0.1], TurtleBot4Directions.EAST)},

        # {'name': 'Position 2',
        #  'pose': navigator.getPoseStamped([-1.1, 1.6], TurtleBot4Directions.NORTH)},

        # {'name': 'Position 3',
        #  'pose': navigator.getPoseStamped([-1.0, 0.05], TurtleBot4Directions.NORTH_WEST)},

        # {'name': 'Position 4',
        #  'pose': navigator.getPoseStamped([3.0, -7.0], TurtleBot4Directions.WEST)},

    # goal_pose.append(navigator.getPoseStamped([-1.55069, 0.0668084], -2.5896))
    # goal_pose.append(navigator.getPoseStamped([-0.761671, -0.852567], -1.02024))
    # goal_pose.append(navigator.getPoseStamped([0.0343325, -1.96793], 0.21976))
    # goal_pose.append(navigator.getPoseStamped([-0.711899, -0.612125], 0.606873))

        {'name': 'Position 1',
         'pose': navigator.getPoseStamped([-1.55069, 0.0668084], -2.5896)},

        {'name': 'Position 2',
         'pose': navigator.getPoseStamped([-0.761671, -0.852567], -1.02024)},

        {'name': 'Position 3',
         'pose': navigator.getPoseStamped([0.0343325, -1.96793], 0.21976)},

        {'name': 'Position 4',
         'pose': navigator.getPoseStamped([-0.711899, -0.0612125], 0.606873)},

        {'name': 'Exit',
         'pose': None}
    ]

    navigator.info('Welcome to the mail delivery service.')

    while True:
        # Create a list of the goals for display
        options_str = 'Please enter the number corresponding to the desired robot goal position:\n'
        for i in range(len(goal_options)):
            options_str += f'    {i}. {goal_options[i]["name"]}\n'

        # Prompt the user for the goal location
        raw_input = input(f'{options_str}Selection: ')

        selected_index = 0

        # Verify that the value input is a number
        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'Invalid goal selection: {raw_input}')
            continue

        # Verify that the user input is within a valid range
        if (selected_index < 0) or (selected_index >= len(goal_options)):
            navigator.error(f'Goal selection out of bounds: {selected_index}')

        # Check for exit
        elif goal_options[selected_index]['name'] == 'Exit':
            selected_index = 0
            navigator.info('Going Home...')
            navigator.startToPose(goal_options[selected_index]['pose'])
            navigator.info('Docking before exiting')
            navigator.dock()
            break

        else:
            # Navigate to requested position
            navigator.startToPose(goal_options[selected_index]['pose'])

    rclpy.shutdown()


if __name__ == '__main__':
    main()
