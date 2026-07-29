#!/usr/bin/env python3

# Example launch file showing how to publish a static TF for a sensor
# mounted on the drone (here: a Garmin rangefinder on an X500 frame).
#
# Copy this file (or the Node action below) into your own tmux
# project's launch/ folder and adjust the translation/rotation and
# frame names for your sensor.

import launch
import os

from launch_ros.actions import Node

def generate_launch_description():

    ld = launch.LaunchDescription()

    # #{ args from ENV

    uav_name = os.getenv('UAV_NAME', 'uav1')

    # #} end of args from ENV

    # #{ fcu_to_garmin static TF

    # garmin is mounted on the right side of the drone body, facing down
    ld.add_action(
        Node(
            package='tf2_ros',
            namespace=uav_name,
            executable='static_transform_publisher',
            name='fcu_to_garmin',
            arguments=[
                '--x', '0.0',
                '--y', '-0.068',
                '--z', '-0.0101',
                '--roll', '-1.5708',
                '--pitch', '1.5708',
                '--yaw', '0',
                '--frame-id', uav_name + '/fcu',
                '--child-frame-id', uav_name + '/garmin',
            ],
        )
    )

    # #} end of fcu_to_garmin static TF

    return ld
