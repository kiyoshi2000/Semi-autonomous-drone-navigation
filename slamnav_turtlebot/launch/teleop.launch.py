#!/usr/bin/env python3
#
#
# Authors: Jeremy Fix

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Define the three containers used for defining the LaunchDescription
    arguments = []
    nodes = []
    launch_files = []

    # Define some arguments that can be changed on the command line if necessary
    teleop_linear_factor = DeclareLaunchArgument("linear_factor", default_value="0.5")
    teleop_angular_factor = DeclareLaunchArgument("angular_factor", default_value="1.0")


    
    arguments += [teleop_linear_factor, teleop_angular_factor]

    # Teleop
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy",
    )

    # include the twist mode 

    slamnav_share_dir = get_package_share_directory("slamnav_turtlebot")
    
    mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[os.path.join(slamnav_share_dir, "config/mux.yaml")],
        remappings=[("/cmd_vel_out", "/commands/velocity")],
        )

    joy_teleop_node = Node(
        package="slamnav_turtlebot",
        executable="joy_teleop",
        name="joyteleop",
        parameters=[
            {
                "linear_factor": LaunchConfiguration("linear_factor"),
                "angular_factor": LaunchConfiguration("angular_factor"),
            }
        ],
        # TODO: Once the multiplexer is added, you need
        # to adapt this remapping

        remappings=[('/cmd_vel', '/joy_vel')]

        #remappings=[("/cmd_vel", "/commands/velocity")],
    )

    slamnav_share_dir = get_package_share_directory("slamnav_turtlebot")
    mux_node = Node(
    package="twist_mux",
    executable="twist_mux",
    name="twist_mux",
    parameters=[os.path.join(slamnav_share_dir, "config/mux.yaml")],
    remappings=[("/cmd_vel_out", "/commands/velocity")],
)
    
    nodes += [joy_node, joy_teleop_node,mux_node]

    # TODO: when adding navigation, we need to use a velocity multiplexer
    # This allows to keep a joystick with precedence over the navigation

    
    mux_node = None
    if mux_node:
        nodes.append(mux_node)
