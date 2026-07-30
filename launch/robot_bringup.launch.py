#!/usr/bin/env python3
# ==============================================================================
#  FineNav Robot Bringup — Launch File
# ==============================================================================
#  Launches the FineNav-Engine navigation stack for real-robot operation.
#
#  Usage:
#    ros2 launch finenav_robot_bringup robot_bringup.launch.py
#    ros2 launch finenav_robot_bringup robot_bringup.launch.py \
#      params_file:=/path/to/custom_params.yaml
# ==============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve paths
    pkg_dir = get_package_share_directory("finenav_robot_bringup")
    default_params = os.path.join(pkg_dir, "config", "robot_params.yaml")

    # ── Launch arguments ──────────────────────────────────────────────────
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to the YAML parameters file for robot bringup.",
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="navigation",
        description="Bringup mode: 'localization' (ESKF only) or 'navigation' (full stack).",
    )

    # ── FineNav Robot Bringup node ────────────────────────────────────────
    bringup_node = Node(
        package="finenav_robot_bringup",
        executable="robot_bringup",
        name="robot_bringup",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"mode": LaunchConfiguration("mode")},
        ],
    )

    # Use TimerAction to ensure parameter loading is complete before node
    # startup (avoids race conditions with transient_local map topics).
    bringup_delayed = TimerAction(
        period=1.0,
        actions=[bringup_node],
    )

    return LaunchDescription([
        params_file_arg,
        mode_arg,
        bringup_delayed,
    ])
