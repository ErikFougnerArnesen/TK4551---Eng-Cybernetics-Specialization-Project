#!/usr/bin/env python3
"""
Launch file for Waypoint Mission

Mirrors the structure of the original offboard_velocity_control.launch.py

Usage:
    ros2 launch px4_offboard waypoint_mission.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # ==============================
    # LAUNCH ARGUMENTS for simulator (Static parameters to dictate target latitude, longitude, takeoff altitude and hovertime over target)
    # ==============================
    
    # Target coordinates (can be overridden from command line)
    target_lat_arg = DeclareLaunchArgument(
        'target_lat',
        default_value='47.397682',
        description='Target latitude'
    )
    
    target_lon_arg = DeclareLaunchArgument(
        'target_lon',
        default_value='8.543242',
        description='Target longitude'
    )
    
    takeoff_alt_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='100.0',
        description='Takeoff altitude in meters'
    )
    
    hover_time_arg = DeclareLaunchArgument(
        'hover_time',
        default_value='5.0',
        description='Time to hover at waypoint in seconds'
    )
    
    # =========
    # PROCESSES (Support script to open terminals and launch SITL, MicroXRCE-DDS bridge)
    # =========
    
    # Start PX4 SITL + Gazebo in a new terminal
    start_simulation = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=PX4 SITL + Gazebo', '--',
            'bash', '-c',
            'cd ~/PX4-Autopilot && make px4_sitl gz_x500; exec bash'
        ],
        output='screen',
        shell=False
    )

    
    # Start Micro XRCE-DDS Agent in a new terminal tab
    start_dds_agent = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title=DDS Agent', '--',
            'bash', '-c',
            'sleep 3 && MicroXRCEAgent udp4 -p 8888; exec bash'
        ],
        output='screen',
        shell=False
    )
    
    # ========================
    # MISSION CONTROL TERMINAL
    # ========================
    
    # Start the mission control terminal in a new window
    start_control_terminal = TimerAction(
        period=8.0,  # Wait for simulation to initialize
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--title=Mission Control', '--',
                    'bash', '-c',
                    'source /opt/ros/humble/setup.bash && '
                    'source ~/ros2_px4_offboard_ws/install/setup.bash && '
                    'ros2 run px4_offboard mission_control_terminal; exec bash'
                ],
                output='screen',
                shell=False
            )
        ]
    )
    
    # =====================
    # WAYPOINT MISSION NODE
    # =====================
    
    # Start the waypoint mission node
    waypoint_mission_node = TimerAction(
        period=6.0,  # Wait for simulation to initialize
        actions=[
            Node(
                package='px4_offboard',
                executable='waypoint_mission',
                name='waypoint_mission',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'target_lat': LaunchConfiguration('target_lat'),
                    'target_lon': LaunchConfiguration('target_lon'),
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hover_time': LaunchConfiguration('hover_time'),
                }]
            )
        ]
    )
    
    # =========================
    # RETURN LAUNCH DESCRIPTION
    # =========================
    
    return LaunchDescription([
        # Arguments
        target_lat_arg,
        target_lon_arg,
        takeoff_alt_arg,
        hover_time_arg,
        
        # Processes
        start_simulation,
        start_dds_agent,
        
        # Nodes (delayed start)
        waypoint_mission_node,
        start_control_terminal,
    ])