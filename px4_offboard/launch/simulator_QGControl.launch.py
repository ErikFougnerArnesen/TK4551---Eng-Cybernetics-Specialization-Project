import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')

    offboard_keepalive_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='offboard_keepalive',
        name='offboard_keepalive',
        output='screen',
    )

    mission_qgc_node = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='mission_qgc',
        name='mission_qgc',
        output='screen',
        parameters=[{
            'target_lat_deg': 47.40025000,
            'target_lon_deg': 8.5069444,
            'altitude_m': 20.0,
            'auto_arm': False,
            'auto_offboard': False,
            'auto_land': True,
        }],
    )

    start_mission_after_keepalive = RegisterEventHandler(
        OnProcessExit(
            target_action=offboard_keepalive_node,
            on_exit=[mission_qgc_node],
        )
    )

    return LaunchDescription([
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),

        # Start keepalive immediately so Offboard appears in QGC
        offboard_keepalive_node,

        # Start mission only after keepalive exits
        start_mission_after_keepalive,

        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        )
    ])
