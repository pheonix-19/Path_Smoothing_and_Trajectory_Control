from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Configuration files
    controller_config = os.path.join(
        FindPackageShare('nav_pipeline').find('nav_pipeline'),
        'config', 'controller.yaml'
    )

    waypoint_file = os.path.join(
        FindPackageShare('nav_pipeline').find('nav_pipeline'),
        'config', 'waypoints.yaml'
    )

    rviz_config = os.path.join(
        FindPackageShare('nav_pipeline').find('nav_pipeline'),
        'config', 'nav_pipeline.rviz'
    )

    # === Waypoint Server ===
    wp = Node(
        package='nav_pipeline',
        executable='waypoint_server',
        name='waypoint_server',
        parameters=[
            controller_config,
            {'waypoint_file': waypoint_file}
        ],
        output='screen',
    )

    # === Path Smoother ===
    smoother = Node(
        package='nav_pipeline',
        executable='path_smoother',
        name='path_smoother',
        parameters=[controller_config],
        output='screen',
    )

    # === Trajectory Generator ===
    traj = Node(
        package='nav_pipeline',
        executable='trajectory_generator',
        name='trajectory_generator',
        parameters=[controller_config],
        output='screen',
    )

    # === Debug Controller ===
    debug = Node(
        package='nav_pipeline',
        executable='debug_controller',
        name='debug_controller',
        output='screen',
    )

    # === Fake Odom Publisher ===
    fake_odom = Node(
        package='nav_pipeline',
        executable='fake_odom_publisher',
        name='fake_odom_publisher',
        parameters=[{
            'publish_rate': 20.0,
            'use_sim_time': False,
            'frame_id': 'odom',
            'child_frame_id': 'base_link'
        }],
        output='screen',
    )

    # === RViz (autoload config) ===
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # === Pure Pursuit Controller ===
    ctrl = Node(
        package='nav_pipeline',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        parameters=[controller_config],
        output='screen',
    )

    # === Launch everything ===
    return LaunchDescription([
        wp,
        smoother,
        traj,
        fake_odom,
        ctrl,
        debug,
        rviz,
    ])