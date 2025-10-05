from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    wayfile_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=os.path.join(
            os.getenv('COLCON_PREFIX_PATH','/opt/ros/jazzy'), 
            'share', 'nav_pipeline', 'config', 'waypoints.yaml'
        )
    )

    waypoint_file = LaunchConfiguration('waypoint_file')

    wp = Node(
        package='nav_pipeline', executable='waypoint_server', name='waypoint_server',
        parameters=[{'waypoint_file': waypoint_file, 'frame_id': 'map'}]
    )

    smoother = Node(
        package='nav_pipeline', executable='path_smoother', name='path_smoother',
        parameters=[{'samples_per_segment': 15, 'resample_ds': 0.03, 'closed': False, 'frame_id': 'map'}]
    )

    traj = Node(
        package='nav_pipeline', executable='trajectory_generator', name='trajectory_generator',
        parameters=[{'v_max': 0.25, 'a_max': 0.5, 'frame_id': 'map', 'publish_rate': 5.0}]
    )

    ctrl = Node(
        package='nav_pipeline', executable='pure_pursuit_controller', name='pure_pursuit_controller',
        parameters=[{'lookahead': 0.5, 'v_nominal': 0.22}]
    )

    # RViz to visualize markers/paths (optional; TB3 empty world does not include map frame -> use odom)
    rviz = Node(
        package='rviz2', executable='rviz2', arguments=['-d', ''],
        output='screen'
    )

    return LaunchDescription([wayfile_arg, wp, smoother, traj, ctrl, rviz])
