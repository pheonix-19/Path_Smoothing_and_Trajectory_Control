from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # --- Arguments ---
    model_arg = DeclareLaunchArgument('model', default_value='burger')
    wayfile_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=os.path.join(
            FindPackageShare('nav_pipeline').find('nav_pipeline'),
            'config', 'waypoints.yaml'
        )
    )

    set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value=LaunchConfiguration('model'))

    # --- Find TB3 gazebo world and spawn files ---
    tb3_gazebo_dir = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'launch', 'empty_world.launch.py'))
    )

    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py'))
    )

    # --- Our pipeline nodes ---
    wp = Node(
        package='nav_pipeline', executable='waypoint_server', name='waypoint_server',
        parameters=[{'waypoint_file': LaunchConfiguration('waypoint_file'), 'frame_id': 'map'}],
        output='screen'
    )

    smoother = Node(
        package='nav_pipeline', executable='path_smoother', name='path_smoother',
        parameters=[{'samples_per_segment': 15, 'resample_ds': 0.03, 'frame_id': 'map'}]
    )

    traj = Node(
        package='nav_pipeline', executable='trajectory_generator', name='trajectory_generator',
        parameters=[{'v_max': 0.25, 'a_max': 0.5, 'frame_id': 'map'}]
    )

    ctrl = Node(
        package='nav_pipeline', executable='pure_pursuit_controller', name='pure_pursuit_controller',
        parameters=[{'lookahead': 0.6, 'v_nominal': 0.22, 'odom_topic': '/odom', 'topic_cmd_vel': '/cmd_vel'}]
    )

    rviz = Node(package='rviz2', executable='rviz2', output='screen')

    return LaunchDescription([
        model_arg,
        wayfile_arg,
        set_model,
        world_launch,
        spawn_tb3,
        wp,
        smoother,
        traj,
        ctrl,
        rviz
    ])
