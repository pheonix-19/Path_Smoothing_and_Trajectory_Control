from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # === Arguments ===
    model_arg = DeclareLaunchArgument('model', default_value='burger')
    wayfile_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=os.path.join(
            FindPackageShare('nav_pipeline').find('nav_pipeline'),
            'config', 'waypoints.yaml'
        )
    )

    set_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value=LaunchConfiguration('model')
    )

    # === TurtleBot3 Gazebo world ===
    tb3_gazebo_dir = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'empty_world.launch.py')
        )
    )

    # --- Spawn robot at first waypoint (0,0) ---
    first_wp = [0.0, 0.0, 0.0]  # x, y, yaw (radians)
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': str(first_wp[0]),
            'y_pose': str(first_wp[1]),
            'z_pose': '0.01',
            'Y_pose': str(first_wp[2])
        }.items(),
    )

    # === Correct ros_gz_bridge (Twist only) ===
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_single',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # === Configuration files ===
    controller_config = os.path.join(
        FindPackageShare('nav_pipeline').find('nav_pipeline'),
        'config', 'controller.yaml'
    )

    # === Waypoint Server ===
    wp = Node(
        package='nav_pipeline',
        executable='waypoint_server',
        name='waypoint_server',
        parameters=[
            controller_config,
            {'waypoint_file': LaunchConfiguration('waypoint_file')}
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

    # === Pure Pursuit Controller ===
    ctrl = Node(
        package='nav_pipeline',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        parameters=[controller_config],
        output='screen',
    )

    # === RViz (autoload config) ===
    rviz_config = os.path.join(
        FindPackageShare('nav_pipeline').find('nav_pipeline'),
        'config', 'nav_pipeline.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        remappings=[('/cmd_vel', '/rviz_cmd_vel')]
    )

    # === Launch everything ===
    return LaunchDescription([
        model_arg,
        wayfile_arg,
        set_model,
        world_launch,
        spawn_tb3,
        bridge,
        wp,
        smoother,
        traj,
        ctrl,
        rviz,
    ])
