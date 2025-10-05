from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    model_arg = DeclareLaunchArgument('model', default_value='burger')
    set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value=LaunchConfiguration('model'))

    # ✅ Properly find turtlebot3_gazebo package from system
    tb3_gazebo_dir = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'launch', 'empty_world.launch.py'))
    )

    return LaunchDescription([
        model_arg,
        set_model,
        world_launch
    ])
