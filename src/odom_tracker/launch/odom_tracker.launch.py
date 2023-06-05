from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        # DeclareLaunchArgument('USE_SIM_TIME_NAME,', default_value='true', description='Grid size in meters'),

        Node(
            package='odom_tracker',
            executable='odom_tracker_node',
            name='odom_tracker',
            output='screen',
        ),
    ])