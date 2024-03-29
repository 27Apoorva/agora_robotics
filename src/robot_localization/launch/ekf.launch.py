# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    # log_level = DeclareLaunchArgument('log_level',
    #                                   default_value='debug',
    #                                   description='Logging level (default: debug)')

    # # Get log level from launch argument
    # log_level_param = LaunchConfiguration('log_level')

    # # Set environment variable for logging level
    # env = {'RCUTILS_LOGGING_DEFAULT_LEVEL': log_level_param}


    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            # env=env,
            # prefix=['xterm -e gdb -ex run --args'],
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
           ),
    ])
