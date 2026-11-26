# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pilsbot_description import profile


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_profile', default_value='pilsbot',
            description='Robot profile: a name shipped with pilsbot_description, or a path.'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    robot_profile = profile.load(LaunchConfiguration('robot_profile').perform(context))
    mcu_node = Node(
      package='pilsbot_driver',
      executable='head_mcu_node',
      parameters=[
          {"serial_port": robot_profile.devices['head_mcu']},
          {"publish_rate": 100},
          {"calibration_val": robot_profile.steering_calibration_flat()},
      ],
      output={
          'stdout': 'screen',
          'stderr': 'screen',
        },
    )
    return [mcu_node]
