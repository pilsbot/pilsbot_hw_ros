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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    example_parameters = [
        {"serial_port": "/dev/ttySparkfunProMicro"},
        {"publish_rate": 100},
        {"calibration_val" : [ # warning: Is noch hardcoded
  50182, -1.56601,
  31881, 0.0,
  14017, 1.56601,
            ]
        }
    ]

    mcu_node = Node(
      package='pilsbot_driver',
      executable='head_mcu_node',
      parameters=example_parameters,
      output={
          'stdout': 'screen',
          'stderr': 'screen',
        },
    )
    return LaunchDescription([
        mcu_node,
    ])
