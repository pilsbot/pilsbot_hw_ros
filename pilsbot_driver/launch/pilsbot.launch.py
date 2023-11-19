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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_launch_args = []

    declared_launch_args.append(DeclareLaunchArgument(
        'controller_config', default_value=TextSubstitution(text='acker_diff_controller.yaml'),
        description='The controller configuration you want to use.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'runtime_config_package', default_value=TextSubstitution(text='pilsbot_control'),
        description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup'))

    declared_launch_args.append(DeclareLaunchArgument(
        'descritption_file', default_value=TextSubstitution(text='pilsbot.urdf.xacro'),
        description='URDF/XACRO description file with the robot.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'description_package', default_value=TextSubstitution(text='pilsbot_description'),
        description=' Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.'))

    # intialise args
    controller_config = LaunchConfiguration('controller_config')
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    description_file = LaunchConfiguration('descritption_file')
    description_package = LaunchConfiguration('description_package')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "urdf",
                    description_file,
                ]
            ),
            " ",
            "controller_config:=",
            controller_config,
            " ",
            "use_fake_hardware:=false",
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    pilsbot_acker_diff_controller_config = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            controller_config,
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description,
                    pilsbot_acker_diff_controller_config,
                    ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    spawn_pb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["pilsbot_velocity_controller"],
        output="screen",
    )
    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_publisher"],
        output="screen",
    )

    spawn_ss_publisher_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=['pilsbot_sensor_broadcaster'],
        output="screen",
    )

    nodes = [control_node, 
        spawn_pb_controller, spawn_jsb_controller, spawn_ss_publisher_controller,
        robot_state_publisher_node]

    return LaunchDescription(declared_launch_args + nodes)
