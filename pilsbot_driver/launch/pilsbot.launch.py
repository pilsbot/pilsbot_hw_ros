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

import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from pilsbot_description import profile

# <ros2_control name=...> in pilsbot_description's URDF.
HARDWARE_COMPONENT = 'PilsbotSystem'
CONTROLLERS = ['joint_state_publisher', 'pilsbot_velocity_controller']


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
        'description_file', default_value=TextSubstitution(text='pilsbot.urdf.xacro'),
        description='URDF/XACRO description file with the robot.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'description_package', default_value=TextSubstitution(text='pilsbot_description'),
        description=' Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'robot_profile', default_value=TextSubstitution(text='pilsbot'),
        description='Robot profile: a name shipped with pilsbot_description, or a path.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'hardware', default_value='real', choices=['real', 'gazebo'],
        description='real: the robot\'s devices through ros2_control_node. gazebo: the '
                    'controller manager runs inside Gazebo\'s gz_ros2_control plugin.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'],
        description='Use /clock (true in simulation).'))

    return LaunchDescription(declared_launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    controller_config = LaunchConfiguration('controller_config')
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    description_file = LaunchConfiguration('description_file')
    description_package = LaunchConfiguration('description_package')
    robot_profile = profile.load(LaunchConfiguration('robot_profile').perform(context))
    hardware = LaunchConfiguration('hardware').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    # The controller's geometry comes from the robot profile, use_sim_time from
    # the launch argument. A params file (not a dict) reaches the controllers'
    # own nodes, not just the manager.
    with tempfile.NamedTemporaryFile(
            'w', prefix='pilsbot_profile_', suffix='.yaml', delete=False) as overrides:
        yaml.safe_dump({
            'controller_manager': {'ros__parameters': {'use_sim_time': use_sim_time}},
            'joint_state_publisher': {'ros__parameters': {'use_sim_time': use_sim_time}},
            'pilsbot_velocity_controller': {'ros__parameters': {
                **robot_profile.controller_overrides(), 'use_sim_time': use_sim_time}},
        }, overrides)

    xacro_hardware_args = [f"hardware:={hardware}"]
    if hardware == 'real':
        # Transition: pilsbot_description's use_fake_hardware defaults to true
        # (Gazebo Classic) until pilsbot/pilsbot_description#6 replaces it with
        # `hardware`. xacro ignores undeclared arguments, so this line is
        # harmless afterwards; drop it once #6 is merged.
        xacro_hardware_args += [" ", "use_fake_hardware:=false"]
    if hardware == 'gazebo':
        # gz_ros2_control reads the controller parameters from the URDF.
        xacro_hardware_args += [" ", f"controller_overrides:={overrides.name}"]

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
            *xacro_hardware_args,
            " ",
            f"robot_profile:={robot_profile.path}",
        ]
    )

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)}

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
                    overrides.name,
                    # Start the hardware inactive: Humble's controller manager aborts
                    # if activation fails at startup (e.g. hoverboard switched off).
                    # hardware_activator activates it as soon as the devices answer.
                    {'hardware_components_initial_state.inactive': [HARDWARE_COMPONENT]},
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
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Controllers are loaded inactive; the activator activates them once the
    # hardware is active, and again whenever the hardware dropped out.
    spawn_pb_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pilsbot_velocity_controller", "--inactive"],
        output="screen",
    )
    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_publisher", "--inactive"],
        output="screen",
    )
    hardware_activator = Node(
        package="pilsbot_driver",
        executable="hardware_activator",
        parameters=[{'hardware_component': HARDWARE_COMPONENT,
                     'controllers': CONTROLLERS,
                     'use_sim_time': use_sim_time}],
        output="screen",
    )

    # Without the controller manager nothing drives or stops the robot. End the
    # whole launch so the container exits: on the robot, useful-scripts'
    # startall.bash then runs disable_motors.bash and systemd starts the stack
    # again. startall.bash disables the motors from an EXIT trap
    # (pilsbot/useful-scripts#2), so that happens whatever the exit code;
    # before that trap, its `set -e` skipped disable_motors.bash unless the
    # stack exited 0. A Shutdown makes `ros2 launch` exit 0 either way.
    shutdown_without_controller_manager = RegisterEventHandler(OnProcessExit(
        target_action=control_node,
        on_exit=[Shutdown(reason='the controller manager (ros2_control_node) exited')]))

    common = [spawn_pb_controller, spawn_jsb_controller,
              hardware_activator,
              robot_state_publisher_node]

    # In Gazebo the controller manager lives in the gz_ros2_control plugin,
    # which also activates the hardware itself: the activator then only
    # activates the controllers.
    if hardware == 'gazebo':
        return common
    return [control_node, shutdown_without_controller_manager] + common
