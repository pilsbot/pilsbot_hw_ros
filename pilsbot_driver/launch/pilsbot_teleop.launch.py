from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        'joystick_dev', default_value=TextSubstitution(text=''),
        description='Device file to hardware joystick. Default: from the robot profile'))

    declared_launch_args.append(DeclareLaunchArgument(
        'joystick_conf', default_value=TextSubstitution(text=''),
        description='Mapping file in pilsbot_driver/config, e.g. medion_0678.yaml. '
                    'Default: from the robot profile'))


    included_pilsbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/pilsbot.launch.py']),
        launch_arguments={
        "controller_config" : LaunchConfiguration('controller_config'),
        "runtime_config_package" : LaunchConfiguration('runtime_config_package'),
        "description_file" : LaunchConfiguration('description_file'),
        "description_package" : LaunchConfiguration('description_package'),
        "robot_profile" : LaunchConfiguration('robot_profile'),
        }.items()
    )

    included_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/teleop.launch.py']),
        launch_arguments={
        "joystick_dev" : LaunchConfiguration("joystick_dev"),
        "joystick_conf" : LaunchConfiguration("joystick_conf"),
        "robot_profile" : LaunchConfiguration('robot_profile'),
        }.items()
    )


    return LaunchDescription(declared_launch_args +
                            [included_pilsbot_launch] +
                            [included_teleop_launch])
