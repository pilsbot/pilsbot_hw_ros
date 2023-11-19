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
        'joystick_dev', default_value=TextSubstitution(text='/dev/input/js0'),
        description='Device file to hardware joystick'))

    declared_launch_args.append(DeclareLaunchArgument(
        'joystick_conf', default_value=TextSubstitution(text='medion_0678.yaml'),
        description='Config value for joystick mapper'))

    joystick_mapping_config = PathJoinSubstitution(
        [
            FindPackageShare("pilsbot_driver"),
            "config",
            LaunchConfiguration('joystick_conf'),
        ]
    )

    joystick_in = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            {"dev" : LaunchConfiguration('joystick_dev')}
            ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    joystick_mapper = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joystick_mapping_config],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    twist_2_acker = Node(
        package='acker_diff_controller',
        executable='cmd_vel_to_ackermann.py',
        parameters=[
            {"message_type" : "AckermannDriveStamped"},
            {"ackermann_cmd_topic" : "/pilsbot_velocity_controller/cmd_vel"}
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )


    nodes = [
        joystick_in,
        joystick_mapper,
        twist_2_acker
    ]

    return LaunchDescription(declared_launch_args + 
                            nodes)
