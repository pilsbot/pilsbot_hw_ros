from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pilsbot_description import profile


def generate_launch_description():

    declared_launch_args = []

    declared_launch_args.append(DeclareLaunchArgument(
        'robot_profile', default_value=TextSubstitution(text='pilsbot'),
        description='Robot profile: a name shipped with pilsbot_description, or a path.'))

    declared_launch_args.append(DeclareLaunchArgument(
        'joystick_device_id', default_value=TextSubstitution(text=''),
        description='SDL index of the gamepad (joy_node device_id). Default: from the robot profile'))

    declared_launch_args.append(DeclareLaunchArgument(
        'joystick_device_name', default_value=TextSubstitution(text=''),
        description='SDL name of the gamepad; takes precedence over the index. '
                    'Default: from the robot profile'))

    declared_launch_args.append(DeclareLaunchArgument(
        'joystick_conf', default_value=TextSubstitution(text=''),
        description='Mapping file in pilsbot_driver/config, e.g. medion_0678.yaml. '
                    'Default: from the robot profile'))

    return LaunchDescription(declared_launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    robot_profile = profile.load(LaunchConfiguration('robot_profile').perform(context))
    joystick_device_id = int(LaunchConfiguration('joystick_device_id').perform(context)
                             or robot_profile.joystick_device_id)
    joystick_device_name = (LaunchConfiguration('joystick_device_name').perform(context)
                            or robot_profile.joystick_device_name)
    joystick_conf = (LaunchConfiguration('joystick_conf').perform(context)
                     or robot_profile.joystick_mapping + '.yaml')

    joystick_mapping_config = PathJoinSubstitution(
        [
            FindPackageShare("pilsbot_driver"),
            "config",
            joystick_conf,
        ]
    )

    joystick_in = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            # Humble's joy_node is SDL-based and has no 'dev' parameter.
            {'device_id': joystick_device_id, 'device_name': joystick_device_name},
            {'coalesce_interval_ms' : 2}    # this might or might not work. Well.
            ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    joystick_mapper = Node(
        package='teleop_acker_joy',
        executable='teleop_acker_node',
        name='teleop_acker_joy_node',
        parameters=[joystick_mapping_config],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        remappings=[
            ('/cmd/vel', '/pilsbot_velocity_controller/cmd_vel'),
        ]
    )

    # twist_2_acker = Node(
    #     package='acker_diff_controller',
    #     executable='cmd_vel_to_ackermann.py',
    #     parameters=[
    #         {"message_type" : "AckermannDriveStamped"},
    #         {"ackermann_cmd_topic" : "/pilsbot_velocity_controller/cmd_vel"}
    #     ],
    #     output={
    #         'stdout': 'screen',
    #         'stderr': 'screen',
    #     }
    # )


    return [
        joystick_in,
        joystick_mapper,
        # twist_2_acker
    ]
