# What the driver launch files start for hardware:=real and hardware:=gazebo.
# The launch descriptions are expanded without starting anything (no Gazebo,
# no devices); only xacro's command line is built, not run.
import importlib.util
import pathlib

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.utilities import evaluate_parameters

LAUNCH_DIR = pathlib.Path(__file__).resolve().parent.parent / 'launch'
CONTROLLERS = ['joint_state_publisher', 'pilsbot_velocity_controller']


def load(launch_file):
    spec = importlib.util.spec_from_file_location(
        launch_file.replace('.', '_'), LAUNCH_DIR / launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def expand(launch_file, **arguments):
    """Declare the arguments and run the OpaqueFunctions, like `ros2 launch` does."""
    context = LaunchContext()
    context.extend_locals({'current_launch_file_directory': str(LAUNCH_DIR)})
    context.launch_configurations.update(arguments)
    actions = []
    for entity in load(launch_file).entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)  # applies the default and checks the choices
        elif isinstance(entity, OpaqueFunction):
            actions += entity.visit(context) or []
        else:
            actions.append(entity)
    return context, actions


def evaluate(context, substitutions):
    return perform_substitutions(context, normalize_to_list_of_substitutions(substitutions))


def nodes(actions):
    return {node.node_executable: node for node in actions if isinstance(node, Node)}


def spawner_arguments(actions):
    return sorted(node._Node__arguments for node in actions
                  if isinstance(node, Node) and node.node_executable == 'spawner')


def parameters(context, node):
    """The node's parameters from dicts, merged; robot_description is left out."""
    merged = {}
    for entry in node._Node__parameters:
        if not isinstance(entry, dict):
            continue
        entry = {name: value for name, value in entry.items()
                 if not isinstance(value, ParameterValue)}
        for evaluated in evaluate_parameters(context, [entry]):
            merged.update(evaluated)
    return merged


def parameter_files(context, node):
    return [str(entry.evaluate(context)) for entry in node._Node__parameters
            if not isinstance(entry, dict)]


def robot_description(actions):
    rsp = nodes(actions)['robot_state_publisher']
    for entry in rsp._Node__parameters:
        for value in entry.values():
            if isinstance(value, ParameterValue):
                return value.value[0]  # the xacro Command
    raise AssertionError('robot_state_publisher has no robot_description')


def xacro_arguments(context, actions):
    """xacro's command line for robot_description, as name -> value."""
    command = perform_substitutions(context, robot_description(actions).command)
    return dict(arg.split(':=', 1) for arg in command.split() if ':=' in arg)


# --- hardware:=gazebo --------------------------------------------------------

def test_gazebo_starts_no_controller_manager_and_no_shutdown_handler():
    _, actions = expand('pilsbot.launch.py', hardware='gazebo', use_sim_time='true')

    assert 'ros2_control_node' not in nodes(actions)
    assert not [a for a in actions if isinstance(a, RegisterEventHandler)]


def test_gazebo_keeps_the_robots_start_sequence():
    context, actions = expand('pilsbot.launch.py', hardware='gazebo', use_sim_time='true')

    assert spawner_arguments(actions) == [
        ['joint_state_publisher', '--inactive'], ['pilsbot_velocity_controller', '--inactive']]
    activator = parameters(context, nodes(actions)['hardware_activator'])
    assert activator['hardware_component'] == 'PilsbotSystem'
    assert list(activator['controllers']) == CONTROLLERS


def test_gazebo_sets_use_sim_time_on_state_publisher_and_activator():
    context, actions = expand('pilsbot.launch.py', hardware='gazebo', use_sim_time='true')

    for executable in ('robot_state_publisher', 'hardware_activator'):
        assert parameters(context, nodes(actions)[executable])['use_sim_time'] is True


def test_gazebo_passes_the_controller_parameters_to_xacro():
    context, actions = expand('pilsbot.launch.py', hardware='gazebo', use_sim_time='true')

    xacro = xacro_arguments(context, actions)
    assert xacro['hardware'] == 'gazebo'
    assert xacro['controller_config'] == 'acker_diff_controller.yaml'
    assert 'use_fake_hardware' not in xacro
    with open(xacro['controller_overrides']) as f:
        overrides = yaml.safe_load(f)
    for node in ['controller_manager'] + CONTROLLERS:
        assert overrides[node]['ros__parameters']['use_sim_time'] is True
    controller = overrides['pilsbot_velocity_controller']['ros__parameters']
    assert controller['wheel_separation'] == pytest.approx(0.62)
    assert controller['wheel_base_distance'] == pytest.approx(0.67)


# --- hardware:=real (the default) -------------------------------------------

def test_real_is_the_default_and_starts_the_controller_manager():
    context, actions = expand('pilsbot.launch.py')

    assert sorted(nodes(actions)) == [
        'hardware_activator', 'robot_state_publisher', 'ros2_control_node', 'spawner']
    assert spawner_arguments(actions) == [
        ['joint_state_publisher', '--inactive'], ['pilsbot_velocity_controller', '--inactive']]
    control_node = nodes(actions)['ros2_control_node']
    assert parameters(context, control_node)['hardware_components_initial_state.inactive'] == \
        ('PilsbotSystem',)
    config, overrides = parameter_files(context, control_node)
    assert config.endswith('pilsbot_control/config/acker_diff_controller.yaml')
    with open(overrides) as f:
        controller = yaml.safe_load(f)['pilsbot_velocity_controller']['ros__parameters']
    assert controller['wheel_separation'] == pytest.approx(0.62)


def test_real_ends_the_launch_when_the_controller_manager_exits():
    _, actions = expand('pilsbot.launch.py', hardware='real')

    [handler] = [a for a in actions if isinstance(a, RegisterEventHandler)]
    target = handler.event_handler._OnActionEventBase__action_matcher
    assert target is nodes(actions)['ros2_control_node']


def test_real_passes_hardware_real_to_xacro():
    context, actions = expand('pilsbot.launch.py', hardware='real')

    xacro = xacro_arguments(context, actions)
    assert xacro['hardware'] == 'real'
    assert 'controller_overrides' not in xacro


def test_real_selects_the_robots_driver_in_the_installed_description():
    # Until pilsbot_description knows `hardware`, use_fake_hardware defaults
    # to true there and would select Gazebo Classic's GazeboSystem.
    context, actions = expand('pilsbot.launch.py', hardware='real')

    urdf = robot_description(actions).perform(context)
    assert '<plugin>pilsbot_driver/PilsbotDriver</plugin>' in urdf
    assert 'GazeboSystem' not in urdf


def test_real_runs_on_wall_time_by_default():
    context, actions = expand('pilsbot.launch.py', hardware='real')

    for executable in ('robot_state_publisher', 'hardware_activator'):
        assert parameters(context, nodes(actions)[executable])['use_sim_time'] is False


# --- invalid values ----------------------------------------------------------

@pytest.mark.parametrize('launch_file', ['pilsbot.launch.py', 'pilsbot_teleop.launch.py'])
@pytest.mark.parametrize('value', ['', 'sim', 'Gazebo'])
def test_an_unknown_hardware_fails_the_launch(launch_file, value):
    with pytest.raises(RuntimeError, match='"hardware".*real.*gazebo'):
        expand(launch_file, hardware=value)


@pytest.mark.parametrize('launch_file',
                         ['pilsbot.launch.py', 'pilsbot_teleop.launch.py', 'teleop.launch.py'])
def test_an_unknown_use_sim_time_fails_the_launch(launch_file):
    with pytest.raises(RuntimeError, match='"use_sim_time"'):
        expand(launch_file, use_sim_time='yes')


# --- pass-through and teleop -------------------------------------------------

def test_pilsbot_teleop_passes_hardware_and_use_sim_time_to_both_includes():
    context, actions = expand('pilsbot_teleop.launch.py', hardware='gazebo', use_sim_time='true')

    includes = {}
    for include in actions:
        if isinstance(include, IncludeLaunchDescription):
            source = include.launch_description_source
            source.get_launch_description(context)  # expands the location
            includes[pathlib.Path(source.location).name] = {
                evaluate(context, key): evaluate(context, value)
                for key, value in include.launch_arguments}
    assert includes['pilsbot.launch.py']['hardware'] == 'gazebo'
    assert includes['pilsbot.launch.py']['use_sim_time'] == 'true'
    assert includes['teleop.launch.py']['use_sim_time'] == 'true'
    assert 'hardware' not in includes['teleop.launch.py']


@pytest.mark.parametrize('use_sim_time, expected', [('true', True), ('false', False)])
def test_teleop_nodes_get_use_sim_time(use_sim_time, expected):
    context, actions = expand('teleop.launch.py', use_sim_time=use_sim_time)

    for executable in ('joy_node', 'teleop_acker_node'):
        assert parameters(context, nodes(actions)[executable])['use_sim_time'] is expected
