# What the hardware activator does each round (every 5 s), given the state of
# the hardware component and the controllers in the controller manager.
# It keeps retrying: at startup and whenever the hardware drops out.
from pilsbot_hardware_activator.plan import Plan
from pilsbot_hardware_activator.plan import plan

CONTROLLERS = ('joint_state_publisher', 'pilsbot_velocity_controller')
BOTH_INACTIVE = {'joint_state_publisher': 'inactive', 'pilsbot_velocity_controller': 'inactive'}
BOTH_ACTIVE = {'joint_state_publisher': 'active', 'pilsbot_velocity_controller': 'active'}


def test_waits_while_the_controller_manager_does_not_list_the_hardware():
    assert plan(None, {}, CONTROLLERS) == Plan()


def test_activates_hardware_that_starts_inactive():
    assert plan('inactive', BOTH_INACTIVE, CONTROLLERS) == Plan(activate_hardware=True)


def test_activates_hardware_that_fell_back_to_unconfigured():
    # A failed activation or a read error leaves the hardware unconfigured.
    assert plan('unconfigured', BOTH_INACTIVE, CONTROLLERS) == Plan(activate_hardware=True)


def test_activates_the_controllers_once_the_hardware_is_active():
    assert plan('active', BOTH_INACTIVE, CONTROLLERS) == Plan(
        activate_controllers=CONTROLLERS)


def test_activates_controllers_that_are_loaded_and_leaves_the_rest_to_the_spawner():
    states = {'joint_state_publisher': 'inactive'}
    assert plan('active', states, CONTROLLERS) == Plan(
        activate_controllers=('joint_state_publisher',))


def test_configures_controllers_left_unconfigured_by_a_failed_activation():
    states = {'joint_state_publisher': 'unconfigured', 'pilsbot_velocity_controller': 'inactive'}
    assert plan('active', states, CONTROLLERS) == Plan(
        configure_controllers=('joint_state_publisher',), activate_controllers=CONTROLLERS)


def test_does_nothing_when_everything_is_active():
    assert plan('active', BOTH_ACTIVE, CONTROLLERS) == Plan()


def test_re_activates_after_the_hardware_dropped_out_while_driving():
    # The controller manager deactivated the controllers with the hardware.
    assert plan('unconfigured', BOTH_INACTIVE, CONTROLLERS) == Plan(activate_hardware=True)
    assert plan('active', BOTH_INACTIVE, CONTROLLERS) == Plan(activate_controllers=CONTROLLERS)


def test_gives_up_on_finalized_hardware():
    # Finalized cannot be left; only a restart of the controller manager helps.
    assert plan('finalized', BOTH_INACTIVE, CONTROLLERS) == Plan(give_up=True)
