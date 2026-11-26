"""
Decide what the hardware activator does in one round.

The controller manager starts the Pilsbot hardware inactive
(hardware_components_initial_state) and the spawners load the controllers
inactive, so teleop, GNSS, lighting, Foxglove and recording keep running while
the hoverboard or Head MCU is missing. The activator activates the hardware as
soon as it can, then the controllers, and does so again whenever the hardware
drops out.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Plan:
    activate_hardware: bool = False
    configure_controllers: tuple = ()
    activate_controllers: tuple = ()
    give_up: bool = False


def plan(hardware_state, controller_states, controllers):
    """
    Return the Plan for this round.

    hardware_state is the lifecycle label of the hardware component, or None
    while the controller manager doesn't list it. controller_states maps loaded
    controller names to their state; controllers is the ordered list to run.
    """
    if hardware_state is None:
        return Plan()
    if hardware_state == 'finalized':
        return Plan(give_up=True)
    if hardware_state != 'active':
        return Plan(activate_hardware=True)

    loaded = [c for c in controllers if c in controller_states]
    return Plan(
        configure_controllers=tuple(c for c in loaded if controller_states[c] == 'unconfigured'),
        activate_controllers=tuple(c for c in loaded if controller_states[c] != 'active'),
    )
