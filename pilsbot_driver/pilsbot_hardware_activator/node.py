"""
Keep the Pilsbot hardware and its controllers active.

Every `period` seconds: read the hardware and controller states from the
controller manager, then act on the Plan (see plan.py). One round runs at a
time; activating the hardware blocks for a few seconds while the driver
retries its serial ports. A call the controller manager doesn't answer within
`call_timeout` seconds ends the round, so the next one starts afresh.
"""
import sys

from controller_manager_msgs.srv import ConfigureController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareComponents
from controller_manager_msgs.srv import SetHardwareComponentState
from controller_manager_msgs.srv import SwitchController
from lifecycle_msgs.msg import State
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pilsbot_hardware_activator.call import call
from pilsbot_hardware_activator.call import CallTimeout
from pilsbot_hardware_activator.plan import plan

# Log a still-failing activation about once a minute, not every round.
_FAILURE_LOG_EVERY = 12


class HardwareActivator(Node):

    def __init__(self, **kwargs):
        super().__init__('hardware_activator', **kwargs)
        self.hardware = self.declare_parameter('hardware_component', 'PilsbotSystem').value
        self.controllers = tuple(self.declare_parameter(
            'controllers', ['joint_state_publisher', 'pilsbot_velocity_controller']).value)
        period = self.declare_parameter('period', 5.0).value
        cm = self.declare_parameter('controller_manager', '/controller_manager').value
        # Activating the hardware takes a few seconds (serial retries); no answer
        # within this time means the controller manager died or hangs.
        self.call_timeout = self.declare_parameter('call_timeout', 30.0).value

        # The timer's coroutine awaits these clients (and their deadline timers);
        # in the node's default (mutually exclusive) group their responses could
        # never be processed.
        self.clients_group = clients = ReentrantCallbackGroup()
        self.list_hardware = self.create_client(
            ListHardwareComponents, f'{cm}/list_hardware_components', callback_group=clients)
        self.set_hardware_state = self.create_client(
            SetHardwareComponentState, f'{cm}/set_hardware_component_state',
            callback_group=clients)
        self.list_controllers = self.create_client(
            ListControllers, f'{cm}/list_controllers', callback_group=clients)
        self.configure_controller = self.create_client(
            ConfigureController, f'{cm}/configure_controller', callback_group=clients)
        self.switch_controller = self.create_client(
            SwitchController, f'{cm}/switch_controller', callback_group=clients)

        self.busy = False
        self.last_hardware_state = None
        self.failed_attempts = 0
        self.gave_up = False
        self.timer = self.create_timer(period, self.round)
        self.get_logger().info(
            f'Keeping {self.hardware} and {", ".join(self.controllers)} active '
            f'(checking every {period:g} s)')

    async def round(self):
        if self.busy or self.gave_up:
            return
        self.busy = True
        try:
            await self._round()
        except CallTimeout as e:
            self.get_logger().warn(f'{e} (controller manager dead or hanging?); retrying')
        finally:
            self.busy = False

    async def _call(self, client, request):
        return await call(self, client, request, self.call_timeout, self.clients_group)

    async def _round(self):
        hardware = await self._call(self.list_hardware, ListHardwareComponents.Request())
        controllers = await self._call(self.list_controllers, ListControllers.Request())
        if hardware is None or controllers is None:
            return
        hardware_state = next(
            (c.state.label for c in hardware.component if c.name == self.hardware), None)
        controller_states = {c.name: c.state for c in controllers.controller}
        self._log_hardware_state(hardware_state)

        p = plan(hardware_state, controller_states, self.controllers)
        if p.give_up:
            self.gave_up = True
            self.get_logger().error(
                f'{self.hardware} is finalized and cannot be activated again; '
                'restart the controller manager.')
            return
        if p.activate_hardware:
            await self._activate_hardware()
            return
        for name in p.configure_controllers:
            await self._call(self.configure_controller, ConfigureController.Request(name=name))
        if p.activate_controllers:
            request = SwitchController.Request(
                activate_controllers=list(p.activate_controllers),
                strictness=SwitchController.Request.BEST_EFFORT, activate_asap=True)
            response = await self._call(self.switch_controller, request)
            if response is not None and response.ok:
                self.get_logger().info(f'Activated {", ".join(p.activate_controllers)}')
            else:
                self.get_logger().warn(
                    f'Could not activate {", ".join(p.activate_controllers)}; retrying')

    async def _activate_hardware(self):
        request = SetHardwareComponentState.Request(
            name=self.hardware,
            target_state=State(id=State.PRIMARY_STATE_ACTIVE, label='active'))
        response = await self._call(self.set_hardware_state, request)
        if response is not None and response.ok:
            self.get_logger().info(
                f'{self.hardware} is active after {self.failed_attempts + 1} attempt(s)')
            self.failed_attempts = 0
            # So a later drop-out is logged as a state change.
            self.last_hardware_state = 'active'
            return
        if self.failed_attempts % _FAILURE_LOG_EVERY == 0:
            self.get_logger().warn(
                f'Could not activate {self.hardware} (hoverboard or Head MCU not reachable?); '
                'see the controller manager log. Retrying.')
        self.failed_attempts += 1

    def _log_hardware_state(self, state):
        if state == self.last_hardware_state:
            return
        if state is None:
            self.get_logger().info(f'Waiting for the controller manager to list {self.hardware}')
        elif state != 'active':
            self.get_logger().warn(f'{self.hardware} is {state}; activating it')
        self.last_hardware_state = state


def main(args=None):
    rclpy.init(args=args)
    node = HardwareActivator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
