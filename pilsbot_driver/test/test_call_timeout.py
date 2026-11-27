# The hardware activator's service calls give up after a deadline, so a
# controller manager that dies or hangs mid-call cannot block it forever.
import time

from controller_manager_msgs.srv import ListHardwareComponents
import pytest
import rclpy
import rclpy.client
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.task import Future

from pilsbot_hardware_activator.call import call
from pilsbot_hardware_activator.call import CallTimeout
from pilsbot_hardware_activator.node import HardwareActivator

TIMEOUT = 0.5


@pytest.fixture
def context():
    context = rclpy.Context()
    rclpy.init(context=context)
    yield context
    rclpy.shutdown(context=context)


@pytest.fixture
def executor(context):
    executor = SingleThreadedExecutor(context=context)
    yield executor
    executor.shutdown()


def serve_list_hardware(context, name):
    """
    Offer ListHardwareComponents under name.

    Requests are answered only while the returned node is spun; otherwise they
    arrive but are never answered, as with a dead or hanging controller manager.
    Keep a reference to it: a collected node takes its service along.
    """
    server = rclpy.create_node('fake_controller_manager', context=context)
    server.create_service(ListHardwareComponents, name, lambda _, response: response)
    return server


def run(executor, coroutine_function, timeout_sec=5.0):
    task = executor.create_task(coroutine_function)
    start = time.monotonic()
    executor.spin_until_future_complete(task, timeout_sec=timeout_sec)
    assert task.done(), 'still waiting'
    return task, time.monotonic() - start


def test_returns_the_response_of_a_service_that_answers(context, executor):
    server = serve_list_hardware(context, '/answering/list')
    node = rclpy.create_node('caller', context=context)
    group = ReentrantCallbackGroup()
    client = node.create_client(ListHardwareComponents, '/answering/list', callback_group=group)
    executor.add_node(server)
    executor.add_node(node)
    assert client.wait_for_service(timeout_sec=5.0)

    async def call_it():
        return await call(node, client, ListHardwareComponents.Request(), TIMEOUT, group)

    task, _ = run(executor, call_it)
    assert isinstance(task.result(), ListHardwareComponents.Response)
    assert not list(node.timers), 'the deadline timer is left behind'


def test_returns_none_while_the_service_is_not_available(context, executor):
    node = rclpy.create_node('caller', context=context)
    group = ReentrantCallbackGroup()
    client = node.create_client(ListHardwareComponents, '/nobody/list', callback_group=group)
    executor.add_node(node)

    async def call_it():
        return await call(node, client, ListHardwareComponents.Request(), TIMEOUT, group)

    task, _ = run(executor, call_it)
    assert task.result() is None


def test_gives_up_on_a_service_that_never_answers(context, executor):
    hung = serve_list_hardware(context, '/hung/list')
    node = rclpy.create_node('caller', context=context)
    group = ReentrantCallbackGroup()
    client = node.create_client(ListHardwareComponents, '/hung/list', callback_group=group)
    executor.add_node(node)
    assert client.wait_for_service(timeout_sec=5.0)

    async def call_it():
        try:
            await call(node, client, ListHardwareComponents.Request(), TIMEOUT, group)
        except CallTimeout as e:
            return e

    task, elapsed = run(executor, call_it)
    assert isinstance(task.result(), CallTimeout)
    assert task.result().service == '/hung/list'
    assert TIMEOUT <= elapsed < TIMEOUT + 2.0
    # A late response would be dropped.
    assert not client._pending_requests
    assert not list(node.timers)
    hung.destroy_node()


class PreHumbleFixFuture(Future):
    """
    A Future as in Humble's rclpy before 3.3.16: cancel() does not complete it.

    There, cancel() only flagged the future and never set it done, so a
    coroutine awaiting it was never resumed.
    """

    def cancel(self):
        self.cancel_requested = True


def test_gives_up_even_where_cancel_does_not_complete_the_future(
        context, executor, monkeypatch):
    monkeypatch.setattr(rclpy.client, 'Future', PreHumbleFixFuture)
    hung = serve_list_hardware(context, '/hung_old/list')
    node = rclpy.create_node('caller', context=context)
    group = ReentrantCallbackGroup()
    client = node.create_client(ListHardwareComponents, '/hung_old/list', callback_group=group)
    executor.add_node(node)
    assert client.wait_for_service(timeout_sec=5.0)

    async def call_it():
        try:
            await call(node, client, ListHardwareComponents.Request(), TIMEOUT, group)
        except CallTimeout as e:
            return e

    task, elapsed = run(executor, call_it)
    assert isinstance(task.result(), CallTimeout)
    assert TIMEOUT <= elapsed < TIMEOUT + 2.0
    assert not client._pending_requests
    assert not list(node.timers)
    hung.destroy_node()


def test_activator_keeps_going_after_the_controller_manager_hangs_mid_round(context, executor):
    hung = serve_list_hardware(context, '/hung_cm/list_hardware_components')
    activator = HardwareActivator(context=context, parameter_overrides=[
        Parameter('controller_manager', value='/hung_cm'),
        Parameter('call_timeout', value=TIMEOUT),
        # Rounds are driven by the test.
        Parameter('period', value=1000.0),
    ])
    executor.add_node(activator)
    assert activator.list_hardware.wait_for_service(timeout_sec=5.0)

    for _ in range(2):
        _, elapsed = run(executor, activator.round)
        assert not activator.busy
        # The round really called (and waited for) the controller manager again,
        # instead of returning at once because it believed itself still busy.
        assert elapsed >= TIMEOUT
    hung.destroy_node()
