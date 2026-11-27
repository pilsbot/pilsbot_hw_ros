"""
Service calls that give up after a deadline.

If the controller manager dies or restarts while a request is pending, its
response never comes; awaiting call_async() alone would block the activator
forever.
"""


class CallTimeout(Exception):
    """The service did not answer within the deadline."""

    def __init__(self, service, timeout_sec):
        super().__init__(f'{service} did not answer within {timeout_sec:g} s')
        self.service = service
        self.timeout_sec = timeout_sec


async def call(node, client, request, timeout_sec, callback_group):
    """
    Call the service and return its response, or None if it isn't available.

    Raises CallTimeout if the response does not arrive within timeout_sec; the
    request is then dropped, so a late response is ignored. The deadline timer
    runs in callback_group, which must not be the group of the awaiting
    callback (it is busy while the caller awaits).
    """
    if not client.service_is_ready():
        return None
    future = client.call_async(request)

    def give_up():
        # Drop the request first, so a response arriving from now on is ignored.
        client.remove_pending_request(future)
        if not future.done():
            # Completing the future resumes the awaiting coroutine, which then
            # raises CallTimeout. Not future.cancel(): in Humble's rclpy before
            # 3.3.16 it does not mark the future done, so the await would
            # never return.
            future.set_exception(CallTimeout(client.srv_name, timeout_sec))

    timer = node.create_timer(timeout_sec, give_up, callback_group=callback_group)
    try:
        return await future
    finally:
        node.destroy_timer(timer)
