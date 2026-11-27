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
        if not future.done():
            client.remove_pending_request(future)
            # Resumes the awaiting coroutine; the future yields no result.
            future.cancel()

    timer = node.create_timer(timeout_sec, give_up, callback_group=callback_group)
    try:
        response = await future
    finally:
        node.destroy_timer(timer)
    if future.cancelled():
        raise CallTimeout(client.srv_name, timeout_sec)
    return response
