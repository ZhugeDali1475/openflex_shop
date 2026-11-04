"""Awaitable helpers that are driven by the rclpy executor, not asyncio."""

from rclpy.task import Future


async def ros_sleep(node, delay_sec: float) -> None:
    """Yield an action coroutine until a one-shot ROS timer expires.

    ROS 2 Humble's rclpy executor drives coroutine callbacks without creating a
    standard asyncio event loop, so ``asyncio.sleep`` cannot be used here.
    """

    if delay_sec <= 0.0:
        return
    future = Future()

    def complete() -> None:
        if not future.done():
            future.set_result(True)

    timer = node.create_timer(delay_sec, complete)
    try:
        await future
    finally:
        node.destroy_timer(timer)
