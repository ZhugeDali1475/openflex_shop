import time


class TelemetryUnavailable(RuntimeError):
    pass


def wait_for_fresh_stop(stationary, timeout):
    deadline = time.monotonic() + timeout
    since = None
    while time.monotonic() < deadline:
        try:
            confirmed = stationary()
        except TelemetryUnavailable:
            confirmed = False
        now = time.monotonic()
        if now >= deadline:
            return False
        if confirmed:
            if since is None:
                since = now
            if now - since >= 0.5:
                return True
        else:
            since = None
        time.sleep(0.02)
    return False
