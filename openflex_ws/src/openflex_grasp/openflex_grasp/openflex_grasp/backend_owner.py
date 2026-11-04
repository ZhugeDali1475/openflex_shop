"""Process-level ownership guard for real grasp backend nodes."""

from __future__ import annotations

import fcntl
import os
from pathlib import Path


class BackendOwner:
    """Hold a non-blocking advisory lock for one ROS backend owner."""

    def __init__(self, name: str, lock_dir: str | None = None):
        directory = Path(lock_dir or os.environ.get("OPENFLEX_BACKEND_LOCK_DIR", "/tmp"))
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f"openflex_{name}.lock"
        self._file = self.path.open("a+")
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self._file.close()
            raise RuntimeError(
                f"backend owner {name!r} is already held; stop the other grasp launch first"
            ) from error
        self._file.seek(0)
        self._file.truncate()
        self._file.write(f"pid={os.getpid()}\n")
        self._file.flush()

    def close(self) -> None:
        if self._file is None:
            return
        fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
        self._file.close()
        self._file = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
