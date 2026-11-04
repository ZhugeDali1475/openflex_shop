from dataclasses import dataclass
import threading
import time
from typing import Generic, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class CacheEntry(Generic[T]):
    value: T
    expires_at: float


class TtlCache(Generic[T]):
    """Small thread-safe TTL primitive; detection ownership is added in P5-09."""

    def __init__(self, *, clock=time.monotonic):
        self._clock = clock
        self._values: dict[str, CacheEntry[T]] = {}
        self._lock = threading.Lock()

    def put(self, key: str, value: T, ttl_s: float) -> None:
        if not key:
            raise ValueError("cache key is required")
        if ttl_s <= 0.0:
            raise ValueError("cache ttl_s must be positive")
        with self._lock:
            self._values[key] = CacheEntry(value, self._clock() + float(ttl_s))

    def get(self, key: str) -> T | None:
        with self._lock:
            entry = self._values.get(key)
            if entry is None:
                return None
            if entry.expires_at <= self._clock():
                self._values.pop(key, None)
                return None
            return entry.value

    def pop(self, key: str) -> T | None:
        with self._lock:
            entry = self._values.pop(key, None)
            if entry is None or entry.expires_at <= self._clock():
                return None
            return entry.value

    def purge_expired(self) -> int:
        now = self._clock()
        with self._lock:
            expired = [key for key, entry in self._values.items() if entry.expires_at <= now]
            for key in expired:
                self._values.pop(key, None)
        return len(expired)
