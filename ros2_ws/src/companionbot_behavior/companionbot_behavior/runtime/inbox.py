"""Bounded, synchronous ingress from ROS/console/audio into one session owner."""
from collections import deque
from dataclasses import dataclass
from queue import Empty
import threading


@dataclass(frozen=True)
class TextInput:
    text: str
    input_id: str
    provenance: dict
    expires_at_s: float


class SessionInbox:
    def __init__(self, is_control, capacity=64):
        self.capacity, self.is_control = capacity, is_control
        self._items = deque()
        self._lock = threading.Lock()
        self._closed = False
        self.accepted = self.rejected = self.superseded = 0

    def put(self, item):
        with self._lock:
            if self._closed:
                self.rejected += 1
                return False
            if self.is_control(item):
                self.superseded += len(self._items)
                self._items.clear()
            elif len(self._items) >= self.capacity:
                self.rejected += 1
                return False
            self._items.append(item)
            self.accepted += 1
            return True

    def get_nowait(self):
        with self._lock:
            if not self._items:
                raise Empty
            return self._items.popleft()

    def close(self):
        with self._lock:
            self._closed = True
            self.superseded += len(self._items)
            self._items.clear()

    def snapshot(self):
        with self._lock:
            return {"capacity": self.capacity, "pending": len(self._items),
                    "accepted": self.accepted, "rejected": self.rejected,
                    "superseded": self.superseded, "closed": self._closed}
