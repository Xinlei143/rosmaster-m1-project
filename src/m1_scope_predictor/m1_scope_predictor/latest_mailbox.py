"""A blocking, one-element mailbox that always retains the latest item."""

import threading


class LatestMailbox:
    def __init__(self):
        self._condition = threading.Condition()
        self._item = None
        self._closed = False
        self.dropped = 0

    def put(self, item):
        with self._condition:
            if self._closed:
                return False
            replaced = self._item is not None
            if replaced:
                self.dropped += 1
            self._item = item
            self._condition.notify()
            return replaced

    def get(self):
        with self._condition:
            while self._item is None and not self._closed:
                self._condition.wait()
            if self._item is None:
                return None
            item = self._item
            self._item = None
            return item

    def take(self):
        """Return the current item immediately, or None when empty."""
        with self._condition:
            item = self._item
            self._item = None
            return item

    def close(self):
        with self._condition:
            self._closed = True
            self._item = None
            self._condition.notify_all()
