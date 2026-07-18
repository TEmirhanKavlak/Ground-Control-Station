"""
event_bus.py
────────────
Thread-safe publish/subscribe sistemi.

MAVLink thread → publish() → queue
GUI thread      → poll()   → queue'dan alır → callback çağırır

Tkinter mainloop'u içinde her 50ms'de bir poll() çağrılır.
"""

import queue
import threading
import logging
import time
from collections import defaultdict
from typing import Callable, Any

log = logging.getLogger("eventbus")


class EventBus:
    """
    Basit, thread-safe event bus.
    Subscriber'lar SADECE GUI thread'inden eklenmeli.
    publish() herhangi bir thread'den çağrılabilir.
    """

    def __init__(self):
        self._queue: queue.Queue = queue.Queue()
        self._subscribers: dict[str, list[Callable]] = defaultdict(list)
        self._lock = threading.Lock()

    # ──────────────────────────────────────
    #  Producer tarafı (herhangi thread)
    # ──────────────────────────────────────

    def publish(self, event: str, data: Any = None) -> None:
        """Thread-safe event yayınla."""
        self._queue.put_nowait((event, data))

    # ──────────────────────────────────────
    #  Consumer tarafı (GUI thread)
    # ──────────────────────────────────────

    def subscribe(self, event: str, callback: Callable) -> None:
        """Bir event'e abone ol. GUI thread'inden çağrılmalı."""
        with self._lock:
            self._subscribers[event].append(callback)

    def unsubscribe(self, event: str, callback: Callable) -> None:
        with self._lock:
            try:
                self._subscribers[event].remove(callback)
            except ValueError:
                pass

    def poll(self, max_events: int = 100, max_ms: float = 8.0) -> None:
        """
        Kuyruktaki event'leri işle.
        Tkinter after() döngüsüyle her ~50ms çağrılmalı.
        """
        processed = 0
        deadline = time.perf_counter() + (max_ms / 1000.0)
        while processed < max_events:
            if processed and time.perf_counter() >= deadline:
                break
            try:
                event, data = self._queue.get_nowait()
            except queue.Empty:
                break

            with self._lock:
                handlers = list(self._subscribers.get(event, []))

            for cb in handlers:
                try:
                    cb(data)
                except Exception as e:
                    log.error(f"EventBus callback hatası [{event}]: {e}", exc_info=True)

            processed += 1

    def clear(self) -> None:
        """Kuyruğu temizle."""
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
