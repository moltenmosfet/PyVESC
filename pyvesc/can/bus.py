"""
VescCanBus — python-can wrapper for the native VESC CAN protocol.

Owns the bus, a background RX thread (can.Notifier), and a live telemetry
snapshot per controller id fed by the VESCs' periodic STATUS broadcasts.
Telemetry is push-based on the wire (broadcast rates set in each VESC's app
config) — the host never requests it.

Raw-record support: ``attach_candump_logger(path)`` writes every frame on the
bus in candump format. With ``receive_own_messages=True`` (the default; needs
socketcan or the virtual interface) the log contains BOTH directions —
commands and telemetry — making it the immutable stimulus+response record of
a run.

SAFETY: nothing in this module is a safety device. ``stop_all()`` is a
convenience; the trusted stop is the hardwired e-stop/contactor chain.
"""

import threading
import time
from dataclasses import dataclass, field, replace
from typing import Callable, Dict, Iterable, List, Optional

import can

from . import frames
from .frames import (CanPacketId, Pong, Status1, Status2, Status3, Status4,
                     Status5, Status6, VescFrame)

# Identity used in PING payloads so PONGs route back to us. Any id no VESC on
# the bus uses; 254 is conventional for a non-VESC host.
HOST_ID = 0xFE


@dataclass(frozen=True)
class NodeTelemetry:
    """Immutable snapshot of the last-seen STATUS frames from one node.

    Stamps are time.monotonic() at decode. ``None`` fields = never seen (check
    before use; a VESC only broadcasts the STATUS messages enabled in its app
    config).
    """
    controller_id: int
    status1: Optional[Status1] = None
    status2: Optional[Status2] = None
    status3: Optional[Status3] = None
    status4: Optional[Status4] = None
    status5: Optional[Status5] = None
    status6: Optional[Status6] = None
    stamps: Dict[str, float] = field(default_factory=dict)

    def age(self, name: str = 'status1',
            now: Optional[float] = None) -> Optional[float]:
        """Seconds since the named status was last seen; None if never."""
        stamp = self.stamps.get(name)
        if stamp is None:
            return None
        return (time.monotonic() if now is None else now) - stamp


class VescCanBus:
    """The one object that owns the CAN socket.

    Use as a context manager. Construct with an interface spec, or inject an
    existing ``can.BusABC`` (tests use the ``virtual`` interface this way).
    """

    def __init__(self, channel: str = 'can0', interface: str = 'socketcan',
                 bus: Optional[can.BusABC] = None,
                 receive_own_messages: bool = True, **bus_kwargs):
        if bus is not None:
            self._bus = bus
        else:
            self._bus = can.Bus(channel=channel, interface=interface,
                                receive_own_messages=receive_own_messages,
                                **bus_kwargs)
        self._lock = threading.Lock()
        self._telemetry: Dict[int, NodeTelemetry] = {}
        self._subscribers: List[Callable[[int, object], None]] = []
        self._pong_event = threading.Event()
        self._pongs: Dict[int, int] = {}  # responder_id -> hw_type
        self._loggers: List[can.Listener] = []
        self._notifier = can.Notifier(self._bus, [self._on_message])

    # --- context manager / lifecycle -----------------------------------------

    def __enter__(self) -> 'VescCanBus':
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        self._notifier.stop()
        for logger in self._loggers:
            logger.stop()
        self._bus.shutdown()

    # --- RX path ---------------------------------------------------------------

    def _on_message(self, msg: can.Message) -> None:
        if not msg.is_extended_id:
            return
        decoded = frames.decode_frame(msg.arbitration_id, bytes(msg.data))
        if decoded is None:
            return
        controller_id, obj = decoded

        if isinstance(obj, Pong):
            if controller_id == HOST_ID:  # addressed to us
                with self._lock:
                    self._pongs[obj.controller_id] = obj.hw_type
                self._pong_event.set()
            return

        slot = _STATUS_SLOTS[type(obj)]
        now = time.monotonic()
        with self._lock:
            telem = self._telemetry.get(controller_id) \
                or NodeTelemetry(controller_id=controller_id)
            stamps = dict(telem.stamps)
            stamps[slot] = now
            self._telemetry[controller_id] = replace(
                telem, **{slot: obj, 'stamps': stamps})

        for fn in self._subscribers:
            fn(controller_id, obj)

    def telemetry(self, controller_id: int) -> NodeTelemetry:
        """Latest snapshot for a node (empty snapshot if never heard from)."""
        with self._lock:
            return self._telemetry.get(controller_id) \
                or NodeTelemetry(controller_id=controller_id)

    def known_nodes(self) -> List[int]:
        with self._lock:
            return sorted(self._telemetry)

    def subscribe(self, fn: Callable[[int, object], None]) -> None:
        """Register a callback fired on every decoded status:
        ``fn(controller_id, status_obj)``. Runs on the RX thread — keep it fast
        and never block in it."""
        self._subscribers.append(fn)

    # --- TX path ---------------------------------------------------------------

    def send(self, frame: VescFrame, timeout: float = 0.1) -> None:
        self._bus.send(can.Message(arbitration_id=frame.arbitration_id,
                                   data=frame.data, is_extended_id=True),
                       timeout=timeout)

    def stop_all(self) -> None:
        """Broadcast SET_CURRENT 0 to every node. A software convenience —
        NOT the safety path."""
        self.send(frames.encode_set_current(frames.BROADCAST_ID, 0.0))

    # --- enumeration -------------------------------------------------------------

    def ping(self, controller_id: int, timeout: float = 0.2) -> Optional[int]:
        """PING one node. Returns its hw_type (0 = VESC) or None on timeout."""
        with self._lock:
            self._pongs.pop(controller_id, None)
        deadline = time.monotonic() + timeout
        self.send(frames.encode_ping(controller_id, HOST_ID))
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._pong_event.clear()
            with self._lock:
                if controller_id in self._pongs:
                    return self._pongs[controller_id]
            self._pong_event.wait(remaining)
            with self._lock:
                if controller_id in self._pongs:
                    return self._pongs[controller_id]

    def scan(self, ids: Iterable[int] = range(0, 254),
             timeout: float = 0.05) -> Dict[int, int]:
        """Enumerate the bus: {controller_id: hw_type} for nodes that answer."""
        found = {}
        for cid in ids:
            hw = self.ping(cid, timeout=timeout)
            if hw is not None:
                found[cid] = hw
        return found

    # --- raw record ---------------------------------------------------------------

    def attach_candump_logger(self, path: str) -> can.Listener:
        """Log every bus frame to ``path`` in candump format (both directions
        when the interface supports receive_own_messages). This file is the
        immutable stimulus+response record of a run."""
        logger = can.CanutilsLogWriter(path)
        self._loggers.append(logger)
        self._notifier.add_listener(logger)
        return logger

    def detach_logger(self, logger: can.Listener) -> None:
        self._notifier.remove_listener(logger)
        if logger in self._loggers:
            self._loggers.remove(logger)
        logger.stop()


_STATUS_SLOTS = {
    Status1: 'status1',
    Status2: 'status2',
    Status3: 'status3',
    Status4: 'status4',
    Status5: 'status5',
    Status6: 'status6',
}
