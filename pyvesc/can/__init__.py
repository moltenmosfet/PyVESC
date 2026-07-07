"""
Native VESC CAN protocol — the dyno runtime link.

This subpackage speaks VESC's native CAN protocol (single 8-byte frames,
extended 29-bit IDs) directly over SocketCAN. It is entirely separate from the
COMM packet protocol in the rest of pyvesc: no framing, no CRC, no shared code.
The COMM path (serial/TCP) remains the bench-setup tool; this is what a control
loop runs on.

Layers:
- ``frames``  — pure codec: command/CONF_* encoders, STATUS_1..6 + PONG
                decoders. No I/O; usable and testable everywhere.
- ``bus``     — ``VescCanBus``: python-can wrapper with a background RX thread
                maintaining a live, thread-safe telemetry snapshot per
                controller id, plus candump logging and PING enumeration.
                Requires python-can (``pip install pyvesc[can]``).
- ``node``    — ``VescCanNode``: typed façade for one controller id.

All wire formats verified against bldc firmware 7.00 source
(comm/comm_can.c, datatypes.h); see test_can_frames.py for the drift guard.
"""

from .frames import (  # noqa: F401
    CanPacketId,
    VescFrame,
    Status1, Status2, Status3, Status4, Status5, Status6, Pong,
    make_arbitration_id, split_arbitration_id, decode_frame,
    encode_set_duty, encode_set_current, encode_set_current_brake,
    encode_set_rpm, encode_set_pos, encode_set_current_rel,
    encode_set_current_brake_rel, encode_set_handbrake,
    encode_set_handbrake_rel, encode_conf_current_limits,
    encode_conf_current_limits_in, encode_conf_foc_erpms,
    encode_conf_battery_cut, encode_ping, encode_shutdown,
    BROADCAST_ID,
)

# bus/node need python-can; import lazily so `pyvesc.can.frames` stays
# dependency-free (e.g. for offline log decoding).
try:
    from .bus import VescCanBus, NodeTelemetry  # noqa: F401
    from .node import VescCanNode  # noqa: F401
except ImportError:  # pragma: no cover - python-can not installed
    pass
