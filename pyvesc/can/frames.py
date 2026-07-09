"""
Pure codec for VESC's native CAN protocol. No I/O — encoders return
(arbitration_id, data) pairs, decoders take them.

Wire format (verified against bldc fw 7.00, comm/comm_can.c + datatypes.h):

- Extended 29-bit arbitration id: ``(packet_id << 8) | controller_id``.
- controller_id 255 = broadcast (every VESC on the bus accepts the command).
- All multi-byte fields are BIG-endian.
- "float32" on the wire = int32 of ``value * scale``; "float16" = int16 of
  ``value * scale`` (a scaled integer, NOT IEEE half precision).

Two traps this module encodes correctly (both from the fw source; neither is
in the VESC documentation):

1. The optional off-delay field sits at OPPOSITE ends of the payload:
   ``SET_CURRENT``           -> [off_delay f16×1e3][current f32×1e3]  (prepended)
   ``SET_CURRENT_REL``       -> [current_rel f32×1e5][off_delay f16×1e3] (appended)
   The off-delay is a per-command watchdog: the VESC auto-zeroes the command
   after that many seconds unless it is refreshed.

2. ``CONF_FOC_ERPMS`` sets ``foc_openloop_rpm`` / ``foc_sl_erpm`` (FOC
   sensorless thresholds) — it is NOT a motor speed limit. There is no CAN
   packet for ``l_min_erpm``/``l_max_erpm``; a runtime speed envelope must be
   enforced by the host (or configured over COMM at setup time).
"""

import struct
from enum import IntEnum
from typing import List, NamedTuple, Optional, Tuple, Union

BROADCAST_ID = 0xFF

_INT16_MIN, _INT16_MAX = -0x8000, 0x7FFF
_INT32_MIN, _INT32_MAX = -0x80000000, 0x7FFFFFFF


class CanPacketId(IntEnum):
    """CAN_PACKET_ID from bldc/datatypes.h (fw 7.00)."""
    SET_DUTY = 0
    SET_CURRENT = 1
    SET_CURRENT_BRAKE = 2
    SET_RPM = 3
    SET_POS = 4
    FILL_RX_BUFFER = 5
    FILL_RX_BUFFER_LONG = 6
    PROCESS_RX_BUFFER = 7
    PROCESS_SHORT_BUFFER = 8
    STATUS = 9
    SET_CURRENT_REL = 10
    SET_CURRENT_BRAKE_REL = 11
    SET_CURRENT_HANDBRAKE = 12
    SET_CURRENT_HANDBRAKE_REL = 13
    STATUS_2 = 14
    STATUS_3 = 15
    STATUS_4 = 16
    PING = 17
    PONG = 18
    DETECT_APPLY_ALL_FOC = 19
    DETECT_APPLY_ALL_FOC_RES = 20
    CONF_CURRENT_LIMITS = 21
    CONF_STORE_CURRENT_LIMITS = 22
    CONF_CURRENT_LIMITS_IN = 23
    CONF_STORE_CURRENT_LIMITS_IN = 24
    CONF_FOC_ERPMS = 25
    CONF_STORE_FOC_ERPMS = 26
    STATUS_5 = 27
    POLL_TS5700N8501_STATUS = 28
    CONF_BATTERY_CUT = 29
    CONF_STORE_BATTERY_CUT = 30
    SHUTDOWN = 31
    STATUS_6 = 58
    # Molten MOSFET private block (200-209) — moltenmosfet/vesc_firmware fork
    # only; chosen far above upstream's allocation frontier.
    MM_SET_ID_DISSIPATE = 200
    MM_STATUS_DISSIPATION = 201


class VescFrame(NamedTuple):
    """One encoded CAN frame: pass to the bus as an extended-id message."""
    arbitration_id: int
    data: bytes


def make_arbitration_id(packet_id: int, controller_id: int) -> int:
    if not 0 <= controller_id <= 0xFF:
        raise ValueError("controller_id must be 0..255, got %r" % controller_id)
    return (int(packet_id) << 8) | controller_id


def split_arbitration_id(arbitration_id: int) -> Tuple[int, int]:
    """-> (packet_id, controller_id). packet_id is int (may be unknown to the enum)."""
    return arbitration_id >> 8, arbitration_id & 0xFF


# --- scaled-integer packing (the VESC "float32"/"float16") -------------------

def _scaled32(value: float, scale: float, what: str) -> bytes:
    raw = int(round(value * scale))
    if not _INT32_MIN <= raw <= _INT32_MAX:
        raise ValueError("%s=%r overflows int32 at scale %g" % (what, value, scale))
    return struct.pack('!i', raw)


def _scaled16(value: float, scale: float, what: str) -> bytes:
    raw = int(round(value * scale))
    if not _INT16_MIN <= raw <= _INT16_MAX:
        raise ValueError("%s=%r overflows int16 at scale %g" % (what, value, scale))
    return struct.pack('!h', raw)


def _off_delay_bytes(off_delay: float) -> bytes:
    # int16 at ×1e3 caps the wire format at 32.767 s
    if not 0.0 <= off_delay <= 32.767:
        raise ValueError("off_delay must be 0..32.767 s, got %r" % off_delay)
    return _scaled16(off_delay, 1e3, "off_delay")


def _check_rel(rel: float, what: str) -> float:
    if not -1.0 <= rel <= 1.0:
        raise ValueError("%s must be in [-1, 1], got %r" % (what, rel))
    return rel


# --- command encoders ---------------------------------------------------------

def encode_set_duty(controller_id: int, duty: float) -> VescFrame:
    """Duty-cycle command, fraction in [-1, 1]. Useless for dyno control
    (uncontrolled torque) — present for completeness/bench poking."""
    _check_rel(duty, "duty")
    return VescFrame(make_arbitration_id(CanPacketId.SET_DUTY, controller_id),
                     _scaled32(duty, 1e5, "duty"))


def encode_set_current(controller_id: int, current_a: float,
                       off_delay_s: Optional[float] = None) -> VescFrame:
    """Motor (q-axis) current in amps — THE torque command (T ≈ Kt·Iq).
    Sign convention: + drives, − regens/brakes through zero into reverse torque.

    off_delay_s: per-command watchdog — the VESC zeroes the command that many
    seconds after the last refresh. NOTE the field is PREPENDED (fw quirk).
    """
    data = _scaled32(current_a, 1e3, "current_a")
    if off_delay_s is not None:
        data = _off_delay_bytes(off_delay_s) + data
    return VescFrame(make_arbitration_id(CanPacketId.SET_CURRENT, controller_id), data)


def encode_set_current_brake(controller_id: int, current_a: float) -> VescFrame:
    """Braking current in amps (dissipative braking toward zero speed;
    magnitude command, does not push through zero into drive)."""
    return VescFrame(make_arbitration_id(CanPacketId.SET_CURRENT_BRAKE, controller_id),
                     _scaled32(current_a, 1e3, "current_a"))


def encode_set_rpm(controller_id: int, erpm: float) -> VescFrame:
    """Speed setpoint in ELECTRICAL rpm (mechanical rpm × pole pairs), driven
    by the VESC's internal speed PID. The constant-speed dyno mode."""
    return VescFrame(make_arbitration_id(CanPacketId.SET_RPM, controller_id),
                     _scaled32(erpm, 1e0, "erpm"))


def encode_set_pos(controller_id: int, degrees: float) -> VescFrame:
    """Position setpoint in degrees (VESC position PID; needs an encoder and is
    single-turn/wrap-sensitive — for multi-turn actuators prefer a host-side
    position loop over SET_CURRENT)."""
    return VescFrame(make_arbitration_id(CanPacketId.SET_POS, controller_id),
                     _scaled32(degrees, 1e6, "degrees"))


def encode_set_current_rel(controller_id: int, rel: float,
                           off_delay_s: Optional[float] = None) -> VescFrame:
    """Current as a fraction [-1, 1] of the VESC's own configured limit.
    NOTE the off-delay field is APPENDED here (opposite of SET_CURRENT)."""
    _check_rel(rel, "rel")
    data = _scaled32(rel, 1e5, "rel")
    if off_delay_s is not None:
        data = data + _off_delay_bytes(off_delay_s)
    return VescFrame(make_arbitration_id(CanPacketId.SET_CURRENT_REL, controller_id), data)


def encode_set_current_brake_rel(controller_id: int, rel: float) -> VescFrame:
    _check_rel(rel, "rel")
    return VescFrame(make_arbitration_id(CanPacketId.SET_CURRENT_BRAKE_REL, controller_id),
                     _scaled32(rel, 1e5, "rel"))


def encode_set_handbrake(controller_id: int, current_a: float) -> VescFrame:
    """Handbrake current in amps: holds the rotor at standstill with an
    open-loop current (heats the motor; standstill hold only)."""
    return VescFrame(make_arbitration_id(CanPacketId.SET_CURRENT_HANDBRAKE, controller_id),
                     _scaled32(current_a, 1e3, "current_a"))


def encode_set_handbrake_rel(controller_id: int, rel: float) -> VescFrame:
    _check_rel(rel, "rel")
    return VescFrame(make_arbitration_id(CanPacketId.SET_CURRENT_HANDBRAKE_REL, controller_id),
                     _scaled32(rel, 1e5, "rel"))


def encode_set_id_dissipate(controller_id: int, current_a: float,
                            off_delay_s: float) -> VescFrame:
    """Molten MOSFET fork only: inject d-axis current to burn energy as winding
    heat (~zero torque; torque keeps priority over the injection in firmware).

    off_delay_s is MANDATORY — it is the command's watchdog. The firmware
    clamps it to [0.05, 5.0] s and ramps the injection to zero on expiry, so
    the host must refresh continuously to sustain a dump. Frames without the
    off-delay field are ignored by the firmware.

    Payload: [current f32×1e3][off_delay f16×1e3] (appended, like
    SET_CURRENT_REL — not the SET_CURRENT prepend quirk).
    """
    if current_a < 0.0:
        raise ValueError("dissipation current is a magnitude, got %r" % current_a)
    if not 0.0 < off_delay_s <= 5.0:
        raise ValueError("off_delay_s must be in (0, 5.0] s (firmware cap), got %r"
                         % off_delay_s)
    data = _scaled32(current_a, 1e3, "current_a") + _off_delay_bytes(off_delay_s)
    return VescFrame(make_arbitration_id(CanPacketId.MM_SET_ID_DISSIPATE, controller_id), data)


# --- CONF_* envelope encoders (RAM by default; store=True persists to flash) --

def encode_conf_current_limits(controller_id: int, min_a: float, max_a: float,
                               store: bool = False) -> VescFrame:
    """Motor current envelope (l_current_min/max) — the torque envelope.
    min_a is the regen/reverse-torque side and must be <= 0 in VESC convention."""
    pid = (CanPacketId.CONF_STORE_CURRENT_LIMITS if store
           else CanPacketId.CONF_CURRENT_LIMITS)
    return VescFrame(make_arbitration_id(pid, controller_id),
                     _scaled32(min_a, 1e3, "min_a") + _scaled32(max_a, 1e3, "max_a"))


def encode_conf_current_limits_in(controller_id: int, min_a: float, max_a: float,
                                  store: bool = False) -> VescFrame:
    """Input (battery-side) current envelope (l_in_current_min/max).
    min_a bounds regen INTO the supply — this is the absorption ceiling."""
    pid = (CanPacketId.CONF_STORE_CURRENT_LIMITS_IN if store
           else CanPacketId.CONF_CURRENT_LIMITS_IN)
    return VescFrame(make_arbitration_id(pid, controller_id),
                     _scaled32(min_a, 1e3, "min_a") + _scaled32(max_a, 1e3, "max_a"))


def encode_conf_foc_erpms(controller_id: int, foc_openloop_rpm: float,
                          foc_sl_erpm: float, store: bool = False) -> VescFrame:
    """Sets foc_openloop_rpm / foc_sl_erpm (FOC sensorless thresholds).
    WARNING: despite the name this is NOT a speed limit — there is no CAN
    packet for l_min_erpm/l_max_erpm."""
    pid = (CanPacketId.CONF_STORE_FOC_ERPMS if store
           else CanPacketId.CONF_FOC_ERPMS)
    return VescFrame(make_arbitration_id(pid, controller_id),
                     _scaled32(foc_openloop_rpm, 1e3, "foc_openloop_rpm")
                     + _scaled32(foc_sl_erpm, 1e3, "foc_sl_erpm"))


def encode_conf_battery_cut(controller_id: int, start_v: float, end_v: float,
                            store: bool = False) -> VescFrame:
    """Battery cutoff voltages (l_battery_cut_start/end): output starts
    derating at start_v, reaches zero at end_v."""
    pid = (CanPacketId.CONF_STORE_BATTERY_CUT if store
           else CanPacketId.CONF_BATTERY_CUT)
    return VescFrame(make_arbitration_id(pid, controller_id),
                     _scaled32(start_v, 1e3, "start_v") + _scaled32(end_v, 1e3, "end_v"))


def encode_ping(controller_id: int, sender_id: int) -> VescFrame:
    """PING a node. The PONG comes back with arbitration controller-field ==
    sender_id and payload [responder_id, hw_type]."""
    if not 0 <= sender_id <= 0xFF:
        raise ValueError("sender_id must be 0..255, got %r" % sender_id)
    return VescFrame(make_arbitration_id(CanPacketId.PING, controller_id),
                     bytes([sender_id]))


def encode_shutdown(controller_id: int) -> VescFrame:
    """Power off (only on hardware with a shutdown circuit; no-op otherwise)."""
    return VescFrame(make_arbitration_id(CanPacketId.SHUTDOWN, controller_id), b'')


# --- COMM-over-CAN tunnel (ids 5-8) -------------------------------------------
# Carries un-framed COMM packets (no start/stop bytes) over CAN — how VESC Tool
# talks through a bridge, and how a CAN-only host reads fw version and fault
# codes (STATUS frames don't carry faults). Requests <= 6 bytes ride
# PROCESS_SHORT_BUFFER; longer ones are chunked via FILL_RX_BUFFER (byte
# offset, <=255) / FILL_RX_BUFFER_LONG (u16 offset) and finalized by
# PROCESS_RX_BUFFER carrying [sender, send_flag, len:u16, crc16:u16] with
# CRC-16/XMODEM over the payload. Wire format from comm_can_send_buffer().

def crc16_xmodem(data: bytes) -> int:
    from crccheck.crc import CrcXmodem
    return CrcXmodem().calc(data)


def encode_comm_frames(controller_id: int, sender_id: int, payload: bytes,
                       send_flag: int = 0) -> List[VescFrame]:
    """Encode one COMM payload for the tunnel; returns the frame sequence to
    send in order. send_flag 0 = process and reply over CAN to sender_id."""
    if not payload:
        raise ValueError("empty COMM payload")
    if len(payload) <= 6:
        return [VescFrame(
            make_arbitration_id(CanPacketId.PROCESS_SHORT_BUFFER, controller_id),
            bytes([sender_id, send_flag]) + payload)]
    out = []
    i = 0
    while i <= 255 and i < len(payload):          # byte-offset chunks, 7 each
        chunk = payload[i:i + 7]
        out.append(VescFrame(
            make_arbitration_id(CanPacketId.FILL_RX_BUFFER, controller_id),
            bytes([i]) + chunk))
        i += len(chunk)
    while i < len(payload):                        # u16-offset chunks, 6 each
        chunk = payload[i:i + 6]
        out.append(VescFrame(
            make_arbitration_id(CanPacketId.FILL_RX_BUFFER_LONG, controller_id),
            struct.pack('!H', i) + chunk))
        i += len(chunk)
    out.append(VescFrame(
        make_arbitration_id(CanPacketId.PROCESS_RX_BUFFER, controller_id),
        struct.pack('!BBHH', sender_id, send_flag, len(payload),
                    crc16_xmodem(payload))))
    return out


# --- STATUS decoders -----------------------------------------------------------

class Status1(NamedTuple):
    """STATUS (id 9): the loop-rate telemetry frame."""
    rpm: float        # electrical rpm
    current: float    # motor current, A (filtered)
    duty: float       # duty fraction


class Status2(NamedTuple):
    amp_hours: float          # consumed, Ah
    amp_hours_charged: float  # regenerated, Ah


class Status3(NamedTuple):
    watt_hours: float
    watt_hours_charged: float


class Status4(NamedTuple):
    temp_fet: float     # °C (filtered)
    temp_motor: float   # °C (filtered)
    current_in: float   # input/battery current, A (− = regen into supply)
    pid_pos: float      # degrees


class Status5(NamedTuple):
    tachometer: int     # cumulative commutation steps (signed)
    v_in: float         # bus voltage, V


class Status6(NamedTuple):
    adc1: float  # V
    adc2: float  # V
    adc3: float  # V
    ppm: float


class StatusDissipation(NamedTuple):
    """MM_STATUS_DISSIPATION (fork id 201): d-axis dump telemetry. Broadcast
    alongside STATUS_1 but only while the injection is armed."""
    id_meas: float      # measured d-axis current, A (filtered; negative = injecting)
    iq_meas: float      # measured q-axis current, A (filtered)
    id_diss_now: float  # ramped injection magnitude the firmware is applying, A
    p_copper: float     # firmware copper-loss estimate 1.5·Rs·(id²+iq²), W


class Pong(NamedTuple):
    """PONG payload. Note: the frame's controller-id field is the ADDRESSEE
    (the pinger's sender_id); the responder is in the payload."""
    controller_id: int  # responder
    hw_type: int        # HW_TYPE_* (0 = VESC)


DecodedStatus = Union[Status1, Status2, Status3, Status4, Status5, Status6,
                      StatusDissipation, Pong]


def _i16(data: bytes, off: int) -> int:
    return struct.unpack_from('!h', data, off)[0]


def _i32(data: bytes, off: int) -> int:
    return struct.unpack_from('!i', data, off)[0]


def decode_frame(arbitration_id: int,
                 data: bytes) -> Optional[Tuple[int, DecodedStatus]]:
    """Decode one extended-id frame.

    Returns (controller_id_field, decoded) for STATUS_1..6 and PONG frames,
    None for anything else (commands, buffer-transfer, BMS...). For PONG the
    controller_id_field is the addressee; the responder id is in the payload.
    """
    packet_id, controller_id = split_arbitration_id(arbitration_id)
    try:
        pid = CanPacketId(packet_id)
    except ValueError:
        return None

    if pid == CanPacketId.STATUS and len(data) >= 8:
        return controller_id, Status1(rpm=float(_i32(data, 0)),
                                      current=_i16(data, 4) / 10.0,
                                      duty=_i16(data, 6) / 1000.0)
    if pid == CanPacketId.STATUS_2 and len(data) >= 8:
        return controller_id, Status2(amp_hours=_i32(data, 0) / 1e4,
                                      amp_hours_charged=_i32(data, 4) / 1e4)
    if pid == CanPacketId.STATUS_3 and len(data) >= 8:
        return controller_id, Status3(watt_hours=_i32(data, 0) / 1e4,
                                      watt_hours_charged=_i32(data, 4) / 1e4)
    if pid == CanPacketId.STATUS_4 and len(data) >= 8:
        return controller_id, Status4(temp_fet=_i16(data, 0) / 10.0,
                                      temp_motor=_i16(data, 2) / 10.0,
                                      current_in=_i16(data, 4) / 10.0,
                                      pid_pos=_i16(data, 6) / 50.0)
    if pid == CanPacketId.STATUS_5 and len(data) >= 6:
        return controller_id, Status5(tachometer=_i32(data, 0),
                                      v_in=_i16(data, 4) / 10.0)
    if pid == CanPacketId.STATUS_6 and len(data) >= 8:
        return controller_id, Status6(adc1=_i16(data, 0) / 1e3,
                                      adc2=_i16(data, 2) / 1e3,
                                      adc3=_i16(data, 4) / 1e3,
                                      ppm=_i16(data, 6) / 1e3)
    if pid == CanPacketId.MM_STATUS_DISSIPATION and len(data) >= 8:
        return controller_id, StatusDissipation(
            id_meas=_i16(data, 0) / 10.0,
            iq_meas=_i16(data, 2) / 10.0,
            id_diss_now=_i16(data, 4) / 10.0,
            p_copper=float(struct.unpack_from('!H', data, 6)[0]))
    if pid == CanPacketId.PONG and len(data) >= 1:
        hw_type = data[1] if len(data) >= 2 else 0
        return controller_id, Pong(controller_id=data[0], hw_type=hw_type)
    return None
