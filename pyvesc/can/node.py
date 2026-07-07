"""
VescCanNode — typed façade for one VESC on the bus.

All commands are engineering units (amps, electrical rpm, degrees, volts).
The node remembers every CONF_* envelope it has applied (``applied_conf``) so
a run manifest can record the envelope the hardware was actually given — the
VESC does not acknowledge CONF_* frames, so this is the record of command,
not a readback.

SAFETY: none of this is a safety device (see bus.py header).
"""

import time
from typing import Dict, Optional, Tuple

from . import frames
from .bus import NodeTelemetry, VescCanBus


class VescCanNode:
    def __init__(self, bus: VescCanBus, controller_id: int):
        if not 0 <= controller_id <= 0xFE:
            raise ValueError("controller_id must be 0..254 for a real node")
        self.bus = bus
        self.controller_id = controller_id
        #: CONF_* values sent to this node this session: name -> (values, monotonic stamp)
        self.applied_conf: Dict[str, Tuple[tuple, float]] = {}

    # --- commands (each resets the VESC's app timeout) -------------------------

    def set_current(self, amps: float, off_delay_s: Optional[float] = None) -> None:
        """Torque command: motor current in amps (+ drive, − brake/regen)."""
        self.bus.send(frames.encode_set_current(self.controller_id, amps, off_delay_s))

    def set_brake_current(self, amps: float) -> None:
        self.bus.send(frames.encode_set_current_brake(self.controller_id, amps))

    def set_rpm(self, erpm: float) -> None:
        """Speed setpoint, electrical rpm (VESC-internal PID)."""
        self.bus.send(frames.encode_set_rpm(self.controller_id, erpm))

    def set_pos(self, degrees: float) -> None:
        self.bus.send(frames.encode_set_pos(self.controller_id, degrees))

    def set_duty(self, duty: float) -> None:
        self.bus.send(frames.encode_set_duty(self.controller_id, duty))

    def set_current_rel(self, rel: float, off_delay_s: Optional[float] = None) -> None:
        self.bus.send(frames.encode_set_current_rel(self.controller_id, rel, off_delay_s))

    def set_handbrake(self, amps: float) -> None:
        self.bus.send(frames.encode_set_handbrake(self.controller_id, amps))

    def stop(self) -> None:
        """Zero the torque command. Convenience, not a safety stop."""
        self.set_current(0.0)

    # --- per-run envelope (RAM unless store=True) --------------------------------

    def conf_current_limits(self, min_a: float, max_a: float,
                            store: bool = False) -> None:
        """Motor-current (torque) envelope. min_a <= 0 is the regen side."""
        self.bus.send(frames.encode_conf_current_limits(
            self.controller_id, min_a, max_a, store))
        self._record_conf('current_limits', (min_a, max_a), store)

    def conf_current_limits_in(self, min_a: float, max_a: float,
                               store: bool = False) -> None:
        """Battery-side envelope; min_a bounds regen into the supply —
        the absorption ceiling."""
        self.bus.send(frames.encode_conf_current_limits_in(
            self.controller_id, min_a, max_a, store))
        self._record_conf('current_limits_in', (min_a, max_a), store)

    def conf_battery_cut(self, start_v: float, end_v: float,
                         store: bool = False) -> None:
        self.bus.send(frames.encode_conf_battery_cut(
            self.controller_id, start_v, end_v, store))
        self._record_conf('battery_cut', (start_v, end_v), store)

    def conf_foc_erpms(self, foc_openloop_rpm: float, foc_sl_erpm: float,
                       store: bool = False) -> None:
        """FOC sensorless thresholds — NOT a speed limit (see frames.py)."""
        self.bus.send(frames.encode_conf_foc_erpms(
            self.controller_id, foc_openloop_rpm, foc_sl_erpm, store))
        self._record_conf('foc_erpms', (foc_openloop_rpm, foc_sl_erpm), store)

    def _record_conf(self, name: str, values: tuple, store: bool) -> None:
        key = name + ('_stored' if store else '')
        self.applied_conf[key] = (values, time.monotonic())

    # --- telemetry -----------------------------------------------------------------

    @property
    def telemetry(self) -> NodeTelemetry:
        return self.bus.telemetry(self.controller_id)

    def is_alive(self, max_age_s: float = 0.5) -> bool:
        """True if STATUS_1 has been seen within max_age_s."""
        age = self.telemetry.age('status1')
        return age is not None and age <= max_age_s

    def ping(self, timeout: float = 0.2) -> Optional[int]:
        return self.bus.ping(self.controller_id, timeout=timeout)
