"""
Bench smoke test for the dyno VESC over USB. Safe with NO motor connected:
every set command is zero-valued, so nothing can spin and no current flows.

Verifies on real hardware:
  1. serial link + firmware version handshake (GetVersion)
  2. telemetry decode (GetValues: bus voltage, temps, fault code)
  3. heartbeat thread (Alive at 10 Hz, link stays fault-free)
  4. the four-quadrant control commands are accepted without raising a fault:
     SetCurrent(0), SetCurrentBrake(0), SetHandbrake(0), SetDutyCycle(0), SetRPM(0)

Usage: ./.venv/bin/python bench_check.py [port] [can_id]
  port: serial device (default /dev/ttyACM0) or a TCP bridge address
        like tcp://192.168.1.50 (see esp32_bridge/)
  can_id: CAN ID of the motor controller when `port` is a bridge on its
          CAN bus rather than the controller itself (e.g. a VESC Express)
"""

import sys
import time

import pyvesc
from pyvesc.VESC import VESC
from pyvesc.messages import SetCurrentBrake, SetHandbrake

DEFAULT_PORT = '/dev/ttyACM0'

FAULT_NONE = b'\x00'

results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ''))
    return ok


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    can_id = int(sys.argv[2]) if len(sys.argv) > 2 else None
    print(f"Connecting to {port}" + (f" (motor controller at CAN {can_id})" if can_id is not None else "") + " ...")

    with VESC(serial_port=port, can_id=can_id) as vesc:
        # 1. handshake
        fw = vesc.get_firmware_version()
        check("GetVersion", fw is not None, f"firmware {fw}")

        # 2. telemetry
        m = vesc.get_measurements()
        ok = m is not None
        check("GetValues decode", ok)
        if not ok:
            print("  telemetry failed — aborting before sending commands")
            return
        print(f"    v_in={m.v_in:.1f} V  temp_fet={m.temp_fet:.1f} C  "
              f"temp_mos1/2/3={m.temp_mos1:.1f}/{m.temp_mos2:.1f}/{m.temp_mos3:.1f} C  "
              f"rpm={m.rpm}  duty={m.duty_cycle_now:.3f}  "
              f"fault={m.mc_fault_code!r}")
        check("bus voltage plausible", 5.0 < m.v_in < 150.0, f"{m.v_in:.1f} V")
        check("temperature plausible", -20.0 < m.temp_fet < 90.0, f"{m.temp_fet:.1f} C")
        check("no fault at connect", m.mc_fault_code == FAULT_NONE)

        # 3. heartbeat: constructor already started it; hold the link and re-poll
        time.sleep(2.0)
        m = vesc.get_measurements()
        check("link alive after 2s of heartbeat",
              m is not None and m.mc_fault_code == FAULT_NONE)

        # 4. four-quadrant command set, all zero-valued (motor absent => no action)
        commands = [
            ("SetCurrent(0)", lambda: vesc.set_current(0)),
            ("SetCurrentBrake(0)", lambda: vesc.write(pyvesc.encode(SetCurrentBrake(0, can_id=vesc.can_id)))),
            ("SetHandbrake(0)", lambda: vesc.write(pyvesc.encode(SetHandbrake(0, can_id=vesc.can_id)))),
            ("SetDutyCycle(0)", lambda: vesc.set_duty_cycle(0)),
            ("SetRPM(0)", lambda: vesc.set_rpm(0)),
        ]
        for name, send in commands:
            send()
            time.sleep(0.2)
            m = vesc.get_measurements()
            # duty threshold: open output floats with ±0.015 of noise (observed);
            # a real actuation would read far above 0.02
            check(f"{name} accepted",
                  m is not None and m.mc_fault_code == FAULT_NONE and abs(m.duty_cycle_now) < 0.02,
                  f"duty={m.duty_cycle_now:.3f} fault={m.mc_fault_code!r}" if m else "no telemetry")

        # release control before disconnect
        vesc.set_current(0)

    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed"
          + (f" — FAILED: {', '.join(failed)}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
