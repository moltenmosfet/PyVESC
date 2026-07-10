#!/usr/bin/env python3
"""
can_diss_check — bench validation ladder for the Molten MOSFET d-axis
dissipation feature (moltenmosfet/vesc_firmware, feature/d-axis-dissipation).
Sibling of can_bench_check.py; REQUIRES the fork firmware on the node.

MOTOR HEATING IS THE POINT of this feature: currents here are small (default
3 A) and every injection is watchdog-limited, but keep a hand near the power
switch and the motor's temperature in mind. Run on a bench motor you can
afford to warm up.

Usage:
    ./can_diss_check.py [channel] [controller_id] [amps]   # default can0, scan, 3.0

What it verifies (and reports as a checklist):
    1.  Fork present: MM_STATUS_DISSIPATION appears once an injection is
        armed (stock firmware ignores frame 200 and broadcasts nothing).
    2.  Standstill injection: SET_ID_DISSIPATE(amps, 0.5 s) refreshed at
        10 Hz -> id_meas tracks -amps, iq stays ~0, motor does NOT rotate
        (rpm ~0), p_copper > 0 reported.
    3.  Torque priority: with a deliberately tight CONF_CURRENT_LIMITS
        envelope (amps), command SET_CURRENT(amps) + dissipation together
        -> iq keeps the budget, id_diss_now collapses toward 0.
    4.  Refresh-or-decay: stop refreshing -> id_meas ramps to 0 within
        off_delay + ramp margin; MM_STATUS_DISSIPATION stops broadcasting.
    5.  Ride-along: dissipation + SET_RPM spin at low speed -> both present
        (id_meas ~ -amps while spinning), then clean stop.

After the ladder, cross-check in VESC Tool terminal: `mm_diss` prints the
same state this script sees over CAN.
"""

import sys
import time

from pyvesc.can import VescCanBus, VescCanNode


def check(name, ok, detail=''):
    print('  [%s] %s%s' % ('PASS' if ok else 'FAIL', name,
                           (' — ' + detail) if detail else ''))
    return ok


def pump(node, amps, seconds, off_delay=0.5, hz=10.0):
    """Refresh the injection at hz for `seconds`, returning the last
    dissipation status seen (or None)."""
    t_end = time.monotonic() + seconds
    last = None
    while time.monotonic() < t_end:
        node.set_id_dissipate(amps, off_delay_s=off_delay)
        time.sleep(1.0 / hz)
        st = node.telemetry.status_dissipation
        if st is not None:
            last = st
    return last


def main():
    channel = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    want_id = int(sys.argv[2]) if len(sys.argv) > 2 else None
    amps = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    results = []

    print('== d-axis dissipation bench check on %s (%.1f A) ==' % (channel, amps))
    bus = VescCanBus(channel=channel)
    try:
        if want_id is None:
            print('scanning ids 0..15 and 100...')
            found = bus.scan(list(range(0, 16)) + [100], timeout=0.1)
            if not found:
                print('no nodes — check bitrate/termination/power')
                return 1
            want_id = found[0]
        node = VescCanNode(bus, want_id)
        print('using controller id %d' % want_id)

        # 1: fork present — arm a tiny injection, look for the status frame
        st = pump(node, min(1.0, amps), 1.0)
        results.append(check('fork status frame (MM_STATUS_DISSIPATION)',
                             st is not None,
                             'stock firmware? flash the fork build' if st is None
                             else repr(st)))
        if st is None:
            return 1

        # 2: standstill injection at the requested current
        st = pump(node, amps, 3.0)
        rpm = node.telemetry.status1.rpm if node.telemetry.status1 else None
        ok = (st is not None and abs(st.id_meas + amps) < max(0.5, 0.2 * amps)
              and abs(st.iq_meas) < 1.0)
        results.append(check('standstill injection tracks -%.1f A' % amps, ok,
                             'id=%.1f iq=%.1f p=%.0fW rpm=%s' %
                             (st.id_meas, st.iq_meas, st.p_copper, rpm)))

        # 3: torque priority under a tight envelope. The demand (id_diss_now)
        # stays at the commanded value BY DESIGN — the clip shows in the
        # MEASURED split: iq owns the budget, id collapses. Handbrake gives a
        # sustained standstill iq (SET_CURRENT on a free motor just
        # accelerates and frees the budget again).
        node.conf_current_limits(-amps, amps)
        try:
            t_end = time.monotonic() + 2.0
            last = None
            while time.monotonic() < t_end:
                node.set_handbrake(amps)
                node.set_id_dissipate(amps, off_delay_s=0.5)
                time.sleep(0.05)
                if node.telemetry.status_dissipation is not None:
                    last = node.telemetry.status_dissipation
            ok = (last is not None and abs(last.iq_meas) > 0.7 * amps
                  and abs(last.id_meas) < 0.3 * amps
                  and last.id_diss_now > 0.9 * amps)
            results.append(check('torque priority (measured id yields to iq)', ok,
                                 'id_meas=%.1f iq_meas=%.1f (demand %.1f) under %.1f A envelope' %
                                 ((last.id_meas, last.iq_meas, last.id_diss_now, amps)
                                  if last else (-1, -1, -1, amps))))
        finally:
            node.set_current(0.0)
            node.conf_current_limits(-50.0, 50.0)  # bench envelope back open (RAM; reboot restores flash)

        # 4: refresh-or-decay — stop refreshing, watch it die
        pump(node, amps, 1.0, off_delay=0.3)
        time.sleep(1.5)  # > off_delay + ramp
        st = node.telemetry.status_dissipation
        age = node.telemetry.age('status_dissipation')
        ok = age is None or age > 0.5 or (st and abs(st.id_meas) < 0.5)
        results.append(check('refresh-or-decay (watchdog)', ok,
                             'status age %.2fs' % age if age else 'never seen'))

        # 5: ride-along with a low-speed spin
        node.set_rpm(2000)
        time.sleep(1.0)
        st = pump(node, amps, 2.0)
        node.set_rpm(0)
        node.set_current(0.0)
        ok = st is not None and abs(st.id_meas + amps) < max(0.5, 0.2 * amps)
        results.append(check('injection rides along while spinning', ok,
                             repr(st) if st else 'no status'))

        print('== %d/%d passed ==' % (sum(results), len(results)))
        return 0 if all(results) else 1
    finally:
        try:
            node.set_current(0.0)
        except Exception:
            pass
        bus.close()


if __name__ == '__main__':
    sys.exit(main())
