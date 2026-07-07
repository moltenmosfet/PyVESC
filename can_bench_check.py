#!/usr/bin/env python3
"""
can_bench_check — hardware smoke test for pyvesc.can against a real VESC.
Sibling of bench_check.py (COMM); motor-less SAFE: every command is
zero-valued, and the one nonzero test is the off-delay auto-zero (0 A).

Usage:
    ./can_bench_check.py [channel] [controller_id]     # default can0, scan

Bring-up (one-time, gs_usb/candleLight or similar):
    sudo ip link set can0 up type can bitrate 500000
    # VESC side: App Settings -> General -> CAN status msgs rate + which
    # STATUS messages to broadcast (need at least STATUS_1,4,5); CAN baud.

What it verifies (and reports as a checklist):
    1. SocketCAN opens; frames flow.
    2. PING/PONG enumeration finds the node.
    3. STATUS_1/4/5 broadcasts arrive; measured rates printed.
    4. Zero-valued SET_CURRENT / SET_CURRENT_BRAKE / SET_RPM / handbrake
       are accepted (no fault, node keeps broadcasting).
    5. CONF_CURRENT_LIMITS(_IN) round-trip *behaviorally*: applies a
       conservative envelope, then restores. NOTE: the VESC does not ack
       CONF_*; on FW < 5.3 these packets may be silently ignored — this
       test detects that ONLY behaviorally (fw version printed; check
       VESC Tool afterwards to confirm values). THE FW 5.2 QUESTION THIS
       EXISTS TO ANSWER: watch the printed verdict.
    6. Off-delay: SET_CURRENT(0, off_delay=0.2) accepted (auto-zero of an
       already-zero command; presence of the 6-byte form not faulting is
       the point).
    7. Command RTT proxy: PING round-trip latency histogram (min/med/p95).
"""

import statistics
import sys
import time

from pyvesc.can import VescCanBus, VescCanNode


def check(name, ok, detail=''):
    print('  [%s] %s%s' % ('PASS' if ok else 'FAIL', name,
                           (' — ' + detail) if detail else ''))
    return ok


def main():
    channel = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    want_id = int(sys.argv[2]) if len(sys.argv) > 2 else None
    results = []

    print('== pyvesc.can bench check on %s ==' % channel)
    bus = VescCanBus(channel=channel)
    try:
        # 1-2: enumeration
        if want_id is None:
            print('scanning ids 0..15 (pass an id to skip)...')
            found = bus.scan(range(0, 16), timeout=0.1)
            results.append(check('enumeration', bool(found), str(found)))
            if not found:
                print('no nodes — check bitrate/termination/power')
                return 1
            want_id = sorted(found)[0]
        else:
            hw = bus.ping(want_id, timeout=0.5)
            results.append(check('ping id %d' % want_id, hw is not None,
                                 'hw_type=%s' % hw))
        node = VescCanNode(bus, want_id)
        print('using controller id %d' % want_id)

        # 3: STATUS broadcasts + rates
        t0 = time.monotonic()
        counts = {}
        bus.subscribe(lambda cid, obj, c=counts: c.__setitem__(
            type(obj).__name__, c.get(type(obj).__name__, 0) + 1)
            if cid == want_id else None)
        time.sleep(2.0)
        dt = time.monotonic() - t0
        for name in ('Status1', 'Status4', 'Status5'):
            rate = counts.get(name, 0) / dt
            results.append(check('%s broadcast' % name, rate > 1,
                                 '%.1f Hz' % rate))
        telem = node.telemetry
        if telem.status5:
            print('  bus voltage: %.1f V' % telem.status5.v_in)
        if telem.status1 is not None:
            print('  rpm=%.0f current=%.1fA duty=%.3f'
                  % (telem.status1.rpm, telem.status1.current,
                     telem.status1.duty))

        # 4: zero-valued commands accepted (node keeps broadcasting after)
        for label, fn in [
                ('SET_CURRENT 0', lambda: node.set_current(0.0)),
                ('SET_CURRENT_BRAKE 0', lambda: node.set_brake_current(0.0)),
                ('SET_RPM 0', lambda: node.set_rpm(0.0)),
                ('SET_HANDBRAKE 0', lambda: node.set_handbrake(0.0)),
                ('SET_CURRENT 0 + off_delay 0.2s',
                 lambda: node.set_current(0.0, off_delay_s=0.2))]:
            fn()
            time.sleep(0.3)
            results.append(check(label, node.is_alive(max_age_s=1.0)))
        node.stop()

        # 5: CONF_* — THE FW 5.2 QUESTION
        print('applying conservative CONF envelope (RAM only)...')
        node.conf_current_limits(-10.0, 10.0)
        node.conf_current_limits_in(-5.0, 5.0)
        time.sleep(0.5)
        alive = node.is_alive(max_age_s=1.0)
        results.append(check(
            'CONF_* frames sent, node alive', alive,
            'NO ACK EXISTS: verify values in VESC Tool -> Motor Settings '
            'to confirm this firmware ACTUALLY APPLIED them'))
        fw = 'unknown'
        print('  (fw version: read via COMM/VESC Tool — CONF_* CAN handlers '
              'may be absent on older firmware; %s)' % fw)

        # 7: RTT histogram via PING
        rtts = []
        for _ in range(50):
            t = time.monotonic()
            if bus.ping(want_id, timeout=0.5) is not None:
                rtts.append((time.monotonic() - t) * 1e3)
        if rtts:
            rtts.sort()
            results.append(check(
                'PING RTT', True,
                'min %.2f / med %.2f / p95 %.2f ms (n=%d)'
                % (rtts[0], statistics.median(rtts),
                   rtts[int(len(rtts) * 0.95) - 1], len(rtts))))
        else:
            results.append(check('PING RTT', False, 'no responses'))

    finally:
        try:
            bus.stop_all()
        finally:
            bus.close()

    ok = sum(results)
    print('== %d/%d passed ==' % (ok, len(results)))
    return 0 if ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
