#!/usr/bin/env python3
"""
can_clamp_check — guided bench ladder for the Molten MOSFET d-axis bus clamp
(moltenmosfet/vesc_firmware, feature/d-axis-dissipation). Sibling of
can_diss_check.py; REQUIRES the fork firmware on the node.

BENCH SETUP (read before running):
  - Current-limited lab PSU, <= 48 V for a 57 V-class board (l_max_vin = 57).
  - INLINE SERIES DIODE between PSU and VESC, heatsinked for the drive
    current. With the diode the bus has NO sink at all — the clamp is the
    only thing holding voltage during regen. That is the point of the test.
  - Set l_battery_regen_cut_start/end in VESC Tool to v_clamp+2 / v_clamp+5
    (band >= 2-3 V; end <= l_max_vin - 2). There is NO CAN setter for these.
  - Energy source is spin-up kinetic energy (no prime mover on the bench):
    keep test rpm modest until the chain is proven.

The script is interactive: it pauses before every step that spins the motor.
Steps 4/5 command a spin-up then a braking step and watch v_bus / i_bus /
clamp telemetry. Deliberate-fault, degradation-chain and endurance testing
(plan steps 3/7/8) stay manual — see documentation/mm_d_axis_dissipation.md
in the firmware repo.

Usage:
    ./can_clamp_check.py [channel] [controller_id] [psu_volts]
      channel      default can0
      controller_id  default: scan
      psu_volts    default 30.0 — v_clamp is set to psu_volts + 4
"""

import sys
import time

from pyvesc.can import VescCanBus, VescCanNode


def check(name, ok, detail=''):
    print('  [%s] %s%s' % ('PASS' if ok else 'FAIL', name,
                           (' — ' + detail) if detail else ''))
    return ok


def confirm(msg):
    input('\n>> %s\n   ENTER to continue (Ctrl-C aborts)... ' % msg)


def wait_status(node, seconds):
    t_end = time.monotonic() + seconds
    last = None
    while time.monotonic() < t_end:
        st = node.telemetry.status_bus_clamp
        if st is not None:
            last = st
        time.sleep(0.02)
    return last


def spin_brake(node, drive_a, brake_a, brake_s, samples):
    """Drive with constant current to natural top speed (max kinetic energy
    for this bus voltage, no speed-PID current spikes into a current-limited
    PSU), then brake, sampling clamp telemetry through the stop.
    Returns (max_vbus, min_ibus, peak_erpm, log)."""
    t_end = time.monotonic() + 2.5
    while time.monotonic() < t_end:
        node.set_current(abs(drive_a), off_delay_s=0.3)
        time.sleep(0.05)
    s1 = node.telemetry.status1
    peak_erpm = s1.rpm if s1 else 0.0
    max_v, min_i = 0.0, 0.0
    log = []
    t_end = time.monotonic() + brake_s
    while time.monotonic() < t_end:
        node.set_current(-abs(brake_a), off_delay_s=0.3)
        st = node.telemetry.status_bus_clamp
        s5 = node.telemetry.status5
        if st is not None:
            max_v = max(max_v, st.v_bus)
            min_i = min(min_i, st.i_bus)
            log.append(st)
        elif s5 is not None:
            max_v = max(max_v, s5.v_in)
        time.sleep(1.0 / samples)
    node.set_current(0.0)
    return max_v, min_i, peak_erpm, log


def main():
    channel = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    want_id = int(sys.argv[2]) if len(sys.argv) > 2 else None
    psu_v = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
    v_clamp = psu_v + 4.0
    results = []

    print('== d-axis bus clamp bench ladder on %s (PSU %.1f V, v_clamp %.1f V) =='
          % (channel, psu_v, v_clamp))
    print('   Series diode in place? Regen-cut configured in VESC Tool? (see header)')

    bus = VescCanBus(channel=channel)
    node = None
    try:
        if want_id is None:
            found = bus.scan(list(range(0, 16)) + [100], timeout=0.1)
            if not found:
                print('no nodes — check bitrate/termination/power')
                return 1
            want_id = found[0]
        node = VescCanNode(bus, want_id)
        print('using controller id %d' % want_id)

        # -- 1: frames only — arm, see telemetry, disarm, see it stop --------
        node.conf_bus_clamp(v_clamp, i_floor=0.0, i_max=5.0,
                            allow_start_modulation=False)
        st = wait_status(node, 1.0)
        ok = st is not None and st.armed
        results.append(check('arm -> MM_STATUS_BUS_CLAMP broadcast', ok,
                             repr(st) if st else
                             'no status — stock firmware? flash the fork build'))
        if not ok:
            return 1
        sane = abs(st.v_bus - psu_v) < max(2.0, 0.1 * psu_v)
        results.append(check('reported v_bus sane vs PSU', sane,
                             'v_bus %.1f V vs PSU %.1f V (check with a meter too)'
                             % (st.v_bus, psu_v)))

        node.disarm_bus_clamp()
        time.sleep(1.0)
        age = node.telemetry.age('status_bus_clamp')
        results.append(check('disarm -> broadcast stops',
                             age is not None and age > 0.5,
                             'last seen %.2fs ago' % age if age else 'never'))

        # -- 2: linearization sanity via the dissipation command -------------
        confirm('Step 2 injects 2 A of d-axis current at STANDSTILL for 3 s '
                '(motor warms slightly). Motor must be free to sit still.')
        node.conf_bus_clamp(v_clamp, i_floor=0.0, i_max=5.0)
        t_end = time.monotonic() + 3.0
        last_i = None
        while time.monotonic() < t_end:
            node.set_id_dissipate(2.0, off_delay_s=0.5)
            time.sleep(0.1)
            st = node.telemetry.status_bus_clamp
            if st is not None:
                last_i = st.i_bus
        node.set_id_dissipate(0.0, off_delay_s=0.1)
        # At 2 A the burn is ~0.5 W -> ~0.02 A of i_bus, at the telemetry
        # resolution limit. The assertion is "no backfeed"; the magnitude is
        # informational (the dissipation ladder proves id tracking properly).
        ok = last_i is not None and last_i >= -0.05
        results.append(check('standstill burn does not backfeed', ok,
                             'i_bus %.2f A — compare to 1.5*Rs*id^2/V by hand'
                             % (last_i if last_i is not None else -99)))

        # -- 3b/4: clamp vs unclamped baseline --------------------------------
        # A bench motor's burnable power is small (1.5*Rs*i_max^2 — tens of
        # watts), so a hard brake CAN out-run the clamp. The honest assertion
        # is comparative: same stimulus, clamp off vs on. The baseline is
        # bounded by the stock regen-cut band + the battery catch — both
        # verified present on this bench.
        confirm('Step 4a (BASELINE, everything disarmed): drives at 3 A to '
                'natural top speed, then brakes 5 A. The bus WILL rise '
                'into the regen-cut band (~29 V). Hands clear.')
        node.disarm_bus_clamp()
        base_max, base_min_i, base_erpm, _ = spin_brake(node, 3.0, 5.0, 2.0, samples=50)
        print('     baseline: peak %.0f erpm, max v_bus %.1f V, min i_bus %.2f A'
              % (base_erpm, base_max, base_min_i))
        time.sleep(1.0)

        v_clamp_4 = psu_v + 1.0
        confirm('Step 4b (CLAMP): same stimulus, clamp %.1f V (PSU + 1), '
                'i_max 20 A. Expect engagement and a lower peak than the '
                'baseline. Hands clear.' % v_clamp_4)
        node.conf_bus_clamp(v_clamp_4, i_floor=0.0, i_max=20.0,
                            clamp_en=True, floor_en=False)
        max_v, _min_i, peak_erpm, log = spin_brake(node, 3.0, 5.0, 2.0, samples=50)
        engaged = any(s.clamp_active for s in log)
        id_max = max((s.id_clamp_now for s in log), default=0.0)
        results.append(check('clamp engages during brake', engaged,
                             'peak %.0f erpm, %d samples, id_now max %.1f A' %
                             (peak_erpm, len(log), id_max)))
        # The design criterion: with adequate i_max the clamp HOLDS the bus at
        # its setpoint. (The unclamped baseline is context, not the metric —
        # its ~30 ms spike-fold-drain excursion aliases against 50 Hz
        # telemetry, while a working clamp produces a sustained, always-caught
        # plateau AT the setpoint. An under-provisioned i_max fails this
        # check honestly: the bus rides up to the regen-cut band instead.)
        results.append(check('clamp holds v_bus at setpoint (+2 V)',
                             max_v < v_clamp_4 + 2.0,
                             'max v_bus %.1f V vs setpoint %.1f V (unclamped baseline saw %.1f V)' %
                             (max_v, v_clamp_4, base_max)))

        # -- 5: floor-PI alone — i_bus never goes negative --------------------
        confirm('Step 5 (FLOOR): gentler brake (2 A), floor mode (i_floor = '
                '0), clamp %.1f V as backstop, i_max 20 A. Regen must be '
                'burned as produced — i_bus should not go meaningfully '
                'negative.' % v_clamp)
        node.conf_bus_clamp(v_clamp, i_floor=0.0, i_max=20.0,
                            clamp_en=True, floor_en=True)
        max_v, min_i, peak_erpm, log = spin_brake(node, 3.0, 2.0, 2.0, samples=50)
        floor_seen = any(s.floor_active for s in log)
        results.append(check('floor holds i_bus >= ~0', min_i > -0.5,
                             'min i_bus %.2f A (floor_active seen: %s, peak %.0f erpm)' %
                             (min_i, floor_seen, peak_erpm)))
        results.append(check('bus stays pinned near PSU', max_v < v_clamp,
                             'max v_bus %.1f V' % max_v))

        # -- 6: erpm gate — auto-restart must REFUSE at standstill ------------
        # Quiesce FIRST: modulation from step 5 takes ~0.5 s to release, and
        # an armed clamp on a still-running loop legitimately engages at any
        # speed (the gate only guards the restart path).
        node.disarm_bus_clamp()
        node.set_current(0.0)
        time.sleep(2.5)
        confirm('Step 6 (RESTART GATE): motor stopped and released; v_clamp '
                'is dropped below PSU voltage with allow_start on. The clamp '
                'must NOT start modulation (observer untrustworthy at 0 rpm).')
        node.conf_bus_clamp(max(psu_v - 2.0, 10.0), i_floor=0.0, i_max=3.0,
                            clamp_en=True, floor_en=False,
                            allow_start_modulation=True)
        time.sleep(2.0)
        st = wait_status(node, 1.0)
        ok = st is not None and not st.started_modulation and st.id_clamp_now < 0.1
        results.append(check('erpm gate refuses standstill restart', ok,
                             repr(st) if st else 'no status'))
        node.conf_bus_clamp(v_clamp, i_floor=0.0, i_max=5.0)  # restore

        print('\n== %d/%d passed ==' % (sum(results), len(results)))
        print('Manual steps remaining (see mm_d_axis_dissipation.md):')
        print('  3: baseline pump w/o clamp -> measure effective C, one deliberate OV fault')
        print('  6b: coast restart AT SPEED (spin, release, drop v_clamp)')
        print('  7: degradation chain (i_max ~3 A, hard brake: saturated -> regen cut -> fault)')
        print('  8: endurance + temp foldback + host-kill mid-burn')
        return 0 if all(results) else 1
    finally:
        try:
            if node is not None:
                node.set_current(0.0)
                node.disarm_bus_clamp()
        except Exception:
            pass
        bus.close()


if __name__ == '__main__':
    sys.exit(main())
