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


def spin_brake(node, erpm, brake_a, brake_s, samples):
    """Spin up, then brake with constant current, sampling clamp telemetry
    and STATUS_5 v_in through the stop. Returns (max_vbus, min_ibus, log)."""
    node.set_rpm(erpm)
    time.sleep(2.0)
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
    return max_v, min_i, log


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
        # i_bus should be positive (drawing) while burning at standstill.
        ok = last_i is not None and last_i > 0.0
        results.append(check('standstill burn draws positive i_bus', ok,
                             'i_bus %.2f A — compare to 1.5*Rs*id^2/V by hand'
                             % (last_i if last_i is not None else -99)))

        # -- 4: clamp-PI alone under a braking pulse --------------------------
        confirm('Step 4 (CLAMP): spins to 3000 erpm, then brakes 3 A for 2 s. '
                'Kinetic energy pumps the bus; the clamp must hold v_bus near '
                '%.1f V with no oscillation. Hands clear.' % v_clamp)
        node.conf_bus_clamp(v_clamp, i_floor=0.0, i_max=10.0,
                            clamp_en=True, floor_en=False)
        max_v, _min_i, log = spin_brake(node, 3000, 3.0, 2.0, samples=50)
        engaged = any(s.clamp_active for s in log)
        held = max_v < v_clamp + 2.0
        results.append(check('clamp engages during brake', engaged,
                             '%d samples, id_now max %.1f A' %
                             (len(log), max((s.id_clamp_now for s in log), default=0.0))))
        results.append(check('v_bus held (< v_clamp + 2 V)', held,
                             'max v_bus %.1f V vs clamp %.1f V' % (max_v, v_clamp)))

        # -- 5: floor-PI alone — i_bus never goes negative --------------------
        confirm('Step 5 (FLOOR): same spin/brake, floor mode (i_floor = 0). '
                'Regen must be burned as produced — i_bus should not go '
                'meaningfully negative and v_bus should stay near PSU voltage.')
        node.conf_bus_clamp(v_clamp, i_floor=0.0, i_max=10.0,
                            clamp_en=True, floor_en=True)
        max_v, min_i, log = spin_brake(node, 3000, 3.0, 2.0, samples=50)
        results.append(check('floor holds i_bus >= ~0', min_i > -0.5,
                             'min i_bus %.2f A' % min_i))
        results.append(check('bus stays pinned near PSU', max_v < v_clamp,
                             'max v_bus %.1f V' % max_v))

        # -- 6: erpm gate — auto-restart must REFUSE at standstill ------------
        confirm('Step 6 (RESTART GATE): with the motor STOPPED, v_clamp is '
                'dropped below PSU voltage with allow_start on. The clamp '
                'must NOT start the motor (observer untrustworthy at 0 rpm).')
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
