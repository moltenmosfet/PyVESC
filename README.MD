# PyVESC — Molten MOSFET fork

Python driver for [VESC](https://vesc-project.com/) motor controllers, two ways in:

- **Native CAN** (`pyvesc.can`) — the dyno runtime link: single-frame commands,
  push-based STATUS telemetry, per-run config envelopes, over SocketCAN.
- **COMM protocol** (serial or TCP) — the bench-setup path: full config,
  firmware flashing, terminal, and the four-quadrant command set.

Forked from [LiamBindle/PyVESC](https://github.com/LiamBindle/PyVESC) with
[PR #38](https://github.com/LiamBindle/PyVESC/pull/38) (lukedempsey: interface
rework, firmware flashing, motor/app config) merged. The API has diverged from
upstream — the original README and the
[readthedocs documentation](http://pyvesc.readthedocs.io/en/latest/) describe
upstream, not this fork.

## What this fork adds

- **Native VESC CAN layer** (`pyvesc.can`, needs `pip install python-can`) —
  frame-exact encoders for the command + `CONF_*` packets, STATUS_1–6 decode
  into a live thread-safe telemetry snapshot per controller id, PING/PONG bus
  enumeration, and candump logging of both directions (the immutable
  stimulus+response record of a run). Wire formats verified against bldc
  fw 7.00 source, with a live enum-drift guard against `../bldc/datatypes.h`.
  Two fw quirks handled that the docs don't mention: the off-delay field is
  *prepended* on `SET_CURRENT` but *appended* on `SET_CURRENT_REL`, and
  `CONF_FOC_ERPMS` is a sensorless threshold, **not** a speed limit.

  ```python
  from pyvesc.can import VescCanBus, VescCanNode

  with VescCanBus(channel="can0") as bus:      # SocketCAN
      absorber = VescCanNode(bus, 10)
      absorber.conf_current_limits_in(-20.0, 80.0)   # absorption ceiling, RAM only
      absorber.set_current(-15.0, off_delay_s=0.2)   # brake, auto-zero in 200 ms
      print(absorber.telemetry.status1.rpm, absorber.telemetry.status5.v_in)
  ```

- **Four-quadrant command set**, hardware-verified: `SetCurrent`,
  `SetCurrentBrake`, `SetHandbrake` (added), `SetRPM`, `SetDutyCycle`, all in
  engineering units (amps, electrical rpm, duty fraction). Protocol fixes for
  FW 5.2: the `GetValues` tail (real `temp_mos1-3`/`avg_vd`/`avg_vq` instead of
  a bogus `time_ms`) and a `GetVersion` off-by-one.
- **TCP transport** — `VESC("tcp://host[:port]")` (default port 65102) in place
  of a serial device, for galvanic isolation from high-power rigs. Built for
  VESC Express; any transparent TCP↔serial bridge (ser2net, ESP-Link) works too.
- **Instance-wide CAN forwarding** — `VESC(..., can_id=N)` wraps every command,
  getter, and heartbeat in `COMM_FORWARD_CAN`, so a controller behind a bridge
  on its CAN bus (e.g. a VESC Express) drives exactly like a direct one.
- **A request/response path hardened for real WiFi** — stale responses are
  drained before each request, replies are keyed by command id, and every read
  has a deadline so a dead link returns `None` instead of hanging a control loop.
- **Test tooling** — `bench_check.py` (COMM) and `can_bench_check.py` (native
  CAN) motor-less hardware smoke tests, TCP loopback tests, and live enum-drift
  guards that check both protocols' ids against the VESC firmware's
  `datatypes.h`.

## Quickstart

```sh
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt   # crccheck, pyserial, python-can
```

### Native CAN (the control-loop path)

```sh
sudo ip link set can0 up type can bitrate 500000   # after every USB replug
./.venv/bin/python can_bench_check.py can0         # scans, checks, RTT stats
```

```python
from pyvesc.can import VescCanBus, VescCanNode

with VescCanBus(channel="can0") as bus:
    node = VescCanNode(bus, bus.scan(range(16)).popitem()[0])  # or known id
    node.conf_current_limits_in(-20.0, 80.0)   # absorption ceiling (RAM only)
    node.set_current(5.0, off_delay_s=0.2)     # 5 A torque, auto-zero in 200 ms
    t = node.telemetry                          # push telemetry, never blocks
    print(t.status1.rpm, t.status4.current_in, t.status5.v_in)
```

### COMM protocol (the bench-setup path)

Three ways to connect, same API from there on:

```python
from pyvesc.VESC import VESC

# USB serial, direct
with VESC("/dev/ttyACM0") as vesc:
    ...

# VESC Express: the Express answers TCP, the motor controller
# sits behind it on CAN — pass the controller's CAN ID
with VESC("tcp://192.168.1.50", can_id=100) as vesc:
    ...

# any transparent TCP<->serial bridge to the VESC's UART port: no can_id
with VESC("tcp://192.168.1.50") as vesc:
    ...
```

```python
vesc.set_current(5.0)         # motor current in amps (negative = regen braking)
vesc.set_rpm(3000)            # electrical rpm
m = vesc.get_measurements()   # v_in, currents, temps, rpm, duty, fault code
print(m.v_in, m.temp_fet, m.mc_fault_code)
```

Smoke-test a rig before trusting it (safe with no motor attached):

```sh
./.venv/bin/python bench_check.py /dev/ttyACM0            # USB
./.venv/bin/python bench_check.py tcp://<express-ip> 100  # VESC Express + CAN ID
```

### Setup notes

- **CAN prerequisites:** enable STATUS messages 1–6 in VESC Tool → App
  Settings → General (STATUS_5 carries bus voltage + tacho; without it the
  telemetry snapshot has no `v_in`). Broadcast rate ~50 Hz default — raise it
  for fast control loops. A dead/unplugged bus jams the socketcan TX queue
  (qlen 10); `ping()`/`scan()` handle it, but consider
  `sudo ip link set can0 txqueuelen 1000`.
- **Failsafe:** the `VESC` class heartbeats at 10 Hz; the VESC's own app timeout
  (default 1000 ms) cuts output when the link dies. That chain is the safety
  story for wireless control — leave the timeout at its default and fix flaky
  links instead of raising it.
- **Getting the TCP link — use a VESC Express.** The
  [VESC Express](https://github.com/vedderb/vesc_express) is the VESC project's
  official ESP32-C3 WiFi/BLE/CAN module (sold as Vesc Express on aliexpress or via Trampa, etc..);
  it exposes the TCP server this driver speaks, on port 65102. One-time
  setup: wire it to the motor controller's CAN port, connect to it with VESC Tool
  (USB or BLE), and under App Settings → WiFi set station mode with your network's
  credentials. Give its MAC a DHCP reservation in the router so the IP survives
  reboots. Then `VESC("tcp://<express-ip>", can_id=<controller id>)`.
- **VESC Express quirks:** find the controller's CAN ID in VESC Tool's CAN device
  list or via `COMM_PING_CAN`. The Express serves one TCP client at a time and
  takes ~2 s to free the slot after a disconnect, so don't hammer reconnects.
  Power it from the CAN connector — a USB cable to the laptop re-creates the
  exact ground path the wireless link exists to break.

## Verified hardware status

| Path | Setup | Result |
| --- | --- | --- |
| USB serial | VESC 6-class, FW 5.2, 12 V bench supply | `bench_check` 11/11 |
| TCP via VESC Express + CAN | Express T on WiFi, VESC **FW 7.00** at CAN ID 100 (re-validated 2026-07-07; first pass was FW 5.2) | `bench_check` 11/11 — the FW 5.2-era `GetValues` decode survives 7.00; RTT med 11 ms / p95 173 ms, 0 misses in 30; **motorized**: `SetRPM(4000)` spun the motor over TCP, stop clean |
| **Native CAN (`pyvesc.can`)** | Mini FSESC 6.7 Pro, **FW 7.00**, CAN ID 100, PCAN-USB @ 500k, e-scooter motor | `can_bench_check` 11/11 (STATUS_5 enabled in app config); PING RTT min 0.77 / med 0.99 / p95 1.55 ms; **live drive verified 2026-07-07**: SET_CURRENT tracked 2.00 A and spun the motor; **off-delay auto-zeroed** 0.5 s after a single send; **CONF_CURRENT_LIMITS applied** (5 A request clamped at exactly 2.00 A); SET_RPM held 3978/4000 erpm; broadcast `stop_all` works |

Both paths are motor-exercised, all six STATUS broadcasts decode at ~48 Hz,
and **regen is verified using rotor inertia as the load** (2026-07-07,
motor clamped): `SET_CURRENT_BRAKE` from 9.8k erpm drove `current_in` to
**−3.3 A into the supply**, tapering with speed; negative `SET_CURRENT`
braked through zero into reverse drive (true 4-quadrant semantics: current
sign = torque, not speed); and the **absorption ceiling works** —
`conf_current_limits_in(-1, 80)` clamped regen at −1.1 A vs −3.3 A
unclamped. Nothing in the driver stack remains hardware-unverified;
sustained high-power regen awaits the rig's dump path.

## Testing

```sh
./.venv/bin/python -m unittest test                # COMM protocol + TCP loopback
./.venv/bin/python -m unittest test_dyno_commands  # 4Q commands + COMM enum guard
./.venv/bin/python -m unittest test_can_frames     # CAN golden bytes + enum guard
./.venv/bin/python -m unittest test_can_bus        # CAN bus/node over virtual interface
./.venv/bin/python bench_check.py [port] [can_id]  # COMM path, real hardware
./.venv/bin/python can_bench_check.py [chan] [id]  # native CAN, real hardware
```

## License & attribution

PyVESC was written by [Liam Bindle](https://github.com/LiamBindle/PyVESC) for the
University of Saskatchewan Space Design Team; the firmware-flashing and config
interface came from lukedempsey's PR #38. Like VESC itself, this fork remains
under the
[Creative Commons Attribution-ShareAlike 4.0 International License](https://creativecommons.org/licenses/by-sa/4.0/).
