# PyVESC — Molten MOSFET fork

Python driver for [VESC](https://vesc-project.com/) motor controllers over serial or
TCP. This is the control layer for the Molten MOSFET dyno: four-quadrant
torque/speed commands, telemetry, and a network transport that keeps the laptop
galvanically isolated from the rig.

Forked from [LiamBindle/PyVESC](https://github.com/LiamBindle/PyVESC) with
[PR #38](https://github.com/LiamBindle/PyVESC/pull/38) (lukedempsey: interface
rework, firmware flashing, motor/app config) merged. The API has diverged from
upstream — the original README and the
[readthedocs documentation](http://pyvesc.readthedocs.io/en/latest/) describe
upstream, not this fork.

## What this fork adds

- **Four-quadrant command set**, hardware-verified: `SetCurrent`,
  `SetCurrentBrake`, `SetHandbrake` (added), `SetRPM`, `SetDutyCycle`, all in
  engineering units (amps, electrical rpm, duty fraction). Protocol fixes for
  FW 5.2: the `GetValues` tail (real `temp_mos1-3`/`avg_vd`/`avg_vq` instead of
  a bogus `time_ms`) and a `GetVersion` off-by-one.
- **TCP transport** — `VESC("tcp://host[:port]")` (default port 65102) in place
  of a serial device, for galvanic isolation from high-power rigs.
- **Instance-wide CAN forwarding** — `VESC(..., can_id=N)` wraps every command,
  getter, and heartbeat in `COMM_FORWARD_CAN`, so a controller behind a bridge
  on its CAN bus (e.g. a VESC Express) drives exactly like a direct one.
- **A request/response path hardened for real WiFi** — stale responses are
  drained before each request, replies are keyed by command id, and every read
  has a deadline so a dead link returns `None` instead of hanging a control loop.
- **Test tooling** — `bench_check.py` (motor-less hardware smoke test, all
  commands zero-valued), TCP loopback tests, and an enum-drift guard that checks
  command ids against the VESC firmware's `datatypes.h`.

## Quickstart

```sh
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt   # crccheck, pyserial
```

Three ways to connect, same API from there on:

```python
from pyvesc.VESC import VESC

# USB serial, direct
with VESC("/dev/ttyACM0") as vesc:
    ...

# transparent ESP32 UART bridge (see esp32_bridge/)
with VESC("tcp://192.168.1.50") as vesc:
    ...

# VESC Express: the Express answers TCP, the motor controller
# sits behind it on CAN — pass the controller's CAN ID
with VESC("tcp://192.168.1.50", can_id=100) as vesc:
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
./.venv/bin/python bench_check.py /dev/ttyACM0          # USB
./.venv/bin/python bench_check.py tcp://<bridge-ip>      # ESP32 UART bridge
./.venv/bin/python bench_check.py tcp://<express-ip> 100 # VESC Express + CAN ID
```

### Setup notes

- **Failsafe:** the `VESC` class heartbeats at 10 Hz; the VESC's own app timeout
  (default 1000 ms) cuts output when the link dies. That chain is the safety
  story for wireless control — leave the timeout at its default and fix flaky
  links instead of raising it.
- **ESP32 UART bridge:** VESC App Settings → App to Use `UART`, baud 115200.
  Wiring, flashing, and the isolation rationale (power the ESP32 from the VESC,
  never from the laptop) are in [esp32_bridge/](esp32_bridge/).
- **VESC Express:** find the controller's CAN ID in VESC Tool's CAN device list
  or via `COMM_PING_CAN`. The Express serves one TCP client at a time and takes
  ~2 s to free the slot after a disconnect, so don't hammer reconnects.

## Verified hardware status

| Path | Setup | Result |
| --- | --- | --- |
| USB serial | VESC 6-class, FW 5.2, 12 V bench supply | `bench_check` 11/11 |
| TCP via VESC Express + CAN | Express T (FW 6.06) on WiFi, VESC FW 5.2 at CAN ID 100 | `bench_check` 11/11; `GetValues` round-trip median ~21 ms, p95 ~110 ms, 0 misses in 100 polls (double-hop WiFi) |
| TCP via ESP32 UART bridge | — | sketch not yet run on hardware |

Not yet exercised (needs a motor on the bench): actual spinning, current
tracking, regen. Every hardware test so far sends zero-valued commands only.

## Testing

```sh
./.venv/bin/python -m unittest test                # protocol + TCP loopback
./.venv/bin/python -m unittest test_dyno_commands  # 4Q commands + enum-drift guard
./.venv/bin/python bench_check.py [port] [can_id]  # on real hardware
```

## License & attribution

PyVESC was written by [Liam Bindle](https://github.com/LiamBindle/PyVESC) for the
University of Saskatchewan Space Design Team; the firmware-flashing and config
interface came from lukedempsey's PR #38. Like VESC itself, this fork remains
under the
[Creative Commons Attribution-ShareAlike 4.0 International License](https://creativecommons.org/licenses/by-sa/4.0/).
