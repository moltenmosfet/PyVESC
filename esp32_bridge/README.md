# ESP32 TCP↔UART bridge for the dyno VESC

Galvanic isolation between the laptop and a multi-kW dyno: the only link is
WiFi. The ESP32 runs a transparent TCP↔UART bridge; `pyvesc` connects to it
with a `tcp://` address instead of a serial device.

```
LAPTOP ── WiFi ── router ── WiFi ── ESP32 ── UART ── VESC
```

```python
from pyvesc.VESC import VESC

with VESC("tcp://192.168.1.50") as vesc:   # port defaults to 65102
    print(vesc.get_measurements().v_in)
```

## Wiring

| ESP32           | VESC UART port (PH2.0 / JST) |
| --------------- | ---------------------------- |
| GPIO16 (RX2)    | TX                           |
| GPIO17 (TX2)    | RX                           |
| GND             | GND                          |
| 5V (VIN)        | 5V                           |

Both sides are 3.3 V logic — direct connection, no level shifter.

**Power the ESP32 from the VESC's 5 V pin.** Powering it from a USB cable to
the laptop re-creates the exact ground path this bridge exists to break. If
you need USB for the debug console during bring-up, do it with the dyno
powered down.

## VESC configuration

VESC Tool → App Settings → General:
- **App to Use:** `UART` (or `PPM and UART` / `ADC and UART`)
- **Baudrate:** 115200 (must match `UART_BAUD` in the sketch)

Leave the timeout at its default (1000 ms). It is the failsafe: `pyvesc`
heartbeats at 10 Hz, and if WiFi drops for more than a second the VESC cuts
output on its own. Do not raise the timeout to "fix" a flaky link — fix the
link.

## Flashing

1. Arduino IDE with the `esp32` board package (3.x; for 2.x cores replace
   `server.accept()` with `server.available()`).
2. Open `esp32_vesc_bridge/esp32_vesc_bridge.ino`, fill in `WIFI_SSID` /
   `WIFI_PASS`.
3. Board: your ESP32 dev module. Flash, then read the assigned IP off the
   serial console at 115200 baud (or give the ESP32's MAC a DHCP reservation
   in the router so the address survives reboots).

The onboard LED (GPIO2) lights while a TCP client is connected.

## Behavior notes

- One client at a time; a **new connection replaces the old one**, so a
  crashed script can't lock you out — just reconnect.
- WiFi modem sleep is disabled in the sketch (`WiFi.setSleep(false)`);
  leaving it on causes 100 ms+ latency spikes that make telemetry polling
  choppy.
- The bridge is a dumb pipe. Everything protocol-level (framing, CRC,
  heartbeats, timeouts) is handled by pyvesc and the VESC themselves, which
  also means VESC Tool's own TCP connection (Connection → TCP) works through
  it for tuning.

## Alternative: VESC Express

If you'd rather not maintain a sketch, the official
[VESC Express](https://github.com/vedderb/vesc_express) firmware on an
ESP32-C3 provides the same TCP bridge (same port, 65102) plus CAN and
BLE, and is maintained by the VESC project. This sketch exists because it
runs on any classic ESP32 lying around and is ~100 lines you can audit.
