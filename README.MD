# PyVESC [![Build Status](https://travis-ci.org/LiamBindle/PyVESC.svg?branch=master)](https://travis-ci.org/LiamBindle/PyVESC) [![Documentation Status](https://readthedocs.org/projects/pyvesc/badge/?version=latest)](http://pyvesc.readthedocs.io/en/latest/?badge=latest)
PyVESC is aimed at being a easy to use and robust python implementation of the
communication protocol used by the
[VESC - Open Source ESC](http://vedder.se/2015/01/vesc-open-source-esc/) by
Benjamin Vedder. Its a great project with a really great community and I'd urge
anyone interested in motor controllers to take a look at it.

That being said, if you're here you probably already know about it. This small
project was written by Liam Bindle for the
[University of Saskatchewan Space Design Team](https://usst.ca)
as our primary language for non-embedded system is Python. You might wonder why
you might need a library to handling packing and parsing VESC messages since
Pythons standard
[struct](https://docs.python.org/3.5/library/struct.html)
module is great for almost exaclty this. PyVESC's usefulness comes from the fact
that PyVESC is:
- Well tested
- Robust in handling corrupt packets in a buffer
- Messages are easily extensible so that PyVESC can be used as a generic
  message/codec protocol (ie. at the USST we use PyVESC for sending messages to
  all of our embedded systems)
- Implements a number of common-error catching exceptions to speed up your
  development

## TCP bridge support (this fork)
`VESC()` accepts a `tcp://host[:port]` address (default port 65102) in place of a
serial device, for driving a VESC through a TCP bridge — galvanic isolation from
high-power test rigs. Two bridge options:

**VESC Express** (official, CAN-attached): the Express answers TCP itself and the
motor controller sits behind it on the CAN bus, so pass its CAN ID — every command,
getter, and heartbeat is then forwarded automatically:

```python
with VESC("tcp://192.168.1.50", can_id=100) as vesc:
    print(vesc.get_measurements().v_in)
```

(Find the ID with `COMM_PING_CAN`, or in VESC Tool's CAN device list. The Express
serves one TCP client at a time and takes ~2 s to free the slot after a disconnect,
so don't hammer reconnects.)

**Transparent ESP32 UART bridge** (this repo): dumb pipe to the VESC's UART port,
no `can_id` needed — `VESC("tcp://<esp32-ip>")`. Firmware, wiring, and the
isolation rationale live in [esp32_bridge/](esp32_bridge/).

The serial path is unchanged: `VESC("/dev/ttyACM0")`. Verified end-to-end against
a VESC Express T (FW 6.06) + VESC FW 5.2 over CAN: `bench_check.py tcp://<ip> 100`
11/11, GetValues round-trip median ~21 ms over double-hop WiFi (p95 ~110 ms —
budget for jitter, the driver drops stale responses and keys replies by command id).

## Documentation
For the latest version of PyVESC's documentation [Read The Docs](http://pyvesc.readthedocs.io/en/latest/)

## Contributing
Obviously this is just a quick and dirty project; however, if you have any ideas,
critiques or changes just let me know or submit a pull request.

## License
Just like VESC, PyVESC is distributed under a
[Creative Commons ShareALike 4.0 International License](https://creativecommons.org/licenses/by-sa/4.0/).
