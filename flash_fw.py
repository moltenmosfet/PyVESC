#!/usr/bin/env python3
"""Flash a VESC firmware .bin over its USB serial port (ChibiOS VCP) via pyvesc.

    ./flash_fw.py <firmware.bin> [serial_port]

Default port: /dev/ttyACM0.

This is the fork's bench flash path for when VESC Tool can't connect — e.g. a
custom-config descriptor that crashes the GUI on connect. USB serial only: it
uses pyvesc's tested erase/write/jump path and gives update_firmware exclusive
access to the port (start_heartbeat=False, no monitor thread). Do NOT try to
flash over the SocketCAN link without a tested CAN uploader.

COMPRESSION IS REQUIRED on hw60: the new-app staging partition is 3 sectors
(384 KB) but the app image is 512 KB, so an uncompressed .bin cannot be staged
(writes run off the partition and the bootloader rejects the image). Pass a
heatshrink-compressed image (window 13 / lookahead 5, the bootloader's build) —
name it *.hs and this tool sets the 0xCC decompress marker automatically. Build
the .hs with the hs_tool encoder (round-trips through the bootloader's own
decoder before emitting). A raw .bin only works on boards whose staging
partition is >= the image.

Safe by default:
  * reads the firmware version first and ABORTS before erasing if the device
    does not respond (never erase a silent/wrong device);
  * the running app is untouched until a successful jump-to-bootloader, so an
    interrupted upload just leaves the old firmware in place (no brick);
  * a trailing USB I/O error at ~100% is usually the board rebooting into the
    bootloader after the jump. It is NOT proof of success.

ALWAYS verify the running firmware afterwards (read fw/config over CAN or USB)
rather than trusting the exit code.
"""
import contextlib
import os
import sys

from pyvesc.VESC import VESC
from pyvesc.firmware import Firmware


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    binpath = sys.argv[1]
    port = sys.argv[2] if len(sys.argv) > 2 else '/dev/ttyACM0'

    lzss = binpath.endswith('.hs')   # heatshrink-compressed -> set 0xCC marker
    fw = Firmware(binpath, lzss=lzss)
    print(f"image: {fw}  ({'COMPRESSED/0xCC' if lzss else 'raw'})", flush=True)

    v = VESC(serial_port=port, start_heartbeat=False, timeout=0.05)
    try:
        ver = v.get_firmware_version()
        if not ver:
            print(f"ABORT: no response on {port} — not erasing.", flush=True)
            return 1
        print(f"connected (fw {getattr(ver, 'comm_fw_version', ver)}). "
              f"erasing + flashing, ~30-90 s...", flush=True)

        def prog(p):
            print(f"  progress: {p}", file=sys.stderr, flush=True)

        try:
            # Silence update_firmware's per-chunk \r spam on stdout; keep our
            # periodic callback on stderr.
            with open(os.devnull, 'w') as dn, contextlib.redirect_stdout(dn):
                ok = v.update_firmware(fw, progress_callback=prog)
            print(f"update_firmware returned: {ok}", flush=True)
        except OSError as e:
            print(f"port I/O error near end ({e}) — likely the reboot into the "
                  f"bootloader after the jump. VERIFY separately.", flush=True)
    finally:
        with contextlib.suppress(Exception):
            v.__exit__(None, None, None)

    print("Done. VERIFY the running firmware over CAN/USB — do NOT trust this "
          "exit code alone.", flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
