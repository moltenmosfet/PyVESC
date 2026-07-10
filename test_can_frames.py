"""
Unit tests for pyvesc.can.frames — golden wire bytes hand-computed from
bldc fw 7.00 source (comm/comm_can.c), plus an enum drift guard that parses
the live ../vesc_firmware/datatypes.h (moltenmosfet fork) when present.

Run: ./.venv/bin/python -m unittest test_can_frames
"""

import os
import re
import struct
import unittest

from pyvesc.can import frames
from pyvesc.can.frames import (CanPacketId, Pong, Status1, Status2, Status3,
                               Status4, Status5, Status6, decode_frame,
                               make_arbitration_id, split_arbitration_id)

BLDC_DATATYPES = os.path.join(os.path.dirname(__file__), '..', 'vesc_firmware', 'datatypes.h')


class TestArbitrationId(unittest.TestCase):
    def test_make_and_split(self):
        arb = make_arbitration_id(CanPacketId.SET_CURRENT, 42)
        self.assertEqual(arb, 0x12A)
        self.assertEqual(split_arbitration_id(arb), (1, 42))

    def test_controller_id_range(self):
        with self.assertRaises(ValueError):
            make_arbitration_id(CanPacketId.SET_CURRENT, 256)


class TestCommandGoldenBytes(unittest.TestCase):
    """Every byte hand-derived from the fw decode path."""

    def test_set_duty(self):
        arb, data = frames.encode_set_duty(1, 0.53)
        self.assertEqual(arb, (0 << 8) | 1)
        self.assertEqual(data, struct.pack('!i', 53000))

    def test_set_current_positive(self):
        arb, data = frames.encode_set_current(42, 5.0)
        self.assertEqual(arb, 0x12A)
        self.assertEqual(data, bytes([0x00, 0x00, 0x13, 0x88]))  # 5000

    def test_set_current_negative(self):
        _, data = frames.encode_set_current(42, -15.5)
        self.assertEqual(data, struct.pack('!i', -15500))

    def test_set_current_off_delay_is_prepended(self):
        # fw: if len >= 6, off-delay f16(x1e3) is read FIRST, then current f32
        _, data = frames.encode_set_current(42, 5.0, off_delay_s=0.5)
        self.assertEqual(len(data), 6)
        self.assertEqual(data[0:2], struct.pack('!h', 500))     # off-delay first
        self.assertEqual(data[2:6], struct.pack('!i', 5000))    # then current

    def test_set_current_rel_off_delay_is_appended(self):
        # fw: current_rel f32(x1e5) first, THEN off-delay f16(x1e3)
        _, data = frames.encode_set_current_rel(5, 0.5, off_delay_s=0.5)
        self.assertEqual(len(data), 6)
        self.assertEqual(data[0:4], struct.pack('!i', 50000))   # rel first
        self.assertEqual(data[4:6], struct.pack('!h', 500))     # off-delay last

    def test_set_current_brake(self):
        arb, data = frames.encode_set_current_brake(3, 12.0)
        self.assertEqual(arb, (2 << 8) | 3)
        self.assertEqual(data, struct.pack('!i', 12000))

    def test_set_rpm(self):
        arb, data = frames.encode_set_rpm(7, -4500)
        self.assertEqual(arb, (3 << 8) | 7)
        self.assertEqual(data, struct.pack('!i', -4500))

    def test_set_pos(self):
        arb, data = frames.encode_set_pos(9, 123.456)
        self.assertEqual(arb, (4 << 8) | 9)
        self.assertEqual(data, struct.pack('!i', 123456000))

    def test_set_current_rel_full_scale(self):
        _, data = frames.encode_set_current_rel(5, -1.0)
        self.assertEqual(data, struct.pack('!i', -100000))

    def test_set_handbrake(self):
        arb, data = frames.encode_set_handbrake(2, 3.0)
        self.assertEqual(arb, (12 << 8) | 2)
        self.assertEqual(data, struct.pack('!i', 3000))

    def test_rel_range_validated(self):
        for bad in (1.01, -1.01, 2.0):
            with self.assertRaises(ValueError):
                frames.encode_set_current_rel(1, bad)
            with self.assertRaises(ValueError):
                frames.encode_set_duty(1, bad)

    def test_off_delay_range_validated(self):
        with self.assertRaises(ValueError):
            frames.encode_set_current(1, 0.0, off_delay_s=-0.1)
        with self.assertRaises(ValueError):
            frames.encode_set_current(1, 0.0, off_delay_s=40.0)


class TestConfGoldenBytes(unittest.TestCase):
    def test_current_limits(self):
        arb, data = frames.encode_conf_current_limits(1, -60.0, 120.0)
        self.assertEqual(arb, (21 << 8) | 1)
        self.assertEqual(data, struct.pack('!ii', -60000, 120000))

    def test_current_limits_store_variant(self):
        arb, _ = frames.encode_conf_current_limits(1, -60.0, 120.0, store=True)
        self.assertEqual(arb >> 8, 22)

    def test_current_limits_in(self):
        arb, data = frames.encode_conf_current_limits_in(1, -20.0, 80.0)
        self.assertEqual(arb, (23 << 8) | 1)
        self.assertEqual(data, struct.pack('!ii', -20000, 80000))

    def test_battery_cut(self):
        arb, data = frames.encode_conf_battery_cut(1, 60.0, 56.0)
        self.assertEqual(arb, (29 << 8) | 1)
        self.assertEqual(data, struct.pack('!ii', 60000, 56000))

    def test_foc_erpms(self):
        arb, data = frames.encode_conf_foc_erpms(1, 700.0, 2500.0)
        self.assertEqual(arb, (25 << 8) | 1)
        self.assertEqual(data, struct.pack('!ii', 700000, 2500000))

    def test_ping(self):
        arb, data = frames.encode_ping(100, 0xFE)
        self.assertEqual(arb, (17 << 8) | 100)
        self.assertEqual(data, bytes([0xFE]))


class TestStatusDecode(unittest.TestCase):
    """Frames constructed exactly as comm_can_send_statusN builds them."""

    def test_status1(self):
        data = struct.pack('!ihh', -4500, -125, 530)
        cid, s = decode_frame((9 << 8) | 100, data)
        self.assertEqual(cid, 100)
        self.assertIsInstance(s, Status1)
        self.assertEqual(s.rpm, -4500.0)
        self.assertAlmostEqual(s.current, -12.5)
        self.assertAlmostEqual(s.duty, 0.53)

    def test_status2(self):
        data = struct.pack('!ii', 12345, 5000)
        _, s = decode_frame((14 << 8) | 1, data)
        self.assertIsInstance(s, Status2)
        self.assertAlmostEqual(s.amp_hours, 1.2345)
        self.assertAlmostEqual(s.amp_hours_charged, 0.5)

    def test_status3(self):
        data = struct.pack('!ii', 98765, 43210)
        _, s = decode_frame((15 << 8) | 1, data)
        self.assertIsInstance(s, Status3)
        self.assertAlmostEqual(s.watt_hours, 9.8765)
        self.assertAlmostEqual(s.watt_hours_charged, 4.3210)

    def test_status4(self):
        data = struct.pack('!hhhh', 635, 482, -87, 6170)
        cid, s = decode_frame((16 << 8) | 7, data)
        self.assertEqual(cid, 7)
        self.assertIsInstance(s, Status4)
        self.assertAlmostEqual(s.temp_fet, 63.5)
        self.assertAlmostEqual(s.temp_motor, 48.2)
        self.assertAlmostEqual(s.current_in, -8.7)
        self.assertAlmostEqual(s.pid_pos, 123.4)

    def test_status5_with_reserved_tail(self):
        data = struct.pack('!ihh', 987654, 842, 0)  # fw sends 8 bytes
        _, s = decode_frame((27 << 8) | 1, data)
        self.assertIsInstance(s, Status5)
        self.assertEqual(s.tachometer, 987654)
        self.assertAlmostEqual(s.v_in, 84.2)

    def test_status6(self):
        data = struct.pack('!hhhh', 1234, 2345, 50, 500)
        _, s = decode_frame((58 << 8) | 1, data)
        self.assertIsInstance(s, Status6)
        self.assertAlmostEqual(s.adc1, 1.234)
        self.assertAlmostEqual(s.ppm, 0.5)

    def test_pong(self):
        # PONG arb controller-field = addressee; payload = [responder, hw_type]
        cid, p = decode_frame((18 << 8) | 0xFE, bytes([100, 0]))
        self.assertEqual(cid, 0xFE)
        self.assertIsInstance(p, Pong)
        self.assertEqual(p.controller_id, 100)
        self.assertEqual(p.hw_type, 0)

    def test_commands_and_unknown_ids_return_none(self):
        self.assertIsNone(decode_frame((1 << 8) | 5, struct.pack('!i', 1000)))
        self.assertIsNone(decode_frame((200 << 8) | 5, b'\x00' * 8))

    def test_short_frame_returns_none(self):
        self.assertIsNone(decode_frame((9 << 8) | 1, b'\x00' * 4))


class TestEnumMatchesFirmware(unittest.TestCase):
    """Drift guard: CanPacketId vs the live bldc checkout when present."""

    def test_hardcoded_anchor_values(self):
        # Anchors survive even without a bldc checkout on disk.
        expected = {
            'SET_DUTY': 0, 'SET_CURRENT': 1, 'SET_CURRENT_BRAKE': 2,
            'SET_RPM': 3, 'SET_POS': 4, 'STATUS': 9, 'SET_CURRENT_REL': 10,
            'SET_CURRENT_HANDBRAKE': 12, 'STATUS_2': 14, 'STATUS_3': 15,
            'STATUS_4': 16, 'PING': 17, 'PONG': 18,
            'CONF_CURRENT_LIMITS': 21, 'CONF_STORE_CURRENT_LIMITS': 22,
            'CONF_CURRENT_LIMITS_IN': 23, 'CONF_STORE_CURRENT_LIMITS_IN': 24,
            'CONF_FOC_ERPMS': 25, 'CONF_STORE_FOC_ERPMS': 26, 'STATUS_5': 27,
            'CONF_BATTERY_CUT': 29, 'CONF_STORE_BATTERY_CUT': 30,
            'SHUTDOWN': 31, 'STATUS_6': 58,
            # Molten MOSFET fork private block
            'MM_SET_ID_DISSIPATE': 200, 'MM_STATUS_DISSIPATION': 201,
            'MM_CONF_BUS_CLAMP': 202, 'MM_STATUS_BUS_CLAMP': 203,
        }
        for name, value in expected.items():
            self.assertEqual(getattr(CanPacketId, name), value, name)

    @unittest.skipUnless(os.path.exists(BLDC_DATATYPES),
                         "no bldc checkout at ../bldc")
    def test_against_live_datatypes_h(self):
        with open(BLDC_DATATYPES) as f:
            src = f.read()
        # Pull explicit `CAN_PACKET_X = N` assignments from the enum.
        fw = {m.group(1): int(m.group(2))
              for m in re.finditer(r'CAN_PACKET_(\w+)\s*=\s*(\d+)', src)}
        self.assertGreater(len(fw), 30, "datatypes.h parse failed")
        for member in CanPacketId:
            self.assertIn(member.name, fw, member.name)
            self.assertEqual(member.value, fw[member.name], member.name)


class TestSetIdDissipate(unittest.TestCase):
    """Molten MOSFET fork command. Golden bytes hand-derived from the fork's
    comm_can.c decode: [current f32x1e3][off_delay f16x1e3], both mandatory."""

    def test_golden_bytes(self):
        arb, data = frames.encode_set_id_dissipate(100, 5.0, 0.5)
        self.assertEqual(arb, (200 << 8) | 100)
        self.assertEqual(data, struct.pack('!i', 5000) + struct.pack('!h', 500))

    def test_off_delay_is_appended_and_mandatory(self):
        _, data = frames.encode_set_id_dissipate(1, 12.5, 5.0)
        self.assertEqual(len(data), 6)                       # never a 4-byte frame
        self.assertEqual(data[0:4], struct.pack('!i', 12500))
        self.assertEqual(data[4:6], struct.pack('!h', 5000))

    def test_rejects_negative_current(self):
        with self.assertRaises(ValueError):
            frames.encode_set_id_dissipate(1, -1.0, 0.5)

    def test_rejects_off_delay_outside_firmware_cap(self):
        with self.assertRaises(ValueError):
            frames.encode_set_id_dissipate(1, 1.0, 0.0)
        with self.assertRaises(ValueError):
            frames.encode_set_id_dissipate(1, 1.0, 5.1)


class TestStatusDissipationDecode(unittest.TestCase):
    def test_golden_decode(self):
        # id=-4.2A, iq=10.0A, diss_now=4.2A, p_copper=137W
        data = (struct.pack('!h', -42) + struct.pack('!h', 100) +
                struct.pack('!h', 42) + struct.pack('!H', 137))
        got = decode_frame((201 << 8) | 100, data)
        self.assertIsNotNone(got)
        controller_id, st = got
        self.assertEqual(controller_id, 100)
        self.assertIsInstance(st, frames.StatusDissipation)
        self.assertAlmostEqual(st.id_meas, -4.2)
        self.assertAlmostEqual(st.iq_meas, 10.0)
        self.assertAlmostEqual(st.id_diss_now, 4.2)
        self.assertEqual(st.p_copper, 137.0)

    def test_short_frame_returns_none(self):
        self.assertIsNone(decode_frame((201 << 8) | 100, b'\x00' * 6))


class TestConfBusClamp(unittest.TestCase):
    """Molten MOSFET fork command. Golden bytes hand-derived from the fork's
    comm_can.c decode: [v_clamp f16x10][i_floor f16x100][i_max f16x10][flags u8]."""

    def test_golden_bytes(self):
        arb, data = frames.encode_conf_bus_clamp(100, 48.0, i_floor=0.5, i_max=10.0)
        self.assertEqual(arb, (202 << 8) | 100)
        self.assertEqual(data, struct.pack('!h', 480) + struct.pack('!h', 50)
                         + struct.pack('!h', 100) + bytes([0x03]))  # clamp|floor

    def test_flags_bits(self):
        _, data = frames.encode_conf_bus_clamp(1, 30.0, floor_en=False,
                                               allow_start_modulation=True)
        self.assertEqual(data[6], 0x05)  # clamp | allow_start

    def test_disarm_is_all_zero(self):
        arb, data = frames.encode_bus_clamp_disarm(100)
        self.assertEqual(arb, (202 << 8) | 100)
        self.assertEqual(data, b'\x00' * 7)

    def test_rejects_nonpositive_v_clamp_and_all_disabled(self):
        with self.assertRaises(ValueError):
            frames.encode_conf_bus_clamp(1, 0.0)
        with self.assertRaises(ValueError):
            frames.encode_conf_bus_clamp(1, 48.0, clamp_en=False, floor_en=False)


class TestStatusBusClampDecode(unittest.TestCase):
    def test_golden_decode(self):
        # v=48.2 V, i_bus=-1.25 A, id_now=7.5 A, armed+clamp_active+saturated
        data = (struct.pack('!h', 482) + struct.pack('!h', -125) +
                struct.pack('!h', 75) + bytes([0x01 | 0x02 | 0x08]))
        got = decode_frame((203 << 8) | 100, data)
        self.assertIsNotNone(got)
        controller_id, st = got
        self.assertEqual(controller_id, 100)
        self.assertIsInstance(st, frames.StatusBusClamp)
        self.assertAlmostEqual(st.v_bus, 48.2)
        self.assertAlmostEqual(st.i_bus, -1.25)
        self.assertAlmostEqual(st.id_clamp_now, 7.5)
        self.assertTrue(st.armed)
        self.assertTrue(st.clamp_active)
        self.assertFalse(st.floor_active)
        self.assertTrue(st.saturated)
        self.assertFalse(st.started_modulation)

    def test_short_frame_returns_none(self):
        self.assertIsNone(decode_frame((203 << 8) | 100, b'\x00' * 6))


class TestCommTunnelEncode(unittest.TestCase):
    def test_short_payload_single_frame(self):
        out = frames.encode_comm_frames(100, 0xFE, b'\x00')  # COMM_FW_VERSION
        self.assertEqual(len(out), 1)
        arb, data = out[0]
        self.assertEqual(arb, (8 << 8) | 100)          # PROCESS_SHORT_BUFFER
        self.assertEqual(data, bytes([0xFE, 0, 0x00]))

    def test_long_payload_chunked_with_crc(self):
        payload = bytes(range(20))                      # > 6 -> FILL path
        out = frames.encode_comm_frames(7, 0xFE, payload)
        # 20 bytes -> 3 FILL chunks (7+7+6) + PROCESS_RX_BUFFER
        self.assertEqual(len(out), 4)
        self.assertEqual(out[0].arbitration_id, (5 << 8) | 7)
        self.assertEqual(out[0].data, bytes([0]) + payload[0:7])
        self.assertEqual(out[1].data, bytes([7]) + payload[7:14])
        self.assertEqual(out[2].data, bytes([14]) + payload[14:20])
        fin = out[3]
        self.assertEqual(fin.arbitration_id, (7 << 8) | 7)
        self.assertEqual(fin.data, struct.pack(
            '!BBHH', 0xFE, 0, 20, frames.crc16_xmodem(payload)))

    def test_very_long_payload_uses_long_chunks(self):
        payload = bytes(300)
        out = frames.encode_comm_frames(7, 0xFE, payload)
        # offsets 0..252 in 7-byte FILLs (37 frames, ends at 259 > 255? no:
        # loop takes chunks while i <= 255: last starts at 252 -> covers 259)
        long_frames = [f for f in out
                       if f.arbitration_id >> 8 == 6]   # FILL_RX_BUFFER_LONG
        self.assertTrue(long_frames)
        off = struct.unpack('!H', long_frames[0].data[:2])[0]
        self.assertGreater(off, 255)
        # reassemble everything and confirm byte-exactness
        buf = bytearray(400)
        for arb, data in out:
            pid = arb >> 8
            if pid == 5:
                buf[data[0]:data[0] + len(data) - 1] = data[1:]
            elif pid == 6:
                o = struct.unpack('!H', data[:2])[0]
                buf[o:o + len(data) - 2] = data[2:]
        self.assertEqual(bytes(buf[:300]), payload)


if __name__ == '__main__':
    unittest.main()
