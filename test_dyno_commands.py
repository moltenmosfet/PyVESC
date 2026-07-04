"""
Unit tests for the VESC command set needed for four-quadrant dyno operation.

The absorber VESC must be able to command torque of either sign at speed of
either sign (all four quadrants), hold the rotor, keep the link alive, and
stream telemetry:

  Control:   SetCurrent (signed torque — the primary 4Q command),
             SetCurrentBrake, SetHandbrake (standstill hold),
             SetRPM (signed, speed-hold mode), SetDutyCycle (signed, backup),
             Alive (heartbeat / motor watchdog)
  Telemetry: GetValues (signed rpm/current/duty, temps, fault code),
             GetVersion (handshake)

Wire format and scaling are asserted against the firmware source in
../bldc (comm/commands.c, datatypes.h COMM_PACKET_ID).
"""

import struct
from unittest import TestCase

from crccheck.crc import CrcXmodem

import pyvesc
from pyvesc.messages import (
    VedderCmd,
    SetCurrent, SetCurrentBrake, SetHandbrake, SetRPM, SetDutyCycle, Alive,
    GetValues, GetVersion,
)


def unwrap(packet):
    """Validate a short-format VESC frame and return its payload bytes."""
    assert packet[0] == 2, "expected short-packet start byte"
    payload_len = packet[1]
    payload = packet[2:2 + payload_len]
    crc = struct.unpack('!H', packet[2 + payload_len:4 + payload_len])[0]
    assert crc == CrcXmodem().calc(payload), "bad CRC"
    assert packet[4 + payload_len] == 3, "bad terminator"
    return payload


class TestFourQuadrantSetters(TestCase):
    """Encoding of every control command, in both signs where applicable."""

    def assert_int32_payload(self, msg, expected_id, expected_value):
        payload = unwrap(pyvesc.encode(msg))
        self.assertEqual(payload[0], expected_id)
        self.assertEqual(struct.unpack('!i', payload[1:5])[0], expected_value)
        self.assertEqual(len(payload), 5)

    def test_current_positive_torque(self):
        # quadrant I/III: motoring
        self.assert_int32_payload(SetCurrent(20.0), 6, 20000)

    def test_current_negative_torque(self):
        # quadrant II/IV: regenerative absorption — the dyno's bread and butter
        self.assert_int32_payload(SetCurrent(-20.0), 6, -20000)

    def test_current_brake(self):
        self.assert_int32_payload(SetCurrentBrake(7.5), 7, 7500)

    def test_handbrake(self):
        self.assert_int32_payload(SetHandbrake(3.25), 10, 3250)

    def test_rpm_both_directions(self):
        self.assert_int32_payload(SetRPM(12000), 8, 12000)
        self.assert_int32_payload(SetRPM(-12000), 8, -12000)

    def test_duty_both_directions(self):
        self.assert_int32_payload(SetDutyCycle(0.5), 5, 50000)
        self.assert_int32_payload(SetDutyCycle(-0.5), 5, -50000)

    def test_zero_current_is_release(self):
        self.assert_int32_payload(SetCurrent(0), 6, 0)

    def test_alive_heartbeat(self):
        payload = unwrap(pyvesc.encode(Alive()))
        self.assertEqual(payload, bytes([30]))

    def test_alive_forwarded_over_can(self):
        payload = unwrap(pyvesc.encode(Alive(can_id=42)))
        self.assertEqual(payload, bytes([34, 42, 30]))  # COMM_FORWARD_CAN wrap

    def test_set_current_forwarded_over_can(self):
        payload = unwrap(pyvesc.encode(SetCurrent(-5.0, can_id=42)))
        self.assertEqual(payload[0:2], bytes([34, 42]))
        self.assertEqual(payload[2], 6)
        self.assertEqual(struct.unpack('!i', payload[3:7])[0], -5000)

    def test_signed_round_trip(self):
        msg, consumed, _ = pyvesc.decode(pyvesc.encode(SetCurrent(-15.5)), recv=False)
        self.assertEqual(msg.current, -15.5)


class TestCommandIdsMatchFirmware(TestCase):
    """Guard against enum drift vs bldc/datatypes.h COMM_PACKET_ID."""

    def test_ids(self):
        expected = {
            'COMM_FW_VERSION': 0,
            'COMM_GET_VALUES': 4,
            'COMM_SET_DUTY': 5,
            'COMM_SET_CURRENT': 6,
            'COMM_SET_CURRENT_BRAKE': 7,
            'COMM_SET_RPM': 8,
            'COMM_SET_HANDBRAKE': 10,
            'COMM_ALIVE': 30,
            'COMM_FORWARD_CAN': 34,
        }
        for name, value in expected.items():
            self.assertEqual(getattr(VedderCmd, name), value, name)


class TestTelemetry(TestCase):
    def test_get_values_request(self):
        payload = unwrap(pyvesc.encode_request(GetValues))
        self.assertEqual(payload, bytes([4]))

    def test_get_version_request(self):
        payload = unwrap(pyvesc.encode_request(GetVersion))
        self.assertEqual(payload, bytes([0]))

    def test_get_version_decode(self):
        # firmware replies [id, major, minor, hw_name..., uuid...]; the
        # variable-length tail must be ignored
        payload = bytes([0, 5, 2]) + b'60\x00' + bytes(12)
        from pyvesc.protocol.packet.codec import frame
        msg, consumed, _ = pyvesc.decode(frame(payload))
        self.assertEqual(msg.fw_version_major, 5)
        self.assertEqual(msg.fw_version_minor, 2)
        self.assertEqual(str(msg), "5.2")

    def test_get_values_decode_firmware_layout(self):
        """Decode a response built exactly as commands.c COMM_GET_VALUES packs it
        (FW 6.x, full mask), including the trailing status byte pyvesc must
        tolerate and ignore. Values are signed as they would be in quadrant II
        (spinning reverse, absorbing)."""
        payload = bytes([4]) + struct.pack(
            '!hhiiiihihiiiiiiBiBhhhiiB',
            335,        # temp_fet        33.5 C
            210,        # temp_motor      21.0 C
            -1234,      # avg_motor_current  -12.34 A (absorbing)
            -567,       # avg_input_current   -5.67 A (charging the bus)
            0,          # avg_id
            -1234,      # avg_iq
            -153,       # duty_cycle_now  -0.153
            -8420,      # rpm             -8420 erpm (reverse)
            842,        # v_in            84.2 V
            12345,      # amp_hours       1.2345 Ah
            5000,       # amp_hours_charged 0.5 Ah
            20000,      # watt_hours      2.0 Wh
            10000,      # watt_hours_charged 1.0 Wh
            -123456,    # tachometer
            234567,     # tachometer_abs
            0,          # mc_fault_code   FAULT_CODE_NONE
            12345678,   # pid_pos_now     12.345678
            88,         # controller_id
            301, 302, 303,  # temp_mos1..3  30.1/30.2/30.3 C
            -1234,      # avg_vd          -1.234 V
            45678,      # avg_vq          45.678 V
            0,          # status byte (FW 6.x) — must be ignored
        )
        from pyvesc.protocol.packet.codec import frame
        msg, consumed, _ = pyvesc.decode(frame(payload))

        self.assertAlmostEqual(msg.temp_fet, 33.5)
        self.assertAlmostEqual(msg.temp_motor, 21.0)
        self.assertAlmostEqual(msg.avg_motor_current, -12.34)
        self.assertAlmostEqual(msg.avg_input_current, -5.67)
        self.assertAlmostEqual(msg.duty_cycle_now, -0.153)
        self.assertEqual(msg.rpm, -8420)
        self.assertAlmostEqual(msg.v_in, 84.2)
        self.assertAlmostEqual(msg.amp_hours, 1.2345)
        self.assertEqual(msg.tachometer, -123456)
        self.assertEqual(msg.tachometer_abs, 234567)
        self.assertEqual(msg.mc_fault_code, b'\x00')
        self.assertAlmostEqual(msg.pid_pos_now, 12.345678)
        self.assertEqual(msg.app_controller_id, bytes([88]))
        self.assertAlmostEqual(msg.temp_mos1, 30.1)
        self.assertAlmostEqual(msg.temp_mos3, 30.3)
        self.assertAlmostEqual(msg.avg_vd, -1.234)
        self.assertAlmostEqual(msg.avg_vq, 45.678)
