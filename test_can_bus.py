"""
Integration tests for pyvesc.can.bus/node over python-can's in-process
``virtual`` interface, with a fake VESC on the other end.

Run: ./.venv/bin/python -m unittest test_can_bus
"""

import os
import struct
import tempfile
import threading
import time
import unittest

import can

from pyvesc.can import frames
from pyvesc.can.bus import HOST_ID, VescCanBus
from pyvesc.can.node import VescCanNode

CHANNEL = 'pyvesc_can_test'


def wait_for(predicate, timeout=2.0, interval=0.005):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


class FakeVesc:
    """Minimal VESC-on-the-bus: answers PING, records commands, and can
    broadcast STATUS frames on demand."""

    def __init__(self, channel: str, controller_id: int):
        self.controller_id = controller_id
        self.bus = can.Bus(interface='virtual', channel=channel)
        self.received = []  # (packet_id, controller_id, bytes)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=0.05)
            if msg is None or not msg.is_extended_id:
                continue
            packet_id, cid = frames.split_arbitration_id(msg.arbitration_id)
            if cid not in (self.controller_id, 0xFF):
                continue
            self.received.append((packet_id, cid, bytes(msg.data)))
            if packet_id == frames.CanPacketId.PING:
                sender = msg.data[0]
                pong = can.Message(
                    arbitration_id=(int(frames.CanPacketId.PONG) << 8) | sender,
                    data=bytes([self.controller_id, 0]), is_extended_id=True)
                self.bus.send(pong)

    def send_status1(self, rpm: int, current_a: float, duty: float):
        data = struct.pack('!ihh', rpm, int(current_a * 10), int(duty * 1000))
        self.bus.send(can.Message(
            arbitration_id=(int(frames.CanPacketId.STATUS) << 8) | self.controller_id,
            data=data, is_extended_id=True))

    def send_status5(self, tacho: int, v_in: float):
        data = struct.pack('!ihh', tacho, int(v_in * 10), 0)
        self.bus.send(can.Message(
            arbitration_id=(int(frames.CanPacketId.STATUS_5) << 8) | self.controller_id,
            data=data, is_extended_id=True))

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.bus.shutdown()


class TestVescCanBus(unittest.TestCase):
    def setUp(self):
        self.fake = FakeVesc(CHANNEL, controller_id=100)
        self.host = VescCanBus(bus=can.Bus(interface='virtual', channel=CHANNEL,
                                           receive_own_messages=True))
        self.node = VescCanNode(self.host, 100)

    def tearDown(self):
        self.host.close()
        self.fake.close()

    def test_command_reaches_node_with_correct_bytes(self):
        self.node.set_current(-42.5)
        self.assertTrue(wait_for(lambda: len(self.fake.received) >= 1))
        packet_id, cid, data = self.fake.received[0]
        self.assertEqual(packet_id, frames.CanPacketId.SET_CURRENT)
        self.assertEqual(cid, 100)
        self.assertEqual(data, struct.pack('!i', -42500))

    def test_status_broadcast_lands_in_snapshot(self):
        self.fake.send_status1(rpm=3000, current_a=12.5, duty=0.4)
        self.fake.send_status5(tacho=555, v_in=84.2)
        self.assertTrue(wait_for(
            lambda: self.node.telemetry.status1 is not None
            and self.node.telemetry.status5 is not None))
        t = self.node.telemetry
        self.assertEqual(t.status1.rpm, 3000)
        self.assertAlmostEqual(t.status1.current, 12.5)
        self.assertAlmostEqual(t.status5.v_in, 84.2)
        self.assertLess(t.age('status1'), 1.0)
        self.assertTrue(self.node.is_alive(max_age_s=2.0))

    def test_never_heard_node_is_empty_and_dead(self):
        other = VescCanNode(self.host, 7)
        self.assertIsNone(other.telemetry.status1)
        self.assertIsNone(other.telemetry.age('status1'))
        self.assertFalse(other.is_alive())

    def test_ping_pong(self):
        hw_type = self.host.ping(100, timeout=2.0)
        self.assertEqual(hw_type, 0)

    def test_ping_timeout_for_absent_node(self):
        self.assertIsNone(self.host.ping(55, timeout=0.1))

    def test_stop_all_broadcasts_zero_current(self):
        self.host.stop_all()
        self.assertTrue(wait_for(lambda: any(
            p == frames.CanPacketId.SET_CURRENT and c == 0xFF and d == b'\x00' * 4
            for p, c, d in self.fake.received)))

    def test_subscriber_hook_fires(self):
        seen = []
        self.host.subscribe(lambda cid, obj: seen.append((cid, obj)))
        self.fake.send_status1(rpm=100, current_a=1.0, duty=0.1)
        self.assertTrue(wait_for(lambda: len(seen) >= 1))
        self.assertEqual(seen[0][0], 100)

    def test_conf_is_recorded_on_node(self):
        self.node.conf_current_limits(-60.0, 120.0)
        self.node.conf_current_limits_in(-20.0, 80.0)
        self.assertIn('current_limits', self.node.applied_conf)
        self.assertEqual(self.node.applied_conf['current_limits'][0], (-60.0, 120.0))
        self.assertTrue(wait_for(lambda: len(self.fake.received) >= 2))

    def test_candump_logger_records_both_directions(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'run.log')
            logger = self.host.attach_candump_logger(path)
            self.node.set_current(5.0)              # TX (own message echo)
            self.fake.send_status1(2000, 5.0, 0.2)  # RX
            self.assertTrue(wait_for(
                lambda: self.node.telemetry.status1 is not None))
            self.host.detach_logger(logger)
            with open(path) as f:
                lines = [ln for ln in f.read().splitlines() if ln.strip()]
            self.assertGreaterEqual(len(lines), 2)


if __name__ == '__main__':
    unittest.main()
