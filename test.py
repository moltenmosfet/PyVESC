from unittest import TestCase


class TestPacket(TestCase):
    def exact_single_frame(self, length):
        """
        Simplest test possible. Create a packet, then parse a buffer containing this packet. Size of buffer is exactly
        one packet (no excess).
        :param length: Number of bytes in payload.
        """
        import random
        import pyvesc.protocol.packet.codec as vesc_packet
        correct_payload_index = None
        if length < 256:
            correct_payload_index = 2
        else:
            correct_payload_index = 3
        test_payload = bytes(random.getrandbits(8) for i in range(length))
        # test framing
        packet = vesc_packet.frame(test_payload)
        self.assertEqual(len(packet), correct_payload_index + length + 3, "size of packet")
        buffer = bytearray(packet)
        # test Parser
        parsed, consumed = vesc_packet.unframe(buffer)
        buffer = buffer[consumed:]
        self.assertEqual(parsed, test_payload)
        self.assertEqual(len(buffer), 0)

    def exact_two_frames(self, length1, length2):
        """
        Check that if there is more than one packet in a buffer, that the unpacker will properly unpack the packets.
        Size of buffer for this test is exactly two packets.
        :param length1: Length of first payload
        :param length2: Length of second payload
        """
        import random
        import pyvesc.protocol.packet.codec as vesc_packet
        correct_payload_index1 = None
        correct_payload_index2 = None
        if length1 < 256:
            correct_payload_index1 = 2
        else:
            correct_payload_index1 = 3
        if length2 < 256:
            correct_payload_index2 = 2
        else:
            correct_payload_index2 = 3
        test_payload1 = bytes(random.getrandbits(8) for i in range(length1))
        test_payload2 = bytes(random.getrandbits(8) for i in range(length2))
        # test framing
        packet1 = vesc_packet.frame(test_payload1)
        packet2 = vesc_packet.frame(test_payload2)
        self.assertEqual(len(packet1), correct_payload_index1 + length1 + 3, "size of packet")
        self.assertEqual(len(packet2), correct_payload_index2 + length2 + 3, "size of packet")
        buffer = bytearray(packet1 + packet2)
        # test Parser
        parsed, consumed = vesc_packet.unframe(buffer)
        buffer = buffer[consumed:]
        self.assertEqual(parsed, test_payload1)
        self.assertEqual(len(buffer), len(packet2))
        parsed, consumed = vesc_packet.unframe(buffer)
        buffer = buffer[consumed:]
        self.assertEqual(parsed, test_payload2)
        self.assertEqual(len(buffer), 0)

    def parse_buffer(self, length):
        import random
        import pyvesc.protocol.packet.codec as vesc_packet
        correct_payload_index = None
        if length < 256:
            correct_payload_index = 2
        else:
            correct_payload_index = 3
        test_payload = bytes(random.getrandbits(8) for i in range(length))
        packet = vesc_packet.frame(test_payload)

        # test on small buffers
        for n in range(0, 5):
            in_buffer = bytearray(packet[:n])
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, None)
            self.assertEqual(in_buffer, out_buffer)
        # test on buffer almost big enough
        for n in range(len(packet) - 4, len(packet)):
            in_buffer = bytearray(packet[:n])
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, None)
            self.assertEqual(in_buffer, out_buffer)
        # test on buffer slightly too big
        extension = b'\x02\x04\x07'
        extended_packet = packet + b'\x02\x04\x07'
        for n in range(len(packet) + 1, len(packet) + 4):
            in_buffer = bytearray(extended_packet[:n])
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, test_payload)
            self.assertEqual(out_buffer, extension[:n - len(packet)])

    def test_small_packets(self):
        for length in range(1, 5):
            self.exact_single_frame(length)
            self.exact_two_frames(length, length + 2)
            self.parse_buffer(length)

    def test_med_packets(self):
        for length in range(254, 258):
            self.exact_single_frame(length)
            self.exact_two_frames(length, length - 23)
            self.parse_buffer(length)

    def test_large_packets(self):
        for length in range(1022, 1024):
            self.exact_single_frame(length)
            self.exact_two_frames(length, length + 100)
            self.parse_buffer(length)

    def test_corrupt_detection(self):
        import pyvesc.protocol.packet.codec as vesc_packet
        # make a good packet
        test_payload = b'Te!'
        good_packet = b'\x02\x03Te!B\x92\x03'
        corrupt_packets = []
        # corrupt first byte
        corrupt = b'\x01\x03Te!B\x92\x03'
        corrupt_packets.append(corrupt)
        # corrupt payload_length (to be smaller and larger)
        smaller_corrupt = b'\x02\x02Te!B\x92\x03'
        larger_corrupt = b'\x02\x04Te!B\x92\x03\x03'
        corrupt_packets.append(smaller_corrupt)
        corrupt_packets.append(larger_corrupt)
        # corrupt first byte in payload
        corrupt = b'\x02\x03se!B\x92\x03'
        corrupt_packets.append(corrupt)
        # corrupt last byte in payload
        corrupt = b'\x02\x03TeyB\x92\x03'
        corrupt_packets.append(corrupt)
        # corrupt crc
        corrupt = b'\x02\x03Te!\xaa\x91\x03'
        corrupt_packets.append(corrupt)
        # corrupt terminator
        corrupt = b'\x02\x03Te!B\x92\x09'
        corrupt_packets.append(corrupt)
        # check that exceptions are given on each corrupt packet
        for corrupt in corrupt_packets:
            in_buffer = bytearray(corrupt)
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, None)
            self.assertTrue(consumed > 0)   # if a packet is corrupt then at least something should be consumed
            # get correct out_cuffer (in all of these cases it is just consuming to the next valid start byte (no more no less)
            self.assertEqual(consumed, vesc_packet.Stateless._next_possible_packet_index(in_buffer))
        # check that the good packet is parsed
        in_buffer = bytearray(good_packet)
        parsed, consumed = vesc_packet.unframe(in_buffer)
        out_buffer = in_buffer[consumed:]
        self.assertEqual(parsed, test_payload)
        self.assertEqual(out_buffer, b'')

    def test_corrupt_recovery(self):
        import pyvesc.protocol.packet.codec as vesc_packet
        # make a good packet
        test_payload = b'Te!'
        good_packet = b'\x02\x03Te!B\x92\x03'
        packet_to_recover = b'\x02\x04!\xe1$ 8\xbb\x03'  # goal is to recover this packet
        payload_to_recover = b'!\xe1$ '
        after_goal = b'\x05\x09\x01'  # mimic another corrupt packet after
        corrupt_packets = []
        # corrupt first byte
        corrupt = b'\x01\x03Te!B\x92\x03'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # corrupt payload_length (to be smaller and larger)
        smaller_corrupt = b'\x02\x02Te!B\x92\x03'
        larger_corrupt = b'\x02\x04Te!B\x92\x03\x03'
        corrupt_packets.append(smaller_corrupt + packet_to_recover + after_goal)
        corrupt_packets.append(larger_corrupt + packet_to_recover + after_goal)
        # corrupt first byte in payload
        corrupt = b'\x02\x03se!B\x92\x03'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # corrupt last byte in payload
        corrupt = b'\x02\x03TeyB\x92\x03'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # corrupt crc
        corrupt = b'\x02\x03Te!\xaa\x91\x03'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # corrupt terminator
        corrupt = b'\x02\x03Te!B\x92\x09'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # check that exceptions are given on each corrupt packet
        for corrupt in corrupt_packets:
            in_buffer = bytearray(corrupt)
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, payload_to_recover)
            found_packet_start = corrupt.find(packet_to_recover)
            self.assertTrue(consumed == (found_packet_start + len(packet_to_recover)))
        # check that the good packet is parsed
        in_buffer = bytearray(good_packet)
        parsed, consumed = vesc_packet.unframe(in_buffer)
        out_buffer = in_buffer[consumed:]
        self.assertEqual(parsed, test_payload)
        self.assertEqual(out_buffer, b'')


class TestMsg(TestCase):
    def setUp(self):
        import copy
        from pyvesc.protocol.base import VESCMessage
        self._initial_registry = copy.deepcopy(VESCMessage._msg_registry)

    def tearDown(self):
        from pyvesc.protocol.base import VESCMessage
        VESCMessage._msg_registry = self._initial_registry
        self._initial_registry = None

    def verify_packing_and_unpacking(self, msg):
        from pyvesc.protocol.base import VESCMessage
        payload_bytestring = VESCMessage.pack(msg)
        parsed_msg = VESCMessage.unpack(payload_bytestring)
        self.assertEqual(parsed_msg.id, msg.id)
        for name in [names[0] for names in msg.send_fields]:
            self.assertEqual(getattr(parsed_msg, name), getattr(msg, name))

    def test_single_message(self):
        from pyvesc.protocol.base import VESCMessage

        class TestMsg1(metaclass=VESCMessage):
            id = 0xE0
            send_fields = [
                ('f1', 'B'),
                ('f2', 'H'),
                ('f3', 'i'),
                ('f4', 'L'),
                ('f5', 'b'),
                ('f6', 'I'),
            ]

        test_message = TestMsg1(27, 25367, -1124192846, 2244862237, 17, 73262)
        self.verify_packing_and_unpacking(test_message)

    def test_multiple_messages(self):
        from pyvesc.protocol.base import VESCMessage

        class testMsg1(metaclass=VESCMessage):
            id = 0xE1
            send_fields = [
                ('f1', 'B'),
                ('f2', 'H'),
                ('f3', 'i'),
                ('f4', 'L'),
                ('f5', 'b'),
                ('f6', 'I'),
            ]

        class testMsg2(metaclass=VESCMessage):
            id = 0xE2
            send_fields = [
                ('f1', 'B'),
                ('f2', 'B'),
            ]

        class testMsg3(metaclass=VESCMessage):
            id = 0xE3
            send_fields = [
                ('f1', 'i'),
                ('f2', 'i'),
            ]

        class testMsg4(metaclass=VESCMessage):
            id = 0xE4
            send_fields = [
                ('f1', 'i'),
                ('f2', 's'),
                ('f3', 'i'),
                ('f4', 'B'),
                ('f5', 'i'),
            ]

        test_message1 = testMsg1(27, 25367, -1124192846, 2244862237, 17, 73262)
        test_message12 = testMsg1(82, 45132, 382136436, 27374, 18, 72134)
        test_message2 = testMsg2(27, 13)
        test_message22 = testMsg2(52, 19)
        test_message3 = testMsg3(-7841, 4611)
        test_message32 = testMsg3(-123, 4123)
        test_message4 = testMsg4(4531, 'hello world', 1421, 34, 14215)
        self.verify_packing_and_unpacking(test_message1)
        self.verify_packing_and_unpacking(test_message2)
        self.verify_packing_and_unpacking(test_message3)
        self.verify_packing_and_unpacking(test_message4)

    def test_errors(self):
        from pyvesc.protocol.base import VESCMessage

        # try to make two messages with the same ID
        class testMsg1(metaclass=VESCMessage):
            id = 0xE5
            send_fields = [
                ('f1', 'H'),
                ('f2', 'H'),
            ]
        caught = False
        try:
            class testMsg2(metaclass=VESCMessage):
                id = 0xE5
                send_fields = [
                    ('f1', 'B'),
                    ('f2', 'B'),
                ]
        except TypeError as e:
            caught = True
        self.assertTrue(caught)

        # check that message classes are final
        caught = False
        try:
            class testMsg4(testMsg1):
                id = 0xE6
                send_fields = [
                    ('f1', 'B'),
                    ('f2', 'B'),
                ]
        except TypeError as e:
            caught = True
        self.assertTrue(caught)

        # check that no more than 1 string field is allowed
        caught = False
        try:
            class testMsg7(metaclass=VESCMessage):
                id = 0xE7
                send_fields = [
                    ('f1', 's'),
                    ('f2', 's'),
                ]
        except TypeError as e:
            caught = True
        self.assertTrue(caught)

        # check that 's' is used instead of 'p'
        caught = False
        try:
            class testMsg8(metaclass=VESCMessage):
                id = 0xE8
                send_fields = [
                    ('f1', 'p'),
                ]
        except TypeError as e:
            caught = True
        self.assertTrue(caught)

        # try to fill a message with the wrong number of arguments
        caught = False
        try:
            testmessage1 = testMsg1(2, 4, 5)  # should be 2 args
        except AttributeError as e:
            caught = True
        self.assertTrue(caught)


class TestInterface(TestCase):
    def setUp(self):
        import copy
        from pyvesc.protocol.base import VESCMessage
        self._initial_registry = copy.deepcopy(VESCMessage._msg_registry)

    def tearDown(self):
        from pyvesc.protocol.base import VESCMessage
        VESCMessage._msg_registry = self._initial_registry
        self._initial_registry = None

    def verify_encode_decode(self, msg):
        from pyvesc.protocol.interface import encode, decode
        encoded = encode(msg)
        decoded, consumed, payload_total = decode(encoded)
        self.assertEqual(consumed, len(encoded))
        for field in msg._send_field_names:
            self.assertEqual(getattr(msg, field), getattr(decoded, field))

    def test_interface(self):
        from pyvesc.protocol.base import VESCMessage

        class testMsg1(metaclass=VESCMessage):
            id = 0xE9
            send_fields = [
                ('f1', 'B'),
                ('f2', 'H'),
                ('f3', 'i'),
                ('f4', 'L'),
                ('f5', 'b'),
                ('f6', 'I'),
            ]

        class testMsg2(metaclass=VESCMessage):
            id = 0xEA
            send_fields = [
                ('f1', 'B'),
                ('f2', 'B'),
            ]

        class testMsg3(metaclass=VESCMessage):
            id = 0xEB
            send_fields = [
                ('f1', 'i'),
                ('f2', 'i'),
            ]

        class testMsg4(metaclass=VESCMessage):
            id = 0xEC
            send_fields = [
                ('f1', 'i'),
                ('f2', 's'),
                ('f3', 'i'),
                ('f4', 'B'),
                ('f5', 'i'),
            ]
        test_message1 = testMsg1(27, 25367, -1124192846, 2244862237, 17, 73262)
        test_message12 = testMsg1(82, 45132, 382136436, 27374, 18, 72134)
        test_message2 = testMsg2(27, 13)
        test_message22 = testMsg2(52, 19)
        test_message3 = testMsg3(-7841, 4611)
        test_message32 = testMsg3(-123, 4123)
        test_message4 = testMsg4(4531, 'hello world', 1421, 34, 14215)
        self.verify_encode_decode(test_message1)
        self.verify_encode_decode(test_message2)
        self.verify_encode_decode(test_message3)
        self.verify_encode_decode(test_message4)


class TestTCPTransport(TestCase):
    """
    Tests for the TCP<->UART bridge transport, using loopback servers that
    stand in for the ESP32 bridge + VESC.
    """

    def _start_server(self, handler):
        """
        Start a single-connection TCP server on an ephemeral port. `handler`
        runs in a thread with the accepted connection. Returns the port.
        """
        import socket
        import threading
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind(('127.0.0.1', 0))
        srv.listen(1)
        port = srv.getsockname()[1]

        def run():
            conn, _ = srv.accept()
            try:
                handler(conn)
            finally:
                conn.close()
                srv.close()

        t = threading.Thread(target=run, daemon=True)
        t.start()
        self.addCleanup(t.join, 2.0)
        return port

    def test_from_url(self):
        from pyvesc.transport import TCPTransport
        with self.assertRaises(ValueError):
            TCPTransport.from_url('/dev/ttyACM0')
        with self.assertRaises(ValueError):
            TCPTransport.from_url('http://host:1')
        # reachability isn't needed to check URL parsing: connection refused
        # on loopback proves host/port were parsed and used
        with self.assertRaises(ConnectionError):
            TCPTransport.from_url('tcp://127.0.0.1:1', connect_timeout=0.2)

    def test_write_read_roundtrip(self):
        import threading
        import time
        from pyvesc.transport import TCPTransport

        got = {}
        request_seen = threading.Event()

        def handler(conn):
            got['request'] = conn.recv(64)
            conn.sendall(b'\x11\x22\x33')
            request_seen.set()
            # hold the connection open while the client reads
            conn.recv(64)

        port = self._start_server(handler)
        transport = TCPTransport('127.0.0.1', port)
        self.addCleanup(transport.close)
        transport.write(b'ping')
        self.assertTrue(request_seen.wait(2.0))
        self.assertEqual(got['request'], b'ping')

        deadline = time.time() + 2.0
        while transport.in_waiting < 3 and time.time() < deadline:
            time.sleep(0.01)
        self.assertEqual(transport.in_waiting, 3)
        self.assertEqual(transport.read(2), b'\x11\x22')
        self.assertEqual(transport.read(2), b'\x33')
        transport.close()
        self.assertFalse(transport.is_open)

    def test_peer_close_drains_then_raises(self):
        import time
        from pyvesc.transport import TCPTransport

        def handler(conn):
            conn.sendall(b'tail')
            # returning closes the connection immediately

        port = self._start_server(handler)
        transport = TCPTransport('127.0.0.1', port)
        self.addCleanup(transport.close)

        # buffered bytes must remain readable after the peer closes
        deadline = time.time() + 2.0
        payload = b''
        while len(payload) < 4 and time.time() < deadline:
            try:
                payload += transport.read(transport.in_waiting)
            except ConnectionError:
                break
            time.sleep(0.01)
        self.assertEqual(payload, b'tail')

        # once drained, the dead link must surface loudly
        with self.assertRaises(ConnectionError):
            deadline = time.time() + 2.0
            while time.time() < deadline:
                transport.in_waiting
                time.sleep(0.01)

    def test_vesc_over_tcp(self):
        import struct
        from pyvesc.VESC import VESC
        from pyvesc.messages.getters import GetRotorPosition
        from pyvesc.protocol.packet.codec import frame

        # framed GetRotorPosition response: rotor_pos field scaled by 100000
        response = frame(struct.pack('>Bi', GetRotorPosition.id, 12345600))

        def handler(conn):
            conn.recv(64)  # the request
            conn.sendall(response)
            conn.recv(64)  # hold open until the client disconnects

        port = self._start_server(handler)
        with VESC('tcp://127.0.0.1:{}'.format(port), start_heartbeat=False) as vesc:
            reply = vesc.write(b'\x00', num_read_bytes=GetRotorPosition._recv_full_msg_size)
            self.assertIsNotNone(reply)
            self.assertAlmostEqual(reply.rotor_pos, 123.456)
