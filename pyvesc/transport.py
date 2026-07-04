"""
TCP transport for talking to a VESC through a TCP<->UART bridge (e.g. an
ESP32 running a transparent bridge, or VESC Express hardware).

Exposes the subset of the pyserial API that pyvesc.VESC uses (write, read,
in_waiting, flush, close, is_open) so serial and TCP transports are
interchangeable.
"""

import select
import socket
from urllib.parse import urlsplit


class TCPTransport(object):
    # VESC Tool's default TCP bridge port; VESC Express uses it too
    DEFAULT_PORT = 65102

    def __init__(self, host, port=DEFAULT_PORT, timeout=0.05, connect_timeout=5.0):
        """
        :param host: hostname or IP of the TCP<->UART bridge
        :param port: TCP port the bridge listens on
        :param timeout: socket send timeout in seconds (reads are non-blocking)
        :param connect_timeout: timeout for the initial connection in seconds
        """
        try:
            self._sock = socket.create_connection((host, port), timeout=connect_timeout)
        except OSError as e:
            raise ConnectionError(
                "Could not connect to VESC bridge at {}:{}: {}".format(host, port, e))
        # VESC packets are tiny; Nagle would batch them and add latency
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(timeout)
        self._buffer = bytearray()
        self.is_open = True

    @classmethod
    def from_url(cls, url, **kwargs):
        """
        Build a transport from a "tcp://host[:port]" address string.
        """
        parsed = urlsplit(url)
        if parsed.scheme != 'tcp' or not parsed.hostname:
            raise ValueError("Expected an address of the form tcp://host[:port], got {!r}".format(url))
        return cls(parsed.hostname, parsed.port or cls.DEFAULT_PORT, **kwargs)

    def _pump(self):
        """
        Move any bytes waiting on the socket into the local buffer.
        """
        while self.is_open:
            readable, _, _ = select.select([self._sock], [], [], 0)
            if not readable:
                return
            chunk = self._sock.recv(4096)
            if not chunk:
                # peer closed; already-buffered bytes stay readable, and the
                # link death surfaces once the buffer is drained
                self.is_open = False
                return
            self._buffer.extend(chunk)

    @property
    def in_waiting(self):
        self._pump()
        if not self._buffer and not self.is_open:
            raise ConnectionError("VESC bridge closed the TCP connection")
        return len(self._buffer)

    def read(self, size):
        self._pump()
        data = bytes(self._buffer[:size])
        del self._buffer[:size]
        return data

    def write(self, data):
        self._sock.sendall(data)
        return len(data)

    def flush(self):
        pass  # sendall already handed everything to the kernel

    def close(self):
        # is_open may already be False after a peer close; the fd still needs closing
        self.is_open = False
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._sock.close()
