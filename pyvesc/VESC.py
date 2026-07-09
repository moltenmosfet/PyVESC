from .protocol.interface import encode_request, encode, decode
from .protocol.packet import codec as vesc_packet_codec
from .protocol.base import VESCMessage
from .transport import TCPTransport
from .messages.getters import GetVersion, GetMotorConfig, GetAppConfig, GetValues
from .messages.setters import (
    SetMotorConfig, SetAppConfig, SetRPM, SetCurrent, SetDutyCycle,
    SetServoPosition, EraseNewApp, WriteNewAppData, WriteNewAppDataLZO,
    JumpToBootloader, TerminalCmd, SetRotorPositionMode, Reboot, Alive,
    SetIdDissipate
)
import time
import threading
import logging


logger = logging.getLogger(__name__)

# because people may want to use this library for their own messaging, do not make this a required package
try:
    import serial
except ImportError:
    serial = None

read_lock = threading.Lock()


class VESC(object):
    def __init__(self, serial_port, has_sensor=False, start_heartbeat=True, baudrate=115200, timeout=0.05,
                 can_id=None):
        """
        :param serial_port: Serial device to use for communication (i.e. "COM3" or "/dev/tty.usbmodem0"),
                            or the address of a TCP bridge as "tcp://host[:port]"
                            (default port 65102, e.g. "tcp://192.168.1.50" for a VESC Express)
        :param has_sensor: Whether or not the bldc motor is using a hall effect sensor
        :param start_heartbeat: Whether or not to automatically start the heartbeat thread that will keep commands
                                alive.
        :param baudrate: baudrate for the serial communication. Ignored for TCP (the bridge fixes the UART baudrate).
        :param timeout: timeout for the serial/socket communication
        :param can_id: Default CAN target for every command and getter. Set this when the device
                       we connect to is not the motor controller itself but a bridge on its CAN bus
                       (e.g. a VESC Express: connect tcp://<express-ip>, can_id=<controller id>).
                       Individual setter calls can still override with their own can_id kwarg.
        """

        self.can_id = can_id

        if isinstance(serial_port, str) and serial_port.startswith('tcp://'):
            self.serial_port = TCPTransport.from_url(serial_port, timeout=timeout)
        else:
            if serial is None:
                raise ImportError("Need to install pyserial in order to use the VESCMotor class over serial.")
            self.serial_port = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)
        if has_sensor:
            self.serial_port.write(encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF, can_id=can_id)))

        # heartbeat messages sent every cycle; forwarded CAN targets are appended
        # via start_heartbeat(can_id=...)
        self.alive_msgs = [encode(Alive(can_id=can_id))]

        self.heart_beat_thread = threading.Thread(target=self._heartbeat_cmd_func)
        self._stop_heartbeat = threading.Event()

        if start_heartbeat:
            self.start_heartbeat()

        self._message_monitor_thread = threading.Thread(target=self._message_monitor)

        # thread to monitor messages to receive unscheuled prints from ESC for debugging,
        # currently disabled as it was interfering sometimes and is not needed
        self._stop_message_monitor = threading.Event()
        # self._message_monitor_thread.start()

        # store message info for getting values so it doesn't need to calculate it every time
        msg = GetValues(can_id=can_id)
        self._get_values_msg = encode_request(msg)
        self._get_values_msg_expected_length = msg._recv_full_msg_size
        self._get_values_msg_id = msg.id

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_heartbeat()
        try:
            if self.serial_port.is_open:
                self.serial_port.flush()
                self.serial_port.close()
        except Exception as e:
            logger.error("Error closing serial port: {}".format(e))
            logger.error("This is likely due to the motor being disconnected before the connection could be closed.")

    def _message_monitor(self):
        """
        A function that continuously reads the serial port for messages and decodes them.
        """
        while not self._stop_message_monitor.is_set():
            ret = self.read(1, expect_string=True, expect_anything=False)
            if ret is not None:
                print(ret)
            time.sleep(0.01)

    def _heartbeat_cmd_func(self):
        """
        Continuous function calling that keeps the motor alive
        """
        while not self._stop_heartbeat.is_set():
            time.sleep(0.1)
            for msg in self.alive_msgs:
                self.write(msg, is_heartbeat=True)

    def start_heartbeat(self, can_id=None):
        """
        Starts a repetitive calling of the last set cmd to keep the motor alive.

        :param can_id: Optional CAN ID of an additional VESC to forward heartbeats to.
                       May be called multiple times to add several forwarded targets.
        """
        if can_id is not None:
            self.alive_msgs.append(encode(Alive(can_id=can_id)))
        if not self.heart_beat_thread.is_alive():
            self.heart_beat_thread.start()

    def stop_heartbeat(self):
        """
        Stops the heartbeat thread and resets the last cmd function. THIS MUST BE CALLED BEFORE THE OBJECT GOES OUT OF
        SCOPE UNLESS WRAPPING IN A WITH STATEMENT (Assuming the heartbeat was started).
        """
        self._stop_heartbeat.set()
        if self.heart_beat_thread.is_alive():
            try:
                self.heart_beat_thread.join()
            except Exception as e:
                logger.error("Error stopping heartbeat: {}".format(e))

    def write(self, data, num_read_bytes=None, is_heartbeat=False, expect_string=False, max_wait=0.5,
              expected_msg_id=None):
        """
        A write wrapper function implemented like this to try and make it easier to incorporate other communication
        methods than UART in the future.
        :param data: the byte string to be sent
        :param num_read_bytes: number of bytes to read for decoding response
        :param is_heartbeat: whether or not this is a heartbeat message, can be used for filtering debug prints
        :param expect_string: whether or not to expect a string response
        :param max_wait: overall deadline in seconds for the response
        :param expected_msg_id: command id the response must carry; other frames are discarded
        :return: decoded response from buffer
        """
        if num_read_bytes is not None:
            # a response is expected: drop stale responses that piled up during a
            # link stall, so this request cannot be answered by old data
            with read_lock:
                stale = self.serial_port.in_waiting
                if stale:
                    logger.debug("Discarding {} stale bytes before request".format(stale))
                    self.serial_port.read(stale)
        try:
            self.serial_port.write(data)
        except Exception as e:
            logger.error("Error writing to serial port: {}".format(e))
        if not is_heartbeat:
            logging.debug("Data sent: {}".format(data))
        if num_read_bytes is not None:
            return self.read(num_read_bytes, expect_string=expect_string, max_wait=max_wait,
                             expected_msg_id=expected_msg_id)

    def read(self, num_read_bytes=None, timeout=0.1, expect_string=False, expect_anything=True, max_wait=0.5,
             expected_msg_id=None):
        """
        Read a response from the transport.

        Fixed-size responses return as soon as a frame decodes; if a link stall
        delivered several frames in one burst, the newest matching frame wins.
        String responses (terminal output, config reads) can span multiple
        frames, so they are collected until the line goes quiet for `timeout`
        seconds. Either way `max_wait` bounds the total time so a dead link
        cannot hang the caller — important when driving a dyno over WiFi.

        :param num_read_bytes: expected payload size; kept for API compatibility, the exit
                               condition is a successful decode rather than a byte count
        :param timeout: quiet-time in seconds that ends a multi-frame string read
        :param expect_string: whether the response may span multiple frames
        :param expect_anything: if False, return None as soon as the line is found idle
        :param max_wait: overall deadline in seconds for the response
        :param expected_msg_id: command id the response must carry (ignored for strings);
                                None accepts any frame
        :return: decoded response, or None if nothing valid arrived in time
        """
        payload = b''
        match = None

        with read_lock:
            deadline = time.time() + max_wait
            t_quiet = None
            while time.time() < deadline:
                waiting = self.serial_port.in_waiting
                if waiting:
                    payload += self.serial_port.read(waiting)
                    t_quiet = time.time()
                    if not expect_string:
                        # unframe everything available and keep the newest matching frame
                        while True:
                            frame_payload, consumed = vesc_packet_codec.unframe(payload)
                            if consumed == 0:
                                break
                            payload = payload[consumed:]
                            if frame_payload is None:
                                continue
                            if expected_msg_id is None or frame_payload[0] == expected_msg_id:
                                match = frame_payload
                            else:
                                logger.debug(
                                    "Ignoring frame with command id {} while waiting for {}".format(
                                        frame_payload[0], expected_msg_id))
                        # only return once the burst is fully consumed, so a stale
                        # frame can't win over a fresher one right behind it
                        if match is not None and not payload:
                            return VESCMessage.unpack(match, unpack_send_fields=False)
                elif not payload and not expect_anything and match is None:
                    # just probing the line, don't wait for a response
                    return None
                elif t_quiet is not None and time.time() - t_quiet > timeout:
                    # multi-frame response finished (line went quiet)
                    break
                time.sleep(0.01)

        if not expect_string:
            return VESCMessage.unpack(match, unpack_send_fields=False) if match is not None else None

        response, consumed, msg_payload = decode(payload, recv=True)
        logging.debug("Data response: {}".format(msg_payload))
        return response

    def update_firmware(self, firmware, progress_callback=None):

        logging.info("Erasing")

        erase_res = self.fw_erase_new_app(firmware.size)
        if erase_res.erase_new_app_result != 1:
            logging.error("Erase failed")
            progress_callback("Erase Failed")
            return False

        logging.info("Sending firmware")

        offset = 0
        time_since_last_progress_update = time.time()

        while firmware.size > 0:
            fw_chunk = firmware.get_next_chunk()

            # check if the chunk is empty, don't send
            has_data = False
            for i in fw_chunk:
                if i != 0xff:
                    has_data = True
                    break

            if has_data:
                fw_result = self.fw_write_new_app_data(offset, fw_chunk)

                if fw_result.write_new_app_result != 1 or fw_result.write_new_app_result is None:
                    logging.error("Write failed")
                    logging.error(fw_result)
                    progress_callback("Flashing Failed")
                    return False

            offset += firmware.chunk_size

            UPDATE_INTERVAL_SECS = 10
            if time.time() - time_since_last_progress_update > UPDATE_INTERVAL_SECS:
                time_since_last_progress_update = time.time()
                logging.info(
                    "Progress: {:.2f}%, Size: {}/{}kB".format(firmware.get_progress(offset), offset, firmware.original_size))
                if progress_callback is not None:
                    progress_callback(int(firmware.get_progress(offset)))

            # stream updates quickly to stdout
            print("\rProgress: {:.2f}%, Size: {}kB, to be written to {}".format(
                firmware.get_progress(offset), offset, offset + firmware.chunk_size), end='\r')
            firmware.clear_chunk()

        logging.info("Firmware upload complete, jumping to bootloader.")
        try:
            self.fw_jump_to_bootloader()
        except Exception as e:
            logging.error(
                "Error jumping to bootloader, this is likely the motor rebooting before a connection could be closed: {}".format(e))

        return True

    def set_rpm(self, new_rpm, **kwargs):
        """
        Set the electronic RPM value (a.k.a. the RPM value of the stator)
        :param new_rpm: new rpm value
        :param kwargs: optional can_id to forward the command over CAN
        """
        kwargs.setdefault('can_id', self.can_id)
        self.write(encode(SetRPM(new_rpm, **kwargs)))

    def set_current(self, new_current, **kwargs):
        """
        :param new_current: new current in amps for the motor
        :param kwargs: optional can_id to forward the command over CAN
        """
        kwargs.setdefault('can_id', self.can_id)
        self.write(encode(SetCurrent(new_current, **kwargs)))

    def set_id_dissipate(self, current, off_delay=0.5, **kwargs):
        """Molten MOSFET fork only: d-axis dissipation injection (winding-heat
        energy dump; ~zero torque, torque keeps priority in firmware).

        :param current: dissipation current magnitude in amps
        :param off_delay: command validity window in seconds (firmware clamps
            to [0.05, 5.0]); refresh faster than this to sustain a dump
        :param kwargs: optional can_id to forward the command over CAN
        """
        kwargs.setdefault('can_id', self.can_id)
        self.write(encode(SetIdDissipate(current, off_delay, **kwargs)))

    def set_duty_cycle(self, new_duty_cycle, **kwargs):
        """
        :param new_duty_cycle: Value of duty cycle to be set (fraction, range [-1, 1]).
        :param kwargs: optional can_id to forward the command over CAN
        """
        kwargs.setdefault('can_id', self.can_id)
        self.write(encode(SetDutyCycle(new_duty_cycle, **kwargs)))

    def set_servo(self, new_servo_pos, **kwargs):
        """
        :param new_servo_pos: New servo position. valid range [0, 1]
        :param kwargs: optional can_id to forward the command over CAN
        """
        kwargs.setdefault('can_id', self.can_id)
        self.write(encode(SetServoPosition(new_servo_pos, **kwargs)))

    def get_measurements(self):
        """
        :return: A msg object with attributes containing the measurement values
        """
        return self.write(self._get_values_msg, num_read_bytes=self._get_values_msg_expected_length,
                          expected_msg_id=self._get_values_msg_id)

    def get_firmware_version(self):
        msg = GetVersion(can_id=self.can_id)
        return self.write(encode_request(msg), num_read_bytes=msg._recv_full_msg_size, expected_msg_id=msg.id)

    def get_rpm(self):
        """
        :return: Current motor rpm
        """
        return self.get_measurements().rpm

    def get_duty_cycle(self):
        """
        :return: Current applied duty-cycle
        """
        return self.get_measurements().duty_now

    def get_v_in(self):
        """
        :return: Current input voltage
        """
        return self.get_measurements().v_in

    def get_motor_current(self):
        """
        :return: Current motor current
        """
        return self.get_measurements().current_motor

    def get_incoming_current(self):
        """
        :return: Current incoming current
        """
        return self.get_measurements().current_in

    def fw_erase_new_app(self, fw_size):
        """
        Erase app data
        """
        # TODO: Revert this to actual fw size
        msg = EraseNewApp(fw_size, can_id=self.can_id)
        # flash erase can take several seconds before the VESC acks
        return self.write(encode(msg), num_read_bytes=msg._recv_full_msg_size, max_wait=30.0,
                          expected_msg_id=msg.id)

    def reboot(self):
        """
        Reboot VESC
        """
        msg = Reboot(can_id=self.can_id)
        return str(self.write(encode_request(msg), num_read_bytes=msg._recv_full_msg_size))

    def fw_write_new_app_data(self, offset, data):
        """
        Write new app data
        """
        msg = WriteNewAppData(offset, data, can_id=self.can_id)
        return self.write(encode(msg), num_read_bytes=msg._recv_full_msg_size, max_wait=5.0, expected_msg_id=msg.id)

    def fw_write_new_app_data_lzo(self, offset, data):
        """
        Write new app data
        """
        msg = WriteNewAppDataLZO(offset, data, can_id=self.can_id)
        return self.write(encode(msg), num_read_bytes=msg._recv_full_msg_size, max_wait=5.0, expected_msg_id=msg.id)

    def fw_jump_to_bootloader(self):
        """
        Jump to bootloader
        set number of read bytes to None as we don't expect a response
        """
        msg = JumpToBootloader(can_id=self.can_id)
        # stop heartbeat, as we are about to reset the device
        self.stop_heartbeat()

        return self.write(encode_request(msg), num_read_bytes=None)

    def send_terminal_cmd(self, cmd):
        """
        Send terminal command
        """
        msg = TerminalCmd(cmd, can_id=self.can_id)
        return self.write(encode(msg), num_read_bytes=msg._recv_full_msg_size, expect_string=True)

    def get_motor_configuration(self):
        """
        Get the motor configuration parameters
        """
        msg = GetMotorConfig(can_id=self.can_id)
        res = self.write(encode(msg), num_read_bytes=msg._recv_full_msg_size, expect_string=True)
        return res

    def set_motor_configuration(self, data):
        """
        Set the motor configuration parameters
        """
        msg = SetMotorConfig(data, can_id=self.can_id)
        res = self.write(encode(msg), num_read_bytes=msg._recv_full_msg_size, expect_string=True)
        return res

    def get_app_configuration(self):
        """
        Get the app configuration parameters
        """
        msg = GetAppConfig(can_id=self.can_id)
        res = self.write(encode(msg), num_read_bytes=msg._recv_full_msg_size, expect_string=True)
        return res

    def set_app_configuration(self, data):
        """
        Set the app configuration parameters
        """
        msg = SetAppConfig(data, can_id=self.can_id)
        res = self.write(encode(msg), num_read_bytes=msg._recv_full_msg_size, expect_string=True)
        return res
