#!/usr/bin/env python3
import threading
import time
from typing import Optional, Tuple

import rospy
import serial
from std_msgs.msg import Int32, String


VALID_STM32_LINES = {
    "PONG",
    "ACK:DROP:1",
    "ACK:DROP:2",
    "ACK:DROP:3",
    "ACK:LOCK",
    "ERR:BUSY",
    "ERR:INVALID",
}


class DropControllerSerial:
    def __init__(self) -> None:
        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baud_rate = int(rospy.get_param("~baud_rate", 115200))
        self.command_timeout = float(rospy.get_param("~command_timeout", 2.0))
        self.reconnect_interval = float(rospy.get_param("~reconnect_interval", 2.0))
        self.send_ping_on_connect = bool(rospy.get_param("~send_ping_on_connect", True))
        self.lock_after_drop = bool(rospy.get_param("~lock_after_drop", False))
        self.lock_delay = float(rospy.get_param("~lock_delay", 0.5))
        if self.command_timeout <= 0.0:
            self.command_timeout = 2.0
        if self.reconnect_interval <= 0.0:
            self.reconnect_interval = 2.0
        if self.lock_delay < 0.0:
            self.lock_delay = 0.0

        self.status_pub = rospy.Publisher("/craic/drop_status", String, queue_size=10)
        rospy.Subscriber("/craic/drop_cmd", Int32, self._drop_cmd_cb, queue_size=10)

        self._serial_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._ser: Optional[serial.Serial] = None
        self._pending_drop: Optional[Tuple[int, float]] = None
        self._stop_event = threading.Event()

        self._reader_thread = threading.Thread(target=self._serial_worker)
        self._reader_thread.daemon = True
        self._reader_thread.start()
        self._timeout_timer = rospy.Timer(rospy.Duration(0.1), self._check_timeout)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "drop_controller_serial started: port=%s baud=%d timeout=%.2fs reconnect=%.2fs lock_after_drop=%s",
            self.serial_port,
            self.baud_rate,
            self.command_timeout,
            self.reconnect_interval,
            self.lock_after_drop,
        )

    def shutdown(self) -> None:
        self._stop_event.set()
        self._close_serial()

    def _publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))

    def _drop_cmd_cb(self, msg: Int32) -> None:
        value = int(msg.data)
        if value not in (1, 2, 3):
            self._publish_status("ERR:INVALID_DROP_CMD:%d" % value)
            return

        with self._pending_lock:
            if self._pending_drop is not None:
                self._publish_status("ERR:BUSY")
                return
            self._pending_drop = (value, time.monotonic())

        if not self._write_line("DROP:%d" % value):
            with self._pending_lock:
                if self._pending_drop is not None and self._pending_drop[0] == value:
                    self._pending_drop = None

    def _serial_worker(self) -> None:
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            if not self._is_connected():
                self._try_connect()
                if not self._is_connected():
                    self._stop_event.wait(self.reconnect_interval)
                    continue

            try:
                with self._serial_lock:
                    ser = self._ser
                if ser is None:
                    continue

                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                self._handle_serial_line(line)
            except (serial.SerialException, OSError) as exc:
                rospy.logwarn("STM32 drop serial disconnected: %s", exc)
                self._close_serial()
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "STM32 drop serial read error: %s", exc)

    def _is_connected(self) -> bool:
        with self._serial_lock:
            return self._ser is not None and self._ser.is_open

    def _try_connect(self) -> None:
        try:
            ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.1,
                write_timeout=1.0,
            )
            with self._serial_lock:
                self._ser = ser
            rospy.loginfo("Connected to STM32 drop controller: %s @ %d", self.serial_port, self.baud_rate)
            if self.send_ping_on_connect:
                self._write_line("PING")
        except (serial.SerialException, OSError) as exc:
            rospy.logwarn_throttle(5.0, "Waiting for STM32 drop serial %s: %s", self.serial_port, exc)

    def _close_serial(self) -> None:
        with self._serial_lock:
            ser = self._ser
            self._ser = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _write_line(self, line: str) -> bool:
        payload = ("%s\r\n" % line).encode("utf-8")
        try:
            with self._serial_lock:
                if self._ser is None or not self._ser.is_open:
                    raise serial.SerialException("not_connected")
                self._ser.write(payload)
                self._ser.flush()
            rospy.loginfo("Sent STM32 drop command: %s", line)
            return True
        except (serial.SerialException, OSError) as exc:
            self._publish_status("ERR:SERIAL_WRITE:%s" % exc)
            rospy.logwarn("STM32 drop serial write failed: %s", exc)
            self._close_serial()
            return False

    def _handle_serial_line(self, line: str) -> None:
        if line in VALID_STM32_LINES:
            status = line
        else:
            status = "UNKNOWN:%s" % line

        self._publish_status(status)
        rospy.loginfo("STM32 drop status: %s", status)

        if line.startswith("ACK:DROP:"):
            self._handle_drop_ack(line)

    def _handle_drop_ack(self, line: str) -> None:
        try:
            ack_drop = int(line.split(":")[-1])
        except ValueError:
            return

        should_lock = False
        with self._pending_lock:
            if self._pending_drop is not None and self._pending_drop[0] == ack_drop:
                self._pending_drop = None
                should_lock = self.lock_after_drop

        if should_lock:
            timer = threading.Timer(self.lock_delay, self._send_lock)
            timer.daemon = True
            timer.start()

    def _send_lock(self) -> None:
        if not rospy.is_shutdown() and not self._stop_event.is_set():
            self._write_line("LOCK")

    def _check_timeout(self, _event) -> None:
        now = time.monotonic()
        timed_out_drop = None

        with self._pending_lock:
            if self._pending_drop is not None:
                drop_id, sent_at = self._pending_drop
                if now - sent_at >= self.command_timeout:
                    timed_out_drop = drop_id
                    self._pending_drop = None

        if timed_out_drop is not None:
            self._publish_status("ERR:TIMEOUT:DROP:%d" % timed_out_drop)


if __name__ == "__main__":
    rospy.init_node("drop_controller_serial")
    DropControllerSerial()
    rospy.spin()
