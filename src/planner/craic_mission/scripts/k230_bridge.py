#!/usr/bin/env python3
import socket
from typing import Iterable, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class K230Bridge:
    def __init__(self) -> None:
        self.mode = rospy.get_param("~mode", "udp")
        self.udp_host = rospy.get_param("~udp_host", "0.0.0.0")
        self.udp_port = int(rospy.get_param("~udp_port", 9000))
        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.serial_baud = int(rospy.get_param("~serial_baud", 115200))
        self.serial_timeout = float(rospy.get_param("~serial_timeout", 0.2))
        self.reconnect_interval = float(rospy.get_param("~reconnect_interval", 2.0))
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.qr_result_topic = rospy.get_param("~qr_result_topic", "/craic/qr_result")
        self.target_detected_topic = rospy.get_param("~target_detected_topic", "/craic/target_detected")
        self.ring_pose_topic = rospy.get_param("~ring_pose_topic", "/craic/ring_pose")
        self.special_target_topic = rospy.get_param("~special_target_topic", "/craic/special_target")
        self.landing_marker_topic = rospy.get_param("~landing_marker_topic", "/craic/landing_marker")
        self.raw_topic = rospy.get_param("~raw_topic", "/craic/k230/raw")

        self.qr_pub = rospy.Publisher(self.qr_result_topic, String, queue_size=1)
        self.target_pub = rospy.Publisher(self.target_detected_topic, String, queue_size=10)
        self.ring_pub = rospy.Publisher(self.ring_pose_topic, PoseStamped, queue_size=1)
        self.special_pub = rospy.Publisher(self.special_target_topic, String, queue_size=10)
        self.landing_pub = rospy.Publisher(self.landing_marker_topic, String, queue_size=10)
        self.raw_pub = rospy.Publisher(self.raw_topic, String, queue_size=10)

    def run(self) -> None:
        if self.mode == "udp":
            self._run_udp()
        elif self.mode == "serial":
            self._run_serial()
        else:
            raise ValueError("unsupported K230 bridge mode: %s" % self.mode)

    def _run_udp(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.udp_host, self.udp_port))
        sock.settimeout(0.2)
        rospy.logwarn("[k230 bridge] listening UDP %s:%d", self.udp_host, self.udp_port)

        while not rospy.is_shutdown():
            try:
                data, _addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            text = data.decode("utf-8", errors="ignore")
            for line in text.splitlines():
                self._handle_line(line.strip())

    def _run_serial(self) -> None:
        try:
            import serial
        except ImportError:
            rospy.logerr("pyserial is not installed; use UDP mode or install python3-serial")
            return

        ser = None
        rospy.logwarn("[k230 bridge] listening serial %s @ %d", self.serial_port, self.serial_baud)

        while not rospy.is_shutdown():
            if ser is None or not ser.is_open:
                try:
                    ser = serial.Serial(self.serial_port, self.serial_baud, timeout=self.serial_timeout)
                    rospy.logwarn("[k230 bridge] serial connected: %s @ %d", self.serial_port, self.serial_baud)
                except serial.SerialException as exc:
                    rospy.logwarn_throttle(5.0, "[k230 bridge] waiting serial %s: %s", self.serial_port, exc)
                    rospy.sleep(max(self.reconnect_interval, 0.2))
                    continue

            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self._handle_line(line)
            except (serial.SerialException, OSError) as exc:
                rospy.logwarn("[k230 bridge] serial disconnected, reconnecting: %s", exc)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                rospy.sleep(max(self.reconnect_interval, 0.2))

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _handle_line(self, line: str) -> None:
        if not line:
            return

        self.raw_pub.publish(String(data=line))
        text = line.strip()
        upper = text.upper()

        if upper.startswith("ROS_MSG:"):
            self._handle_ros_msg(text[8:].strip(), line)
            return

        if upper.startswith("QR:"):
            self._publish_qr(text[3:])
        elif upper.startswith("TARGET:"):
            self._publish_target(text[7:])
        elif upper.startswith("RING:"):
            self._publish_ring(text[5:], require_confidence=False)
        else:
            rospy.logwarn("[k230 bridge] unknown line: %s", line)

    def _handle_ros_msg(self, payload: str, raw_line: str) -> None:
        parts = [part.strip() for part in payload.split(",")]
        if not parts or not parts[0]:
            rospy.logwarn("[k230 bridge] ignore empty ROS_MSG line: %s", raw_line)
            return

        msg_type = parts[0].lower()
        body = ",".join(parts[1:])
        if msg_type == "qr":
            self._publish_qr(body)
        elif msg_type == "target":
            self._publish_target(body)
        elif msg_type == "ring":
            self._publish_ring(body, require_confidence=True)
        elif msg_type == "special":
            self._publish_special(body)
        elif msg_type == "landing":
            self._publish_landing(body)
        else:
            rospy.logwarn("[k230 bridge] unknown ROS_MSG type '%s' in line: %s", msg_type, raw_line)

    def _publish_qr(self, payload: str) -> None:
        qr = payload.strip()
        parts = [part.strip() for part in qr.split(",")]
        if len(parts) != 3 or parts[2].lower() not in ("left", "right"):
            rospy.logwarn("[k230 bridge] ignore invalid QR payload: %s", payload)
            return

        normalized = "%s,%s,%s" % (parts[0].lower(), parts[1].lower(), parts[2].lower())
        self.qr_pub.publish(String(data=normalized))
        rospy.logwarn("[k230 bridge] QR %s", normalized)

    def _publish_target(self, payload: str) -> None:
        parts = [part.strip() for part in payload.split(",")]
        if len(parts) < 4:
            rospy.logwarn("[k230 bridge] ignore invalid TARGET payload: %s", payload)
            return
        try:
            label = parts[0].lower()
            confidence = float(parts[1])
            center_x = float(parts[2])
            center_y = float(parts[3])
        except ValueError:
            rospy.logwarn("[k230 bridge] ignore invalid TARGET payload: %s", payload)
            return
        if not label:
            rospy.logwarn("[k230 bridge] ignore invalid TARGET payload: %s", payload)
            return

        normalized = "%s,%.3f,%.1f,%.1f" % (label, confidence, center_x, center_y)
        self.target_pub.publish(String(data=normalized))
        rospy.logwarn("[k230 bridge] TARGET %s", normalized)

    def _publish_ring(self, payload: str, require_confidence: bool = False) -> None:
        values = self._parse_floats(payload.split(","))
        min_len = 4 if require_confidence else 3
        if values is None or len(values) < min_len:
            rospy.logwarn("[k230 bridge] ignore invalid RING payload: %s", payload)
            return

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = values[0]
        msg.pose.position.y = values[1]
        msg.pose.position.z = values[2]
        msg.pose.orientation.w = 1.0
        self.ring_pub.publish(msg)
        if len(values) >= 4:
            rospy.logwarn("[k230 bridge] RING %.2f %.2f %.2f confidence=%.2f", values[0], values[1], values[2], values[3])
        else:
            rospy.logwarn("[k230 bridge] RING %.2f %.2f %.2f", values[0], values[1], values[2])

    def _publish_special(self, payload: str) -> None:
        values = self._parse_floats(payload.split(","))
        if values is None or len(values) < 3:
            rospy.logwarn("[k230 bridge] ignore invalid SPECIAL payload: %s", payload)
            return

        normalized = "%.3f,%.1f,%.1f" % (values[0], values[1], values[2])
        self.special_pub.publish(String(data=normalized))
        rospy.logwarn("[k230 bridge] SPECIAL %s", normalized)

    def _publish_landing(self, payload: str) -> None:
        parts = [part.strip() for part in payload.split(",")]
        if len(parts) < 4:
            rospy.logwarn("[k230 bridge] ignore invalid LANDING payload: %s", payload)
            return

        side = parts[0].lower()
        if side not in ("left", "right"):
            rospy.logwarn("[k230 bridge] ignore invalid LANDING payload: %s", payload)
            return

        values = self._parse_floats(parts[1:4])
        if values is None:
            rospy.logwarn("[k230 bridge] ignore invalid LANDING payload: %s", payload)
            return

        normalized = "%s,%.3f,%.3f,%.3f" % (side, values[0], values[1], values[2])
        self.landing_pub.publish(String(data=normalized))
        rospy.logwarn("[k230 bridge] LANDING %s", normalized)

    @staticmethod
    def _parse_floats(items: Iterable[str]) -> Optional[list]:
        values = []
        for item in items:
            try:
                values.append(float(item.strip()))
            except ValueError:
                return None
        return values


if __name__ == "__main__":
    rospy.init_node("k230_bridge")
    K230Bridge().run()
