#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String


def _normalize_qr(items):
    if len(items) != 4:
        return None
    side = items[3].strip().lower()
    if side not in ("left", "right"):
        return None
    class_a = items[1].strip().lower()
    class_b = items[2].strip().lower()
    if not class_a or not class_b:
        return None
    return "%s,%s,%s" % (class_a, class_b, side)


def _normalize_target(items):
    if len(items) < 5:
        return None
    label = items[1].strip().lower()
    if not label:
        return None
    try:
        confidence = float(items[2])
        center_x = float(items[3])
        center_y = float(items[4])
    except ValueError:
        return None
    return "%s,%.3f,%.1f,%.1f" % (label, confidence, center_x, center_y)


def main():
    rospy.init_node("k230_usb_serial_node", anonymous=False)

    serial_port = rospy.get_param("~serial_port", "/dev/ttyACM0")
    baud_rate = int(rospy.get_param("~baud_rate", 115200))
    serial_timeout = float(rospy.get_param("~serial_timeout", 1.0))
    reconnect_interval = float(rospy.get_param("~reconnect_interval", 2.0))
    msg_prefix = rospy.get_param("~msg_prefix", "ROS_MSG:")
    raw_topic = rospy.get_param("~raw_topic", "/craic/k230/raw")
    qr_result_topic = rospy.get_param("~qr_result_topic", "/craic/qr_result")
    target_detected_topic = rospy.get_param("~target_detected_topic", "/craic/target_detected")
    loop_hz = float(rospy.get_param("~loop_hz", 50.0))
    if loop_hz <= 0:
        loop_hz = 50.0

    raw_pub = rospy.Publisher(raw_topic, String, queue_size=10)
    qr_pub = rospy.Publisher(qr_result_topic, String, queue_size=1)
    target_pub = rospy.Publisher(target_detected_topic, String, queue_size=10)
    rate = rospy.Rate(loop_hz)
    ser = None

    rospy.loginfo(
        "k230_serial_node started: port=%s baud=%d raw=%s qr=%s target=%s",
        serial_port,
        baud_rate,
        raw_topic,
        qr_result_topic,
        target_detected_topic,
    )

    while not rospy.is_shutdown():
        if ser is None or not ser.is_open:
            try:
                ser = serial.Serial(serial_port, baud_rate, timeout=serial_timeout)
                rospy.loginfo("Connected to serial: %s @ %d", serial_port, baud_rate)
            except serial.SerialException as exc:
                rospy.logwarn_throttle(5.0, "Waiting for serial %s: %s", serial_port, exc)
                rospy.sleep(max(reconnect_interval, 0.2))
                continue

        try:
            raw_data = ser.readline()
            if not raw_data:
                rate.sleep()
                continue

            line = raw_data.decode("utf-8", errors="ignore").strip()
            if not line:
                rate.sleep()
                continue

            raw_pub.publish(String(data=line))
            if not line.startswith(msg_prefix):
                rospy.logdebug("Ignored non-K230 protocol line: %s", line)
                rate.sleep()
                continue

            payload = line[len(msg_prefix):].strip()
            items = [item.strip() for item in payload.split(",")]
            if not items:
                rate.sleep()
                continue

            msg_type = items[0].lower()
            if msg_type == "qr":
                normalized = _normalize_qr(items)
                if normalized is None:
                    rospy.logwarn("Invalid K230 QR line: %s", line)
                else:
                    qr_pub.publish(String(data=normalized))
                    rospy.loginfo("Published QR: %s", normalized)
            elif msg_type == "target":
                normalized = _normalize_target(items)
                if normalized is None:
                    rospy.logwarn("Invalid K230 target line: %s", line)
                else:
                    target_pub.publish(String(data=normalized))
                    rospy.loginfo("Published target: %s", normalized)
            else:
                rospy.logwarn("Unknown K230 ROS_MSG type '%s': %s", msg_type, line)

            rate.sleep()
        except (serial.SerialException, OSError) as exc:
            rospy.logwarn("Serial disconnected, reconnecting: %s", exc)
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            rospy.sleep(max(reconnect_interval, 0.2))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Serial read error: %s", exc)
            rate.sleep()

    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
