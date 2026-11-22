#!/usr/bin/env python3
"""Receive BNO085 quaternion + linear acceleration over UDP and publish sensor_msgs/Imu."""

import json
import math
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class UdpImuReceiver(Node):
    def __init__(self):
        super().__init__("udp_imu_receiver")
        self.declare_parameter("port", 5005)
        self.declare_parameter("topic", "/imu/data")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("log_every_n", 30)

        port = int(self.get_parameter("port").value)
        topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))

        self.publisher = self.create_publisher(Imu, topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

        self.count = 0
        self.last_seq = None
        self.timer = self.create_timer(0.002, self.poll)
        self.get_logger().info(
            f"Listening UDP on 0.0.0.0:{port}; publishing quaternion + acceleration on {topic}"
        )

    @staticmethod
    def validate_vector(values, length):
        if not isinstance(values, list) or len(values) != length:
            return None
        try:
            result = [float(v) for v in values]
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in result):
            return None
        return result

    def poll(self):
        for _ in range(100):
            try:
                data, addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warning(f"UDP receive error: {exc}")
                break

            try:
                packet = json.loads(data.decode("utf-8"))
                q = self.validate_vector(packet.get("q"), 4)
                accel = self.validate_vector(packet.get("a"), 3)
                if q is None or accel is None:
                    continue

                norm = math.sqrt(sum(v * v for v in q))
                if not math.isfinite(norm) or norm < 1e-9:
                    continue
                q = [v / norm for v in q]
                seq = packet.get("seq")
            except (ValueError, TypeError, UnicodeDecodeError, AttributeError):
                continue

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel

            # No gyro is transmitted yet; -1 marks this field as unavailable.
            msg.angular_velocity_covariance[0] = -1.0
            # Unknown covariance: zeros are used rather than claiming zero uncertainty.
            self.publisher.publish(msg)

            self.count += 1
            self.last_seq = seq
            if self.count % self.log_every_n == 0:
                self.get_logger().info(
                    f"Received {self.count} packets from {addr[0]} | seq={seq} | "
                    f"a_lin=({accel[0]:+.3f},{accel[1]:+.3f},{accel[2]:+.3f}) m/s^2"
                )

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UdpImuReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
