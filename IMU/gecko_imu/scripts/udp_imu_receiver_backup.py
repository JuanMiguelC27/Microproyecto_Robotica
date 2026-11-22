#!/usr/bin/env python3
"""Receive BNO085 XYZW quaternions over UDP and publish ROS 2 sensor_msgs/Imu."""
import json
import math
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class UdpImuReceiver(Node):
    def __init__(self):
        super().__init__('udp_imu_receiver')
        self.declare_parameter('port', 5005)
        self.declare_parameter('topic', '/imu/data')
        port = int(self.get_parameter('port').value)
        topic = str(self.get_parameter('topic').value)
        self.publisher = self.create_publisher(Imu, topic, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)
        self.count = 0
        self.timer = self.create_timer(0.005, self.poll)
        self.get_logger().info(f'Listening UDP on 0.0.0.0:{port}; publishing {topic}')

    def poll(self):
        for _ in range(30):
            try:
                data, addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warning(f'UDP receive error: {exc}')
                break
            try:
                packet = json.loads(data.decode('utf-8'))
                q = packet['q']
                if not isinstance(q, list) or len(q) != 4:
                    continue
                q = [float(v) for v in q]
                norm = math.sqrt(sum(v*v for v in q))
                if not all(math.isfinite(v) for v in q) or norm < 1e-9:
                    continue
                q = [v/norm for v in q]
            except (ValueError, TypeError, KeyError, UnicodeDecodeError):
                continue
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
            # Orientation covariance is unknown, not zero.
            msg.orientation_covariance[0] = 0.0
            # Angular velocity and linear acceleration are not supplied.
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
            self.publisher.publish(msg)
            self.count += 1
            if self.count % 30 == 0:
                self.get_logger().info(f'Received {self.count} IMU packets from {addr[0]}')

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = UdpImuReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
