#!/usr/bin/env python3
"""Continuous synthetic IMU publisher for testing gecko_imu in ROS 2 Humble.

Publishes sensor_msgs/msg/Imu on /imu/data with a sinusoidal rotation about
one selected axis (roll, pitch, yaw) or all three axes simultaneously.

Examples:
  ros2 run ... (not required; this can be run directly with Python)
  python3 imu_test_publisher.py --axis roll
  python3 imu_test_publisher.py --axis pitch --amplitude-deg 30
  python3 imu_test_publisher.py --axis yaw --amplitude-deg 45 --frequency 0.15
  python3 imu_test_publisher.py --axis all
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert roll, pitch, yaw (radians) to ROS quaternion (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


class SyntheticImuPublisher(Node):
    def __init__(self, axis: str, amplitude_deg: float,
                 frequency_hz: float, rate_hz: float, topic: str):
        super().__init__('imu_test_publisher')

        if axis not in ('roll', 'pitch', 'yaw', 'all'):
            raise ValueError("axis must be roll, pitch, yaw, or all")
        if not 0.0 < amplitude_deg <= 90.0:
            raise ValueError('amplitude-deg must be in (0, 90]')
        if frequency_hz <= 0.0 or rate_hz <= 0.0:
            raise ValueError('frequency and rate must be positive')

        self.axis = axis
        self.amplitude = math.radians(amplitude_deg)
        self.frequency = frequency_hz
        self.start_time = self.get_clock().now()
        self.publisher = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_sample)
        self.topic = topic

        self.get_logger().info(
            f'Publishing synthetic IMU on {topic}: axis={axis}, '
            f'amplitude=+/-{amplitude_deg:.1f} deg, '
            f'frequency={frequency_hz:.3f} Hz, rate={rate_hz:.1f} Hz'
        )
        self.get_logger().info('Press Ctrl+C to stop.')

    def publish_sample(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        wave = math.sin(2.0 * math.pi * self.frequency * elapsed)

        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        if self.axis == 'roll':
            roll = self.amplitude * wave
        elif self.axis == 'pitch':
            pitch = self.amplitude * wave
        elif self.axis == 'yaw':
            yaw = self.amplitude * wave
        else:
            # Different phase offsets make the three axes distinguishable.
            roll = self.amplitude * wave
            pitch = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * elapsed + 2.0 * math.pi / 3.0
            )
            yaw = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * elapsed + 4.0 * math.pi / 3.0
            )

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Approximate orientation covariance (rad^2); this is synthetic data.
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        self.publisher.publish(msg)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Publish a continuous synthetic IMU orientation on ROS 2.'
    )
    parser.add_argument('--axis', choices=['roll', 'pitch', 'yaw', 'all'],
                        default='roll', help='Axis to oscillate (default: roll)')
    parser.add_argument('--amplitude-deg', type=float, default=25.0,
                        help='Peak angle in degrees, 0 < value <= 90 (default: 25)')
    parser.add_argument('--frequency', type=float, default=0.15,
                        help='Oscillation frequency in Hz (default: 0.15)')
    parser.add_argument('--rate', type=float, default=30.0,
                        help='Publishing rate in Hz (default: 30)')
    parser.add_argument('--topic', default='/imu/data',
                        help='ROS topic (default: /imu/data)')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = None
    try:
        node = SyntheticImuPublisher(
            axis=args.axis,
            amplitude_deg=args.amplitude_deg,
            frequency_hz=args.frequency,
            rate_hz=args.rate,
            topic=args.topic,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (ValueError, Exception) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'Error: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
