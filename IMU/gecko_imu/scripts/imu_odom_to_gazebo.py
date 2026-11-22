
#!/usr/bin/env python3
"""Apply the estimated IMU odometry pose to a Gazebo Fortress model.

Subscribes:
    /imu/estimated_odom (nav_msgs/Odometry)

Calls:
    /world/empty/set_pose (ros_gz_interfaces/SetEntityPose)

IMPORTANT:
    Run this INSTEAD OF imu_to_gazebo.py.
    Both nodes must not command the same model simultaneously.

The odometry frame is assumed to align with the Gazebo world at startup.
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose


class ImuOdomToGazebo(Node):

    def __init__(self):

        super().__init__('imu_odom_to_gazebo')

        defaults = {
            'odom_topic': '/imu/estimated_odom',
            'service_name': '/world/empty/set_pose',
            'entity_name': 'gecko',
            'entity_id': 10,
            'entity_type': 2,
            'update_rate_hz': 15.0,
            'max_message_age_sec': 0.5,
            'x_offset': 0.0,
            'y_offset': 0.0,
            'z_offset': 0.000004376,
            'log_every_n': 30,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value

        self.odom_topic = str(p('odom_topic'))
        self.service_name = str(p('service_name'))
        self.entity_name = str(p('entity_name'))

        self.entity_id = int(p('entity_id'))
        self.entity_type = int(p('entity_type'))

        self.max_age = float(p('max_message_age_sec'))

        self.offset = (
            float(p('x_offset')),
            float(p('y_offset')),
            float(p('z_offset')),
        )

        rate = float(p('update_rate_hz'))

        self.log_every_n = max(
            1, int(p('log_every_n'))
        )

        if not math.isfinite(rate) or rate <= 0:
            raise ValueError(
                'update_rate_hz must be positive and finite'
            )

        self.latest_pose = None
        self.latest_receipt_time = None

        self.request_in_flight = False
        self.sent_count = 0

        self.warned_service = False
        self.warned_stale = False

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.client = self.create_client(
            SetEntityPose,
            self.service_name
        )

        self.timer = self.create_timer(
            1.0 / rate,
            self.update_pose
        )

        self.get_logger().info(
            f'Subscribed to {self.odom_topic}'
        )

        self.get_logger().info(
            f'Gazebo pose service: {self.service_name}'
        )

        self.get_logger().info(
            f'Controlling model={self.entity_name}, '
            f'id={self.entity_id}, rate={rate:.1f} Hz'
        )

        self.get_logger().warning(
            'This node controls the full XYZ + quaternion pose. '
            'Stop imu_to_gazebo.py before running it.'
        )

    def odom_callback(self, msg: Odometry):

        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation

        values = [
            pos.x, pos.y, pos.z,
            q.x, q.y, q.z, q.w
        ]

        if not all(math.isfinite(float(v)) for v in values):

            self.get_logger().warning(
                'Ignoring non-finite odometry pose'
            )
            return

        norm = math.sqrt(
            q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        )

        if norm < 1e-9:

            self.get_logger().warning(
                'Ignoring near-zero odometry quaternion'
            )
            return

        self.latest_pose = (
            (
                float(pos.x),
                float(pos.y),
                float(pos.z)
            ),
            (
                float(q.x)/norm,
                float(q.y)/norm,
                float(q.z)/norm,
                float(q.w)/norm
            )
        )

        self.latest_receipt_time = (
            self.get_clock().now().nanoseconds * 1e-9
        )

    def update_pose(self):

        if (
            self.latest_pose is None
            or self.latest_receipt_time is None
        ):
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        age = now - self.latest_receipt_time

        if age < 0 or age > self.max_age:

            if not self.warned_stale:

                self.get_logger().warning(
                    f'Odometry is stale ({age:.3f}s); '
                    'holding Gazebo pose'
                )

                self.warned_stale = True

            return

        self.warned_stale = False

        if self.request_in_flight:
            return

        if not self.client.service_is_ready():

            if not self.warned_service:

                self.get_logger().warning(
                    f'Waiting for {self.service_name}; '
                    'check ros_gz_bridge'
                )

                self.warned_service = True

            return

        self.warned_service = False

        position, quaternion = self.latest_pose

        req = SetEntityPose.Request()

        req.entity.id = self.entity_id
        req.entity.name = self.entity_name
        req.entity.type = self.entity_type

        req.pose.position.x = (
            position[0] + self.offset[0]
        )

        req.pose.position.y = (
            position[1] + self.offset[1]
        )

        req.pose.position.z = (
            position[2] + self.offset[2]
        )

        req.pose.orientation.x = quaternion[0]
        req.pose.orientation.y = quaternion[1]
        req.pose.orientation.z = quaternion[2]
        req.pose.orientation.w = quaternion[3]

        self.request_in_flight = True

        future = self.client.call_async(req)

        future.add_done_callback(
            self.service_done
        )

    def service_done(self, future):

        self.request_in_flight = False

        try:

            response = future.result()

            if response is None or not response.success:

                self.get_logger().error(
                    f'Gazebo rejected pose update for '
                    f'{self.entity_name}'
                )

                return

            self.sent_count += 1

            if (
                self.sent_count % self.log_every_n == 0
                and self.latest_pose
            ):

                pos, _ = self.latest_pose

                self.get_logger().info(
                    f'Pose updates={self.sent_count} | '
                    f'position=({pos[0]:+.3f}, '
                    f'{pos[1]:+.3f}, '
                    f'{pos[2]:+.3f}) m'
                )

        except Exception as exc:

            self.get_logger().error(
                f'SetEntityPose call failed: {exc}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = ImuOdomToGazebo()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

