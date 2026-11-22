#!/usr/bin/env python3
"""Pitch-triggered dynamic wall manager for Gazebo Fortress / ROS 2 Humble.

This node ONLY creates and removes the wall, at the pose requested by
gecko_twin_node.py, and tells the twin when the wall exists. Keep
imu_displacement_estimator.py running for /imu/estimated_pitch_deg.
"""

import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from visualization_msgs.msg import Marker

from gecko_motion import finite_pose

LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
WALL_SIZE = (0.10, 2.0, 2.0)


class WallClimbManager(Node):
    def __init__(self):
        super().__init__('wall_climb_manager')

        default_wall = str(
            Path(__file__).resolve().parents[1] / 'models/wall/wall.sdf'
        )

        defaults = {
            'pitch_topic': '/imu/estimated_pitch_deg',
            'create_service': '/world/empty/create',
            'remove_service': '/world/empty/remove',
            'wall_name': 'gecko_wall',
            'wall_sdf': default_wall,
            'enter_pitch_deg': -60.0,
            'exit_pitch_deg': -50.0,
            'enter_hold_sec': 0.30,
            'exit_hold_sec': 0.50,
            'log_every_n': 30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value

        self.pitch_topic = str(p('pitch_topic'))
        self.create_service_name = str(p('create_service'))
        self.remove_service_name = str(p('remove_service'))
        self.wall_name = str(p('wall_name'))
        self.wall_sdf = os.path.abspath(os.path.expanduser(str(p('wall_sdf'))))

        self.enter_pitch = float(p('enter_pitch_deg'))
        self.exit_pitch = float(p('exit_pitch_deg'))
        self.enter_hold = float(p('enter_hold_sec'))
        self.exit_hold = float(p('exit_hold_sec'))
        self.log_every_n = max(1, int(p('log_every_n')))

        if self.exit_pitch <= self.enter_pitch:
            raise ValueError(
                'exit_pitch_deg must be greater than enter_pitch_deg '
                '(for example enter=-60, exit=-50).'
            )
        if self.enter_hold < 0 or self.exit_hold < 0:
            raise ValueError('Hold times cannot be negative.')
        if not os.path.isfile(self.wall_sdf):
            raise FileNotFoundError(
                f'Wall SDF not found: {self.wall_sdf}'
            )

        self.spawn_client = self.create_client(
            SpawnEntity, self.create_service_name
        )
        self.delete_client = self.create_client(
            DeleteEntity, self.remove_service_name
        )
        self.pitch_sub = self.create_subscription(
            Float32, self.pitch_topic, self.pitch_callback, 10
        )
        self.target_sub = self.create_subscription(
            PoseStamped, '/gecko/wall/target_pose', self.target_callback, 10
        )
        self.active_pub = self.create_publisher(Bool, '/gecko/wall/active', LATCHED_QOS)
        self.pose_pub = self.create_publisher(PoseStamped, '/gecko/wall/pose', LATCHED_QOS)
        self.marker_pub = self.create_publisher(Marker, '/gecko/wall/marker', LATCHED_QOS)

        # FLOOR -> SPAWNING -> WALL_CLIMB -> DELETING -> FLOOR
        self.state = 'FLOOR'
        self.condition_since: Optional[float] = None
        self.target_pose: Optional[PoseStamped] = None
        self.spawned_pose: Optional[PoseStamped] = None
        self.last_pitch = 0.0
        self.sample_count = 0
        self.wall_exists = False
        self.operation_in_progress = False
        self.warned_create_unavailable = False
        self.warned_remove_unavailable = False
        self.active_pub.publish(Bool(data=False))

        self.get_logger().info(f'Subscribed to {self.pitch_topic}')
        self.get_logger().info(
            f'Gazebo create={self.create_service_name}, '
            f'remove={self.remove_service_name}'
        )
        self.get_logger().info(
            f'Wall SDF: {self.wall_sdf}; name={self.wall_name}'
        )
        self.get_logger().info(
            f'Entry: pitch <= {self.enter_pitch:.1f} deg '
            f'for {self.enter_hold:.2f}s; '
            f'exit: pitch >= {self.exit_pitch:.1f} deg '
            f'for {self.exit_hold:.2f}s'
        )
        self.get_logger().warning(
            'This node only manages wall creation/removal. '
            'It does not move the robot.'
        )

    def target_callback(self, msg: PoseStamped):
        position, orientation = msg.pose.position, msg.pose.orientation
        if finite_pose((position.x, position.y, position.z, orientation.x,
                        orientation.y, orientation.z, orientation.w)):
            self.target_pose = msg

    def publish_marker(self, action: int):
        marker = Marker()
        marker.ns = self.wall_name
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = action
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        if self.spawned_pose is not None:
            marker.pose = self.spawned_pose.pose
        marker.scale.x, marker.scale.y, marker.scale.z = WALL_SIZE
        marker.color.r, marker.color.g, marker.color.b = 0.65, 0.68, 0.72
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def pitch_callback(self, msg: Float32):
        pitch = float(msg.data)
        if not (pitch == pitch and abs(pitch) < 1e6):
            self.get_logger().warning('Ignoring invalid pitch sample.')
            return

        self.last_pitch = pitch
        self.sample_count += 1
        now = time.monotonic()

        if self.sample_count % self.log_every_n == 0:
            self.get_logger().info(
                f'state={self.state} | pitch={pitch:+.2f} deg'
            )

        if self.operation_in_progress:
            return

        if self.state == 'FLOOR':
            if pitch <= self.enter_pitch:
                if self.condition_since is None:
                    self.condition_since = now
                elif now - self.condition_since >= self.enter_hold:
                    self.spawn_wall()
            else:
                self.condition_since = None

        elif self.state == 'WALL_CLIMB':
            if pitch >= self.exit_pitch:
                if self.condition_since is None:
                    self.condition_since = now
                elif now - self.condition_since >= self.exit_hold:
                    self.delete_wall()
            else:
                self.condition_since = None

    def spawn_wall(self):
        if self.wall_exists:
            self.state = 'WALL_CLIMB'
            self.condition_since = None
            return

        if not self.spawn_client.service_is_ready():
            if not self.warned_create_unavailable:
                self.get_logger().warning(
                    f'Waiting for Gazebo spawn service {self.create_service_name}'
                )
                self.warned_create_unavailable = True
            self.condition_since = time.monotonic() - self.enter_hold
            return

        if self.target_pose is None:
            self.condition_since = time.monotonic() - self.enter_hold
            return

        self.spawned_pose = self.target_pose
        request = SpawnEntity.Request()
        factory = request.entity_factory
        factory.name = self.wall_name
        factory.sdf_filename = self.wall_sdf
        factory.allow_renaming = False

        # Set the model's world pose explicitly. wall.sdf itself is authored
        # at the origin to avoid applying the offset twice.
        factory.pose = self.spawned_pose.pose

        self.state = 'SPAWNING'
        self.condition_since = None
        self.operation_in_progress = True
        self.get_logger().info(
            f'Pitch {self.last_pitch:+.2f} deg held below threshold. '
            f'Creating wall "{self.wall_name}".'
        )
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_done)

    def spawn_done(self, future):
        self.operation_in_progress = False
        try:
            response = future.result()
            if response is not None and response.success:
                self.wall_exists = True
                self.state = 'WALL_CLIMB'
                self.condition_since = None
                self.warned_create_unavailable = False
                self.pose_pub.publish(self.spawned_pose)
                self.active_pub.publish(Bool(data=True))
                self.publish_marker(Marker.ADD)
                self.get_logger().info(
                    f'Wall "{self.wall_name}" created successfully. '
                    'Wall-climb state active.'
                )
            else:
                self.state = 'FLOOR'
                self.condition_since = None
                self.get_logger().error(
                    'Gazebo did not confirm wall creation. '
                    'Check the SDF path and create service.'
                )
        except Exception as exc:
            self.state = 'FLOOR'
            self.condition_since = None
            self.get_logger().error(f'Wall creation service failed: {exc}')

    def delete_wall(self):
        if not self.wall_exists:
            self.state = 'FLOOR'
            self.condition_since = None
            return

        if not self.delete_client.service_is_ready():
            if not self.warned_remove_unavailable:
                self.get_logger().warning(
                    f'Waiting for Gazebo remove service {self.remove_service_name}'
                )
                self.warned_remove_unavailable = True
            self.condition_since = time.monotonic() - self.exit_hold
            return

        request = DeleteEntity.Request()
        request.entity.name = self.wall_name
        request.entity.type = 2  # MODEL; explicit type is important in Fortress.

        self.state = 'DELETING'
        self.condition_since = None
        self.operation_in_progress = True
        self.get_logger().info(
            f'Pitch {self.last_pitch:+.2f} deg held above exit threshold. '
            f'Removing wall "{self.wall_name}".'
        )
        future = self.delete_client.call_async(request)
        future.add_done_callback(self.delete_done)

    def delete_done(self, future):
        self.operation_in_progress = False
        try:
            response = future.result()
            if response is not None and response.success:
                self.wall_exists = False
                self.state = 'FLOOR'
                self.condition_since = None
                self.warned_remove_unavailable = False
                self.active_pub.publish(Bool(data=False))
                self.publish_marker(Marker.DELETE)
                self.get_logger().info(
                    f'Wall "{self.wall_name}" removed successfully. '
                    'Floor state active.'
                )
            else:
                self.state = 'WALL_CLIMB'
                self.condition_since = None
                self.get_logger().error(
                    'Gazebo did not confirm wall removal. '
                    'The wall-climb state is retained.'
                )
        except Exception as exc:
            self.state = 'WALL_CLIMB'
            self.condition_since = None
            self.get_logger().error(f'Wall removal service failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WallClimbManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f'wall_climb_manager stopped: {exc}')
        else:
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
