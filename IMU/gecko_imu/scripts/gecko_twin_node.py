#!/usr/bin/env python3
"""Digital twin of the Gecko for ROS 2 Humble / Gazebo Fortress.

Orientation comes only from the real vehicle's IMU. Wheel, servo and turbine
motion, and the displacement of the twin, come from the RViz control panel.
This node is the single authority over the model pose and its joints.

Mount parameters describe the fixed transform Base_link -> imu_link (RPY degrees).
The incoming IMU quaternion is assumed to follow ROS REP-145 conventions.
"""

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int8
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from gecko_geometry import IDENTITY, Quat, q_from_rpy
from gecko_joints import (
    SERVO_JOINTS,
    TURBINE_JOINTS,
    VELOCITY_JOINTS,
    WHEEL_JOINTS,
    position_topic,
    velocity_topic,
)
from gecko_motion import (
    DRIVE_STOP,
    GeckoMotion,
    clamp_servo,
    clamp_turbine,
    effective_drive,
    finite_pose,
    sanitize_drive,
    wall_from_center,
    wheel_velocities,
)

SET_POSE_TIMEOUT_S = 1.0
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def q_normalize(q: Quat) -> Quat:
    n = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(n) or n < 1e-12:
        raise ValueError('Invalid or near-zero quaternion')
    return tuple(v / n for v in q)  # type: ignore[return-value]


def q_conjugate(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def q_multiply(a: Quat, b: Quat) -> Quat:
    """Hamilton product a*b in ROS xyzw order."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GeckoTwinNode(Node):
    def __init__(self):
        super().__init__('gecko_twin_node')

        defaults = {
            'imu_topic': '/imu/data',
            'service_name': '/world/empty/set_pose',
            'entity_name': 'gecko',
            'entity_id': 0,
            'entity_type': 2,  # Gazebo MODEL
            'world_frame': 'world',
            'base_frame': 'Base_link',
            'imu_frame': 'imu_link',
            # Fixed orientation of imu_link relative to Base_link.
            # The model's +X ("front": Wheel_F*, Turbine_F) is the real vehicle's
            # BACK as seen by the BNO085 frame, so the IMU frame is yawed 180 deg
            # about Z with respect to Base_link. With the startup calibration this
            # is equivalent to q' = Rz(pi) * dq * Rz(pi)^-1: roll and pitch change
            # sign, yaw is kept. Set to 0.0 to undo the correction.
            'imu_mount_roll_deg': 0.0,
            'imu_mount_pitch_deg': 0.0,
            'imu_mount_yaw_deg': 180.0,
            # IMU origin relative to Base_link, metres.
            'imu_offset_x': 0.0, 'imu_offset_y': 0.0, 'imu_offset_z': 0.0,
            'auto_calibrate': True,
            'calibration_service': '/imu/calibrate',
            'update_rate_hz': 30.0,
            'speed': 0.15,
            'wheel_radius': 0.015,
            'turbine_max_rad_s': 60.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value
        self.service_name = str(p('service_name'))
        self.entity_name = str(p('entity_name'))
        self.entity_id = int(p('entity_id'))
        self.entity_type = int(p('entity_type'))
        self.world_frame = str(p('world_frame'))
        self.base_frame = str(p('base_frame'))
        self.imu_frame = str(p('imu_frame'))
        self.auto_calibrate = bool(p('auto_calibrate'))
        self.imu_offset = (
            float(p('imu_offset_x')), float(p('imu_offset_y')),
            float(p('imu_offset_z')),
        )
        mount_rpy_deg = (
            float(p('imu_mount_roll_deg')),
            float(p('imu_mount_pitch_deg')),
            float(p('imu_mount_yaw_deg')),
        )
        self.q_base_imu = q_normalize(q_from_rpy(*(math.radians(v) for v in mount_rpy_deg)))

        rate = float(p('update_rate_hz'))
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('update_rate_hz must be positive and finite')
        speed = float(p('speed'))
        self.wheel_omega = speed / float(p('wheel_radius'))
        self.turbine_max = float(p('turbine_max_rad_s'))

        self.motion = GeckoMotion(speed=speed)
        self.latest_q_world_imu: Optional[Quat] = None
        self.q_reference_world_base: Optional[Quat] = None

        self.drive = DRIVE_STOP
        self.drive_stamp = -math.inf
        self.servo_angle = 0.0
        self.turbine_f = 0.0
        self.turbine_r = 0.0
        self.joint_positions: Dict[str, float] = {
            j: 0.0 for j in VELOCITY_JOINTS + SERVO_JOINTS
        }

        self.wall_active = False
        self.wall_center: Optional[Tuple[Tuple[float, float, float], float]] = None

        self.request_stamp: Optional[float] = None
        self.warned_service_unavailable = False
        self.last_tick = time.monotonic()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.client = self.create_client(SetEntityPose, self.service_name)

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/gecko/wall/target_pose', 10)
        self.controls_pub = self.create_publisher(
            Float64MultiArray, '/gecko/state/controls', LATCHED_QOS
        )
        self.velocity_pubs = {
            j: self.create_publisher(Float64, velocity_topic(j), 10) for j in VELOCITY_JOINTS
        }
        self.position_pubs = {
            j: self.create_publisher(Float64, position_topic(j), 10) for j in SERVO_JOINTS
        }

        self.create_subscription(
            Imu, str(p('imu_topic')), self.imu_callback, qos_profile_sensor_data
        )
        self.create_subscription(Int8, '/gecko/cmd/drive', self.drive_callback, 10)
        self.create_subscription(Float64, '/gecko/cmd/servo_angle', self.servo_callback, 10)
        self.create_subscription(Float64, '/gecko/cmd/turbine_f', self.turbine_f_callback, 10)
        self.create_subscription(Float64, '/gecko/cmd/turbine_r', self.turbine_r_callback, 10)
        self.create_subscription(Bool, '/gecko/wall/active', self.wall_active_callback, LATCHED_QOS)
        self.create_subscription(
            PoseStamped, '/gecko/wall/pose', self.wall_pose_callback, LATCHED_QOS
        )
        self.calibration_service = self.create_service(
            Trigger, str(p('calibration_service')), self.calibrate_callback
        )

        self.timer = self.create_timer(1.0 / rate, self.update)
        self.publish_static_imu_transform()
        self.publish_controls()

        self.get_logger().info(
            f'Gecko twin ready: model={self.entity_name}, service={self.service_name}'
        )
        self.get_logger().info(f'IMU mount RPY Base_link -> imu_link (deg): {mount_rpy_deg}')

    # ------------------------------ Commands ------------------------------

    def drive_callback(self, msg: Int8):
        self.drive = sanitize_drive(int(msg.data))
        self.drive_stamp = time.monotonic()

    def servo_callback(self, msg: Float64):
        self.servo_angle = clamp_servo(float(msg.data), self.servo_angle)
        self.publish_controls()

    def turbine_f_callback(self, msg: Float64):
        self.turbine_f = clamp_turbine(float(msg.data), self.turbine_f)
        self.publish_controls()

    def turbine_r_callback(self, msg: Float64):
        self.turbine_r = clamp_turbine(float(msg.data), self.turbine_r)
        self.publish_controls()

    def publish_controls(self):
        self.controls_pub.publish(Float64MultiArray(
            data=[self.servo_angle, self.turbine_f, self.turbine_r]
        ))

    # -------------------------------- Wall --------------------------------

    def wall_active_callback(self, msg: Bool):
        self.wall_active = bool(msg.data)
        if self.wall_active:
            self.apply_wall()
        else:
            self.wall_center = None
            self.motion.set_wall(None)

    def wall_pose_callback(self, msg: PoseStamped):
        position = msg.pose.position
        orientation = msg.pose.orientation
        if not finite_pose((position.x, position.y, position.z, orientation.x,
                            orientation.y, orientation.z, orientation.w)):
            self.get_logger().warning('Invalid wall pose ignored.')
            return
        self.wall_center = ((position.x, position.y, position.z), yaw_of(orientation))
        self.apply_wall()

    def apply_wall(self):
        if self.wall_active and self.wall_center is not None:
            self.motion.set_wall(wall_from_center(*self.wall_center))

    # --------------------------------- IMU --------------------------------

    def publish_static_imu_transform(self):
        """Publish the fixed Base_link -> imu_link mounting transform on /tf_static."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.imu_frame
        t.transform.translation.x = self.imu_offset[0]
        t.transform.translation.y = self.imu_offset[1]
        t.transform.translation.z = self.imu_offset[2]
        t.transform.rotation.x, t.transform.rotation.y = self.q_base_imu[0:2]
        t.transform.rotation.z, t.transform.rotation.w = self.q_base_imu[2:4]
        self.tf_static_broadcaster.sendTransform(t)

    def sensor_to_base(self, q_world_imu: Quat) -> Quat:
        """q_world_base = q_world_imu * inverse(q_base_imu)."""
        return q_normalize(q_multiply(q_world_imu, q_conjugate(self.q_base_imu)))

    def imu_callback(self, msg: Imu):
        # REP-145 uses orientation_covariance[0] = -1 when orientation is unavailable.
        if msg.orientation_covariance[0] == -1.0:
            self.get_logger().warning('IMU orientation is marked unavailable; sample ignored.')
            return
        q = msg.orientation
        try:
            self.latest_q_world_imu = q_normalize(
                (float(q.x), float(q.y), float(q.z), float(q.w))
            )
        except ValueError:
            self.get_logger().warning('Invalid/near-zero IMU quaternion; sample ignored.')
            return

        if self.q_reference_world_base is None and self.auto_calibrate:
            self.q_reference_world_base = self.sensor_to_base(self.latest_q_world_imu)
            self.get_logger().info(
                'Calibration captured: current physical pose is now 0 degrees in Gazebo.'
            )

    def calibrate_callback(self, request: Trigger.Request,
                           response: Trigger.Response) -> Trigger.Response:
        del request
        if self.latest_q_world_imu is None:
            response.success = False
            response.message = 'No valid IMU sample received yet; cannot calibrate.'
            return response
        self.q_reference_world_base = self.sensor_to_base(self.latest_q_world_imu)
        response.success = True
        response.message = 'Calibration updated; current physical orientation is now zero.'
        self.get_logger().info(response.message)
        return response

    def relative_orientation(self) -> Quat:
        """Calibrated orientation, or level until the IMU has been calibrated."""
        if self.latest_q_world_imu is None or self.q_reference_world_base is None:
            return IDENTITY
        q_world_base = self.sensor_to_base(self.latest_q_world_imu)
        return q_normalize(q_multiply(
            q_conjugate(self.q_reference_world_base), q_world_base
        ))

    # -------------------------------- Cycle -------------------------------

    def update(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        q_imu = self.relative_orientation()
        drive = effective_drive(self.drive, now - self.drive_stamp)
        position = self.motion.step(q_imu, drive, dt)
        q = self.motion.orient(q_imu)

        velocities = wheel_velocities(self.motion.wheel_drive(drive), self.wheel_omega)
        velocities[TURBINE_JOINTS[0]] = self.turbine_f * self.turbine_max
        velocities[TURBINE_JOINTS[1]] = self.turbine_r * self.turbine_max
        for joint, velocity in velocities.items():
            self.joint_positions[joint] += velocity * min(dt, 0.1)
        for joint in SERVO_JOINTS:
            self.joint_positions[joint] = self.servo_angle

        stamp = self.get_clock().now().to_msg()
        self.publish_transform(stamp, position, q)
        self.publish_joint_states(stamp, velocities)
        self.publish_wall_target(stamp, q_imu)
        self.publish_gazebo_commands(velocities)
        self.send_gazebo_pose(position, q)

    def publish_transform(self, stamp, position, q: Quat):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.base_frame
        msg.transform.translation.x = position[0]
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(msg)

    def publish_joint_states(self, stamp, velocities: Dict[str, float]):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(WHEEL_JOINTS + TURBINE_JOINTS + SERVO_JOINTS)
        msg.position = [self.joint_positions[j] for j in msg.name]
        msg.velocity = [velocities.get(j, 0.0) for j in msg.name]
        self.joint_pub.publish(msg)

    def publish_wall_target(self, stamp, q_imu: Quat):
        center, yaw = self.motion.wall_target(q_imu)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = center
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.target_pub.publish(msg)

    def publish_gazebo_commands(self, velocities: Dict[str, float]):
        for joint, publisher in self.velocity_pubs.items():
            publisher.publish(Float64(data=velocities[joint]))
        for publisher in self.position_pubs.values():
            publisher.publish(Float64(data=self.servo_angle))

    def send_gazebo_pose(self, position, q: Quat):
        now = time.monotonic()
        if self.request_stamp is not None and now - self.request_stamp < SET_POSE_TIMEOUT_S:
            return
        if not self.client.service_is_ready():
            if not self.warned_service_unavailable:
                self.get_logger().warning(
                    f'Waiting for {self.service_name}; check ros_gz_bridge.'
                )
                self.warned_service_unavailable = True
            return
        self.warned_service_unavailable = False

        req = SetEntityPose.Request()
        req.entity.id = self.entity_id
        req.entity.name = self.entity_name
        req.entity.type = self.entity_type
        req.pose.position.x, req.pose.position.y, req.pose.position.z = position
        req.pose.orientation.x = q[0]
        req.pose.orientation.y = q[1]
        req.pose.orientation.z = q[2]
        req.pose.orientation.w = q[3]

        self.request_stamp = now
        self.client.call_async(req).add_done_callback(self.service_done_callback)

    def service_done_callback(self, future):
        self.request_stamp = None
        try:
            if not future.result().success:
                self.get_logger().error(f'Gazebo rejected pose update for {self.entity_name}.')
        except Exception as exc:
            self.get_logger().error(f'Gazebo SetEntityPose call failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GeckoTwinNode()
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
