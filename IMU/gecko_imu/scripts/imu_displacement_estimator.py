#!/usr/bin/env python3
"""Approximate IMU dead-reckoning for the Gecko visual simulation.

ROS 2 Humble.
Input:
  /imu/data (sensor_msgs/msg/Imu)
    - orientation: BNO085 quaternion, ROS xyzw order
    - linear_acceleration: gravity-compensated acceleration in imu_link [m/s^2]

Outputs:
  /imu/estimated_odom          (nav_msgs/msg/Odometry)
  /imu/estimated_path          (nav_msgs/msg/Path; bounded history)
  /imu/estimated_acceleration  (geometry_msgs/msg/Vector3Stamped)
  /imu/estimated_pitch_deg     (std_msgs/msg/Float32)
  /imu_displacement/reset      (std_srvs/srv/Trigger)

IMPORTANT:
  This is deliberately an approximate visual estimator, not reliable odometry.
  Double integration drifts. It uses a stationary startup calibration, bias
  subtraction, low-pass filtering, deadband, dt-gap rejection, speed/position
  limits and an optional heuristic zero-velocity update (ZUPT).
"""

import math
from collections import deque
from statistics import mean, pstdev
from typing import Deque, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # x, y, z, w


def q_normalize(q: Quat) -> Quat:
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid or near-zero quaternion")
    return tuple(v / norm for v in q)  # type: ignore[return-value]


def q_conjugate(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def q_multiply(a: Quat, b: Quat) -> Quat:
    """Hamilton product, quaternions in ROS x,y,z,w order."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate vector v by unit quaternion q: q * [v,0] * conjugate(q)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # Expanded quaternion-vector rotation; avoids temporary quaternion objects.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def quaternion_pitch_rad(q: Quat) -> float:
    """Return ROS-style Euler pitch, with result in [-pi/2, pi/2]."""
    x, y, z, w = q
    sin_pitch = 2.0 * (w * y - z * x)
    return math.asin(max(-1.0, min(1.0, sin_pitch)))


def vec_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_scale(v: Vec3, scalar: float) -> Vec3:
    return (v[0] * scalar, v[1] * scalar, v[2] * scalar)


def vec_norm(v: Vec3) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def vec_limit(v: Vec3, max_norm: float) -> Vec3:
    n = vec_norm(v)
    if n > max_norm > 0.0:
        return vec_scale(v, max_norm / n)
    return v


def stamp_to_seconds(msg: Imu) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class ImuDisplacementEstimator(Node):
    def __init__(self) -> None:
        super().__init__("imu_displacement_estimator")

        defaults = {
            "imu_topic": "/imu/data",
            "odom_topic": "/imu/estimated_odom",
            "path_topic": "/imu/estimated_path",
            "acceleration_topic": "/imu/estimated_acceleration",
            "pitch_topic": "/imu/estimated_pitch_deg",
            "odom_frame": "odom",
            "base_frame": "Base_link",
            # Yaw of imu_link relative to Base_link (deg). Must match
            # imu_mount_yaw_deg of gecko_twin_node.py: the model front is the
            # BNO085 back, so orientation and acceleration are rotated into the
            # model frame (roll/pitch change sign, yaw is kept).
            "imu_mount_yaw_deg": 180.0,
            # Leave the robot still during this startup calibration window.
            "calibration_duration_sec": 2.0,
            "calibration_min_samples": 25,
            "calibration_max_stddev_mps2": 0.12,
            # Signal conditioning.
            "lowpass_alpha": 0.35,
            "acceleration_deadband_mps2": 0.04,
            "max_acceleration_mps2": 3.0,
            # Time and motion limits for a visual estimate.
            "max_dt_sec": 0.12,
            "max_speed_mps": 0.80,
            "max_displacement_m": 5.0,
            # Heuristic ZUPT: assumes near-zero accel + stable attitude means stopped.
            # This cannot distinguish rest from constant-velocity motion.
            "enable_zupt": True,
            "zupt_accel_threshold_mps2": 0.07,
            "zupt_angular_rate_threshold_radps": 0.12,
            "zupt_hold_sec": 0.80,
            # Path history limit to avoid unbounded memory growth.
            "path_max_poses": 500,
            "path_min_spacing_m": 0.015,
            "log_every_n": 30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value
        self.imu_topic = str(p("imu_topic"))
        self.odom_frame = str(p("odom_frame"))
        self.base_frame = str(p("base_frame"))
        half = math.radians(float(p("imu_mount_yaw_deg"))) / 2.0
        self.q_base_imu: Quat = (0.0, 0.0, math.sin(half), math.cos(half))
        self.calibration_duration = float(p("calibration_duration_sec"))
        self.calibration_min_samples = int(p("calibration_min_samples"))
        self.calibration_max_std = float(p("calibration_max_stddev_mps2"))
        self.alpha = float(p("lowpass_alpha"))
        self.deadband = float(p("acceleration_deadband_mps2"))
        self.max_accel = float(p("max_acceleration_mps2"))
        self.max_dt = float(p("max_dt_sec"))
        self.max_speed = float(p("max_speed_mps"))
        self.max_displacement = float(p("max_displacement_m"))
        self.enable_zupt = bool(p("enable_zupt"))
        self.zupt_accel_threshold = float(p("zupt_accel_threshold_mps2"))
        self.zupt_rate_threshold = float(p("zupt_angular_rate_threshold_radps"))
        self.zupt_hold = float(p("zupt_hold_sec"))
        self.path_max_poses = max(2, int(p("path_max_poses")))
        self.path_min_spacing = max(0.0, float(p("path_min_spacing_m")))
        self.log_every_n = max(1, int(p("log_every_n")))

        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("lowpass_alpha must be in (0, 1]")
        if self.calibration_duration <= 0.0 or self.max_dt <= 0.0:
            raise ValueError("calibration_duration_sec and max_dt_sec must be positive")
        if self.max_speed <= 0.0 or self.max_accel <= 0.0:
            raise ValueError("max_speed_mps and max_acceleration_mps2 must be positive")

        self.odom_pub = self.create_publisher(Odometry, str(p("odom_topic")), 10)
        self.path_pub = self.create_publisher(Path, str(p("path_topic")), 10)
        self.accel_pub = self.create_publisher(
            Vector3Stamped, str(p("acceleration_topic")), 10
        )
        self.pitch_pub = self.create_publisher(Float32, str(p("pitch_topic")), 10)

        self.subscription = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data
        )
        self.reset_service = self.create_service(
            Trigger, "/imu_displacement/reset", self.reset_callback
        )

        self.path_poses: Deque[PoseStamped] = deque(maxlen=self.path_max_poses)
        self.sample_count = 0
        self.dropped_time_steps = 0
        self._reset_estimator_state()

        self.get_logger().info(f"Subscribing to {self.imu_topic}")
        self.get_logger().info("Publishing approximate displacement on /imu/estimated_odom")
        self.get_logger().info(
            f"Startup calibration: {self.calibration_duration:.2f}s, "
            f"minimum {self.calibration_min_samples} samples; keep the robot still."
        )
        self.get_logger().warning(
            "Double integration is approximate and will drift; this node is for visualization, not navigation."
        )
        if self.enable_zupt:
            self.get_logger().warning(
                "Heuristic ZUPT is enabled: low acceleration + stable attitude for "
                f"{self.zupt_hold:.2f}s sets velocity to zero. Constant-speed motion may be mistaken for rest."
            )

    def _reset_estimator_state(self) -> None:
        self.q_reference: Optional[Quat] = None
        self.calibration_start: Optional[float] = None
        self.calibration_samples: List[Vec3] = []
        self.accel_bias: Vec3 = (0.0, 0.0, 0.0)
        self.calibrated = False

        self.position: Vec3 = (0.0, 0.0, 0.0)
        self.velocity: Vec3 = (0.0, 0.0, 0.0)
        self.filtered_accel: Vec3 = (0.0, 0.0, 0.0)
        self.previous_accel: Vec3 = (0.0, 0.0, 0.0)
        self.previous_stamp: Optional[float] = None
        self.previous_q_relative: Optional[Quat] = None
        self.low_motion_start: Optional[float] = None
        self.last_path_position: Optional[Vec3] = None
        self.path_poses.clear()
        self.sample_count = 0
        self.dropped_time_steps = 0

    def reset_callback(self, request: Trigger.Request,
                       response: Trigger.Response) -> Trigger.Response:
        del request
        self._reset_estimator_state()
        response.success = True
        response.message = (
            "Estimator reset. Keep the robot still; a new orientation reference "
            "and acceleration-bias calibration will be collected."
        )
        self.get_logger().warning(response.message)
        return response

    def _read_imu(self, msg: Imu) -> Tuple[Quat, Vec3]:
        if msg.orientation_covariance[0] == -1.0:
            raise ValueError("orientation unavailable")
        if msg.linear_acceleration_covariance[0] == -1.0:
            raise ValueError("linear acceleration unavailable")

        q = q_normalize((
            float(msg.orientation.x), float(msg.orientation.y),
            float(msg.orientation.z), float(msg.orientation.w),
        ))
        a = (
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        )
        if not all(math.isfinite(v) for v in a):
            raise ValueError("non-finite acceleration")
        # Express orientation and acceleration in Base_link (model) axes.
        q = q_normalize(q_multiply(q, q_conjugate(self.q_base_imu)))
        a = q_rotate(self.q_base_imu, a)
        return q, a

    def _relative_orientation(self, q_current: Quat) -> Quat:
        assert self.q_reference is not None
        return q_normalize(q_multiply(q_conjugate(self.q_reference), q_current))

    def _collect_calibration(self, stamp: float, q: Quat, accel_body: Vec3) -> None:
        if self.q_reference is None:
            self.q_reference = q
            self.calibration_start = stamp
            self.calibration_samples = []

        q_rel = self._relative_orientation(q)
        accel_reference = q_rotate(q_rel, accel_body)
        self.calibration_samples.append(accel_reference)

        elapsed = stamp - (self.calibration_start if self.calibration_start is not None else stamp)
        if elapsed < self.calibration_duration:
            return
        if len(self.calibration_samples) < self.calibration_min_samples:
            # Keep collecting until enough valid samples are available.
            return

        axes = list(zip(*self.calibration_samples))
        stddevs = tuple(pstdev(axis) for axis in axes)
        max_std = max(stddevs)

        if max_std > self.calibration_max_std:
            self.get_logger().warning(
                f"Calibration window was not sufficiently still (max axis stddev="
                f"{max_std:.3f} m/s^2). Restarting; keep the robot motionless."
            )
            self.q_reference = q
            self.calibration_start = stamp
            self.calibration_samples = []
            return

        self.accel_bias = tuple(mean(axis) for axis in axes)  # type: ignore[assignment]
        self.calibrated = True
        self.previous_stamp = stamp
        self.previous_q_relative = self._relative_orientation(q)
        self.previous_accel = (0.0, 0.0, 0.0)
        self.filtered_accel = (0.0, 0.0, 0.0)
        self.position = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.last_path_position = None

        self.get_logger().info(
            "Calibration complete. Relative orientation and displacement start at zero. "
            f"Acceleration bias in initial robot frame: "
            f"({self.accel_bias[0]:+.4f}, {self.accel_bias[1]:+.4f}, "
            f"{self.accel_bias[2]:+.4f}) m/s^2; samples={len(self.calibration_samples)}; "
            f"stddev=({stddevs[0]:.4f}, {stddevs[1]:.4f}, {stddevs[2]:.4f})."
        )
        self._publish_outputs(stamp, self._relative_orientation(q), (0.0, 0.0, 0.0))

    def imu_callback(self, msg: Imu) -> None:
        try:
            q_current, accel_body = self._read_imu(msg)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"Ignoring invalid IMU sample: {exc}")
            return

        stamp = stamp_to_seconds(msg)
        if stamp <= 0.0:
            self.get_logger().warning("Ignoring IMU sample with zero/invalid timestamp")
            return

        if not self.calibrated:
            self._collect_calibration(stamp, q_current, accel_body)
            return

        q_relative = self._relative_orientation(q_current)
        # Current body-frame acceleration rotated into the initial robot frame.
        accel_reference = q_rotate(q_relative, accel_body)
        accel_corrected = vec_sub(accel_reference, self.accel_bias)
        accel_corrected = vec_limit(accel_corrected, self.max_accel)

        # First-order low-pass filter.
        self.filtered_accel = vec_add(
            vec_scale(self.filtered_accel, 1.0 - self.alpha),
            vec_scale(accel_corrected, self.alpha),
        )

        # Deadband each component to suppress small stationary noise.
        accel_filtered = tuple(
            0.0 if abs(v) < self.deadband else v
            for v in self.filtered_accel
        )
        accel_filtered = vec_limit(accel_filtered, self.max_accel)

        if self.previous_stamp is None:
            self.previous_stamp = stamp
            self.previous_accel = accel_filtered  # type: ignore[assignment]
            self.previous_q_relative = q_relative
            self._publish_outputs(stamp, q_relative, accel_filtered)  # type: ignore[arg-type]
            return

        dt = stamp - self.previous_stamp
        if dt <= 0.0:
            return

        if dt > self.max_dt:
            self.dropped_time_steps += 1
            # Do not integrate over a long UDP/scheduler gap: it would invent a large jump.
            self.previous_stamp = stamp
            self.previous_accel = accel_filtered  # type: ignore[assignment]
            self.previous_q_relative = q_relative
            if self.dropped_time_steps == 1 or self.dropped_time_steps % 20 == 0:
                self.get_logger().warning(
                    f"Skipped integration interval dt={dt:.3f}s (> {self.max_dt:.3f}s). "
                    "Position is held for this sample."
                )
            self._publish_outputs(stamp, q_relative, accel_filtered)  # type: ignore[arg-type]
            return

        # Estimate angular rate from consecutive orientation samples for ZUPT gating.
        angular_rate = 0.0
        if self.previous_q_relative is not None:
            q_delta = q_normalize(q_multiply(
                q_conjugate(self.previous_q_relative), q_relative
            ))
            angle = 2.0 * math.acos(max(-1.0, min(1.0, abs(q_delta[3]))))
            angular_rate = angle / dt

        low_motion = (
            vec_norm(accel_filtered) <= self.zupt_accel_threshold
            and angular_rate <= self.zupt_rate_threshold
        )

        if self.enable_zupt and low_motion:
            if self.low_motion_start is None:
                self.low_motion_start = stamp
            elif stamp - self.low_motion_start >= self.zupt_hold:
                self.velocity = (0.0, 0.0, 0.0)
                accel_filtered = (0.0, 0.0, 0.0)
        else:
            self.low_motion_start = None

        # Trapezoidal double integration (acceleration -> velocity -> position).
        velocity_new = vec_add(
            self.velocity,
            vec_scale(vec_add(self.previous_accel, accel_filtered), 0.5 * dt),
        )
        velocity_new = vec_limit(velocity_new, self.max_speed)

        position_new = vec_add(
            self.position,
            vec_scale(vec_add(self.velocity, velocity_new), 0.5 * dt),
        )

        # Bound total displacement so a sensor spike cannot send the model away.
        if vec_norm(position_new) > self.max_displacement:
            position_new = vec_limit(position_new, self.max_displacement)
            velocity_new = (0.0, 0.0, 0.0)
            self.get_logger().warning(
                "Maximum estimated displacement reached; position clamped and velocity reset."
            )

        self.velocity = velocity_new
        self.position = position_new
        self.previous_accel = accel_filtered  # type: ignore[assignment]
        self.previous_stamp = stamp
        self.previous_q_relative = q_relative
        self.sample_count += 1

        self._publish_outputs(stamp, q_relative, accel_filtered)  # type: ignore[arg-type]

        if self.sample_count % self.log_every_n == 0:
            pitch_deg = math.degrees(quaternion_pitch_rad(q_relative))
            self.get_logger().info(
                f"p=({self.position[0]:+.3f},{self.position[1]:+.3f},"
                f"{self.position[2]:+.3f}) m | "
                f"v=({self.velocity[0]:+.3f},{self.velocity[1]:+.3f},"
                f"{self.velocity[2]:+.3f}) m/s | pitch={pitch_deg:+.1f} deg"
            )

    def _publish_outputs(self, stamp_sec: float, q: Quat, accel: Vec3) -> None:
        # Preserve the PC receive timestamp assigned by udp_imu_receiver.py.
        stamp = self.get_clock().now().to_msg()
        if stamp_sec > 0.0 and math.isfinite(stamp_sec):
            stamp.sec = int(stamp_sec)
            stamp.nanosec = int((stamp_sec - stamp.sec) * 1e9)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Odometry twist is expressed in child_frame_id (Base_link).
        velocity_body = q_rotate(q_conjugate(q), self.velocity)
        odom.twist.twist.linear.x = velocity_body[0]
        odom.twist.twist.linear.y = velocity_body[1]
        odom.twist.twist.linear.z = velocity_body[2]
        self.odom_pub.publish(odom)

        accel_msg = Vector3Stamped()
        accel_msg.header.stamp = stamp
        accel_msg.header.frame_id = self.odom_frame
        accel_msg.vector.x = accel[0]
        accel_msg.vector.y = accel[1]
        accel_msg.vector.z = accel[2]
        self.accel_pub.publish(accel_msg)

        pitch_msg = Float32()
        pitch_msg.data = math.degrees(quaternion_pitch_rad(q))
        self.pitch_pub.publish(pitch_msg)

        self._maybe_publish_path(stamp, q)

    def _maybe_publish_path(self, stamp, q: Quat) -> None:
        if self.last_path_position is not None:
            delta = vec_sub(self.position, self.last_path_position)
            if vec_norm(delta) < self.path_min_spacing:
                return

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.odom_frame
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1]
        pose.pose.position.z = self.position[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path_poses.append(pose)
        self.last_path_position = self.position

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.odom_frame
        path.poses = list(self.path_poses)
        self.path_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ImuDisplacementEstimator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f"Estimator stopped: {type(exc).__name__}: {exc}")
        else:
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
