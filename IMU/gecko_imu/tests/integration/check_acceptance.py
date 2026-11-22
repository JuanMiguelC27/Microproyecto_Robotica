#!/usr/bin/env python3
"""End-to-end acceptance check of the Gecko twin against a real Gazebo server.

RViz is closed after start-up so that its panel does not compete with the
commands published here. Run it through tests/run_ros.sh.
"""

import math
import os
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))

import rclpy  # noqa: E402
from sensor_msgs.msg import Imu, JointState  # noqa: E402
from std_msgs.msg import Float64, Int8  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from gecko_geometry import q_from_rpy, support  # noqa: E402

START = ROOT / 'start_gecko_pc.sh'
POSE_INFO = '/world/empty/pose/info'
JOINT_STATE = '/world/empty/model/gecko/joint_state'
DOWN, UP = (0.0, 0.0, -1.0), (0.0, 0.0, 1.0)
WHEELS = ('Wheel_FL_Joint', 'Wheel_RL_Joint', 'Wheel_FR_Joint', 'Wheel_RR_Joint')


class Rig:
    def __init__(self):
        self.node = rclpy.create_node('gecko_acceptance')
        self.pitch = 0.0
        self.drive = 0
        self.poses, self.joints = {}, None
        self.node.create_subscription(TFMessage, POSE_INFO, self._on_pose, 10)
        self.node.create_subscription(JointState, JOINT_STATE, self._on_joints, 10)
        self.imu_pub = self.node.create_publisher(Imu, '/imu/data', 10)
        self.drive_pub = self.node.create_publisher(Int8, '/gecko/cmd/drive', 10)
        self.servo_pub = self.node.create_publisher(Float64, '/gecko/cmd/servo_angle', 10)
        self.turbine_f_pub = self.node.create_publisher(Float64, '/gecko/cmd/turbine_f', 10)
        self.turbine_r_pub = self.node.create_publisher(Float64, '/gecko/cmd/turbine_r', 10)
        self.node.create_timer(0.02, self._publish_imu)
        self.node.create_timer(0.1, self._publish_drive)

    def _on_pose(self, msg):
        for transform in msg.transforms:
            self.poses[transform.child_frame_id] = transform.transform

    def _on_joints(self, msg):
        self.joints = msg

    def _publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.y = math.sin(self.pitch / 2.0)
        msg.orientation.w = math.cos(self.pitch / 2.0)
        self.imu_pub.publish(msg)

    def _publish_drive(self):
        self.drive_pub.publish(Int8(data=self.drive))

    def spin(self, seconds, every=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if every is not None:
                every()

    def wait_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return True
        return False

    def velocity(self, joint):
        return self.joints.velocity[self.joints.name.index(joint)]

    def position(self, joint):
        return self.joints.position[self.joints.name.index(joint)]

    def base(self):
        return self.poses['gecko'].translation

    def yaw(self):
        r = self.poses['gecko'].rotation
        return math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z))


class Report:
    def __init__(self):
        self.failed = []

    def check(self, criterion, good, detail):
        print(f'{"PASS" if good else "FAIL"} criterio {criterion}: {detail}', flush=True)
        if not good:
            self.failed.append(criterion)


def wheel_signs(rig):
    return tuple(round(rig.velocity(j)) for j in WHEELS)


def check_driving(rig, report):
    expected = {
        1: (10, 10, 10, 10),
        2: (-10, -10, -10, -10),
        3: (10, 10, -10, -10),
        4: (-10, -10, 10, 10),
    }
    names = {1: 'adelante', 2: 'atras', 3: 'derecha', 4: 'izquierda'}
    for drive, signs in expected.items():
        yaw_before = rig.yaw()
        rig.drive = drive
        rig.spin(1.0)
        report.check(3, wheel_signs(rig) == signs, f'{names[drive]} {wheel_signs(rig)}')
        rig.drive = 0
        rig.spin(0.5)
        if drive in (3, 4):
            # Right turns clockwise (negative yaw), left counter-clockwise, ~66 deg/s.
            turned = math.degrees(math.remainder(rig.yaw() - yaw_before, math.tau))
            good = 40 <= abs(turned) <= 90 and (turned < 0) == (drive == 3)
            report.check(3, good, f'{names[drive]} giro {turned:+.0f} deg en Gazebo')

    start = rig.base().x
    begin = time.monotonic()
    rig.drive = 1
    rig.spin(1.0)
    rig.drive = 0
    elapsed = time.monotonic() - begin
    rig.spin(0.7)
    moved = rig.base().x - start
    report.check(3, abs(moved - 0.15 * elapsed) <= 0.03,
                 f'avanzo {moved:.3f} m en {elapsed:.2f} s')


def check_servos_and_turbines(rig, report):
    rig.servo_pub.publish(Float64(data=0.5))
    rig.turbine_f_pub.publish(Float64(data=0.5))
    rig.turbine_r_pub.publish(Float64(data=-1.0))
    rig.spin(4.0)
    front = rig.position('Turbine_Servo_F_Joint')
    rear = rig.position('Turbine_Servo_R_Joint')
    report.check(4, abs(front - 0.5) <= 0.0175 and abs(rear - 0.5) <= 0.0175,
                 f'servos F={front:.4f} R={rear:.4f} rad, sostenidos 4 s')
    turbine_f, turbine_r = rig.velocity('Turbine_F_Joint'), rig.velocity('Turbine_R_Joint')
    report.check(5, abs(turbine_f - 30.0) <= 3.0 and abs(turbine_r + 60.0) <= 6.0,
                 f'turbinas F={turbine_f:.1f} R={turbine_r:.1f} rad/s, sostenidas 4 s')


def check_floor_contact(rig, report):
    for degrees in (45.0, -45.0):
        rig.pitch = math.radians(degrees)
        rig.spin(1.0)
        rig.drive = 1
        rig.spin(1.0)
        q = q_from_rpy(0.0, rig.pitch, 0.0)
        error = abs(rig.base().z - support(q, DOWN))
        rig.drive = 0
        report.check(6, error < 0.001, f'pitch {degrees:+.0f}: error de altura {error*1000:.3f} mm')
    rig.pitch = 0.0
    rig.spin(1.0)


def check_wall(rig, report):
    rig.pitch = math.radians(-75.0)
    q = q_from_rpy(0.0, rig.pitch, 0.0)
    appeared = rig.wait_until(lambda: 'gecko_wall' in rig.poses, 15.0)
    report.check(7, appeared, 'la pared aparece con pitch -75')
    if not appeared:
        return
    rig.spin(1.0)
    wall, base = rig.poses['gecko_wall'].translation, rig.base()
    report.check(7, wall.x > base.x, f'pared en x={wall.x:.3f}, Gecko en x={base.x:.3f}')

    z0 = rig.base().z
    rig.drive = 1
    rig.spin(2.0)
    report.check(7, rig.base().z > z0 + 0.2, f'sube de {z0:.3f} a {rig.base().z:.3f} m')
    rig.spin(13.0)
    limit = 2.0 - support(q, UP)
    report.check(7, abs(rig.base().z - limit) <= 0.005,
                 f'se detiene en z={rig.base().z:.3f}, limite {limit:.3f} m')
    rig.drive = 2
    rig.spin(2.0)
    report.check(7, rig.base().z < limit - 0.2, f'baja a {rig.base().z:.3f} m')
    rig.drive = 0

    rig.pitch = 0.0
    rig.spin(3.0)
    rig.poses.pop('gecko_wall', None)
    rig.spin(1.5)
    report.check(8, 'gecko_wall' not in rig.poses, 'la pared desaparece con pitch 0')
    landed = rig.wait_until(lambda: abs(rig.base().z - 0.0284) <= 0.001, 8.0)
    report.check(8, landed, f'el Gecko baja hasta z={rig.base().z:.4f} m')


def check_stability(rig, report):
    rig.drive = 1
    samples, begin = [], time.monotonic()

    def sample():
        now = time.monotonic() - begin
        rig.pitch = math.radians(20.0) * math.sin(2.0 * math.pi * 0.5 * now)
        if now > 1.0:
            samples.append(rig.velocity('Wheel_FL_Joint'))

    rig.spin(9.0, sample)
    rig.drive = 0
    rig.pitch = 0.0
    deviation = statistics.pstdev(samples)
    report.check(9, deviation < 1.0, f'desviacion de rueda {deviation:.4f} rad/s en 8 s')


def run_checks(report):
    rig = Rig()
    ready = rig.wait_until(lambda: rig.joints is not None and 'gecko' in rig.poses, 30.0)
    if not ready:
        report.check(2, False, 'Gazebo no publico la pose o los joints del Gecko')
        return
    rig.spin(3.0)

    out = subprocess.run(['ign', 'model', '-m', 'gecko', '-l'],
                         capture_output=True, text=True).stdout
    links = out.count('- Link [')
    report.check(2, links == 9, f'{links} links en Gazebo')

    check_driving(rig, report)
    check_servos_and_turbines(rig, report)
    check_floor_contact(rig, report)
    check_wall(rig, report)
    check_stability(rig, report)
    rig.node.destroy_node()


def main():
    env = dict(os.environ, GECKO_SKIP_HOTSPOT='1', GECKO_HEADLESS='1')
    report = Report()
    bridge = None
    try:
        subprocess.run([str(START), 'start'], env=env, check=True,
                       stdout=subprocess.DEVNULL, timeout=180)
        subprocess.run(['pkill', '-x', 'rviz2'], check=False)
        bridge = subprocess.Popen([
            '/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge',
            f'{POSE_INFO}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            f'{JOINT_STATE}@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        rclpy.init()
        run_checks(report)
        rclpy.shutdown()
    finally:
        if bridge is not None:
            os.killpg(bridge.pid, signal.SIGTERM)
        subprocess.run([str(START), 'stop'], env=env, check=False,
                       stdout=subprocess.DEVNULL, timeout=60)

    if report.failed:
        print(f'acceptance_failed: {sorted(set(report.failed))}')
        return 1
    print('acceptance_ok')
    return 0


if __name__ == '__main__':
    sys.exit(main())
