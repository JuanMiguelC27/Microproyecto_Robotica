#!/usr/bin/env python3
"""Check that joints keep their commanded motion while set_pose moves the model."""

import math
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))

import rclpy  # noqa: E402
from ros_gz_interfaces.srv import SetEntityPose  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

import gecko_joints as gj  # noqa: E402
from generate_gecko_models import generate  # noqa: E402

SOURCE = ROOT.parents[1] / 'Micro2_URDFV2/urdf/Micro2_URDFV2.urdf'
JOINT_STATE = '/world/empty/model/gecko/joint_state'
WHEEL_CMD, TURBINE_CMD, SERVO_CMD = 10.0, 60.0, 0.5
DURATION, SETTLE = 10.0, 2.0


def start(*command):
    return subprocess.Popen(command, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def wait_for_service(name, timeout=60.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        out = subprocess.run(['ign', 'service', '-l'], capture_output=True, text=True).stdout
        if name in out:
            return
        time.sleep(0.5)
    raise RuntimeError(f'Gazebo did not offer {name}')


def spawn(urdf):
    out = subprocess.run([
        'ign', 'service', '-s', '/world/empty/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean', '--timeout', '15000',
        '--req', f'sdf_filename: "{urdf}", name: "gecko"',
    ], capture_output=True, text=True).stdout
    if 'data: true' not in out:
        raise RuntimeError(f'Gazebo rejected the model: {out}')


def measure():
    rclpy.init()
    node = rclpy.create_node('joint_motion_check')
    samples = {}
    node.create_subscription(JointState, JOINT_STATE, lambda m: samples.update(
        {'last': m, 'all': samples.get('all', []) + [(time.monotonic(), m)]}), 50)
    client = node.create_client(SetEntityPose, '/world/empty/set_pose')
    pubs = {j: node.create_publisher(Float64, gj.velocity_topic(j), 10)
            for j in gj.VELOCITY_JOINTS}
    pubs.update({j: node.create_publisher(Float64, gj.position_topic(j), 10)
                 for j in gj.SERVO_JOINTS})
    if not client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError('set_pose bridge unavailable')

    commands = {j: WHEEL_CMD for j in gj.WHEEL_JOINTS}
    commands.update({j: TURBINE_CMD for j in gj.TURBINE_JOINTS})
    commands.update({j: SERVO_CMD for j in gj.SERVO_JOINTS})

    begin = time.monotonic()
    pending, calls = None, 0
    while time.monotonic() - begin < DURATION:
        now = time.monotonic() - begin
        for joint, value in commands.items():
            pubs[joint].publish(Float64(data=value))
        if pending is None or pending.done():
            pitch = math.radians(30.0) * math.sin(2.0 * math.pi * 0.5 * now)
            req = SetEntityPose.Request()
            req.entity.name, req.entity.type = 'gecko', 2
            req.pose.position.z = 0.2
            req.pose.orientation.y = math.sin(pitch / 2.0)
            req.pose.orientation.w = math.cos(pitch / 2.0)
            pending = client.call_async(req)
            calls += 1
        rclpy.spin_once(node, timeout_sec=1.0 / 30.0)

    node.destroy_node()
    rclpy.shutdown()
    return begin, samples.get('all', []), calls


def evaluate(begin, samples, calls):
    window = [m for t, m in samples if t - begin >= SETTLE]
    if not window:
        print('FAIL: no joint states received')
        return False
    series = {}
    for msg in window:
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            series.setdefault(name, []).append((pos, vel))

    print(f'set_pose calls={calls} joint_state samples={len(window)}')
    ok = {'wheels': True, 'turbines': True, 'servos': True}
    for joint in gj.WHEEL_JOINTS + gj.TURBINE_JOINTS:
        vel = [v for _, v in series[joint]]
        mean, dev = statistics.fmean(vel), statistics.pstdev(vel)
        target, tol, key = ((WHEEL_CMD, 1.0, 'wheels') if joint in gj.WHEEL_JOINTS
                            else (TURBINE_CMD, 6.0, 'turbines'))
        good = abs(mean - target) <= tol and dev < tol
        ok[key] &= good
        print(f'{joint}: mean={mean:.3f} rad/s std={dev:.3f} {"ok" if good else "BAD"}')
    for joint in gj.SERVO_JOINTS:
        final = series[joint][-1][0]
        good = abs(final - SERVO_CMD) <= 0.0175
        ok['servos'] &= good
        print(f'{joint}: final={final:.4f} rad {"ok" if good else "BAD"}')
    print(' '.join(f'{k}_{"ok" if v else "FAIL"}'
                   for k, v in ok.items()))
    return all(ok.values())


def main():
    processes = []
    try:
        with tempfile.TemporaryDirectory() as tmp:
            urdf = generate(SOURCE, Path(tmp))['gazebo']
            processes.append(start('ign', 'gazebo', '-s', '-r', 'empty.sdf'))
            wait_for_service('/world/empty/create')
            spawn(urdf)
            processes.append(start(
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                *gj.bridge_args(),
                '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                f'{JOINT_STATE}@sensor_msgs/msg/JointState[ignition.msgs.Model',
            ))
            return 0 if evaluate(*measure()) else 1
    finally:
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(1.0)
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


if __name__ == '__main__':
    sys.exit(main())
