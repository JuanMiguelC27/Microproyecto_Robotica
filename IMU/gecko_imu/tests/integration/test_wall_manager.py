"""Integration tests of the wall manager against fake Gazebo services."""

import math
import subprocess
import sys
import time
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts/wall_climb_manager.py'
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FakeGazebo:
    def __init__(self):
        self.node = rclpy.create_node('fake_gazebo_for_wall')
        self.accept_spawn = True
        self.spawns, self.removes = [], []
        self.active, self.pose, self.markers = [], None, []
        self.node.create_service(SpawnEntity, '/world/empty/create', self._on_spawn)
        self.node.create_service(DeleteEntity, '/world/empty/remove', self._on_remove)
        self.node.create_subscription(Bool, '/gecko/wall/active',
                                      lambda m: self.active.append(m.data), LATCHED)
        self.node.create_subscription(PoseStamped, '/gecko/wall/pose', self._on_pose, LATCHED)
        self.node.create_subscription(Marker, '/gecko/wall/marker', self.markers.append, LATCHED)
        self.pitch = self.node.create_publisher(Float32, '/imu/estimated_pitch_deg', 10)
        self.target = self.node.create_publisher(PoseStamped, '/gecko/wall/target_pose', 10)

    def _on_pose(self, msg):
        self.pose = msg

    def _on_spawn(self, request, response):
        self.spawns.append(request.entity_factory)
        response.success = self.accept_spawn
        return response

    def _on_remove(self, request, response):
        self.removes.append(request.entity)
        response.success = True
        return response

    def run(self, seconds, pitch, target=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pitch.publish(Float32(data=float(pitch)))
            if target is not None:
                self.target.publish(target)
            rclpy.spin_once(self.node, timeout_sec=0.03)

    def wait_ready(self, timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.pitch.get_subscription_count() > 0 \
                    and self.target.get_subscription_count() > 0:
                self.run(0.5, 0.0)
                return
        raise AssertionError('wall manager did not start')


def target_pose():
    msg = PoseStamped()
    msg.header.frame_id = 'world'
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 1.0, 2.0, 1.0
    msg.pose.orientation.z = math.sin(math.radians(45))
    msg.pose.orientation.w = math.cos(math.radians(45))
    return msg


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def gazebo(ros):
    process = subprocess.Popen([sys.executable, str(SCRIPT)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    fake = FakeGazebo()
    try:
        fake.wait_ready()
        yield fake
    finally:
        process.terminate()
        process.wait(timeout=10)
        fake.node.destroy_node()


def test_spawns_at_target_pose(gazebo):
    gazebo.run(1.0, -75.0, target_pose())

    factory = gazebo.spawns[0]
    assert factory.name == 'gecko_wall'
    assert (factory.pose.position.x, factory.pose.position.y) == (1.0, 2.0)
    assert factory.pose.orientation.z == pytest.approx(math.sin(math.radians(45)))
    assert gazebo.active[-1] is True
    assert gazebo.pose.pose.position.y == 2.0

    marker = gazebo.markers[-1]
    assert (marker.type, marker.action) == (Marker.CUBE, Marker.ADD)
    assert (marker.scale.x, marker.scale.y, marker.scale.z) == pytest.approx((0.10, 2.0, 2.0))


def test_waits_for_target_pose(gazebo):
    gazebo.run(1.0, -75.0)
    assert gazebo.spawns == []
    gazebo.run(1.0, -75.0, target_pose())
    assert len(gazebo.spawns) == 1


def test_rejected_spawn_stays_inactive(gazebo):
    gazebo.accept_spawn = False
    gazebo.run(1.5, -75.0, target_pose())
    assert len(gazebo.spawns) >= 2
    assert True not in gazebo.active


def test_removes_wall_on_exit(gazebo):
    gazebo.run(1.0, -75.0, target_pose())
    gazebo.run(1.2, -40.0, target_pose())

    assert gazebo.removes[0].name == 'gecko_wall'
    assert gazebo.removes[0].type == 2
    assert gazebo.active[-1] is False
    assert gazebo.markers[-1].action == Marker.DELETE


def test_invalid_target_pose_is_ignored(gazebo):
    target = target_pose()
    target.pose.position.x = math.nan
    gazebo.run(1.0, -75.0, target)
    assert gazebo.spawns == []
