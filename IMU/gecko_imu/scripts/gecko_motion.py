#!/usr/bin/env python3
"""Kinematics of the Gecko twin: floor driving, wall climbing and descent.

Tilt always comes from the IMU. The joystick turns the vehicle in place about
the world vertical, and that yaw is added on top of the IMU orientation. This
module also decides where the vehicle is, keeping its envelope in contact with
the active surface.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Iterable, Optional, Tuple

from gecko_geometry import (
    IDENTITY, WHEEL_CENTERS, Quat, Vec3, cross, dot, rotate, support,
)
from gecko_joints import WHEEL_JOINTS

DRIVE_STOP = 0
DRIVE_FORWARD = 1
DRIVE_BACK = 2
DRIVE_RIGHT = 3
DRIVE_LEFT = 4

DRIVE_SPEED = 0.15

# Skid steer: the wheels on each side roll at DRIVE_SPEED in opposite directions,
# so the body turns at that speed divided by half the track width (~66 deg/s).
TRACK_HALF_WIDTH = max(abs(c[1]) for c in WHEEL_CENTERS)
TURN_RATE = DRIVE_SPEED / TRACK_HALF_WIDTH

MAX_DT = 0.1
MIN_PROJECTION = 0.1
SERVO_LIMIT = 0.7854
MAX_POSE_VALUE = 1000.0

WALL_HALF_THICKNESS = 0.05
WALL_CENTER_Z = 1.0
WALL_GAP = 0.01

UP: Vec3 = (0.0, 0.0, 1.0)
DOWN: Vec3 = (0.0, 0.0, -1.0)
BODY_X: Vec3 = (1.0, 0.0, 0.0)

_LEFT_WHEELS = ('Wheel_FL_Joint', 'Wheel_RL_Joint')
_TURN_SIGNS = {DRIVE_LEFT: 1.0, DRIVE_RIGHT: -1.0}
_DRIVE_SIGNS = {
    DRIVE_STOP: (0.0, 0.0),
    DRIVE_FORWARD: (1.0, 1.0),
    DRIVE_BACK: (-1.0, -1.0),
    DRIVE_RIGHT: (1.0, -1.0),
    DRIVE_LEFT: (-1.0, 1.0),
}


class Mode(Enum):
    FLOOR = 'floor'
    WALL = 'wall'
    DESCENT = 'descent'


@dataclass(frozen=True)
class Wall:
    heading: Vec3  # Horizontal unit vector pointing from the vehicle into the wall.
    face: Vec3     # Any point on the wall face, laterally centred.


def wall_from_center(center: Vec3, yaw: float) -> Wall:
    heading = (math.cos(yaw), math.sin(yaw), 0.0)
    face = (
        center[0] - heading[0] * WALL_HALF_THICKNESS,
        center[1] - heading[1] * WALL_HALF_THICKNESS,
        center[2],
    )
    return Wall(heading=heading, face=face)


def finite_pose(values: Iterable[float]) -> bool:
    """Reject poses that are not finite or lie absurdly far from the world origin."""
    return all(math.isfinite(v) and abs(v) <= MAX_POSE_VALUE for v in values)


def sanitize_drive(value: int) -> int:
    return value if value in _DRIVE_SIGNS else DRIVE_STOP


def clamp_servo(rad: float, current: float) -> float:
    if not math.isfinite(rad):
        return current
    return max(-SERVO_LIMIT, min(SERVO_LIMIT, rad))


def clamp_turbine(value: float, current: float) -> float:
    if not math.isfinite(value):
        return current
    return max(-1.0, min(1.0, value))


def effective_drive(drive: int, age_s: float, timeout: float = 0.5) -> int:
    """Stop the wheels when the panel has gone silent."""
    return drive if age_s <= timeout else DRIVE_STOP


def wheel_velocities(drive: int, omega: float = 10.0) -> Dict[str, float]:
    left, right = _DRIVE_SIGNS[sanitize_drive(drive)]
    return {
        joint: omega * (left if joint in _LEFT_WHEELS else right)
        for joint in WHEEL_JOINTS
    }


def yaw_then(q: Quat, yaw: float) -> Quat:
    """Rotate orientation q by yaw about the world vertical (q_z(yaw) * q)."""
    s, c = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    x, y, z, w = q
    return (c * x - s * y, c * y + s * x, c * z + s * w, c * w - s * z)


def _unit(v: Vec3) -> Optional[Vec3]:
    """Normalize v, or return None when it is too short to define a direction."""
    norm = math.sqrt(dot(v, v))
    if norm < MIN_PROJECTION:
        return None
    return (v[0] / norm, v[1] / norm, v[2] / norm)


class GeckoMotion:
    """Every public method takes the calibrated IMU orientation (q_imu)."""

    def __init__(self, speed: float = DRIVE_SPEED, descent_speed: float = 0.5,
                 wall_top: float = 2.0, wall_half_width: float = 1.0):
        self.speed = speed
        self.turn_rate = speed / TRACK_HALF_WIDTH
        self.yaw = 0.0
        self.descent_speed = descent_speed
        self.wall_top = wall_top
        self.wall_half_width = wall_half_width
        self.mode = Mode.FLOOR
        self.heading: Vec3 = BODY_X
        self.position: Vec3 = (0.0, 0.0, support(IDENTITY, DOWN))
        self._wall: Optional[Wall] = None

    def set_wall(self, wall: Optional[Wall]) -> None:
        if wall is not None:
            self._wall = wall
            self.mode = Mode.WALL
        elif self.mode is Mode.WALL:
            self._wall = None
            self.mode = Mode.DESCENT

    def orient(self, q_imu: Quat) -> Quat:
        """Orientation of the twin: the IMU plus the yaw turned with the joystick."""
        return yaw_then(q_imu, self.yaw)

    def wheel_drive(self, drive: int) -> int:
        """Return the wheel command; the joystick is ignored during a descent."""
        return DRIVE_STOP if self.mode is Mode.DESCENT else drive

    def wall_target(self, q_imu: Quat) -> Tuple[Vec3, float]:
        """Return the centre and yaw a wall would have if it appeared now."""
        q = self.orient(q_imu)
        self._update_heading(q)
        h = self.heading
        offset = support(q, h) + WALL_GAP + WALL_HALF_THICKNESS
        center = (
            self.position[0] + h[0] * offset,
            self.position[1] + h[1] * offset,
            WALL_CENTER_Z,
        )
        return center, math.atan2(h[1], h[0])

    def step(self, q_imu: Quat, drive: int, dt: float) -> Vec3:
        dt = min(dt, MAX_DT)
        if dt <= 0.0:
            return self.position

        drive = sanitize_drive(drive)
        # On the wall or falling, a turn about the world vertical would roll the
        # vehicle over its long axis, so the joystick only turns it on the floor.
        if self.mode is Mode.FLOOR and drive in _TURN_SIGNS:
            self.yaw = math.remainder(
                self.yaw + _TURN_SIGNS[drive] * self.turn_rate * dt, math.tau)

        q = self.orient(q_imu)
        self._update_heading(q)
        distance = self.speed * dt * {DRIVE_FORWARD: 1.0, DRIVE_BACK: -1.0}.get(drive, 0.0)

        if self.mode is Mode.WALL and self._wall is not None:
            self._step_wall(q, self._wall, distance)
        elif self.mode is Mode.DESCENT:
            self._step_descent(q, dt)
        else:
            self._step_floor(q, distance)
        return self.position

    def _update_heading(self, q: Quat) -> None:
        forward = rotate(q, BODY_X)
        heading = _unit((forward[0], forward[1], 0.0))
        if heading is not None:
            self.heading = heading

    def _step_floor(self, q: Quat, distance: float) -> None:
        self.position = (
            self.position[0] + self.heading[0] * distance,
            self.position[1] + self.heading[1] * distance,
            support(q, DOWN),
        )

    def _step_descent(self, q: Quat, dt: float) -> None:
        floor_z = support(q, DOWN)
        z = self.position[2] - self.descent_speed * dt
        if z <= floor_z:
            z = floor_z
            self.mode = Mode.FLOOR
        self.position = (self.position[0], self.position[1], z)

    def _step_wall(self, q: Quat, wall: Wall, distance: float) -> None:
        h = wall.heading
        lateral = cross(UP, h)
        forward = rotate(q, BODY_X)
        into_wall = dot(forward, h)
        along = _unit((
            forward[0] - into_wall * h[0],
            forward[1] - into_wall * h[1],
            forward[2] - into_wall * h[2],
        ))

        relative = (self.position[0] - wall.face[0], self.position[1] - wall.face[1], 0.0)
        side = dot(relative, lateral)
        z = self.position[2]
        if along is not None:
            side += dot(along, lateral) * distance
            z += along[2] * distance

        negative_lateral = (-lateral[0], -lateral[1], -lateral[2])
        side = max(-self.wall_half_width + support(q, negative_lateral),
                   min(self.wall_half_width - support(q, lateral), side))
        z = max(support(q, DOWN), min(self.wall_top - support(q, UP), z))

        depth = -support(q, h)
        self.position = (
            wall.face[0] + h[0] * depth + lateral[0] * side,
            wall.face[1] + h[1] * depth + lateral[1] * side,
            z,
        )
