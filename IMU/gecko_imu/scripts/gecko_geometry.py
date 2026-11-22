#!/usr/bin/env python3
"""Rigid-body envelope of the Gecko, expressed in Base_link (metres)."""

import math
from itertools import product
from typing import Sequence, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # ROS order: x, y, z, w

IDENTITY: Quat = (0.0, 0.0, 0.0, 1.0)

BODY_Y: Vec3 = (0.0, 1.0, 0.0)

WHEEL_RADIUS = 0.015
WHEEL_CENTERS: Sequence[Vec3] = (
    (0.142, 0.12909, -0.013419),
    (0.142, -0.12909, -0.011481),
    (-0.14196, -0.12909, -0.012381),
    (-0.142, 0.12909, -0.013381),
)

# Discs that contain the servo and turbine sweep over the full +/-45 deg travel.
SERVO_RADIUS = 0.056
SERVO_CENTERS: Sequence[Vec3] = tuple(
    (x, y, 0.0415) for x, y in product((0.09, -0.09), (0.0672, -0.0672))
)

CHASSIS_CORNERS: Sequence[Vec3] = tuple(
    product((0.153, -0.153), (0.1308, -0.1308), (-0.0209, 0.068))
)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def q_from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate v from the body frame to the world frame."""
    axis = (q[0], q[1], q[2])
    t = tuple(2.0 * c for c in cross(axis, v))
    u = cross(axis, t)  # type: ignore[arg-type]
    return (
        v[0] + q[3] * t[0] + u[0],
        v[1] + q[3] * t[1] + u[1],
        v[2] + q[3] * t[2] + u[2],
    )


def support(q: Quat, u: Vec3) -> float:
    """Return how far the vehicle reaches from Base_link along the unit vector u."""
    axis = dot(rotate(q, BODY_Y), u)
    disc_reach = math.sqrt(max(0.0, 1.0 - axis * axis))

    wheels = max(dot(rotate(q, c), u) for c in WHEEL_CENTERS) + WHEEL_RADIUS * disc_reach
    servos = max(dot(rotate(q, c), u) for c in SERVO_CENTERS) + SERVO_RADIUS * disc_reach
    chassis = max(dot(rotate(q, c), u) for c in CHASSIS_CORNERS)
    return max(wheels, servos, chassis)
