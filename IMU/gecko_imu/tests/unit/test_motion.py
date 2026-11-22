from math import asin, atan2, degrees, inf, nan, radians

import pytest
from pytest import approx

from gecko_geometry import IDENTITY, q_from_rpy, support
from gecko_motion import (
    DRIVE_BACK,
    DRIVE_FORWARD,
    DRIVE_LEFT,
    DRIVE_RIGHT,
    DRIVE_STOP,
    TURN_RATE,
    GeckoMotion,
    Mode,
    Wall,
    clamp_servo,
    clamp_turbine,
    effective_drive,
    finite_pose,
    sanitize_drive,
    wall_from_center,
    wheel_velocities,
)

DOWN = (0, 0, -1)
VERTICAL = q_from_rpy(0, radians(-90), 0)


def run(motion, q, drive, seconds, dt=0.1):
    for _ in range(round(seconds / dt)):
        motion.step(q, drive, dt)


def on_wall(q=VERTICAL):
    motion = GeckoMotion()
    motion.step(q, DRIVE_STOP, 0.033)
    motion.set_wall(Wall(heading=(1, 0, 0), face=(1, 0, 0)))
    motion.step(q, DRIVE_STOP, 0.033)
    return motion


def test_starts_on_floor():
    assert GeckoMotion().position == approx((0, 0, 0.028419))


def test_forward_and_back_at_constant_speed():
    motion = GeckoMotion()
    run(motion, IDENTITY, DRIVE_FORWARD, 1.0)
    assert motion.position[0] == approx(0.15)
    run(motion, IDENTITY, DRIVE_BACK, 1.0)
    assert motion.position[0] == approx(0.0, abs=1e-9)


def yaw_deg(q):
    x, y, z, w = q
    return degrees(atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def test_turning_rotates_in_place():
    motion = GeckoMotion()
    run(motion, IDENTITY, DRIVE_RIGHT, 1.0)
    assert motion.position[:2] == approx((0, 0))
    # Right turns clockwise seen from above: negative yaw, at the skid-steer rate.
    assert yaw_deg(motion.orient(IDENTITY)) == approx(-degrees(TURN_RATE))
    run(motion, IDENTITY, DRIVE_LEFT, 2.0)
    assert yaw_deg(motion.orient(IDENTITY)) == approx(degrees(TURN_RATE))


def test_turn_rate_matches_wheel_speed():
    assert TURN_RATE == approx(0.15 / 0.12909)


def test_forward_follows_turned_heading():
    motion = GeckoMotion()
    run(motion, IDENTITY, DRIVE_LEFT, radians(90) / TURN_RATE, dt=radians(90) / TURN_RATE / 20)
    run(motion, IDENTITY, DRIVE_FORWARD, 1.0)
    assert motion.position[:2] == approx((0, 0.15), abs=1e-9)


def test_turn_adds_to_imu_yaw_and_keeps_tilt():
    motion = GeckoMotion()
    run(motion, IDENTITY, DRIVE_LEFT, 0.5)
    q = motion.orient(q_from_rpy(0, radians(20), radians(30)))
    assert yaw_deg(q) == approx(30 + degrees(TURN_RATE * 0.5))
    # Pitch is unchanged by a rotation about the world vertical.
    x, y, z, w = q
    assert degrees(asin(2 * (w * y - z * x))) == approx(20)


def test_turn_is_ignored_on_wall_and_kept_afterwards():
    motion = GeckoMotion()
    run(motion, IDENTITY, DRIVE_RIGHT, 0.5)
    yaw = yaw_deg(motion.orient(IDENTITY))
    motion.set_wall(Wall(heading=(1, 0, 0), face=(1, 0, 0)))
    run(motion, VERTICAL, DRIVE_LEFT, 1.0)
    assert yaw_deg(motion.orient(IDENTITY)) == approx(yaw)


def test_wall_target_uses_turned_heading():
    motion = GeckoMotion()
    run(motion, IDENTITY, DRIVE_LEFT, radians(90) / TURN_RATE, dt=radians(90) / TURN_RATE / 20)
    center, yaw = motion.wall_target(IDENTITY)
    assert yaw == approx(radians(90))
    assert center[1] > 0.2


def test_moves_along_imu_heading():
    motion = GeckoMotion()
    run(motion, q_from_rpy(0, 0, radians(90)), DRIVE_FORWARD, 1.0)
    assert motion.position[:2] == approx((0, 0.15), abs=1e-9)


@pytest.mark.parametrize('pitch', [-60, -30, 30, 60])
def test_tilted_motion_stays_horizontal(pitch):
    q = q_from_rpy(0, radians(pitch), 0)
    motion = GeckoMotion()
    run(motion, q, DRIVE_FORWARD, 1.0)
    assert motion.position[0] == approx(0.15)
    assert motion.position[2] == approx(support(q, DOWN))


def test_near_vertical_keeps_last_heading():
    motion = GeckoMotion()
    motion.step(q_from_rpy(0, 0, radians(90)), DRIVE_STOP, 0.1)
    run(motion, q_from_rpy(0, radians(-89.9), radians(90)), DRIVE_FORWARD, 1.0)
    assert motion.position[:2] == approx((0, 0.15), abs=1e-6)


def test_never_flies_or_sinks_in_any_orientation():
    motion = GeckoMotion()
    for roll in range(-90, 91, 15):
        for pitch in range(-90, 91, 15):
            q = q_from_rpy(radians(roll), radians(pitch), radians(25))
            motion.step(q, DRIVE_FORWARD, 0.033)
            assert abs(motion.position[2] - support(q, DOWN)) < 1e-3


def test_large_dt_is_limited():
    motion = GeckoMotion()
    motion.step(IDENTITY, DRIVE_FORWARD, 5.0)
    assert motion.position[0] == approx(0.015)


def test_non_positive_dt_does_not_move():
    motion = GeckoMotion()
    motion.step(IDENTITY, DRIVE_FORWARD, -1.0)
    assert motion.position[0] == 0.0


def test_wall_target_is_in_front():
    center, yaw = GeckoMotion().wall_target(IDENTITY)
    assert center == approx((0.217, 0, 1.0))
    assert yaw == approx(0.0)


def test_wall_from_center():
    wall = wall_from_center((2.0, 1.0, 1.0), radians(90))
    assert wall.heading == approx((0, 1, 0), abs=1e-9)
    assert wall.face == approx((2.0, 0.95, 1.0))


def test_wall_contact_distance():
    motion = on_wall()
    assert motion.mode is Mode.WALL
    assert motion.position[0] == approx(1 - 0.028419)


def test_climbs_and_descends_wall():
    motion = on_wall()
    z0 = motion.position[2]
    run(motion, VERTICAL, DRIVE_FORWARD, 1.0)
    assert motion.position[2] == approx(z0 + 0.15)
    run(motion, VERTICAL, DRIVE_BACK, 1.0)
    assert motion.position[2] == approx(z0)


def test_wall_limits():
    motion = on_wall()
    run(motion, VERTICAL, DRIVE_FORWARD, 20.0)
    assert motion.position[2] == approx(2.0 - 0.157)
    run(motion, VERTICAL, DRIVE_BACK, 20.0)
    assert motion.position[2] == approx(0.157)


def test_lateral_limit_on_wall():
    q = q_from_rpy(0, radians(-75), radians(60))
    motion = on_wall(q)
    run(motion, q, DRIVE_FORWARD, 30.0)
    assert abs(motion.position[1]) <= 1.0 - support(q, (0, 1, 0)) + 1e-9


def test_descent_ignores_joystick_then_lands():
    motion = on_wall()
    run(motion, VERTICAL, DRIVE_FORWARD, 5.0)
    x, z = motion.position[0], motion.position[2]
    motion.set_wall(None)
    motion.step(VERTICAL, DRIVE_FORWARD, 0.1)
    assert motion.mode is Mode.DESCENT
    assert motion.position[0] == approx(x)
    assert motion.position[2] == approx(z - 0.05)
    run(motion, IDENTITY, DRIVE_STOP, 10.0)
    assert motion.mode is Mode.FLOOR
    assert motion.position[2] == approx(0.028419)


def test_wheel_signs():
    assert set(wheel_velocities(DRIVE_FORWARD).values()) == {10.0}
    assert set(wheel_velocities(DRIVE_BACK).values()) == {-10.0}
    assert set(wheel_velocities(DRIVE_STOP).values()) == {0.0}
    right = wheel_velocities(DRIVE_RIGHT)
    assert right['Wheel_FL_Joint'] == right['Wheel_RL_Joint'] == 10.0
    assert right['Wheel_FR_Joint'] == right['Wheel_RR_Joint'] == -10.0
    left = wheel_velocities(DRIVE_LEFT)
    assert left['Wheel_FL_Joint'] == left['Wheel_RL_Joint'] == -10.0
    assert left['Wheel_FR_Joint'] == left['Wheel_RR_Joint'] == 10.0


@pytest.mark.parametrize('bad', [-1, 5, 99])
def test_unknown_drive_means_stop(bad):
    assert sanitize_drive(bad) == DRIVE_STOP


@pytest.mark.parametrize('bad', [nan, inf, -inf])
def test_invalid_values_keep_current(bad):
    assert clamp_servo(bad, 0.3) == 0.3
    assert clamp_turbine(bad, -0.4) == -0.4


def test_values_are_limited():
    assert clamp_servo(2.0, 0.0) == 0.7854
    assert clamp_servo(-2.0, 0.0) == -0.7854
    assert clamp_turbine(3.0, 0.0) == 1.0
    assert clamp_turbine(-3.0, 0.0) == -1.0


def test_watchdog_stops_drive():
    assert effective_drive(DRIVE_FORWARD, 0.49) == DRIVE_FORWARD
    assert effective_drive(DRIVE_FORWARD, 0.51) == DRIVE_STOP


def test_descent_stops_wheels():
    motion = on_wall()
    run(motion, VERTICAL, DRIVE_FORWARD, 2.0)
    assert motion.wheel_drive(DRIVE_FORWARD) == DRIVE_FORWARD
    motion.set_wall(None)
    motion.step(VERTICAL, DRIVE_FORWARD, 0.033)
    assert motion.mode is Mode.DESCENT
    assert motion.wheel_drive(DRIVE_FORWARD) == DRIVE_STOP


@pytest.mark.parametrize('bad', [nan, inf, -inf, 1e300])
def test_invalid_pose_values_are_rejected(bad):
    assert not finite_pose((bad, 0.0, 1.0))


def test_valid_pose_values_are_accepted():
    assert finite_pose((1.0, -2.0, 1.0, 0.0, 0.0, 0.0, 1.0))
