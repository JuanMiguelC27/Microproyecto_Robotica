from math import radians

from pytest import approx

from gecko_geometry import IDENTITY, q_from_rpy, rotate, support

UP, DOWN, FRONT, BACK, LEFT = (0, 0, 1), (0, 0, -1), (1, 0, 0), (-1, 0, 0), (0, 1, 0)


def test_nose_up_is_negative_pitch():
    assert rotate(q_from_rpy(0, radians(-90), 0), FRONT) == approx((0, 0, 1), abs=1e-9)


def test_level_support():
    assert support(IDENTITY, DOWN) == approx(0.028419)
    assert support(IDENTITY, UP) == approx(0.0975)
    assert support(IDENTITY, FRONT) == approx(0.157)
    assert support(IDENTITY, BACK) == approx(0.157)
    assert support(IDENTITY, LEFT) == approx(0.1308)


def test_vertical_support():
    q = q_from_rpy(0, radians(-90), 0)
    assert support(q, DOWN) == approx(0.157)
    assert support(q, FRONT) == approx(0.028419)
