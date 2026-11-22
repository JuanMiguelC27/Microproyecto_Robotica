"""Front/back correction: the model front (+X) is the BNO085 back (Rz(180 deg))."""
from math import pi, radians, sqrt, sin, cos

from pytest import approx

import gecko_twin_node as twin
from gecko_geometry import q_from_rpy, rotate

NO_ROT = (0.0, 0.0, 0.0, 1.0)


def make(yaw_deg):
    node = twin.GeckoTwinNode.__new__(twin.GeckoTwinNode)
    node.q_base_imu = twin.q_normalize(q_from_rpy(0, 0, radians(yaw_deg)))
    return node


def model_delta(node, q_imu0, q_imu):
    """Replicates GeckoTwinNode.relative_orientation (startup calibration q0)."""
    ref = node.sensor_to_base(q_imu0)
    return twin.q_normalize(twin.q_multiply(twin.q_conjugate(ref), node.sensor_to_base(q_imu)))


def test_default_mount_is_180():
    src = open(twin.__file__).read()
    assert "'imu_mount_yaw_deg': 180.0" in src


def test_front_up_and_left_up_same_side():
    node = make(180.0)
    # Real front up (IMU frame: front toward -X_imu): pitch positive about +Y_imu.
    q = model_delta(node, NO_ROT, q_from_rpy(0, radians(20), 0))
    # model +X (front) must rise, model +Y (left) unchanged in height
    assert rotate(q, (1, 0, 0))[2] > 0.3
    assert rotate(q, (0, 1, 0))[2] == approx(0, abs=1e-9)
    # Real left up (IMU +Y_imu is real right): left side up = roll negative in IMU
    q = model_delta(node, NO_ROT, q_from_rpy(radians(-20), 0, 0))
    assert rotate(q, (0, 1, 0))[2] > 0.3
    assert rotate(q, (1, 0, 0))[2] == approx(0, abs=1e-9)


def test_without_correction_is_inverted():
    q = model_delta(make(0.0), NO_ROT, q_from_rpy(0, radians(20), 0))
    assert rotate(q, (1, 0, 0))[2] < 0


def test_yaw_preserved_and_conjugation():
    node = make(180.0)
    q = model_delta(node, NO_ROT, q_from_rpy(0, 0, radians(90)))
    assert q == approx(q_from_rpy(0, 0, radians(90)), abs=1e-9)
    # also with an arbitrary calibration pose: dq' = Rz dq Rz^-1
    q0 = q_from_rpy(0.1, -0.2, 0.7)
    q1 = twin.q_multiply(q0, q_from_rpy(radians(10), radians(15), radians(30)))
    rz = q_from_rpy(0, 0, pi)
    expect = twin.q_multiply(twin.q_multiply(rz, twin.q_multiply(twin.q_conjugate(q0), q1)),
                             twin.q_conjugate(rz))
    got = model_delta(node, q0, q1)
    assert all(abs(a - b) < 1e-9 for a, b in zip(got, expect)) or \
        all(abs(a + b) < 1e-9 for a, b in zip(got, expect))
