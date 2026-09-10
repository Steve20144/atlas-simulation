"""Geometry and effectiveness matrix tests (PLAN.md section 6/M2)."""

from pathlib import Path

import numpy as np
import pytest

from tiltlab.core.geometry import (
    GeometryFlags,
    Rotor,
    compute_effectiveness_matrix,
    effectiveness_matrix,
    rotors_from_px4_params,
)
from tiltlab.core.params_px4 import params_from_ulog
from tiltlab.scenario import axis_to_tilt_azimuth, tilt_azimuth_to_axis

FIXTURES = Path(__file__).parent / "fixtures"
LOG36 = FIXTURES / "log_36_2026-9-9-14-11-36.ulg"


def test_axis_is_normalised_and_bad_axis_skipped() -> None:
    # AER:162-169: the axis is normalised; a zero axis leaves an all-zero column but counts
    m, n = compute_effectiveness_matrix(
        [
            Rotor((0.0, 0.0, 0.0), (0.0, 0.0, -3.0), 1.0, 0.0),
            Rotor((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0, 0.0),
        ]
    )
    assert n == 2
    assert m[5, 0] == pytest.approx(-1.0)
    assert np.all(m[:, 1] == 0.0)


def test_zero_ct_rotor_skipped_but_counted() -> None:
    m, n = compute_effectiveness_matrix([Rotor((0.1, 0.0, 0.0), (0.0, 0.0, -1.0), 0.0, 0.05)])
    assert n == 1  # AER:156 increments before the CT check at AER:191
    assert np.all(m == 0.0)


def test_km_sign_matches_px4_unit_test() -> None:
    # ActuatorEffectivenessRotorsTest.cpp:53,78,81: axis (0,0,-1), CT 1, KM 0.05 gives yaw +0.05
    # and thrust_z -1 (moment = ct * p x a - ct * km * a, AER:199)
    m = effectiveness_matrix(np.zeros((1, 3)), np.array([[0.0, 0.0, -1.0]]), 1.0, 0.05)
    assert m[2, 0] == pytest.approx(0.05)
    assert m[5, 0] == pytest.approx(-1.0)
    # the propeller torque flag zeroes km (AER:179-181)
    m2 = effectiveness_matrix(
        np.zeros((1, 3)),
        np.array([[0.0, 0.0, -1.0]]),
        1.0,
        0.05,
        GeometryFlags(propeller_torque_disabled=True),
    )
    assert m2[2, 0] == 0.0


def test_cross_product_sign() -> None:
    # rotor on the right (+y) thrusting up (-z) gives negative roll moment: (p x a)_x = py*az
    m = effectiveness_matrix(np.array([[0.0, 0.5, 0.0]]), np.array([[0.0, 0.0, -1.0]]), 2.0, 0.0)
    assert m[0, 0] == pytest.approx(-1.0)
    # rotor in front (+x) thrusting up gives positive pitch moment: (p x a)_y = -px*az
    m = effectiveness_matrix(np.array([[0.5, 0.0, 0.0]]), np.array([[0.0, 0.0, -1.0]]), 2.0, 0.0)
    assert m[1, 0] == pytest.approx(1.0)


def test_hand_computed_6x2() -> None:
    # rotor 0: p = (0.3, -0.2, 0.1), axis (0, 0, -1), ct 6.5, km -0.05
    # rotor 1: p = (-0.1, 0.4, 0.0), axis (1, 0, -1) -> (s, 0, -s), s = 1/sqrt(2), ct 4.0, km 0
    s = 1.0 / np.sqrt(2.0)
    pos = np.array([[0.3, -0.2, 0.1], [-0.1, 0.4, 0.0]])
    axes = np.array([[0.0, 0.0, -1.0], [1.0, 0.0, -1.0]])
    m = effectiveness_matrix(pos, axes, [6.5, 4.0], [-0.05, 0.0])
    assert m.shape == (6, 2)
    # rotor 0: p x a = (py*az - pz*ay, -px*az + pz*ax, px*ay - py*ax) = (0.2, 0.3, 0)
    # moment = 6.5 * (0.2, 0.3, 0) - 6.5 * (-0.05) * (0, 0, -1) = (1.3, 1.95, -0.325)
    np.testing.assert_allclose(m[:, 0], [1.3, 1.95, -0.325, 0.0, 0.0, -6.5], rtol=1e-6, atol=1e-7)
    # rotor 1: a = (s, 0, -s); p x a = (0.4*(-s), -(-0.1)(-s), -0.4*s) = (-0.4s, -0.1s, -0.4s)
    expected1 = 4.0 * np.array([-0.4 * s, -0.1 * s, -0.4 * s, s, 0.0, -s])
    np.testing.assert_allclose(m[:, 1], expected1, rtol=1e-6, atol=1e-7)


@pytest.mark.parametrize(
    ("tilt", "az"),
    [
        (0.0, 0.0),
        (30.0, 90.0),
        (30.0, 270.0),
        (45.0, 0.0),
        (60.0, 135.0),
        (90.0, 359.0),
        (12.5, 200.0),
    ],
)
def test_tilt_azimuth_axis_round_trip(tilt: float, az: float) -> None:
    axis = tilt_azimuth_to_axis(tilt, az)
    assert np.linalg.norm(axis) == pytest.approx(1.0)
    t2, a2 = axis_to_tilt_azimuth(axis)
    assert t2 == pytest.approx(tilt, abs=1e-9)
    assert a2 == pytest.approx(az if tilt > 0 else 0.0, abs=1e-9)


def test_axis_conventions() -> None:
    np.testing.assert_allclose(tilt_azimuth_to_axis(0.0, 123.0), [0.0, 0.0, -1.0], atol=1e-12)
    np.testing.assert_allclose(
        tilt_azimuth_to_axis(30.0, 90.0), [0.0, 0.5, -np.sqrt(3) / 2], atol=1e-12
    )
    assert axis_to_tilt_azimuth([0.0, 0.0, -5.0]) == (0.0, 0.0)
    assert axis_to_tilt_azimuth([1.0, 0.0, -1.0]) == pytest.approx((45.0, 0.0))


def test_log36_rotor_axes() -> None:
    params = params_from_ulog(LOG36)
    rotors = rotors_from_px4_params(params)
    assert len(rotors) == 10
    m, n = compute_effectiveness_matrix(rotors)
    assert n == 10
    for i in range(8):
        # the tilted rotors: axis (1, 0, -1) normalised (AER:164-165); thrust rows = ct * axis
        axis = m[3:6, i] / rotors[i].thrust_coef
        np.testing.assert_allclose(axis, [0.7071, 0.0, -0.7071], atol=1e-4)
        assert axis_to_tilt_azimuth(rotors[i].axis) == pytest.approx((45.0, 0.0))
    for i in (8, 9):
        np.testing.assert_allclose(m[3:6, i] / rotors[i].thrust_coef, [0.0, 0.0, -1.0], atol=1e-6)
