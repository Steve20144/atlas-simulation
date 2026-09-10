"""Fan model tests (PLAN.md section 6/M3)."""

import numpy as np
import pytest

from tiltlab.core.fan import (
    G0,
    XFLY80_NOMINAL_PACK_V,
    Battery,
    FanCurve,
    FanLag,
    estimated_default_xfly80,
)


@pytest.fixture(params=["3280-KV2200", "3665-KV2300 PRO"])
def curve(request: pytest.FixtureRequest) -> FanCurve:
    return estimated_default_xfly80(request.param)


def test_thrust_and_power_monotonic(curve: FanCurve) -> None:
    cmd = np.linspace(0.0, 1.0, 1001)
    assert np.all(np.diff(curve.thrust(cmd)) >= 0.0)
    assert np.all(np.diff(curve.power(cmd)) >= 0.0)
    assert curve.estimated is True
    assert curve.cmd.size == 8


def test_inverse_round_trip(curve: FanCurve) -> None:
    thrust = np.linspace(0.0, curve.max_thrust_n, 501)
    cmd = curve.cmd_from_thrust(thrust)
    assert np.all(cmd >= 0.0) and np.all(cmd <= 1.0)
    np.testing.assert_allclose(curve.thrust(cmd), thrust, rtol=0.0, atol=1e-9)
    # Clamping outside the reachable range.
    assert curve.cmd_from_thrust(-1.0) == 0.0
    assert curve.cmd_from_thrust(curve.max_thrust_n * 2.0) == 1.0
    assert curve.thrust(1.5) == curve.max_thrust_n


def test_ct_for_px4_is_thrust_at_full_command(curve: FanCurve) -> None:
    assert curve.ct_for_px4() == float(curve.thrust(1.0))
    assert curve.ct_for_px4() == curve.max_thrust_n


def test_default_3280_max_thrust() -> None:
    c = estimated_default_xfly80("3280-KV2200")
    assert c.ct_for_px4() == pytest.approx(3.400 * G0, rel=1e-3)
    assert c.max_power_w == pytest.approx(100.0 * XFLY80_NOMINAL_PACK_V)
    assert c.cells == 6
    # thrust proportional to power^(2/3) at every knot
    ratio = c.thrust_n[1:] / c.power_w[1:] ** (2.0 / 3.0)
    np.testing.assert_allclose(ratio, ratio[0], rtol=1e-12)
    # aliases resolve to the same curve
    assert estimated_default_xfly80("xfly80_3280").to_scenario_curve() == c.to_scenario_curve()
    pro = estimated_default_xfly80("3665")
    assert pro.ct_for_px4() == pytest.approx(3.650 * G0, rel=1e-3)
    with pytest.raises(ValueError):
        estimated_default_xfly80("90mm")


def test_lag_reaches_step_after_five_tau() -> None:
    tau = 0.15
    lag = FanLag(tau_s=tau)
    dt = 0.002
    n = int(round(5.0 * tau / dt))
    for _ in range(n):
        x = lag.step(1.0, dt)
    assert abs(float(x) - 1.0) < 0.01
    # exact discrete solution: after one step of dt the state is 1 - exp(-dt/tau)
    lag.reset(0.0)
    assert float(lag.step(1.0, dt)) == pytest.approx(1.0 - np.exp(-dt / tau))
    # vectorised over fans, zero tau is passthrough
    v = FanLag(tau_s=0.0, state=np.zeros(3))
    np.testing.assert_array_equal(v.step(np.array([0.2, 0.5, 1.0]), dt), [0.2, 0.5, 1.0])


def test_battery_current_and_voltage() -> None:
    bat = Battery(cells=6, cell_full_v=4.2, cell_nominal_v=3.7, cell_resistance_ohm=0.003,
                  capacity_ah=10.0)
    assert bat.nominal_pack_v == pytest.approx(22.2)
    assert bat.open_circuit_voltage() == pytest.approx(25.2)
    power = np.array([0.0, 500.0, 2220.0])
    volt = np.full(3, 22.2)
    np.testing.assert_allclose(Battery.current_from_power(power, volt), power / volt)
    v, i = bat.load(2220.0)
    assert float(i) == pytest.approx(2220.0 / float(v))
    ocv = bat.open_circuit_voltage()
    assert float(v) == pytest.approx(ocv - float(i) * bat.pack_resistance_ohm)
    assert float(bat.voltage_under_load(100.0)) == pytest.approx(25.2 - 100.0 * 0.018)
    # derating: unity at nominal voltage, quadratic otherwise
    assert float(bat.thrust_derating(22.2)) == pytest.approx(1.0)
    assert float(bat.thrust_derating(11.1)) == pytest.approx(0.25)
    c = estimated_default_xfly80("3280")
    assert float(bat.derated_thrust(c, 1.0, 11.1)) == pytest.approx(0.25 * c.ct_for_px4())
    # charge integration: 100 A for 36 s is 1 Ah
    bat.consume(100.0, 36.0)
    assert bat.consumed_ah == pytest.approx(1.0)
    assert bat.state_of_charge == pytest.approx(0.9)
    assert bat.open_circuit_voltage() < 25.2


def test_scenario_dict_round_trip(curve: FanCurve) -> None:
    d = curve.to_scenario_curve()
    assert d["estimated"] is True
    assert set(d) == {"cells", "points", "lag_s", "max_continuous_A", "notes", "estimated",
                      "rotor_inertia_kgm2"}
    back = FanCurve.from_scenario_curve(d)
    assert back.to_scenario_curve() == d
    np.testing.assert_array_equal(back.cmd, curve.cmd)
    # PLAN.md section 5 example (no estimated key) loads as measured
    plan = {"cells": 6, "points": [{"cmd": 0.0, "thrust_N": 0, "power_W": 0},
                                   {"cmd": 1.0, "thrust_N": 33.3, "power_W": 2450}],
            "lag_s": 0.15, "max_continuous_A": 100, "notes": "manufacturer max only"}
    c = FanCurve.from_scenario_curve(plan)
    assert c.estimated is False and c.rotor_inertia_kgm2 is None
    assert c.ct_for_px4() == pytest.approx(33.3)
    assert float(c.thrust(0.5)) == pytest.approx(16.65)


def test_validation_rejects_bad_points() -> None:
    good = [{"cmd": 0.0, "thrust_N": 0.0, "power_W": 0.0},
            {"cmd": 1.0, "thrust_N": 10.0, "power_W": 100.0}]
    FanCurve.from_points(good)
    with pytest.raises(ValueError):
        FanCurve.from_points([good[1], good[0]])  # unsorted
    with pytest.raises(ValueError):
        FanCurve.from_points([good[0], {"cmd": 1.2, "thrust_N": 10.0, "power_W": 100.0}])
    with pytest.raises(ValueError):
        FanCurve.from_points([good[0], {"cmd": 1.0, "thrust_N": -1.0, "power_W": 100.0}])
    with pytest.raises(ValueError):
        FanCurve.from_points([good[0], {"cmd": 0.5, "thrust_N": 5.0, "power_W": 50.0},
                              {"cmd": 1.0, "thrust_N": 4.0, "power_W": 100.0}])
    with pytest.raises(ValueError):
        FanCurve.from_points([good[0]])
    # a deadband (flat start) is allowed; zero thrust inverts to the end of the deadband
    dead = FanCurve.from_points([good[0], {"cmd": 0.1, "thrust_N": 0.0, "power_W": 5.0}, good[1]])
    assert float(dead.cmd_from_thrust(0.0)) == pytest.approx(0.1)
    assert float(dead.thrust(dead.cmd_from_thrust(5.0))) == pytest.approx(5.0)
