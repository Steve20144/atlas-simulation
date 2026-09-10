"""M4 acceptance: the static metrics reproduce the two known facts of PLAN.md section 6/M4.

1. All-vertical geometry with the KM yaw model (flown_log40_vertical_km, stock concept):
   yaw authority is roughly one third of roll authority.
2. Faithful dihedral geometry (baseline_dihedral30, stock concept, Fx = Fy = 0 rows): yaw is
   unusable, the min-norm command per N m of yaw is several times the roll cost and the
   bounded LP authority is a fraction of a newton metre.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from tiltlab.core.metrics import DEFAULT_WEIGHTS, compute_metrics, metrics_table
from tiltlab.scenario import Scenario

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"


def load(name: str) -> Scenario:
    return Scenario.model_validate(json.loads((SCENARIOS / f"{name}.json").read_text()))


@pytest.fixture(scope="module")
def stock_metrics() -> dict[str, dict]:
    out = {}
    for name in ("flown_log40_vertical_km", "baseline_dihedral30"):
        out[name] = compute_metrics(load(name), "stock")
    print()
    for m in out.values():
        print(metrics_table(m))
    return out


def test_vertical_km_yaw_is_about_one_third_of_roll(stock_metrics: dict[str, dict]) -> None:
    m = stock_metrics["flown_log40_vertical_km"]
    a = m["authority"]
    ratio = a["yaw"]["plus"] / a["roll"]["plus"]
    ratio_minnorm = a["roll"]["minnorm_cmd_per_unit"] / a["yaw"]["minnorm_cmd_per_unit"]
    print(f" log40 yaw/roll authority {ratio:.3f}, min-norm cost ratio {ratio_minnorm:.3f}")
    assert 0.2 <= ratio <= 0.5
    assert 0.2 <= ratio_minnorm <= 0.5
    assert m["controlled_axes"] == ["roll", "pitch", "yaw", "Fz"]
    assert all(b == "stock_px4" for b in m["badges"].values())


def test_dihedral_stock_yaw_is_unusable(stock_metrics: dict[str, dict]) -> None:
    m = stock_metrics["baseline_dihedral30"]
    a = m["authority"]
    yaw, roll = a["yaw"], a["roll"]
    # PLAN.md M4: about 4.8 units on the inner fans per 0.5 N m yaw at the logged CT 6.5 N;
    # at the fan curve CT (33.3 N) that is 4.8 * 6.5 / 33.3 = 0.94 units, still the full range.
    ct = m["effectiveness"]["ct_N"][0]
    assert yaw["minnorm_cmd_per_0p5Nm"] * ct / 6.5 == pytest.approx(4.8, rel=0.05)
    assert yaw["minnorm_fan_of_max"] in (2, 3, 4, 5)
    assert yaw["minnorm_cmd_per_unit"] > 3.0 * roll["minnorm_cmd_per_unit"]
    unusable = (not yaw["plus_attainable"]) or yaw["plus"] < 0.5
    assert unusable, f"yaw authority {yaw['plus']} N m should be unusable"


def test_hover_solution_balances_weight() -> None:
    sc = load("baseline_dihedral30")
    for concept in ("stock", "fully_actuated"):
        m = compute_metrics(sc, concept)
        h = m["hover"]
        weight = sc.mass.total_kg * sc.environment.gravity
        assert h["exact"]
        assert h["vertical_thrust_N"] == pytest.approx(weight, rel=1e-3)
        assert np.allclose(h["residual"], 0.0, atol=1e-3)
        assert 0.0 < h["headroom"] < 1.0
        assert h["power_W"] > 0.0
        assert len(h["u"]) == 10 and len(h["thrust_N"]) == 10
        assert m["collective"] == pytest.approx(m["collective_hover"])
        assert m["estimated"] and "mass" in m["estimated_sources"]


def test_fully_actuated_axes_and_badges() -> None:
    m = compute_metrics(load("baseline_dihedral30"), "fully_actuated")
    assert m["controlled_axes"] == ["roll", "pitch", "yaw", "Fx", "Fy", "Fz"]
    assert m["badges"]["authority.Fy"] == "needs_fully_actuated_controller"
    assert m["badges"]["authority.roll"] == "stock_px4"
    assert m["badges"]["hover"] == "needs_fully_actuated_controller"
    assert m["authority"]["Fy"]["plus"] > 0.0  # dihedral gives side force
    assert not m["authority"]["Fx"]["plus_attainable"]  # no fan tilts forward
    assert m["conditioning"]["rank"] == 5
    assert m["conditioning"]["null_space_dim"] == 5
    assert m["coupling"]["allocator"] == "CA_METHOD_0"


def test_collective_and_weights() -> None:
    sc = load("flown_log40_vertical_km")
    low = compute_metrics(sc, "stock", collective=0.15)
    assert low["collective"] == pytest.approx(0.15)
    assert low["px4_thrust_sp_z"] < 0.0
    # less collective, less room to reduce thrust: -Fz authority shrinks, +Fz authority grows
    hov = compute_metrics(sc, "stock")
    assert low["authority"]["Fz"]["plus"] < hov["authority"]["Fz"]["plus"]
    weighted = compute_metrics(sc, "stock", weights={"hover_headroom": 1.0, "decoupling": 0.0})
    assert set(weighted["score"]["weights"]) == set(DEFAULT_WEIGHTS)
    assert weighted["score"]["weights"]["decoupling"] == 0.0
    with pytest.raises(ValueError):
        compute_metrics(sc, "stock", weights={"bogus": 1.0})
    with pytest.raises(ValueError):
        compute_metrics(sc, "nope")


def test_coupling_matrix_shape_and_diagonal() -> None:
    m = compute_metrics(load("flown_log40_vertical_km"), "stock")
    c = m["coupling"]
    frac = np.array(c["leakage_fraction"], dtype=float)
    assert frac.shape == (4, 4)
    assert np.array(c["leakage_physical"]).shape == (4, 6)
    # commanded 20 percent of authority is delivered on the commanded axis
    assert np.allclose(np.diag(frac), 1.0, atol=0.05)
    assert c["max_offaxis_fraction"] < 0.05


def test_json_serialisable() -> None:
    m = compute_metrics(load("baseline_dihedral30"), "fully_actuated")
    text = json.dumps(m)
    assert "NaN" not in text and "Infinity" not in text
