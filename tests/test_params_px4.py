"""PX4 .params read/write round trips and the Scenario <-> CA parameter mapping."""

import json
from pathlib import Path

import numpy as np
import pytest

from tiltlab.core.geometry import rotors_from_px4_params, rotors_from_scenario
from tiltlab.core.params_px4 import (
    format_px4_float,
    params_file_from_dict,
    params_file_to_bytes,
    params_from_ulog,
    read_params_file,
    scenario_from_ca_params,
    scenario_to_ca_params,
    write_params_file,
    xfly80_3280_curve,
)
from tiltlab.scenario import Mass, Scenario

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests" / "fixtures"
SCENARIOS = ROOT / "scenarios"
PARAM_FILES = sorted(FIXTURES.glob("*.params"))
SCENARIO_FILES = [
    SCENARIOS / "flown_log36_ax1.json",
    SCENARIOS / "flown_log40_vertical_km.json",
    SCENARIOS / "baseline_dihedral30.json",
]


@pytest.mark.parametrize("path", PARAM_FILES, ids=[p.name for p in PARAM_FILES])
def test_params_file_round_trip_byte_for_byte(path: Path, tmp_path: Path) -> None:
    pf = read_params_file(path)
    assert len(pf.entries) > 1000
    assert {e.type_code for e in pf.entries} == {6, 9}
    out = tmp_path / path.name
    write_params_file(pf, out)
    assert out.read_bytes() == path.read_bytes()


@pytest.mark.parametrize("path", PARAM_FILES, ids=[p.name for p in PARAM_FILES])
def test_float_formatting_matches_qgc_export(path: Path) -> None:
    """Every float value re-formatted from its numeric value matches the file text, except the
    two QGC-edited entries documented in params_px4 (MPC_MAN_Y_MAX '30' / '60')."""
    pf = read_params_file(path)
    mismatches = [
        (e.name, e.raw, format_px4_float(float(e.value)))
        for e in pf.entries
        if e.type_code == 9 and e.raw != format_px4_float(float(e.value))
    ]
    assert all(name == "MPC_MAN_Y_MAX" for name, _, _ in mismatches), mismatches
    assert len(mismatches) <= 1


def test_format_px4_float_cases() -> None:
    assert format_px4_float(1600.0) == "1.6e+03"
    assert format_px4_float(-10.0) == "-1e+01"
    assert format_px4_float(1499.0) == "1499"
    assert format_px4_float(1013.25) == "1013.25"
    assert format_px4_float(0.0001) == "0.0001"
    assert format_px4_float(1e-5) == "1e-05"
    assert format_px4_float(0.0) == "0"
    assert format_px4_float(-0.3) == "-0.3"
    assert format_px4_float(np.float32(0.00052359875)) == "0.00052359875"


def test_params_file_from_dict_writes_crlf_and_types() -> None:
    pf = params_file_from_dict({"CA_ROTOR_COUNT": 10, "CA_ROTOR0_PX": -0.3})
    data = params_file_to_bytes(pf)
    assert b"\r\n" in data and b"\n\n" not in data
    assert b"1\t1\tCA_ROTOR0_PX\t-0.3\t9\r\n" in data
    assert b"1\t1\tCA_ROTOR_COUNT\t10\t6\r\n" in data


def test_ulog_parameters_match_readme_facts() -> None:
    p36 = params_from_ulog(FIXTURES / "log_36_2026-9-9-14-11-36.ulg")
    assert p36["CA_ROTOR_COUNT"] == 10 and p36["CA_METHOD"] == 2 and p36["MC_AIRMODE"] == 0
    assert p36["CA_ROTOR0_KM"] == 0.0  # README: log 36 ran with KM 0, the .params file has -0.05
    assert isinstance(p36["CA_ROTOR_COUNT"], int) and isinstance(p36["CA_ROTOR0_CT"], float)


@pytest.mark.parametrize("path", SCENARIO_FILES, ids=[p.stem for p in SCENARIO_FILES])
def test_scenario_params_round_trip(path: Path) -> None:
    scenario = Scenario.model_validate(json.loads(path.read_text(encoding="utf-8")))
    params = scenario_to_ca_params(scenario)
    assert params["CA_ROTOR_COUNT"] == 10
    assert params["CA_METHOD"] == scenario.control.ca_method
    assert len([k for k in params if k.startswith("CA_ROTOR") and k != "CA_ROTOR_COUNT"]) == 80

    back = scenario_from_ca_params(
        params,
        name=scenario.meta.name,
        created=scenario.meta.created,
        fan_curves=scenario.fan_curves,
        curve_ref=scenario.fans[0].curve_ref,
        mass=scenario.mass,
        rig=scenario.rig,
        control=scenario.control,
    )
    params2 = scenario_to_ca_params(back)
    assert params2 == params

    # geometry seen by PX4 is identical: positions, normalised axes, CT, KM
    r_a = rotors_from_scenario(scenario)
    r_b = rotors_from_px4_params(params2)
    for a, b in zip(r_a, r_b, strict=True):
        np.testing.assert_allclose(a.position, b.position, atol=1e-7)
        na = np.asarray(a.axis) / np.linalg.norm(a.axis)
        nb = np.asarray(b.axis) / np.linalg.norm(b.axis)
        np.testing.assert_allclose(na, nb, atol=1e-6)
        assert np.float32(a.thrust_coef) == np.float32(b.thrust_coef)
        assert np.float32(a.moment_ratio) == np.float32(b.moment_ratio)


def test_flown_scenarios_match_fixture_geometry() -> None:
    """The flown scenarios reproduce the CA_ROTORn_* of the parameter files they were built
    from (log 36: 14150909.params with KM reset to 0; log 40: the v4 rig/airmode file)."""
    cases = {
        "flown_log36_ax1.json": ("14150909.params", True),
        "flown_log40_vertical_km.json": ("20260909_1515_params_v4_rig_airmode.params", False),
    }
    for sc_name, (pf_name, km_reset) in cases.items():
        scenario = Scenario.model_validate(
            json.loads((SCENARIOS / sc_name).read_text(encoding="utf-8"))
        )
        fixture = read_params_file(FIXTURES / pf_name).to_dict()
        params = scenario_to_ca_params(scenario)
        for i in range(10):
            for key in ("PX", "PY", "PZ", "CT"):
                assert np.float32(params[f"CA_ROTOR{i}_{key}"]) == np.float32(
                    fixture[f"CA_ROTOR{i}_{key}"]
                )
            km_expected = 0.0 if km_reset else fixture[f"CA_ROTOR{i}_KM"]
            assert np.float32(params[f"CA_ROTOR{i}_KM"]) == np.float32(km_expected)
            ax = np.array([params[f"CA_ROTOR{i}_A{c}"] for c in "XYZ"])
            fx = np.array([fixture[f"CA_ROTOR{i}_A{c}"] for c in "XYZ"])
            np.testing.assert_allclose(ax / np.linalg.norm(ax), fx / np.linalg.norm(fx), atol=1e-6)


def test_scenario_from_params_records_ct_override() -> None:
    fixture = read_params_file(FIXTURES / "14150909.params").to_dict()
    sc = scenario_from_ca_params(
        fixture,
        name="t",
        created="2026-09-09T00:00:00",
        fan_curves={"xfly80_3280": xfly80_3280_curve()},
        curve_ref="xfly80_3280",
        mass=Mass(total_kg=12.0, estimated=True),
    )
    # 6.5 N as flown differs from the 33.3 N curve, so the flown CT is kept as an override
    assert sc.control.px4_params_override["CA_ROTOR0_CT"] == 6.5
    assert sc.control.reaction_torque is True  # rotor 0 has KM -0.05 in this file
    assert sc.fans_sorted()[0].spin == "CW" and sc.fans_sorted()[0].km == pytest.approx(0.05)
    assert sc.fans_sorted()[0].tilt_deg == pytest.approx(45.0)
