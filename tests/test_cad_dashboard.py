"""Atlas Mass & CoG dashboard ingestion: geometry, frame, EDF detection, mass properties, scenario
build."""

from __future__ import annotations

import json

import numpy as np
import pytest

from tiltlab.cad.fusion_dashboard import (
    FusionFrame,
    Weights,
    build_scenario,
    detect_edfs,
    export_glb,
    fit_reference_cg,
    load_dashboard,
    mass_properties,
    mesh_properties,
    order_edfs_px4,
    volume_centroid_mm,
)
from tiltlab.core.params_px4 import read_params_file
from tiltlab.scenario import Scenario

from .conftest import FIXTURES

HTML = FIXTURES / "cog_dashboard_full_8.html"
PARAMS = FIXTURES / "20260909_1515_params_v4_rig_airmode.params"


@pytest.fixture(scope="module")
def dash():
    return load_dashboard(HTML)


def test_mesh_properties_unit_cube():
    v = [0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1]
    # outward-facing triangles of the unit cube
    i = [
        0,
        2,
        1,
        0,
        3,
        2,
        4,
        5,
        6,
        4,
        6,
        7,
        0,
        1,
        5,
        0,
        5,
        4,
        1,
        2,
        6,
        1,
        6,
        5,
        2,
        3,
        7,
        2,
        7,
        6,
        3,
        0,
        4,
        3,
        4,
        7,
    ]
    vol, cen, inertia = mesh_properties(v, i)
    assert vol == pytest.approx(1.0)
    assert np.allclose(cen, [0.5, 0.5, 0.5])
    # about the origin, unit density: Ixx = Iyy = Izz = 2/3, Ixy = -1/4
    assert np.allclose(np.diag(inertia), 2 / 3)
    assert inertia[0, 1] == pytest.approx(-0.25)


def test_bodies_match_dashboard(dash):
    assert dash.doc == "PHASE_0.1_ASSY"
    assert len(dash.bodies) == 37
    for b in dash.bodies:
        assert np.linalg.norm(b.mesh_centroid_mm - b.com_mm) < 1.5, b.name
        if (
            "Rod" not in b.name
        ):  # rods are low-poly cylinders (44 vertices), their mesh volume is 25 percent low
            assert b.mesh_vol_mm3 / 1000.0 == pytest.approx(b.vol_cm3, rel=0.06), b.name


def test_frame_is_right_handed_and_matches_default():
    R = FusionFrame().rotation()
    assert np.linalg.det(R) == pytest.approx(1.0)
    assert np.allclose(R @ np.array([0, 0, 1.0]), [1, 0, 0])  # Fusion +z -> forward
    assert np.allclose(R @ np.array([0, 1.0, 0]), [0, 0, 1])  # Fusion +y -> down (up is -y)
    assert np.allclose(R @ np.array([1.0, 0, 0]), [0, 1, 0])  # Fusion +x -> right
    R2 = FusionFrame(up="+y").rotation()
    assert np.linalg.det(R2) == pytest.approx(1.0)
    assert np.allclose(R2 @ np.array([0, 1.0, 0]), [0, 0, -1])


def test_levels_match_the_aircraft_description(dash):
    """Inner wing motors at the level of the front fans; each pair outward one step lower
    (50 mm down per 87 mm outward, a 30 degree offset)."""
    frame = FusionFrame()
    edfs = order_edfs_px4(detect_edfs(dash), frame)
    z = [frame.to_frd_m(e.body.com_mm)[2] for e in edfs]  # FRD z, down positive
    y = [abs(frame.to_frd_m(e.body.com_mm)[1]) for e in edfs]
    steps_down = [z[i] - z[i + 2] for i in (0, 2, 4)]  # outer minus next inner, per left fans
    assert all(abs(s - 0.050) < 0.002 for s in steps_down)
    steps_out = [y[i] - y[i + 2] for i in (0, 2, 4)]
    assert all(abs(s - 0.087) < 0.002 for s in steps_out)
    assert np.degrees(np.arctan2(0.050, 0.087)) == pytest.approx(30.0, abs=0.5)
    # the front fans' duct bottom lies at the inner wing motors' centre height
    front = edfs[8].body.vertices_mm
    front_bottom_up = -front[:, 1].max()  # up = -y
    inner_centre_up = -edfs[6].body.com_mm[1]
    assert abs(front_bottom_up - inner_centre_up) < 0.015 * 1e3


def test_ten_edfs_with_duct_axes(dash):
    edfs = detect_edfs(dash)
    assert len(edfs) == 10
    for e in edfs:
        assert 45 <= e.duct_radius_mm <= 55
        assert 120 <= e.length_mm <= 140
    wing = [e for e in edfs if abs(e.body.com_mm[0]) > 30]
    centre = [e for e in edfs if abs(e.body.com_mm[0]) <= 30]
    assert len(wing) == 8 and len(centre) == 2
    for e in wing:  # as modelled: wing fan ducts run along the fore-aft Fusion z axis
        assert abs(e.axis_fusion[2]) > 0.999
    for e in centre:  # centreline fans are vertical (Fusion y)
        assert abs(e.axis_fusion[1]) > 0.999


def test_px4_order_and_symmetry(dash):
    frame = FusionFrame()
    edfs = order_edfs_px4(detect_edfs(dash), frame)
    lat = [frame.to_frd_m(e.body.com_mm)[1] for e in edfs]
    assert [round(abs(v), 3) for v in lat[:8]] == [
        0.371,
        0.371,
        0.284,
        0.284,
        0.197,
        0.197,
        0.11,
        0.11,
    ]
    assert all(lat[i] < 0 < lat[i + 1] for i in range(0, 8, 2))
    for i in range(0, 8, 2):
        a, b = frame.to_frd_m(edfs[i].body.com_mm), frame.to_frd_m(edfs[i + 1].body.com_mm)
        assert abs(a[0] - b[0]) < 1e-3 and abs(a[2] - b[2]) < 1e-3
    fx = [frame.to_frd_m(e.body.com_mm)[0] for e in edfs[8:]]
    assert fx[1] - fx[0] == pytest.approx(0.110, abs=1e-3)


def test_flown_params_have_the_vertical_axis_inverted(dash):
    """With up = -y the lateral positions still match the flown CA_ROTOR file to the rounding, but
    the vertical pattern is mirrored (the file was built with up = +y): the fit residual in z grows
    outward instead of vanishing."""
    frame = FusionFrame()
    edfs = order_edfs_px4(detect_edfs(dash), frame)
    ca = {e.name: e.value for e in read_params_file(PARAMS).entries}
    _cg, res = fit_reference_cg(edfs, ca, frame)
    assert np.abs(res[:8, 1]).max() < 0.005
    assert np.abs(res[:8, 2]).max() > 0.1  # 0.14 m of the outer pair is on the wrong side
    # in the flipped frame the residuals do vanish
    _cg2, res2 = fit_reference_cg(edfs, ca, FusionFrame(up="+y"))
    assert np.abs(res2[:8, 2]).max() < 0.006


def test_volume_centroid_and_glb(dash, tmp_path):
    vc = volume_centroid_mm(dash)
    assert vc.shape == (3,) and abs(vc[0]) < 5  # symmetric aircraft: on the centreline
    frame = FusionFrame()
    front = frame.to_frd_m(dash.by_name("XFLY 80mm EDF:5").com_mm - vc)
    assert 0.35 < front[0] < 0.5  # front fans well ahead of the reference point
    glb = export_glb(dash, frame, vc, tmp_path / "m.glb")
    assert glb.exists() and glb.stat().st_size > 100_000
    assert glb.read_bytes()[:4] == b"glTF"


def test_mass_properties_point_masses(dash):
    frame = FusionFrame()
    w = Weights(by_type={b.type_name: 100.0 for b in dash.bodies})
    mp = mass_properties(dash, w, frame)
    assert mp.total_kg == pytest.approx(3.7)
    expected = np.mean([b.com_mm for b in dash.bodies], axis=0)
    assert np.allclose(mp.cg_fusion_mm, expected)
    assert np.allclose(mp.cg_frd_m, frame.to_frd_m(expected))
    eig = np.linalg.eigvalsh(mp.inertia_frd_kgm2)
    assert eig.min() > 0
    # parallel-axis lower bound: point masses at the centroids
    point = sum(
        0.1 * (float(r @ r) * np.eye(3) - np.outer(r, r))
        for r in [frame.to_frd_m(b.com_mm - expected) for b in dash.bodies]
    )
    assert np.all(np.diag(mp.inertia_frd_kgm2) >= np.diag(point) - 1e-9)
    assert mp.missing == []


def test_weights_overrides_and_missing(dash):
    w = Weights(by_type={"XFLY 80mm EDF:1": 320.0}, overrides={"XFLY 80mm EDF:1": 330.0})
    mp = mass_properties(dash, w, FusionFrame())
    assert mp.total_kg == pytest.approx(0.330)
    assert len(mp.missing) == 36


def test_build_scenario_without_weights(dash, tmp_path):
    frame = FusionFrame()
    template = Scenario.model_validate(
        json.loads(
            (FIXTURES.parent.parent / "scenarios" / "flown_log40_vertical_km.json").read_text()
        )
    )
    edfs = order_edfs_px4(detect_edfs(dash), frame)
    ca = {e.name: e.value for e in read_params_file(PARAMS).entries}
    cg, _ = fit_reference_cg(edfs, ca, frame)
    sc, report = build_scenario(
        dash, frame, template, "t", "2026-09-09T00:00:00", reference_cg_fusion_mm=cg
    )
    assert len(sc.fans) == 10 and sc.mass.estimated
    assert sc.frame.cad_forward_axis == "+Z" and sc.frame.cad_up_axis == "-Y"
    assert len(sc.mass.bodies) == 37 and all(b.volume_m3 > 0 for b in sc.mass.bodies)
    assert [f["cad_tilt_deg"] for f in report["fans"][:8]] == [90.0] * 8
    assert all(f["cad_tilt_deg"] < 1.0 for f in report["fans"][8:])
    Scenario.model_validate(json.loads(json.dumps(sc.model_dump(mode="json"))))


def test_build_scenario_with_weights(dash):
    frame = FusionFrame()
    template = Scenario.model_validate(
        json.loads(
            (FIXTURES.parent.parent / "scenarios" / "flown_log40_vertical_km.json").read_text()
        )
    )
    w = Weights(
        by_type={b.type_name: (320.0 if "EDF" in b.type_name else 150.0) for b in dash.bodies},
        reported_total_g=7250.0,
        reported_cog_mm_fusion=[0, 100, 50],
    )
    sc, report = build_scenario(dash, frame, template, "t", "2026-09-09T00:00:00", weights=w)
    assert sc.mass.total_kg == pytest.approx(10 * 0.320 + 27 * 0.150)
    assert not sc.mass.estimated
    assert sc.mass.cad_reported is not None and sc.mass.cad_reported.mass_kg == pytest.approx(7.25)
    # fan positions are relative to the computed CG
    cg = np.asarray(report["cg_fusion_mm"])
    for f, e in zip(sc.fans, order_edfs_px4(detect_edfs(dash), frame), strict=True):
        assert np.allclose(f.pos_frd_m, frame.to_frd_m(e.body.com_mm - cg), atol=1e-4)
