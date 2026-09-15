"""Landing gear: feet park the airframe at frame.ground_pitch_deg, the Gazebo export carries it."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from vectra.core.gear import (
    collision_boxes,
    depth_m,
    design_gear,
    footprint_margin_m,
    ground_normal_frd,
    rest_height_m,
)
from vectra.export.gazebo import export_gazebo, gz_rest_pose
from vectra.scenario import Scenario

from .conftest import FIXTURES

SCENARIOS = FIXTURES.parent.parent / "scenarios"


def skin() -> np.ndarray:
    """Synthetic airframe skin, FRD metres: a slab body, two low rear ducts and a nose boom."""
    rng = np.random.default_rng(1)

    def block(x0, x1, y0, y1, z0, z1, n=400):
        p = rng.uniform([x0, y0, z0], [x1, y1, z1], size=(n, 3))
        face = rng.integers(0, 6, size=n)  # push every sample onto one face of the block
        for k, (lo, hi) in enumerate(((x0, x1), (y0, y1), (z0, z1))):
            p[face == 2 * k, k] = lo
            p[face == 2 * k + 1, k] = hi
        return p

    return np.vstack(
        [
            block(-0.4, 0.3, -0.15, 0.15, -0.2, -0.1),  # body
            block(-0.35, -0.2, 0.3, 0.45, 0.05, 0.27),  # right rear duct
            block(-0.35, -0.2, -0.45, -0.3, 0.05, 0.27),  # left rear duct
            block(0.3, 0.8, -0.08, 0.08, -0.15, -0.02),  # nose boom
        ]
    )


FEET = {
    "front_r": (0.5, 0.06),
    "front_l": (0.5, -0.06),
    "rear_r": (-0.3, 0.4),
    "rear_l": (-0.3, -0.4),
}


def test_ground_normal_and_depth():
    n = ground_normal_frd(-20.0)
    assert n == pytest.approx((math.sin(math.radians(20)), 0.0, math.cos(math.radians(20))))
    # nose down: the nose tip is deeper than the tail even though both sit at the same z
    assert depth_m([[0.8, 0, 0]], -20.0)[0] > depth_m([[-0.4, 0, 0]], -20.0)[0]
    assert depth_m([[0, 0, 1]], 0.0)[0] == pytest.approx(1.0)


def test_collision_boxes_cover_the_skin():
    v = skin()
    boxes = collision_boxes(v, cell_m=0.1, min_vertices=1)
    assert 20 <= len(boxes) <= 120
    corners = np.vstack([b.corners() for b in boxes])
    # boxes hug the skin; a flat cell is padded to 1 cm, so its faces may sit 5 mm outside
    assert corners.min(axis=0) == pytest.approx(v.min(axis=0), abs=0.0051)
    assert corners.max(axis=0) == pytest.approx(v.max(axis=0), abs=0.0051)
    # every skin vertex lies inside some box
    inside = np.zeros(len(v), dtype=bool)
    for b in boxes:
        c, h = np.asarray(b.centre_frd_m), np.asarray(b.size_m) / 2 + 1e-9
        inside |= np.all(np.abs(v - c) <= h, axis=1)
    assert inside.all()


def test_design_gear_feet_are_the_only_ground_contact():
    v = skin()
    boxes = collision_boxes(v)
    legs = design_gear(v, -20.0, FEET, boxes, clearance_m=0.05, foot_radius_m=0.025)
    assert [leg.name for leg in legs] == list(FEET)
    feet = np.array([leg.foot_frd_m for leg in legs])
    ball_bottom = depth_m(feet, -20.0).ravel() + 0.025
    assert np.ptp(ball_bottom) < 1e-9  # all four balls on one plane
    corners = np.vstack([b.corners() for b in boxes] + [v])
    assert ball_bottom[0] == pytest.approx(depth_m(corners, -20.0).max() + 0.05)
    for leg in legs:  # struts go straight down from the skin
        assert leg.attach_frd_m[:2] == pytest.approx(leg.foot_frd_m[:2])
        assert leg.foot_frd_m[2] > leg.attach_frd_m[2]
    # the front legs are short (the nose is already low at -20 deg), the rear ones longer
    length = {leg.name: leg.foot_frd_m[2] - leg.attach_frd_m[2] for leg in legs}
    assert 0.03 < length["front_r"] < length["rear_r"] < 0.3
    with pytest.raises(ValueError):
        design_gear(v, -20.0, {"off": (0.0, 0.9)}, boxes)


@pytest.fixture(scope="module")
def geared() -> Scenario:
    sc = Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))
    v = skin()
    legs = design_gear(v, -20.0, FEET, collision_boxes(v))
    frame = sc.frame.model_copy(update={"ground_pitch_deg": -20.0, "gear": legs})
    return sc.model_copy(update={"frame": frame})


def test_rest_height_and_footprint(geared: Scenario):
    h = rest_height_m(geared)
    assert 0.2 < h < 0.4
    assert footprint_margin_m(geared) > 0.1  # the CG projects well inside the four feet
    z0, pitch0 = gz_rest_pose(geared)
    assert z0 == pytest.approx(h + 0.01)
    assert pitch0 == pytest.approx(math.radians(geared.frame.hover_pitch_deg + 20.0))
    with pytest.raises(ValueError):
        rest_height_m(
            geared.model_copy(update={"frame": geared.frame.model_copy(update={"gear": []})})
        )


def test_gazebo_export_parks_on_the_gear(geared: Scenario, tmp_path):
    out = export_gazebo(geared, tmp_path)
    model = ET.parse(out["model_sdf"]).getroot().find("model")
    assert model is not None
    z0, pitch0 = gz_rest_pose(geared)
    pose = [float(v) for v in model.find("pose").text.split()]
    assert pose == pytest.approx([0.0, 0.0, z0, 0.0, pitch0, 0.0], abs=1e-4)
    frame = model.find("frame")
    assert frame is not None and frame.get("name") == "airframe"
    assert float(frame.find("pose").text.split()[4]) == pytest.approx(
        -math.radians(geared.frame.hover_pitch_deg), abs=1e-4
    )
    base = next(link for link in model.findall("link") if link.get("name") == "base_link")
    names = [c.get("name") for c in base.findall("collision")]
    assert sorted(n for n in names if n.startswith("foot_")) == sorted(f"foot_{k}" for k in FEET)
    assert sum(n.startswith("strut_") for n in names) == 4
    assert sum(n.startswith("airframe_box_") for n in names) >= 1
    assert "airframe_collision" not in names
    feet = [c for c in base.findall("collision") if c.get("name").startswith("foot_")]
    assert all(c.find("pose").get("relative_to") == "airframe" for c in feet)
    assert all(c.find(".//sphere/radius") is not None and c.find(".//mu") is not None for c in feet)
    airframe = open(out["airframe"], encoding="utf-8").read()
    assert f"PX4_GZ_MODEL_POSE=${{PX4_GZ_MODEL_POSE:=0,0,{z0:.4f},0,{pitch0:.5f},0}}" in airframe
    world = ET.parse(out["world_sdf"]).getroot()
    assert world.findall(".//include") == []  # PX4 still spawns the vehicle itself
    assert "## Landing gear" in open(out["readme"], encoding="utf-8").read()


def test_export_without_gear_is_unchanged(tmp_path):
    sc = Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))
    out = export_gazebo(sc, tmp_path)
    model = ET.parse(out["model_sdf"]).getroot().find("model")
    assert model.find("pose").text.split() == ["0", "0", "0.3", "0", "0", "0"]
    base = next(link for link in model.findall("link") if link.get("name") == "base_link")
    assert [c.get("name") for c in base.findall("collision")] == ["airframe_collision"]
    assert "PX4_GZ_MODEL_POSE" not in open(out["airframe"], encoding="utf-8").read()
