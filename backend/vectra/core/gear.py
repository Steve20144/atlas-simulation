"""Landing gear: feet that park the airframe at frame.ground_pitch_deg on flat ground, and the
collision boxes that keep everything else off it.

All points are airframe FRD, metres, about the scenario origin (the reference CG); pitch is
nose-up positive in degrees. "Depth" is the distance below the origin along the ground normal,
so the deepest point of the airframe is the one that touches the ground first.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vectra.scenario import Leg, Scenario


@dataclass(frozen=True)
class Box:
    """Axis-aligned box in airframe FRD: centre and full size, metres."""

    centre_frd_m: tuple[float, float, float]
    size_m: tuple[float, float, float]

    def corners(self) -> np.ndarray:
        c = np.asarray(self.centre_frd_m, dtype=float)
        h = np.asarray(self.size_m, dtype=float) / 2.0
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        return c + signs * h


def ground_normal_frd(pitch_deg: float) -> np.ndarray:
    """Unit vector from the airframe into the ground (NED down expressed in airframe FRD) when
    the airframe holds pitch_deg nose-up: (-sin, 0, cos). Nose-down pitch tips it forward."""
    th = np.radians(float(pitch_deg))
    return np.array([-np.sin(th), 0.0, np.cos(th)])


def depth_m(points_frd: np.ndarray, pitch_deg: float) -> np.ndarray:
    """Depth (m) of each point below the origin along the ground normal at pitch_deg."""
    return np.atleast_2d(np.asarray(points_frd, dtype=float)) @ ground_normal_frd(pitch_deg)


def collision_boxes(
    vertices_frd: np.ndarray, cell_m: float = 0.1, min_vertices: int = 3
) -> list[Box]:
    """Column boxes on an xy grid of cell_m: every cell holding at least min_vertices mesh
    vertices becomes one box spanning those vertices in x, y and z. Tight to the skin within a
    cell (the bottom overshoots a sloping surface by at most slope x cell_m)."""
    v = np.atleast_2d(np.asarray(vertices_frd, dtype=float))
    if len(v) == 0:
        return []
    cells = np.floor(v[:, :2] / cell_m).astype(int)
    keys, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    boxes: list[Box] = []
    for k in np.flatnonzero(counts >= min_vertices):
        sel = v[inverse.ravel() == k]
        lo, hi = sel.min(axis=0), sel.max(axis=0)
        size = np.maximum(hi - lo, 0.01)
        boxes.append(Box(tuple(((lo + hi) / 2.0).tolist()), tuple(size.tolist())))
    return boxes


def surface_under(
    vertices_frd: np.ndarray, x_m: float, y_m: float, radius_m: float = 0.04
) -> float:
    """FRD z (m) of the lowest airframe vertex within radius_m of (x, y): where a strut meets
    the skin. Raises ValueError when no vertex is that close."""
    v = np.atleast_2d(np.asarray(vertices_frd, dtype=float))
    sel = v[np.hypot(v[:, 0] - x_m, v[:, 1] - y_m) < radius_m]
    if len(sel) == 0:
        raise ValueError(f"no airframe surface within {radius_m} m of ({x_m}, {y_m})")
    return float(sel[:, 2].max())


def design_gear(
    vertices_frd: np.ndarray,
    ground_pitch_deg: float,
    feet_xy_m: dict[str, tuple[float, float]],
    boxes: list[Box] | None = None,
    clearance_m: float = 0.05,
    strut_radius_m: float = 0.012,
    foot_radius_m: float = 0.025,
) -> list[Leg]:
    """Straight struts down (airframe -z to +z) from the skin above each (x, y) in feet_xy_m to a
    ball foot. Every ball bottom lies clearance_m deeper than the deepest airframe point (mesh
    vertices and collision box corners) at ground_pitch_deg, so the feet alone touch the ground
    and the airframe parks at that pitch."""
    n = ground_normal_frd(ground_pitch_deg)
    pts = [np.atleast_2d(np.asarray(vertices_frd, dtype=float))]
    for b in boxes or []:
        pts.append(b.corners())
    deepest = float(np.max(np.vstack(pts) @ n))
    ball_depth = deepest + clearance_m - foot_radius_m  # depth of the ball centres
    legs: list[Leg] = []
    for name, (x, y) in feet_xy_m.items():
        z_attach = surface_under(vertices_frd, x, y)
        z_foot = (ball_depth - n[0] * x) / n[2]
        if z_foot <= z_attach:
            raise ValueError(
                f"leg {name}: foot would sit inside the airframe; move it or raise clearance"
            )
        legs.append(
            Leg(
                name=name,
                attach_frd_m=(float(x), float(y), z_attach),
                foot_frd_m=(float(x), float(y), float(z_foot)),
                radius_m=strut_radius_m,
                foot_radius_m=foot_radius_m,
            )
        )
    return legs


def rest_height_m(scenario: Scenario) -> float:
    """Height (m) of the CG above flat ground when the airframe parks on its gear at
    frame.ground_pitch_deg: the deepest ball bottom, measured from the CG."""
    frame = scenario.frame
    if frame.ground_pitch_deg is None or not frame.gear:
        raise ValueError("scenario has no landing gear")
    cg = np.asarray(scenario.mass.cg_frd_m, dtype=float)
    feet = np.array([leg.foot_frd_m for leg in frame.gear]) - cg
    radii = np.array([leg.foot_radius_m for leg in frame.gear])
    return float(np.max(depth_m(feet, frame.ground_pitch_deg).ravel() + radii))


def footprint_margin_m(scenario: Scenario) -> float:
    """Distance (m) from the CG's ground projection to the nearest edge of the feet polygon at
    frame.ground_pitch_deg; positive means the parked airframe cannot tip over."""
    frame = scenario.frame
    if frame.ground_pitch_deg is None or len(frame.gear) < 3:
        raise ValueError("footprint needs at least three legs")
    n = ground_normal_frd(frame.ground_pitch_deg)
    t = np.array([n[2], 0.0, -n[0]])  # in-plane fore-aft axis, unit
    cg = np.asarray(scenario.mass.cg_frd_m, dtype=float)
    feet = np.array([leg.foot_frd_m for leg in frame.gear]) - cg
    p = np.column_stack([feet @ t, feet[:, 1]])  # plane coordinates; the CG projects to (0, 0)
    hull = _convex_hull(p)
    margins = []
    for a, b in zip(hull, np.roll(hull, -1, axis=0), strict=True):
        e = b - a
        # signed distance of the origin from edge ab, positive on the polygon's inside (ccw hull)
        margins.append(float(e[0] * (-a[1]) - e[1] * (-a[0])) / float(np.hypot(*e)))
    return min(margins)


def _convex_hull(points: np.ndarray) -> np.ndarray:
    """Counter-clockwise convex hull (monotone chain) of 2D points."""
    pts = sorted({(float(x), float(y)) for x, y in points})
    if len(pts) < 3:
        return np.asarray(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    upper: list[tuple[float, float]] = []
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.asarray(lower[:-1] + upper[:-1])


def leg_meshes(legs: list[Leg]) -> dict[str, object]:
    """Strut cylinders and ball feet as trimesh meshes keyed "gear:<name>" (FRD metres), for
    baking into the scenario GLB. The exporter skips these nodes when it builds collision boxes."""
    import trimesh

    out: dict[str, object] = {}
    for leg in legs:
        a, f = np.asarray(leg.attach_frd_m, dtype=float), np.asarray(leg.foot_frd_m, dtype=float)
        strut = trimesh.creation.cylinder(radius=leg.radius_m, segment=[a, f], sections=16)
        ball = trimesh.creation.icosphere(subdivisions=2, radius=leg.foot_radius_m)
        ball.apply_translation(f)
        out[f"gear:{leg.name}"] = trimesh.util.concatenate([strut, ball])
    return out
