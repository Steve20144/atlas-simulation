"""Ingest the Fusion 360 "Atlas Mass & CoG" dashboard HTML into tiltlab mass properties and fan
geometry.

The dashboard embeds ``const DATA = {doc, types:[{name, id, instances:[{name, id, com, vol,
meshes:[{v, i}]}]}]}``:
per component instance a centre of mass ``com`` (mm, Fusion assembly frame), a volume ``vol`` (cm3,
exact from
Fusion) and triangle meshes (``v`` flat xyz mm, ``i`` flat triangle indices, already in the
assembly frame).
Component weights are NOT in the file: the dashboard keeps them in browser localStorage and exports
them as JSON
(``weights_by_type`` in grams, ``overrides`` per instance, ``position_offsets_mm``). Pass that JSON
to
:func:`mass_properties`.

Frames. Fusion assembly frame (mm) -> PX4 FRD body frame (m). The mapping is a user-confirmed
choice (:class:`FusionFrame`). The default ``forward="+z", up="-y"`` is the dashboard's own
display frame and matches the aircraft: the inner wing motors sit at the level of the front
fans and each pair outward is one step lower (50 mm down per 87 mm outward, a 30 degree
offset); the nose clamp is at +z. The flown parameter files have the vertical coordinates
upside down relative to this (they were built with up = +y).
Right-handedness is enforced: right = down x forward.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from tiltlab.scenario import (
    NUM_FANS,
    Body,
    CadReported,
    Coanda,
    Fan,
    Foil,
    Mass,
    Scenario,
    axis_to_tilt_azimuth,
)

AXES = {"x": 0, "y": 1, "z": 2}
EDF_NAME_PATTERN = re.compile(r"EDF", re.IGNORECASE)
EDF_DUCT_RADIUS_MM = (
    38.0,
    60.0,
)  # 80 mm rotor, duct with flanges: every vertex within this radius of the axis
EDF_MIN_LENGTH_MM = 40.0


# ---------------------------------------------------------------- parsing


def _extract_data_object(html: str) -> str:
    """Return the JSON text of ``const DATA = {...}`` (brace matched, string aware)."""
    start = html.index("const DATA = ")
    j = html.index("{", start)
    depth = 0
    in_str = False
    esc = False
    k = j
    while True:
        c = html[k]
        if in_str:
            if esc:
                esc = False
            elif c == "\\":
                esc = True
            elif c == '"':
                in_str = False
        else:
            if c == '"':
                in_str = True
            elif c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    return html[j : k + 1]
        k += 1


def mesh_properties(v: Any, i: Any) -> tuple[float, np.ndarray, np.ndarray]:
    """Volume (mm3), centroid (mm) and unit-density inertia tensor about the origin (mm5) of a
    closed
    triangle mesh, by the divergence theorem over signed tetrahedra (origin, a, b, c)."""
    verts = np.asarray(v, dtype=float).reshape(-1, 3)
    tris = np.asarray(i, dtype=int).reshape(-1, 3)
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    vol6 = np.einsum("ij,ij->i", a, np.cross(b - a, c - a))  # 6 * signed tetra volume
    vol = float(vol6.sum() / 6.0)
    if abs(vol) < 1e-9:
        return 0.0, verts.mean(0), np.zeros((3, 3))
    cen = ((a + b + c) * vol6[:, None]).sum(0) / (24.0 * vol)

    def sq(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
        return p * p + q * q + r * r + p * q + p * r + q * r

    def mixed(
        p1: np.ndarray,
        p2: np.ndarray,
        q1: np.ndarray,
        q2: np.ndarray,
        r1: np.ndarray,
        r2: np.ndarray,
    ) -> np.ndarray:
        return (
            2.0 * (p1 * p2 + q1 * q2 + r1 * r2)
            + p1 * q2
            + p2 * q1
            + p1 * r2
            + p2 * r1
            + q1 * r2
            + q2 * r1
        )

    xx = (vol6 * sq(a[:, 0], b[:, 0], c[:, 0])).sum() / 60.0
    yy = (vol6 * sq(a[:, 1], b[:, 1], c[:, 1])).sum() / 60.0
    zz = (vol6 * sq(a[:, 2], b[:, 2], c[:, 2])).sum() / 60.0
    xy = (vol6 * mixed(a[:, 0], a[:, 1], b[:, 0], b[:, 1], c[:, 0], c[:, 1])).sum() / 120.0
    xz = (vol6 * mixed(a[:, 0], a[:, 2], b[:, 0], b[:, 2], c[:, 0], c[:, 2])).sum() / 120.0
    yz = (vol6 * mixed(a[:, 1], a[:, 2], b[:, 1], b[:, 2], c[:, 1], c[:, 2])).sum() / 120.0
    inertia = np.array([[yy + zz, -xy, -xz], [-xy, xx + zz, -yz], [-xz, -yz, xx + yy]])
    if vol < 0:  # inward-facing winding: flip everything back to positive volume
        vol, inertia = -vol, -inertia
    return vol, cen, inertia


@dataclass
class CadBody:
    """One component instance. Fusion assembly frame, mm and mm-based units."""

    name: str
    type_name: str
    type_id: str
    instance_id: str
    com_mm: np.ndarray  # dashboard centre of mass
    vol_cm3: float  # dashboard (exact Fusion) volume
    mesh_vol_mm3: float
    mesh_centroid_mm: np.ndarray
    mesh_inertia_origin_mm5: np.ndarray  # unit density, about the Fusion origin
    vertices_mm: np.ndarray = field(repr=False)
    meshes_mm: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=list, repr=False)

    @property
    def volume_m3(self) -> float:
        return self.vol_cm3 * 1e-6


@dataclass
class Dashboard:
    doc: str
    bodies: list[CadBody]

    def by_name(self, name: str) -> CadBody:
        for b in self.bodies:
            if b.name == name:
                return b
        raise KeyError(name)


def load_dashboard(path: str | Path) -> Dashboard:
    html = Path(path).read_text(encoding="utf-8", errors="replace")
    data = json.loads(_extract_data_object(html))
    bodies: list[CadBody] = []
    for t in data["types"]:
        for inst in t["instances"]:
            vol_tot, first, inertia, chunks = 0.0, np.zeros(3), np.zeros((3, 3)), []
            meshes: list[tuple[np.ndarray, np.ndarray]] = []
            for m in inst.get("meshes", []):
                if not m.get("v") or not m.get("i"):
                    continue
                vol, cen, ine = mesh_properties(m["v"], m["i"])
                vol_tot += vol
                first += vol * cen
                inertia += ine
                verts = np.asarray(m["v"], dtype=float).reshape(-1, 3)
                chunks.append(verts)
                meshes.append((verts, np.asarray(m["i"], dtype=np.int64).reshape(-1, 3)))
            centroid = first / vol_tot if vol_tot > 1e-9 else np.asarray(inst["com"], dtype=float)
            bodies.append(
                CadBody(
                    name=inst["name"],
                    type_name=t["name"],
                    type_id=t["id"],
                    instance_id=inst["id"],
                    com_mm=np.asarray(inst["com"], dtype=float),
                    vol_cm3=float(inst.get("vol", vol_tot / 1000.0)),
                    mesh_vol_mm3=vol_tot,
                    mesh_centroid_mm=centroid,
                    mesh_inertia_origin_mm5=inertia,
                    vertices_mm=np.vstack(chunks) if chunks else np.zeros((0, 3)),
                    meshes_mm=meshes,
                )
            )
    return Dashboard(doc=str(data.get("doc", "")), bodies=bodies)


def volume_centroid_mm(dash: Dashboard) -> np.ndarray:
    """The dashboard's own reference point without weights: the volume-weighted centroid of all
    bodies (Fusion mm). Not the mass CG; use it only until the component weights exist."""
    vol = np.array([b.vol_cm3 for b in dash.bodies], dtype=float)
    com = np.array([b.com_mm for b in dash.bodies], dtype=float)
    return (vol[:, None] * com).sum(0) / vol.sum()


def export_glb(
    dash: Dashboard, frame: FusionFrame, cg_fusion_mm: np.ndarray, path: str | Path
) -> Path:
    """Write every body mesh as one glTF binary, vertices in FRD metres relative to the CG, one node
    per body (named after the CAD instance) so the viewer can show the real airframe."""
    import trimesh

    R = frame.rotation()
    scene = trimesh.Scene()
    for b in dash.bodies:
        for k, (verts, faces) in enumerate(b.meshes_mm):
            v_frd = ((verts - cg_fusion_mm) @ R.T) * 1e-3
            mesh = trimesh.Trimesh(vertices=v_frd, faces=faces, process=False)
            scene.add_geometry(mesh, node_name=f"{b.name}#{k}", geom_name=f"{b.name}#{k}")
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    scene.export(str(out), file_type="glb")
    return out


# ---------------------------------------------------------------- frame


@dataclass(frozen=True)
class FusionFrame:
    """Which Fusion axes are forward and up. Right = down x forward, so the FRD frame is
    right-handed."""

    forward: str = "+z"
    up: str = "-y"

    @staticmethod
    def _unit(spec: str) -> np.ndarray:
        sign = -1.0 if spec.strip().startswith("-") else 1.0
        e = np.zeros(3)
        e[AXES[spec.strip().lstrip("+-").lower()]] = sign
        return e

    def rotation(self) -> np.ndarray:
        """3x3 matrix R with v_frd = R @ v_fusion."""
        fwd = self._unit(self.forward)
        down = -self._unit(self.up)
        if abs(float(fwd @ down)) > 1e-9:
            raise ValueError("forward and up must be different axes")
        right = np.cross(down, fwd)
        return np.vstack([fwd, right, down])

    def to_frd_m(self, v_fusion_mm: np.ndarray) -> np.ndarray:
        """Fusion mm vector(s) -> FRD metres."""
        return (np.asarray(v_fusion_mm, dtype=float) @ self.rotation().T) * 1e-3

    def as_scenario_frame(self) -> dict[str, str]:
        return {
            "cad_forward_axis": self.forward.upper(),
            "cad_up_axis": self.up.upper(),
            "cad_units": "mm",
        }


# ---------------------------------------------------------------- EDF detection


@dataclass
class EdfBody:
    body: CadBody
    axis_fusion: np.ndarray  # unit, sign undetermined from geometry (duct is symmetric)
    duct_radius_mm: float
    length_mm: float


def fit_duct_axis(vertices_mm: np.ndarray) -> tuple[np.ndarray, float, float] | None:
    """Axis of the cylindrical duct: the direction along which every vertex stays within the duct
    radius.
    Candidates are the three principal axes and the three coordinate axes; the one with the smallest
    maximum radial distance wins. Returns (unit axis, max radius mm, extent mm) or None if not
    duct-like."""
    if len(vertices_mm) < 12:
        return None
    c = vertices_mm.mean(0)
    _, vec = np.linalg.eigh(np.cov((vertices_mm - c).T))
    candidates = [vec[:, k] for k in range(3)] + [np.eye(3)[k] for k in range(3)]
    best: tuple[np.ndarray, float, float] | None = None
    for ax in candidates:
        ax = ax / np.linalg.norm(ax)
        along = (vertices_mm - c) @ ax
        radial = np.linalg.norm((vertices_mm - c) - np.outer(along, ax), axis=1)
        rmax, extent = float(radial.max()), float(along.max() - along.min())
        if best is None or rmax < best[1]:
            best = (ax, rmax, extent)
    assert best is not None
    ax, rmax, extent = best
    if not (EDF_DUCT_RADIUS_MM[0] <= rmax <= EDF_DUCT_RADIUS_MM[1]) or extent < EDF_MIN_LENGTH_MM:
        return None
    return ax, rmax, extent


def detect_edfs(dash: Dashboard) -> list[EdfBody]:
    """Bodies named like an EDF whose mesh is a duct (all vertices within 38 to 60 mm of one axis,
    > 40 mm long)."""
    out: list[EdfBody] = []
    for b in dash.bodies:
        if not EDF_NAME_PATTERN.search(b.name):
            continue
        fit = fit_duct_axis(b.vertices_mm)
        if fit is None:
            continue
        ax, rmax, extent = fit
        out.append(EdfBody(body=b, axis_fusion=ax, duct_radius_mm=rmax, length_mm=extent))
    return out


def order_edfs_px4(edfs: list[EdfBody], frame: FusionFrame) -> list[EdfBody]:
    """PX4 rotor numbering used by the flown parameter files: rotors 0..7 are the wing fans in
    pairs from the
    outermost inwards, left (negative Y) before right; rotors 8 and 9 are the centreline fans,
    rearmost first."""
    pos = {id(e): frame.to_frd_m(e.body.com_mm) for e in edfs}
    centre = [e for e in edfs if abs(pos[id(e)][1]) < 0.03]
    wing = [e for e in edfs if abs(pos[id(e)][1]) >= 0.03]
    wing.sort(key=lambda e: (-round(abs(pos[id(e)][1]), 3), pos[id(e)][1]))
    centre.sort(key=lambda e: pos[id(e)][0])
    return wing + centre


# ---------------------------------------------------------------- mass


@dataclass
class Weights:
    """Grams per type (dashboard ``weights_by_type``) and per-instance overrides (``overrides``)."""

    by_type: dict[str, float] = field(default_factory=dict)
    overrides: dict[str, float] = field(default_factory=dict)
    position_offsets_mm: dict[str, list[float]] = field(default_factory=dict)
    reported_total_g: float | None = None
    reported_cog_mm_fusion: list[float] | None = None

    @classmethod
    def from_dashboard_export(cls, path: str | Path) -> Weights:
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(
            by_type={
                k: float(v)
                for k, v in (d.get("weights_by_type") or {}).items()
                if v not in (None, "")
            },
            overrides={
                k: float(v) for k, v in (d.get("overrides") or {}).items() if v not in (None, "")
            },
            position_offsets_mm={
                k: list(map(float, v)) for k, v in (d.get("position_offsets_mm") or {}).items()
            },
            reported_total_g=(float(d["total_g"]) if d.get("total_g") is not None else None),
            reported_cog_mm_fusion=(
                list(map(float, d["cog_mm_fusion_frame"])) if d.get("cog_mm_fusion_frame") else None
            ),
        )

    def mass_g(self, body: CadBody) -> float | None:
        if body.name in self.overrides:
            return self.overrides[body.name]
        return self.by_type.get(body.type_name)


@dataclass
class MassProperties:
    total_kg: float
    cg_frd_m: np.ndarray
    inertia_frd_kgm2: np.ndarray  # about the CG, FRD axes
    cg_fusion_mm: np.ndarray
    bodies: list[Body]
    missing: list[str]  # bodies without a weight (excluded)


def mass_properties(dash: Dashboard, weights: Weights, frame: FusionFrame) -> MassProperties:
    """Mass-weighted CG (dashboard formula: sum m_i * com_i / M) and inertia about the CG in FRD.

    Each body is treated as uniform density: its mesh inertia (unit density, about the Fusion
    origin) is scaled
    by mass / mesh volume, then shifted to the total CG by the parallel axis theorem and rotated to
    FRD."""
    total_g = 0.0
    first = np.zeros(3)
    bodies: list[Body] = []
    missing: list[str] = []
    contributions: list[tuple[float, CadBody]] = []
    for b in dash.bodies:
        m = weights.mass_g(b)
        if m is None:
            missing.append(b.name)
            continue
        off = np.asarray(weights.position_offsets_mm.get(b.name, [0.0, 0.0, 0.0]), dtype=float)
        total_g += m
        first += m * (b.com_mm + off)
        contributions.append((m, b))
        bodies.append(Body(name=b.name, volume_m3=b.volume_m3, mass_kg=m * 1e-3, source="user"))
    if total_g <= 0:
        raise ValueError("no weights supplied")
    cg_mm = first / total_g
    R = frame.rotation()
    inertia = np.zeros((3, 3))
    for m_g, b in contributions:
        m = m_g * 1e-3
        if b.mesh_vol_mm3 > 1e-9:
            # unit-density mesh inertia about origin (mm5) -> kg m2 about origin for this body's
            # density
            i_origin = (
                b.mesh_inertia_origin_mm5 * (m / b.mesh_vol_mm3) * 1e-6
            )  # mm5 * kg/mm3 * (1e-3 m/mm)^2
            # shift from Fusion origin to the body's own centroid (subtract), then to the total CG
            # (add)
            i_body = i_origin - m * _shift(b.mesh_centroid_mm * 1e-3)
        else:
            i_body = np.zeros((3, 3))
        off = np.asarray(weights.position_offsets_mm.get(b.name, [0.0, 0.0, 0.0]), dtype=float)
        r = (b.com_mm + off - cg_mm) * 1e-3
        inertia += i_body + m * _shift(r)
    inertia_frd = R @ inertia @ R.T
    return MassProperties(
        total_kg=total_g * 1e-3,
        cg_frd_m=frame.to_frd_m(cg_mm),
        inertia_frd_kgm2=inertia_frd,
        cg_fusion_mm=cg_mm,
        bodies=bodies,
        missing=missing,
    )


def _shift(r: np.ndarray) -> np.ndarray:
    """Parallel-axis term for unit mass: |r|^2 I - r r^T."""
    r = np.asarray(r, dtype=float)
    return float(r @ r) * np.eye(3) - np.outer(r, r)


def fit_reference_cg(
    edfs_px4: list[EdfBody], ca_params: dict[str, float | int], frame: FusionFrame
) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares Fusion-frame CG that maps the wing fans (rotors 0..7) onto the CA_ROTORn_P* of
    a parameter
    file. Returns (cg_fusion_mm, residuals_m per rotor 0..9 in FRD). Used when no weights are
    available yet."""
    R = frame.rotation()
    p_cad = (
        np.array([R @ e.body.com_mm for e in edfs_px4]) * 1e-3
    )  # FRD m, relative to the Fusion origin
    p_ref = np.array(
        [[ca_params[f"CA_ROTOR{i}_P{a}"] for a in "XYZ"] for i in range(len(edfs_px4))], dtype=float
    )
    cg_frd = (p_cad[:8] - p_ref[:8]).mean(0)
    residuals = p_cad - cg_frd - p_ref
    return R.T @ (cg_frd * 1e3), residuals


# ---------------------------------------------------------------- foils


FOIL_NAME_PATTERN = re.compile(r"FOIL", re.IGNORECASE)
FOIL_CHANNEL_LENGTH_MM = 200.0
FOIL_CHANNEL_RADIUS_MM = 70.0


def foil_pressure_point(
    dash: Dashboard, edf: EdfBody, frame: FusionFrame, cg_fusion_mm: np.ndarray
) -> tuple[np.ndarray, dict[str, Any]] | None:
    """Where the redirected jet of one motor acts: the centroid of the foil-channel vertices in a
    tube of radius FOIL_CHANNEL_RADIUS_MM that starts at the motor's aft duct exit and runs
    FOIL_CHANNEL_LENGTH_MM aft. Returns (FRD metres relative to the CG, report) or None."""
    aft_fusion = -(frame.rotation().T @ np.array([1.0, 0.0, 0.0]))  # FRD -X expressed in Fusion
    c = edf.body.com_mm
    exit_s = float(((edf.body.vertices_mm - c) @ aft_fusion).max())
    best: tuple[int, np.ndarray, str] | None = None
    for body in dash.bodies:
        if not FOIL_NAME_PATTERN.search(body.name) or len(body.vertices_mm) < 100:
            continue
        rel = body.vertices_mm - c
        s = rel @ aft_fusion
        perp = np.linalg.norm(rel - np.outer(s, aft_fusion), axis=1)
        sel = body.vertices_mm[
            (s > exit_s) & (s < exit_s + FOIL_CHANNEL_LENGTH_MM) & (perp < FOIL_CHANNEL_RADIUS_MM)
        ]
        if len(sel) > 50 and (best is None or len(sel) > best[0]):
            best = (len(sel), sel, body.name)
    if best is None:
        return None
    n, sel, name = best
    centroid = sel.mean(0)
    report = {
        "foil_body": name,
        "channel_points": int(n),
        "duct_exit_mm_aft_of_motor": round(exit_s, 1),
        "centroid_fusion_mm": np.round(centroid, 1).tolist(),
        "centroid_mm_aft_of_motor": round(float((centroid - c) @ aft_fusion), 1),
    }
    return frame.to_frd_m(centroid - cg_fusion_mm), report


# ---------------------------------------------------------------- scenario


def build_scenario(
    dash: Dashboard,
    frame: FusionFrame,
    template: Scenario,
    name: str,
    created: str,
    weights: Weights | None = None,
    reference_cg_fusion_mm: np.ndarray | None = None,
    fan_tilt_azimuth: list[tuple[float, float]] | None = None,
    foils: bool = True,
    foil_deflection_deg: float = 45.0,
) -> tuple[Scenario, dict[str, Any]]:
    """Scenario with fan positions from the CAD and mass properties from the weights.

    With ``foils`` (default) the eight wing motors are modelled as they are built: horizontal
    ducts blowing aft (motor axis tilt 90, azimuth 0) into a left and a right foil that turns the
    jet down by ``foil_deflection_deg``. The force on the airframe then acts at the centroid of
    the foil channel behind each motor (measured from the CAD meshes) along the deflected
    direction. The centreline fans stay as the template has them. With ``foils=False`` fan
    orientation comes from ``fan_tilt_azimuth`` or the template. The as-modelled CAD duct axes
    are returned in the report for reference."""
    edfs = order_edfs_px4(detect_edfs(dash), frame)
    if len(edfs) != NUM_FANS:
        raise ValueError(f"expected {NUM_FANS} EDF bodies, found {len(edfs)}")
    report: dict[str, Any] = {
        "doc": dash.doc,
        "frame": frame.as_scenario_frame(),
        "bodies": len(dash.bodies),
    }

    if weights is not None:
        mp = mass_properties(dash, weights, frame)
        cg_fusion = mp.cg_fusion_mm
        mass = Mass(
            total_kg=mp.total_kg,
            cg_frd_m=tuple(np.round(mp.cg_frd_m, 6).tolist()),
            inertia_frd_kgm2=[[float(x) for x in row] for row in np.round(mp.inertia_frd_kgm2, 6)],
            cad_reported=(
                CadReported(
                    mass_kg=weights.reported_total_g * 1e-3,
                    cg_m=tuple(frame.to_frd_m(np.asarray(weights.reported_cog_mm_fusion)).tolist()),
                )
                if weights.reported_total_g and weights.reported_cog_mm_fusion
                else None
            ),
            bodies=mp.bodies,
            estimated=bool(mp.missing),
            notes=(
                "weights from the Atlas Mass & CoG dashboard export"
                + (
                    f"; bodies without weight, excluded: {', '.join(mp.missing)}"
                    if mp.missing
                    else ""
                )
            ),
        )
        report["missing_weights"] = mp.missing
    else:
        if reference_cg_fusion_mm is None:
            raise ValueError("either weights or reference_cg_fusion_mm is required")
        cg_fusion = np.asarray(reference_cg_fusion_mm, dtype=float)
        mass = template.mass.model_copy(
            update={
                "bodies": [
                    Body(name=b.name, volume_m3=b.volume_m3, mass_kg=0.0, source="user")
                    for b in dash.bodies
                ],
                "estimated": True,
                "notes": (
                    "No component weights yet: export them from the Atlas Mass & CoG dashboard "
                    "(Export button) and rerun scripts/import_cad_dashboard.py --weights. "
                    "Total mass is the template placeholder; the CG used for fan positions is "
                    "the reference CG fitted to the flown CA_ROTOR positions, Fusion mm "
                    f"{np.round(cg_fusion, 1).tolist()}."
                ),
            }
        )
    report["cg_fusion_mm"] = np.round(cg_fusion, 2).tolist()

    fans: list[Fan] = []
    cad_axes: list[dict[str, Any]] = []
    for idx, e in enumerate(edfs):
        pos = frame.to_frd_m(e.body.com_mm - cg_fusion)
        axis_frd = frame.rotation() @ e.axis_fusion
        if foils and idx < 8:
            tilt, az = 90.0, 0.0  # horizontal motor, thrust axis forward, exhaust aft into the foil
        elif fan_tilt_azimuth is not None:
            tilt, az = fan_tilt_azimuth[idx]
        else:
            tilt, az = template.fans[idx].tilt_deg, template.fans[idx].azimuth_deg
        tmpl = template.fans[idx]
        fans.append(
            Fan(
                id=idx,
                output=tmpl.output or f"MAIN{idx + 1}",
                pos_frd_m=tuple(np.round(pos, 4).tolist()),
                tilt_deg=tilt,
                azimuth_deg=az,
                spin=tmpl.spin,
                mirror_of=tmpl.mirror_of,
                curve_ref=tmpl.curve_ref,
                km=tmpl.km,
            )
        )
        up_axis = (
            -axis_frd if axis_frd[2] > 0 else axis_frd
        )  # sign is undetermined; report the upward-pointing choice
        t_cad, a_cad = (
            axis_to_tilt_azimuth(up_axis)
            if abs(up_axis[2]) > 1e-6
            else (90.0, float(np.degrees(np.arctan2(up_axis[1], up_axis[0])) % 360.0))
        )
        cad_axes.append(
            {
                "rotor": idx,
                "cad_body": e.body.name,
                "pos_frd_m": np.round(pos, 4).tolist(),
                "cad_axis_frd": np.round(axis_frd, 4).tolist(),
                "cad_tilt_deg": round(float(t_cad), 2),
                "cad_azimuth_deg": round(float(a_cad), 2),
                "duct_radius_mm": round(e.duct_radius_mm, 1),
                "length_mm": round(e.length_mm, 1),
            }
        )
    report["fans"] = cad_axes

    foils_out: list[Foil] = []
    if foils:
        pressure: dict[int, tuple[float, float, float]] = {}
        channel_report: dict[str, Any] = {}
        for idx, e in enumerate(edfs[:8]):
            pp = foil_pressure_point(dash, e, frame, cg_fusion)
            if pp is None:
                raise ValueError(f"no foil channel found behind {e.body.name}")
            pressure[idx] = tuple(np.round(pp[0], 4).tolist())
            channel_report[str(idx)] = pp[1]
        left = [i for i in pressure if fans[i].pos_frd_m[1] < 0]
        right = [i for i in pressure if fans[i].pos_frd_m[1] >= 0]
        note = (
            "Motors are horizontal and blow aft into the foil (motor axis tilt 90, azimuth 0). "
            f"Wrap {foil_deflection_deg:g} deg as built (thrust up and forward), not read from "
            "the CAD. Pressure points are the centroid of the foil channel behind each motor "
            "measured on the CAD meshes. The jet follows the foil by the Coanda effect: surface "
            "radius about 0.25 m estimated from the channel walls in the CAD (they curve down "
            "about 75 mm over the first 160 mm), jet thickness 0.08 m (duct exit); separation "
            "angle and losses are correlations to calibrate on the rig."
        )
        coanda = Coanda(
            radius_m=0.25,
            jet_thickness_m=0.08,
            estimated=True,
            notes="radius from the PHASE_0.1 channel walls; correlation constants are defaults",
        )
        for fid, ids in (("left", left), ("right", right)):
            foils_out.append(
                Foil(
                    id=fid,
                    fan_ids=sorted(ids),
                    deflection_deg=foil_deflection_deg,
                    pressure_points_frd_m={i: pressure[i] for i in ids},
                    loss_at_90deg=0.0,
                    coanda=coanda,
                    estimated=True,
                    notes=note,
                )
            )
        report["foils"] = channel_report

    scenario = template.model_copy(
        update={
            "foils": foils_out,
            "meta": template.meta.model_copy(
                update={
                    "name": name,
                    "created": created,
                    "notes": (
                        f"Fan positions and mass properties from {dash.doc} "
                        f"(Atlas Mass & CoG dashboard). {template.meta.notes}"
                    ).strip(),
                }
            ),
            "frame": template.frame.model_copy(
                update={
                    **frame.as_scenario_frame(),
                    "cad_origin": tuple(np.round(cg_fusion, 3).tolist()),
                }
            ),
            "mass": mass,
            "fans": fans,
        }
    )
    return Scenario.model_validate(scenario.model_dump()), report
