"""Request and response models for POST /api/sweep."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field

from tiltlab.scenario import Scenario


class SweepRequest(BaseModel):
    scenario: Scenario
    concept: Literal["stock", "fully_actuated"] = "stock"
    collective: float | None = Field(default=None, ge=0.0, le=1.0)
    # Grid values: foil deflections (0..180) when variable resolves to "foil", else fan tilts.
    tilts_deg: list[float] = Field(default_factory=lambda: [float(t) for t in range(0, 181, 15)])
    variable: Literal["auto", "foil", "tilt"] = "auto"
    foil_grouping: Literal["same", "left_right", "per_pair"] = "same"
    azimuth_mode: Literal[
        "inward",
        "outward",
        "forward",
        "aft",
        "alternating",
        "outer_fwd_inner_aft",
        "outer_aft_inner_fwd",
    ] = "forward"
    per_pair: bool = False
    centreline_tilts_deg: list[float] = Field(default_factory=lambda: [0.0])
    centreline_azimuth_deg: float = Field(default=0.0, ge=0.0, le=360.0)
    min_headroom: float = Field(default=0.2, ge=0.0, le=1.0)
    min_yaw_Nm: float = Field(default=0.0, ge=0.0)
    rank_by: Literal["power", "yaw", "yaw_per_kW", "headroom", "score"] = "power"
    top: int = Field(default=25, ge=1, le=5000)


class SweepResponse(BaseModel):
    variable: str
    n_evaluated: int
    n_feasible: int
    truncated: bool
    elapsed_ms: float
    objective: str
    spec: dict[str, Any]
    candidates: list[dict[str, Any]]
    best: dict[str, Any] | None
