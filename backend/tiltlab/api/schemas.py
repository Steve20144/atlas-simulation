"""Pydantic request and response models of the tiltlab REST API.

Physical units in the metrics payload are stated in the keys of core/metrics.py (N, N m, W);
``collective`` is the fraction of the vertical thrust available at u = 1 (dimensionless).
"""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from tiltlab.scenario import Scenario

Concept = Literal["stock", "fully_actuated"]


class MetricsRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    scenario: Scenario
    concept: Concept | None = Field(
        default=None, description="defaults to scenario.control.concept"
    )
    blend: float | None = Field(default=None, ge=0.0, le=1.0)
    collective: float | None = Field(default=None, ge=0.0, le=1.0)
    weights: dict[str, float] | None = None


class MetricsResponse(BaseModel):
    """Output of core.metrics.compute_metrics plus the echoed request context."""

    model_config = ConfigDict(extra="forbid")
    scenario_name: str
    concept: Concept
    blend: float
    controlled_axes: list[str]
    collective: float
    collective_hover: float
    Fz_max_N: float
    Fz_target_N: float
    px4_thrust_sp_z: float
    effectiveness: dict[str, Any]
    hover: dict[str, Any]
    authority: dict[str, dict[str, Any]]
    marginal_power: dict[str, dict[str, Any]]
    coupling: dict[str, Any]
    conditioning: dict[str, Any]
    control: dict[str, Any]
    score: dict[str, Any]
    badges: dict[str, str]
    estimated: bool
    estimated_sources: list[str]
    notes: list[str]
    compute_ms: float


class Px4ParamsPreviewRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    scenario: Scenario
    concept: Concept | None = None


class Px4ParamsPreviewResponse(BaseModel):
    model_config = ConfigDict(extra="forbid")
    concept: Concept
    params: dict[str, float | int]
    extras: dict[str, float | int]
    lines: list[str]


class ScenarioListResponse(BaseModel):
    model_config = ConfigDict(extra="forbid")
    scenarios: list[str]


class ScenarioSaveResponse(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str
    path: str


class ExportParamsRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    scenario: Scenario
    concept: Concept | None = None
    base: str | None = Field(
        default=None, description="backup .params file to merge onto; default latest fixture"
    )


class ExportCsvRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    rows: list[dict[str, Any]] = Field(min_length=1)
    stem: str = "metrics"


class ExportResponse(BaseModel):
    model_config = ConfigDict(extra="forbid")
    path: str


class GazeboLaunchRequest(BaseModel):
    """Which harness to export and start: gz sim SITL (Ubuntu-24.04) or Classic HITL
    (Ubuntu-22.04)."""

    model_config = ConfigDict(extra="forbid")
    scenario: Scenario
    mode: Literal["sitl", "hitl"]


class BoardPushRequest(BaseModel):
    """Push the scenario CA_* set (plus concept extras) to the Pixhawk over USB."""

    model_config = ConfigDict(extra="forbid")
    scenario: Scenario
    concept: Concept | None = None
    port: str = Field(default="auto", description="COMn, /dev/ttyACMn, or auto to pick a Pixhawk")
    backup: bool = Field(default=True, description="write the board current values first")


class BoardParamRequest(BaseModel):
    """Write one parameter (for example SYS_HITL 0/1) with the type the board reports for it."""

    model_config = ConfigDict(extra="forbid")
    name: str = Field(pattern=r"^[A-Z][A-Z0-9_]{1,15}$")
    value: float
    port: str = "auto"


class BoardPortRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    port: str = "auto"
