"""Request and response models for the foil design sheet endpoints."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict

from tiltlab.scenario import Scenario


class FoilSheetRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    scenario: Scenario


class FoilSheetResponse(BaseModel):
    rows: list[dict[str, Any]]
    markdown: str


class FoilSheetExportResponse(BaseModel):
    csv_path: str
    md_path: str
