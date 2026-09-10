"""Timestamped output file names (PLAN.md M9: every export is timestamped)."""

from __future__ import annotations

import re
from datetime import datetime
from pathlib import Path

_STAMP_FORMAT = "%Y%m%d_%H%M"
_STAMP_RE = re.compile(r"^\d{8}_\d{4}_")


def timestamp(now: datetime | None = None) -> str:
    """'YYYYMMDD_HHMM' in local time. `now` is injectable for tests."""
    return (now or datetime.now()).strftime(_STAMP_FORMAT)


def timestamped_name(stem: str, ext: str, now: datetime | None = None) -> str:
    """'YYYYMMDD_HHMM_<stem>.<ext>' in local time; `ext` may carry a leading dot."""
    if not stem:
        raise ValueError("stem must not be empty")
    return f"{timestamp(now)}_{stem}.{ext.lstrip('.')}"


def has_timestamp_prefix(name: str | Path) -> bool:
    """True when a file name starts with the 'YYYYMMDD_HHMM_' prefix produced here."""
    return bool(_STAMP_RE.match(Path(name).name))
