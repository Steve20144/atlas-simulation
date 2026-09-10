"""Shared pytest configuration.

``FIXTURES`` points at the golden ULog and parameter files under tests/fixtures.
"""

from pathlib import Path

FIXTURES = Path(__file__).parent / "fixtures"
