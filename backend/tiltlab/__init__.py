"""tiltlab: tilt-fan multicopter analysis and PX4-in-the-loop simulation."""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("tiltlab")
except PackageNotFoundError:  # pragma: no cover - source checkout without install
    __version__ = "0.1.0"

__all__ = ["__version__"]
