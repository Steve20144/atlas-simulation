"""Return a Pixhawk from HITL to its flight configuration.

Why ``SYS_HITL 0`` alone does not stick: the HITL parameter set (export/gazebo_classic_hitl
.hitl_params) puts SYS_AUTOSTART on PX4's HIL airframe 1001, and that airframe script runs
``param set SYS_HITL 1`` on every boot (ROMFS/px4fmu_common/init.d/airframes/1001_rc_quad_x.hil
line 14, a hard set, not set-default). rcS then starts the sensors in HIL mode and writes
``param set GPS_1_CONFIG 0`` (rcS:324-329) and starts pwm_out_sim (rcS:466-470). Leaving HITL
therefore means restoring SYS_AUTOSTART and every other parameter the HITL set changed, from the
flight parameter backup, in one write, and rebooting.

Values come from the backup .params file where it has them (the same "latest backup" the .params
export merges onto, or a file the caller names). Fallbacks are PX4's own defaults where the pinned
tree states them (EKF2_EN 1: src/modules/ekf2/module.yaml:5-9; GPS_1_CONFIG GPS1 = 201:
src/drivers/gps/module.yaml:12-14) or the value that undoes the HITL setting (CBRK_SUPPLY_CHK 0:
anything but the 894281 key re-enables the check, 1001_rc_quad_x.hil:20; SYS_HAS_MAG/BARO 1;
HIL_ACT_FUNCn 0). Sensor calibration (CAL_ACC0_*, CAL_GYRO0_*, the slot ids) can only come from
the backup; without it the sim device id is cleared and the caller is told to recalibrate.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from vectra.core.params_px4 import PARAM_TYPE_FLOAT, PARAM_TYPE_INT32, ParamFile
from vectra.scenario import NUM_FANS, Scenario

# SYS_AUTOSTART flown in every golden log (tests/fixtures/README.md): PX4 "Generic Quadcopter X".
# Only used when the backup has no SYS_AUTOSTART line; the caller is warned.
FALLBACK_AUTOSTART = 4001

# parameter -> flight value when the backup lacks it (None: skip unless the backup has it)
FLIGHT_FALLBACKS: dict[str, int | None] = {
    "SYS_HITL": 0,
    "SYS_AUTOSTART": FALLBACK_AUTOSTART,
    "EKF2_EN": 1,
    "SYS_HAS_MAG": 1,
    "SYS_HAS_BARO": 1,
    "CBRK_SUPPLY_CHK": 0,
    "GPS_1_CONFIG": 201,
    "UAVCAN_ENABLE": None,
    "SDLOG_MODE": None,
    "SDLOG_BACKEND": None,
}
# IMU calibration the HITL set overwrote with the sim device and identity values
CAL_ID_PARAMS: tuple[str, ...] = ("CAL_ACC0_ID", "CAL_GYRO0_ID", "CAL_ACC1_ID", "CAL_GYRO1_ID")
CAL_FLOAT_PARAMS: tuple[str, ...] = tuple(
    [f"CAL_ACC0_{a}OFF" for a in "XYZ"]
    + [f"CAL_ACC0_{a}SCALE" for a in "XYZ"]
    + [f"CAL_GYRO0_{a}OFF" for a in "XYZ"]
)


@dataclass
class FlightSet:
    """What to write to leave HITL: values, PX4 types, where each value came from, warnings."""

    params: dict[str, int | float] = field(default_factory=dict)
    types: dict[str, int] = field(default_factory=dict)
    sources: dict[str, str] = field(default_factory=dict)  # "backup" | "default" | "hitl_undo"
    warnings: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "params": dict(self.params),
            "sources": dict(self.sources),
            "warnings": list(self.warnings),
        }


def _backup_lookup(backup: ParamFile | None) -> dict[str, tuple[int | float, int]]:
    if backup is None:
        return {}
    return {e.name: (e.value, e.type_code) for e in backup.entries}


def flight_params(
    backup: ParamFile | None,
    scenario: Scenario | None = None,
    rotor_count: int = NUM_FANS,
) -> FlightSet:
    """The parameter set that takes the board from HITL back to flight.

    backup: the flight parameter backup (values win over every fallback). scenario: when given,
    the controller gains the HITL set sized for the simulator (export.gazebo.px4_tuning) are
    restored from the backup as well, so the vehicle flies with the gains it flew before.
    SYS_HITL is always 0.
    """
    have = _backup_lookup(backup)
    out = FlightSet()

    def put(name: str, value: int | float, ptype: int, source: str) -> None:
        out.params[name] = value
        out.types[name] = ptype
        out.sources[name] = source

    for name, fallback in FLIGHT_FALLBACKS.items():
        if name == "SYS_HITL":
            put(name, 0, PARAM_TYPE_INT32, "hitl_undo")
        elif name in have:
            v, t = have[name]
            put(name, int(v) if t == PARAM_TYPE_INT32 else float(v), t, "backup")
        elif fallback is not None:
            put(name, fallback, PARAM_TYPE_INT32, "default")
            if name == "SYS_AUTOSTART":
                out.warnings.append(
                    f"backup has no SYS_AUTOSTART; wrote {fallback} (the airframe flown in the "
                    "golden logs). Check it is your vehicle's airframe before flying."
                )
    for i in range(rotor_count):
        put(f"HIL_ACT_FUNC{i + 1}", 0, PARAM_TYPE_INT32, "hitl_undo")

    missing_cal = [n for n in CAL_ID_PARAMS + CAL_FLOAT_PARAMS if n not in have]
    for name in CAL_ID_PARAMS:
        if name in have:
            put(name, int(have[name][0]), PARAM_TYPE_INT32, "backup")
        else:
            put(name, 0, PARAM_TYPE_INT32, "hitl_undo")
    for name in CAL_FLOAT_PARAMS:
        if name in have:
            put(name, float(have[name][0]), PARAM_TYPE_FLOAT, "backup")
    if missing_cal:
        out.warnings.append(
            "backup lacks the IMU calibration the HITL set overwrote "
            f"({', '.join(missing_cal[:4])}{', ...' if len(missing_cal) > 4 else ''}); the sim "
            "device id was cleared, so recalibrate accelerometer and gyro in QGroundControl "
            "before flying."
        )

    if scenario is not None:
        from vectra.export.gazebo import px4_tuning

        tuning = px4_tuning(scenario)
        kept = [n for n in tuning if n not in have]
        for name in tuning:
            if name in have:
                v, t = have[name]
                put(name, int(v) if t == PARAM_TYPE_INT32 else float(v), t, "backup")
        if kept:
            out.warnings.append(
                "controller gains the HITL set wrote stay on the board because the backup has no "
                f"value for them: {', '.join(kept)}."
            )
    else:
        out.warnings.append(
            "no scenario given, so the controller gains written for HITL (THR_MDL_FAC and the "
            "MC_* rate gains) were left as they are."
        )
    return out
