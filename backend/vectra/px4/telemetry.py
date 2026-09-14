"""Live thrust feed from the Pixhawk over USB: what the stick becomes on the way to the ESCs.

Reads the MAVLink streams PX4 sends on the USB (MAVLINK_MODE_CONFIG) link and keeps the last
samples in memory for the UI to poll:

- RC_CHANNELS (raw stick, us): src/modules/mavlink/streams/RC_CHANNELS.hpp
- MANUAL_CONTROL (throttle after RC calibration, dead zone and mapping; z = (throttle + 1) *
  500, so 0..1000): streams/MANUAL_CONTROL.hpp:75. Not on by default over USB; the feed turns
  it on with `mavlink stream -d /dev/ttyACM0 -s MANUAL_CONTROL -r 10` (mavlink_main.cpp:3447).
- ATTITUDE_TARGET.thrust = |vehicle_attitude_setpoint.thrust_body| in 0..1, the collective the
  attitude controller asks for: streams/ATTITUDE_TARGET.hpp:94.
- SERVO_OUTPUT_RAW instances 0 and 1 = actuator_outputs of the two output drivers (px4io MAIN,
  fmu AUX), the pulse widths in us the ESCs receive: streams/SERVO_OUTPUT_RAW.hpp:71-88.
- HEARTBEAT base_mode for the armed and HIL flags.

The board keeps driving its outputs while the feed runs. It is a measurement, not a lockdown:
PX4 auto-disarms 5 s after a software lockdown outside HITL (Commander.cpp:2314-2329), so the
only safe way to move the stick with the board armed is fans and ESCs unpowered.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from vectra.core.params_px4 import PARAM_TYPE_FLOAT, PARAM_TYPE_INT32
from vectra.px4.board import Link, mode_string, read_params, shell

MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_MODE_FLAG_HIL_ENABLED = 32
USB_MAVLINK_DEV = "/dev/ttyACM0"  # the USB CDC instance on fmu-v6x
STREAMS_HZ: dict[str, float] = {
    "MANUAL_CONTROL": 10.0,
    "RC_CHANNELS": 10.0,
    "ATTITUDE_TARGET": 10.0,
    "SERVO_OUTPUT_RAW_0": 10.0,
    "SERVO_OUTPUT_RAW_1": 10.0,
}
MESSAGE_TYPES = [
    "HEARTBEAT", "RC_CHANNELS", "MANUAL_CONTROL", "ATTITUDE_TARGET", "SERVO_OUTPUT_RAW",
]
# parameters that shape the stick -> thrust curve, read once at start
CURVE_PARAMS: dict[str, int] = {
    "MPC_THR_HOVER": PARAM_TYPE_FLOAT,
    "MPC_MANTHR_MIN": PARAM_TYPE_FLOAT,
    "MPC_THR_MIN": PARAM_TYPE_FLOAT,
    "MPC_THR_MAX": PARAM_TYPE_FLOAT,
    "THR_MDL_FAC": PARAM_TYPE_FLOAT,
    "COM_SPOOLUP_TIME": PARAM_TYPE_FLOAT,
    "MC_AIRMODE": PARAM_TYPE_INT32,
    "RC_MAP_THROTTLE": PARAM_TYPE_INT32,
    "RC3_MIN": PARAM_TYPE_FLOAT,
    "RC3_MAX": PARAM_TYPE_FLOAT,
    "RC3_TRIM": PARAM_TYPE_FLOAT,
    "RC3_DZ": PARAM_TYPE_FLOAT,
}
SAMPLE_PERIOD_S = 0.1
HISTORY = 300  # 30 s at 10 Hz
RECV_TIMEOUT_S = 0.5


@dataclass
class Sample:
    t: float
    rc_raw: int | None
    throttle: float | None  # 0..1 stick after PX4's mapping (MANUAL_CONTROL), or from RC_CHANNELS
    thrust_sp: float | None  # 0..1 collective the attitude controller asks for
    armed: bool
    main_us: list[int] = field(default_factory=list)  # SERVO_OUTPUT_RAW instance 0
    aux_us: list[int] = field(default_factory=list)  # SERVO_OUTPUT_RAW instance 1


class ThrustFeed:
    """Background reader of one board link. start() configures the streams and spawns the
    thread; snapshot() is what the UI polls; stop() ends the thread and closes the link."""

    def __init__(self, m: Link, port: str) -> None:
        self.m = m
        self.port = port
        self.started = time.time()
        self.params: dict[str, float | None] = {}
        self.samples: deque[Sample] = deque(maxlen=HISTORY)
        self.latest: dict[str, Any] = {
            "rc_raw": None, "rc_channels": [], "throttle": None, "throttle_source": None,
            "thrust_sp": None, "armed": False, "hil": False, "mode": None,
            "main_us": [], "aux_us": [],
        }
        self.counts: dict[str, int] = {}
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------ lifecycle
    def start(self) -> None:
        self.params = read_params(self.m, list(CURVE_PARAMS), CURVE_PARAMS)
        shell(
            self.m,
            [f"mavlink stream -d {USB_MAVLINK_DEV} -s {s} -r {r:g}" for s, r in STREAMS_HZ.items()],
        )
        self._thread = threading.Thread(target=self._run, name="thrust-feed", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        try:
            # back to the USB defaults for the streams the feed turned up
            shell(self.m, [f"mavlink stream -d {USB_MAVLINK_DEV} -s {s} -r -1" for s in STREAMS_HZ])
        except Exception:  # the link may already be gone; closing is what matters
            pass
        try:
            self.m.close()
        except Exception:
            pass

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive() and self.error is None

    # ------------------------------------------------------------ reader
    def _run(self) -> None:
        last_sample = 0.0
        try:
            while not self._stop.is_set():
                msg = self.m.recv_match(type=MESSAGE_TYPES, blocking=True, timeout=RECV_TIMEOUT_S)
                if msg is None:
                    time.sleep(0.005)
                    continue
                with self._lock:
                    self._ingest(msg)
                    now = time.time()
                    if now - last_sample >= SAMPLE_PERIOD_S:
                        last_sample = now
                        lt = self.latest
                        self.samples.append(
                            Sample(
                                t=round(now - self.started, 3), rc_raw=lt["rc_raw"],
                                throttle=lt["throttle"], thrust_sp=lt["thrust_sp"],
                                armed=lt["armed"], main_us=list(lt["main_us"]),
                                aux_us=list(lt["aux_us"]),
                            )
                        )
        except Exception as exc:  # link dropped: report, the UI shows it and the user stops
            self.error = f"{type(exc).__name__}: {exc}"

    def _ingest(self, msg: Any) -> None:
        kind = msg.get_type()
        self.counts[kind] = self.counts.get(kind, 0) + 1
        lt = self.latest
        if kind == "HEARTBEAT":
            if getattr(msg, "get_srcComponent", lambda: 1)() != 1:
                return
            lt["armed"] = bool(int(msg.base_mode) & MAV_MODE_FLAG_SAFETY_ARMED)
            lt["hil"] = bool(int(msg.base_mode) & MAV_MODE_FLAG_HIL_ENABLED)
            lt["mode"] = mode_string(msg)
        elif kind == "RC_CHANNELS":
            n = int(getattr(msg, "chancount", 0) or 0)
            chans = [int(getattr(msg, f"chan{i}_raw", 0)) for i in range(1, min(n, 18) + 1)]
            lt["rc_channels"] = chans
            ch = int(self.params.get("RC_MAP_THROTTLE") or 3)
            lt["rc_raw"] = chans[ch - 1] if 0 < ch <= len(chans) else None
            if lt["throttle_source"] != "MANUAL_CONTROL" and lt["rc_raw"]:
                lt["throttle"] = self._throttle_from_rc(lt["rc_raw"])
                lt["throttle_source"] = "RC_CHANNELS"
        elif kind == "MANUAL_CONTROL":
            lt["throttle"] = max(0.0, min(1.0, float(msg.z) / 1000.0))  # MANUAL_CONTROL.hpp:75
            lt["throttle_source"] = "MANUAL_CONTROL"
        elif kind == "ATTITUDE_TARGET":
            lt["thrust_sp"] = float(msg.thrust)
        elif kind == "SERVO_OUTPUT_RAW":
            pw = [int(getattr(msg, f"servo{i}_raw", 0)) for i in range(1, 17)]
            while pw and pw[-1] == 0:
                pw.pop()
            lt["aux_us" if int(getattr(msg, "port", 0)) == 1 else "main_us"] = pw

    def _throttle_from_rc(self, raw: int) -> float:
        """Stick position 0..1 from the pulse width and the RC3 calibration (no dead zone)."""
        lo = float(self.params.get("RC3_MIN") or 1000.0)
        hi = float(self.params.get("RC3_MAX") or 2000.0)
        return max(0.0, min(1.0, (raw - lo) / (hi - lo))) if hi > lo else 0.0

    # ------------------------------------------------------------ view
    def snapshot(self, seconds: float = 30.0) -> dict[str, Any]:
        with self._lock:
            cutoff = (time.time() - self.started) - seconds
            samples = [s.__dict__ for s in self.samples if s.t >= cutoff]
            return {
                "running": self.running,
                "port": self.port,
                "uptime_s": round(time.time() - self.started, 1),
                "error": self.error,
                "latest": dict(self.latest),
                "params": dict(self.params),
                "counts": dict(self.counts),
                "samples": samples,
            }
