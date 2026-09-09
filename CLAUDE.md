# tiltlab conventions
- Read PLAN.md only by section (`grep -n "^### M" PLAN.md` to find offsets). Never cat the whole file.
- One module per session. Finish with `make test` green and one commit: "M<n>: <one line>".
- Python: numpy-vectorised, type hints, docstrings only where units or frames are involved. Every function that takes a vector states its frame (FRD/NED) and unit in the docstring.
- Frames: FRD body, NED world, SI units. Degrees only in UI components.
- Never hard-code CT, mass, inertia or fan positions in code. They come from the Scenario.
- PX4 behaviour: port from the pinned source tree in `third_party/PX4-Autopilot` (git tag v1.17.0, commit d6f12ad1c4f70ad3230afd7d86e971421e02fef4, sparse checkout; recreate with `scripts/fetch_px4.sh`). Cite the file and line in a comment. Do not paraphrase from memory.
- Tests are golden-data tests against tests/fixtures/*.ulg. Do not loosen tolerances to pass. Ground truth extracted from the logs is in tests/fixtures/README.md; take parameters from the log itself (pyulog `initial_parameters`), never from the .params files.
- Frontend: components under 150 lines, state in zustand store, no prop drilling deeper than two levels, no new dependencies without a line in PLAN.md section 3.
- Do not print large arrays, logs or STEP contents into the conversation. Write them to files and summarise in one line.
- Do not explain what you are about to do. Do it, run the test, report pass/fail in one line.
- Commands: `make dev` (backend :8000 + vite :5173), `make test`, `make lint`, `make docker`.

## Where things live in the pinned PX4 v1.17.0 tree (verified paths)
- Effectiveness matrix: `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp` (`computeEffectivenessMatrix`, lines 145 to 225; moment = ct * position.cross(axis) - ct * km * axis, line 199) and `ActuatorEffectivenessMultirotor.{cpp,hpp}`. Note: NOT under src/lib/control_allocation/actuator_effectiveness (that directory only holds the base class).
- Allocation: `src/lib/control_allocation/control_allocation/ControlAllocation.cpp` (clip, normalise, slew), `ControlAllocationPseudoInverse.cpp` (`updateControlAllocationMatrixScale` lines 81 to 148, `allocate` lines 180 to 189), `ControlAllocationSequentialDesaturation.cpp` (desaturation, airmode handling, reads MC_AIRMODE itself).
- Method selection: `src/modules/control_allocator/ControlAllocator.cpp` `update_allocation_method` (line 137). CA_METHOD 2 (AUTO) resolves to SEQUENTIAL_DESATURATION for the multirotor airframe (`ActuatorEffectivenessMultirotor.hpp:47-49`). All golden logs ran CA_METHOD = 2 with CA_AIRFRAME = 0.
- CA_* parameter definitions: `src/modules/control_allocator/module.yaml`.
- Rate PID: `src/lib/rate_control/rate_control.cpp`; rate module `src/modules/mc_rate_control/MulticopterRateControl.cpp`; attitude `src/modules/mc_att_control/mc_att_control_main.cpp` and `AttitudeControl/AttitudeControl.cpp`; position `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`; land detector `src/modules/land_detector`.
- SITL protocol: `src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp`. The `none_iris` target is a custom target in `simulator_mavlink/CMakeLists.txt:62-70` (SYS_AUTOSTART = 10016, airframe `ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris`). `platforms/posix/cmake/sitl_target.cmake` does not exist in v1.17.0.
- HITL: `src/modules/simulation/pwm_out_sim/PWMSim.cpp`, airframe `ROMFS/px4fmu_common/init.d/airframes/1001_rc_quad_x.hil`, board configs `boards/px4/fmu-v6x/*.px4board`. Verified: no fmu-v6x board config enables `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM` (its Kconfig default is n and it is only selected by `COMMON_SIMULATION`, which depends on PLATFORM_POSIX). Stock v1.17.0 v6x firmware therefore cannot run HITL; a custom build with that option enabled is required. The UI checklist and the report must say so.
- uORB messages: `msg/` and `msg/versioned/` (ControlAllocatorStatus.msg in msg/, ActuatorMotors.msg in msg/versioned/; use `find msg -name '<Name>.msg'`). Saturation codes: 0 OK, 2 upper limit, -2 lower limit, 1 / -1 dynamic (slew) variants.

## Environment notes (this development machine, Windows 11)
- Tools are installed in user space and are NOT on PATH in shells spawned by the running Claude Code app. In Git Bash run `source scripts/env.sh` first; in PowerShell `. .\scripts\env.ps1`. Locations: uv and make in `~/.local/bin`, node and npm in `%LOCALAPPDATA%\Programs\nodejs`.
- Python is uv-managed CPython 3.11 (`uv python find 3.11`). Ignore uv's warning "Missing expected target directory for Python minor version link"; the interpreter works. Do not use the system Python 3.14.
- All backend dependencies are declared once in `backend/pyproject.toml`; do not add per-module requirement files. cadquery-ocp ships cp311 win_amd64 wheels. weasyprint (PDF) is optional and may be missing on Windows; the report code must degrade to Markdown only.
- Docker is not installed on this machine. `make docker` must still be correct; its acceptance runs on a Linux host.
- No aircraft STEP file exists on this machine. Only the flight controller STEP (`..\pixhawk6x-pro-3D-simple.stp`, 16 MB) and loose STLs under `..\04 - CAD` are available. CAD tests use a synthetic 10-fan STEP generated with OCP plus the Pixhawk STEP as a real-file smoke test.
- The workflow and agent runs write scratch output under the session scratchpad, never into the repo.
