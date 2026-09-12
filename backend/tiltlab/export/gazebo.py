"""Gazebo (gz sim) harness generated from a Scenario.

Writes a model (SDF 1.9) with the airframe body and one rotor link per fan placed at the
effective force point and pointed along the effective thrust direction (foil-aware), each driven
by the gz MulticopterMotorModel plugin with the fan's effective thrust constant, plus the IMU,
barometer, magnetometer and GPS sensors PX4's gz bridge reads; a world with the matching sensor
systems (PX4 spawns the model into it itself, so the world does not include it); a PX4 posix
airframe file
carrying the same CA_ROTOR* geometry so the allocator in SITL matches the exported params; and a
README with the launch steps. Frames: FRD (x forward, y right, z down) becomes gz body FLU
(x forward, y left, z up): (x, y, z) -> (x, -y, -z).

Generated, not run here (Gazebo needs a Linux host); see the README for the checklist.
"""

from __future__ import annotations

import math
import zlib
from pathlib import Path
from typing import Any
from xml.sax.saxutils import escape

import numpy as np

from tiltlab.core.metrics import compute_metrics, fan_lag_s, inertia_matrix
from tiltlab.export.params import ca_geometry_params
from tiltlab.scenario import Scenario

MAX_ROT_VELOCITY = 1000.0  # rad/s at full command; motorConstant maps it onto the fan CT
ROTOR_MASS_KG = 0.32  # XFly 80 mm EDF unit
# Must not share its numeric prefix with any stock posix airframe: rcS sources every file matching
# `<SYS_AUTOSTART>_*` and keeps the last; v1.17 ships 4010_gz_x500_mono_cam (4001..4021 are used).
AIRFRAME_ID_BASE = 4500  # tiltlab ids are 4500..4899; PX4 v1.17 uses none of them


def airframe_id(name: str) -> int:
    """Autostart id for a scenario name: PX4's rcS matches `<id>_*` and sources the last hit, so two
    tiltlab exports installed side by side must not share an id."""
    return AIRFRAME_ID_BASE + zlib.crc32(name.encode("utf-8")) % 400


def frd_to_flu(v: tuple[float, float, float] | np.ndarray) -> tuple[float, float, float]:
    x, y, z = (float(c) for c in v)
    return (x, -y, -z)


def axis_to_rpy(axis_flu: tuple[float, float, float]) -> tuple[float, float, float]:
    """Roll, pitch, yaw (rad) that rotate the link +z onto axis_flu (zero yaw).

    SDF applies a pose's rpy as R = Rz(yaw) Ry(pitch) Rx(roll) (roll first, fixed axes), so
    the rotated +z is (sin p cos r, -sin r, cos p cos r): pitch = atan2(x, z) and
    roll = atan2(-y, hypot(x, z)). Exact for any axis, including a fan tilted sideways on a
    pitched hover frame where x and y are both non-zero.
    """
    x, y, z = axis_flu
    pitch = math.atan2(x, z) if (x or z) else 0.0
    roll = math.atan2(-y, math.hypot(x, z))
    return (roll, pitch, 0.0)


def _noise(stddev: float) -> str:
    """Zero-mean gaussian sensor noise element in the sensor's own SI unit."""
    return f'<noise type="gaussian"><mean>0.0</mean><stddev>{stddev:g}</stddev></noise>'


def airframe_stl(src_glb: Path, dst_stl: Path) -> bool:
    """Convert the scenario GLB (FRD metres about the CG) to a binary STL in gz body FLU.

    The GLB keeps raw FRD vertices with no root transform. Gazebo's glTF loader draws those as-is
    in its Z-up body frame, which shows the airframe upside down and mirrored (the wing dihedral
    appears to rise outward). STL has no up-axis convention, so FRD (x, y, z) -> FLU (x, -y, -z)
    is applied to the vertices explicitly. Returns False when the conversion is not possible.
    """
    try:
        import trimesh

        scene = trimesh.load(str(src_glb), force="scene")
        merged = trimesh.util.concatenate(list(scene.dump()))
        merged.apply_transform(np.diag([1.0, -1.0, -1.0, 1.0]))
        dst_stl.parent.mkdir(parents=True, exist_ok=True)
        merged.export(str(dst_stl))
        return True
    except Exception:  # noqa: BLE001 - the mesh is cosmetic; the model must still export
        dst_stl.unlink(missing_ok=True)
        return False


def body_mass_kg(scenario: Scenario, rotor_mass_kg: float = ROTOR_MASS_KG) -> float:
    """Mass left for the body link once each rotor link carries rotor_mass_kg, so the whole
    model weighs the scenario's total (otherwise hover needs about 27 percent more thrust than
    tiltlab predicts)."""
    return max(0.5, scenario.mass.total_kg - rotor_mass_kg * len(scenario.fans))


def _inertia(scenario: Scenario) -> dict[str, float]:
    ix, _placeholder = inertia_matrix(scenario)  # FRD, box placeholder when the scenario has none
    # FRD -> FLU flips the sign of the xy and xz products
    return {
        "ixx": float(ix[0, 0]),
        "iyy": float(ix[1, 1]),
        "izz": float(ix[2, 2]),
        "ixy": -float(ix[0, 1]),
        "ixz": -float(ix[0, 2]),
        "iyz": float(ix[1, 2]),
    }


def model_sdf(scenario: Scenario, name: str, mesh_uri: str | None) -> str:
    inertia = _inertia(scenario)
    # the CAD mesh is in the airframe frame; base_link is the hover frame (a nose-up hover is a
    # negative rotation about FLU +Y)
    mesh_pitch = -math.radians(float(scenario.frame.hover_pitch_deg))
    ca = ca_geometry_params(scenario)
    body_visual = (
        f'<visual name="airframe_visual"><pose>0 0 0 0 {mesh_pitch:.5f} 0</pose>'
        f"<geometry><mesh><uri>{escape(mesh_uri)}</uri></mesh>"
        "</geometry><material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse>"
        "</material></visual>"
        if mesh_uri
        else '<visual name="airframe_visual"><geometry><box><size>1.2 0.9 0.3</size></box>'
        "</geometry></visual>"
    )
    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<sdf version="1.9">',
        f'  <model name="{escape(name)}">',
        "    <pose>0 0 0.3 0 0 0</pose>",
        '    <link name="base_link">',
        "      <inertial>",
        # the rotor links carry ROTOR_MASS_KG each; keep the model's total at the scenario mass
        f"        <mass>{body_mass_kg(scenario):.4f}</mass>",
        "        <inertia>"
        + "".join(f"<{k}>{v:.6f}</{k}>" for k, v in inertia.items())
        + "</inertia>",
        "      </inertial>",
        f"      {body_visual}",
        '      <collision name="airframe_collision"><geometry><box><size>1.2 0.9 0.3</size></box>'
        "</geometry></collision>",
        # Sensor names, rates and noise mirror PX4-gazebo-models x500_base/model.sdf; the gz bridge
        # subscribes to .../link/base_link/sensor/<name>/... (GZBridge.cpp v1.17.0, lines 218-310)
        # and PX4's default preflight needs a GPS (navsat) before it will arm.
        '      <sensor name="imu_sensor" type="imu"><gz_frame_id>base_link</gz_frame_id>'
        "<always_on>1</always_on><update_rate>250</update_rate><imu><angular_velocity>"
        + "".join(f"<{a}>{_noise(0.0008726646)}</{a}>" for a in "xyz")
        + "</angular_velocity><linear_acceleration>"
        + "".join(
            f"<{a}>{_noise(s)}</{a}>"
            for a, s in zip("xyz", (0.00637, 0.00637, 0.00686), strict=True)
        )
        + "</linear_acceleration></imu></sensor>",
        '      <sensor name="air_pressure_sensor" type="air_pressure">'
        "<gz_frame_id>base_link</gz_frame_id><always_on>1</always_on><update_rate>50</update_rate>"
        f"<air_pressure><pressure>{_noise(3.0)}</pressure></air_pressure></sensor>",
        '      <sensor name="magnetometer_sensor" type="magnetometer">'
        "<gz_frame_id>base_link</gz_frame_id><always_on>1</always_on><update_rate>100</update_rate>"
        "<magnetometer>"
        + "".join(f"<{a}>{_noise(0.0001)}</{a}>" for a in "xyz")
        + "</magnetometer></sensor>",
        '      <sensor name="navsat_sensor" type="navsat"><gz_frame_id>base_link</gz_frame_id>'
        "<always_on>1</always_on><update_rate>30</update_rate></sensor>",
        "    </link>",
    ]
    for fan in scenario.fans_sorted():
        i = fan.id
        pos = frd_to_flu(scenario.hover_pos(fan))  # base_link is the hover frame
        axis = frd_to_flu(scenario.hover_axis(fan))
        r, p, y = axis_to_rpy(axis)
        # first-order spool lag of this fan's curve (the scenario's estimate, not x500's 12 ms)
        lag = max(0.01, scenario.fan_curves[fan.curve_ref].lag_s) if fan.curve_ref else 0.01
        ct = float(ca[f"CA_ROTOR{i}_CT"])
        km = float(ca[f"CA_ROTOR{i}_KM"])
        turning = "ccw" if fan.spin == "CCW" else "cw"
        parts += [
            f'    <link name="rotor_{i}">',
            f"      <pose>{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} {r:.5f} {p:.5f} {y:.5f}</pose>",
            f"      <inertial><mass>{ROTOR_MASS_KG}</mass><inertia><ixx>0.0003</ixx>"
            "<iyy>0.0003</iyy><izz>0.0005</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
            "</inertia></inertial>",
            f'      <visual name="rotor_{i}_visual"><geometry><cylinder><radius>0.042</radius>'
            "<length>0.02</length></cylinder></geometry><material><ambient>0.2 0.6 0.9 1</ambient>"
            "<diffuse>0.2 0.6 0.9 1</diffuse></material></visual>",
            "    </link>",
            f'    <joint name="rotor_{i}_joint" type="revolute">',
            f"      <parent>base_link</parent><child>rotor_{i}</child>",
            "      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit>"
            "<dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness>"
            "</dynamics></axis>",
            "    </joint>",
            '    <plugin filename="gz-sim-multicopter-motor-model-system" '
            'name="gz::sim::systems::MulticopterMotorModel">',
            f"      <jointName>rotor_{i}_joint</jointName><linkName>rotor_{i}</linkName>",
            f"      <turningDirection>{turning}</turningDirection>",
            f"      <timeConstantUp>{lag:.3f}</timeConstantUp>"
            f"<timeConstantDown>{lag:.3f}</timeConstantDown>",
            f"      <maxRotVelocity>{MAX_ROT_VELOCITY:.0f}</maxRotVelocity>",
            f"      <motorConstant>{ct / MAX_ROT_VELOCITY**2:.10e}</motorConstant>",
            f"      <momentConstant>{abs(km):.6f}</momentConstant>",
            "      <commandSubTopic>command/motor_speed</commandSubTopic>",
            f"      <motorNumber>{i}</motorNumber><actuator_number>{i}</actuator_number>",
            "      <rotorDragCoefficient>0</rotorDragCoefficient>"
            "<rollingMomentCoefficient>0</rollingMomentCoefficient>",
            f"      <motorSpeedPubTopic>motor_speed/{i}</motorSpeedPubTopic>"
            "<rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>",
            "    </plugin>",
        ]
    parts += [
        '    <plugin filename="gz-sim-odometry-publisher-system" '
        'name="gz::sim::systems::OdometryPublisher"><dimensions>3</dimensions>'
        "<odom_publish_frequency>100</odom_publish_frequency></plugin>",
        "  </model>",
        "</sdf>",
        "",
    ]
    return "\n".join(parts)


def world_sdf(name: str) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{escape(name)}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      </physics>
    <gravity>0 0 -9.80665</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
      </plugin>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971</latitude_deg>
        <longitude_deg>8.546164</longitude_deg>
        <elevation>488</elevation>
        </spherical_coordinates>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <direction>-0.5 0.1 -0.9</direction>
        </light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision">
        <geometry>
        <plane>
        <normal>0 0 1</normal>
        <size>200 200</size>
        </plane>
        </geometry>
        </collision>
      <visual name="visual">
        <geometry>
        <plane>
        <normal>0 0 1</normal>
        <size>200 200</size>
        </plane>
        </geometry>
        <material>
          <ambient>0.6 0.6 0.6 1</ambient>
          <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
          </visual>
          </link>
          </model>
    <!-- The vehicle is not included here: PX4's px4-rc.gzsim spawns model://{escape(name)} itself
         as <name>_0 and attaches the gz bridge to that instance. -->
  </world>
</sdf>
"""


def geometry_summary(scenario: Scenario) -> list[str]:
    """One line per fan: tilt/azimuth, foil deflection asked and effective, jet attachment and the
    resulting FRD thrust axis. Lets a harness be told apart from another export of the same
    scenario (a sweep result applied in the UI writes to the same directory name)."""
    out = []
    for fan in scenario.fans_sorted():
        ax = scenario.effective_axis(fan)
        foil = scenario.foil_for(fan.id)
        if foil is None:
            foil_txt = "no foil"
        else:
            eff = scenario.fan_effective_deflection(fan)
            state = "attached" if scenario.fan_jet_attached(fan) else "SEPARATED"
            foil_txt = (
                f"foil {foil.deflection_for(fan.id):.0f} deg (effective {eff:.0f} deg, {state})"
            )
        out.append(
            f"rotor {fan.id}: tilt {fan.tilt_deg:.0f} az {fan.azimuth_deg:.0f}, {foil_txt}, "
            f"thrust axis FRD ({ax[0]:+.3f}, {ax[1]:+.3f}, {ax[2]:+.3f})"
        )
    return out


RATE_CROSSOVER_MAX_RAD_S = 4.0  # rate-loop crossover asked for when the fans are fast enough
LAG_PERIODS_PER_CROSSOVER = (
    3.5  # w_c = 1 / (3.5 x fan lag): 2.5 rang the weakest axis at saturation
)


def px4_tuning(scenario: Scenario) -> dict[str, float]:
    """PX4 controller parameters sized for this airframe from tiltlab's own numbers.

    PX4's default MC_* gains fit a small quad whose motors give about 130 rad/s^2 of angular
    acceleration per unit of normalised torque with a 10 to 25 ms spool. A 12 kg EDF airframe
    gives 7 to 20 rad/s^2 per unit with a 150 ms spool, so with the defaults the rate loop is
    several times slower than the attitude loop above it and the cascade oscillates and flips
    (seen in gz: pitch swinging to +50 then -80 deg within two seconds of lift-off).

    Rule used here, per axis: crossover w_c = min(4 rad/s, 1 / (3.5 * fan lag)), rate P =
    w_c / (torque authority / inertia), I = 0.6 P (yaw 0.5 P), D = 0.05 P (yaw 0), integrator
    limit 0.15, attitude P = w_c / 2.5, autotune off. Authority is tiltlab's attainable torque
    at hover (N m), inertia the scenario's (or the box placeholder). THR_MDL_FAC 1 with a zero
    idle command makes the
    gz thrust (motorConstant * omega^2) linear in PX4's command. MPC_THR_HOVER is the hover
    collective. Axes tiltlab marks unattainable keep the PX4 defaults.
    """
    m = compute_metrics(scenario)
    inertia = _inertia(scenario)
    lag = fan_lag_s(scenario)
    w_c = min(RATE_CROSSOVER_MAX_RAD_S, 1.0 / (LAG_PERIODS_PER_CROSSOVER * lag))
    # autotune is on by default in PX4 and QGC can trigger it; on this airframe it produced gains
    # three to four times these and every axis rang (gz sim 2026-09-11), so it stays off
    out: dict[str, float] = {"THR_MDL_FAC": 1.0, "MC_AT_EN": 0.0, "MC_AT_APPLY": 0.0}
    hover = m.get("hover", {})
    if hover.get("exact") and hover.get("u"):
        out["MPC_THR_HOVER"] = round(min(0.8, max(0.2, float(np.mean(hover["u"])))), 3)
    for axis, key, tag in (
        ("roll", "ixx", "ROLL"),
        ("pitch", "iyy", "PITCH"),
        ("yaw", "izz", "YAW"),
    ):
        a = m.get("authority", {}).get(axis)
        if not a or not (a.get("plus_attainable") and a.get("minus_attainable")):
            continue
        tau = min(float(a["plus"]), float(a["minus"]))
        if tau <= 0.0 or inertia[key] <= 0.0:
            continue
        acc_per_unit = tau / inertia[key]  # rad/s^2 per unit normalised torque
        p = min(0.6, max(0.02, w_c / acc_per_unit))
        out[f"MC_{tag}RATE_P"] = round(p, 4)
        out[f"MC_{tag}RATE_I"] = round((0.5 if axis == "yaw" else 0.6) * p, 4)
        out[f"MC_{tag}RATE_D"] = 0.0 if axis == "yaw" else round(0.05 * p, 5)
        out[f"MC_{tag[0]}R_INT_LIM"] = 0.15  # windup at saturation sustained the pitch limit cycle
        out[f"MC_{tag}RATE_K"] = 1.0
        out[f"MC_{tag}_P"] = round(min(6.5, max(0.5, w_c / 2.5)), 3)
    # Outer loops must be slower than the attitude loop they command. With PX4's quad defaults
    # (MPC_XY_VEL_P_ACC 1.8, MPC_XY_P 0.95) over a 1 rad/s attitude loop the position controller
    # pinned the attitude setpoint at the tilt limit in a 0.25 Hz limit cycle (roll and pitch
    # 78 deg peak to peak, 9 m of wander, gz sim 2026-09-11). Scaled to the attitude bandwidth
    # w_att = w_c / 2.5 these held 0.1 deg attitude noise and 15 cm position in the same sim.
    w_att = w_c / 2.5
    out.update(
        {
            "MPC_XY_VEL_P_ACC": round(0.75 * w_att, 3),
            "MPC_XY_VEL_I_ACC": round(0.14 * w_att, 3),
            "MPC_XY_VEL_D_ACC": round(0.28 * w_att, 3),
            "MPC_XY_P": round(min(2.0, 0.47 * w_att), 3),
            "MPC_Z_VEL_P_ACC": round(1.9 * w_att, 3),
            "MPC_Z_VEL_I_ACC": round(0.94 * w_att, 3),
            "MPC_Z_P": round(min(1.5, 0.56 * w_att), 3),
            "MPC_TILTMAX_AIR": 20.0,
            "MPC_ACC_HOR": 2.0,
            "MPC_ACC_UP_MAX": 2.0,
            "MPC_ACC_DOWN_MAX": 2.0,
            "MPC_JERK_AUTO": 2.0,
            "MPC_XY_VEL_MAX": 3.0,
        }
    )
    # yaw keeps the same attitude/rate ratio as roll and pitch: raising MC_YAW_P to 1.4 w_att with
    # a lower rate gain produced a 0.4 Hz yaw limit cycle (114 deg peak to peak) in gz sim
    # A geometry tuned by hand in the sim keeps its gains in control.px4_params_override, which
    # wins over the rule here and reaches the SITL airframe and the HITL .params through the same
    # call. CA_* entries are geometry, not tuning, and are applied by ca_geometry_params instead.
    out.update(
        {k: float(v) for k, v in scenario.control.px4_params_override.items()
         if not k.startswith("CA_")}
    )
    return out


def px4_airframe(scenario: Scenario, name: str) -> str:
    ca = ca_geometry_params(scenario)
    tuning = px4_tuning(scenario)
    lines = [
        "#!/bin/sh",
        f"# @name tiltlab {name} (gz)",
        "# @type Multirotor",
        "# @class Copter",
        "# Generated by tiltlab from the scenario; CA_ROTOR* match the exported .params.",
        "# Geometry in this export (check this is the deflection set you meant to fly):",
        *(f"#   {line}" for line in geometry_summary(scenario)),
        "",
        ". ${R}etc/init.d/rc.mc_defaults",
        "",
        "PX4_SIMULATOR=${PX4_SIMULATOR:=gz}",
        f"PX4_GZ_WORLD=${{PX4_GZ_WORLD:={name}}}",
        f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={name}}}",
        "",
        "param set-default SIM_GZ_EN 1",
        "param set-default CA_AIRFRAME 0",
        f"param set-default CA_METHOD {int(scenario.control.ca_method)}",
        f"param set-default CA_ROTOR_COUNT {int(ca['CA_ROTOR_COUNT'])}",
    ]
    for i in range(int(ca["CA_ROTOR_COUNT"])):
        for key in ("PX", "PY", "PZ", "AX", "AY", "AZ", "CT", "KM"):
            lines.append(
                f"param set-default CA_ROTOR{i}_{key} {float(ca[f'CA_ROTOR{i}_{key}']):.6g}"
            )
    for i in range(int(ca["CA_ROTOR_COUNT"])):
        lines.append(f"param set-default SIM_GZ_EC_FUNC{i + 1} {101 + i}")
        # idle command 0 (not x500's 150): with THR_MDL_FAC 1 the gz thrust is then exactly
        # linear in PX4's motor command, so the allocator's hover and torque trims are right
        lines.append(f"param set-default SIM_GZ_EC_MIN{i + 1} 0")
        lines.append(f"param set-default SIM_GZ_EC_MAX{i + 1} 1000")
    lines += [
        "",
        "# Controller sizing from tiltlab (see px4_tuning in tiltlab/export/gazebo.py): rate-loop",
        f"# crossover min(4 rad/s, 1/(3.5 x fan lag {fan_lag_s(scenario):.2f} s)) over torque",
        "# authority / inertia per axis; THR_MDL_FAC 1 linearises motorConstant x omega^2.",
        *(f"param set-default {k} {v:g}" for k, v in tuning.items()),
        "",
        "# SITL sends MAVLink to localhost only. Inside WSL2 broadcasts never reach Windows",
        "# (tested: unicast to the host does, subnet and limited broadcasts do not), so the GCS",
        "# link is started here, before rc.mavlink, aimed at the Windows host (WSL's default",
        "# gateway) on QGC's port. rc.mavlink then finds port 18570 occupied ('already occupied'",
        "# is expected) and this stays the only GCS link. Override with PX4_GCS_HOST and",
        "# PX4_GCS_PORT; skipped outside WSL unless PX4_GCS_HOST is set.",
        "param set-default MAV_0_BROADCAST 1",
        'if [ -n "${PX4_GCS_HOST}" ] || grep -qi microsoft /proc/version 2>/dev/null',
        "then",
        "\tgcs_host=${PX4_GCS_HOST:-$(ip route show default 2>/dev/null"
        " | awk '{print $3}' | head -n 1)}",
        "\tgcs_port=${PX4_GCS_PORT:-14550}",
        '\tif [ -n "${gcs_host}" ]',
        "\tthen",
        '\t\techo "INFO  [init] tiltlab: MAVLink GCS link to ${gcs_host}:${gcs_port} (WSL2 host)"',
        "\t\tmavlink start -x -u 18570 -r 4000000 -f -t ${gcs_host} -o ${gcs_port}",
        "\tfi",
        "fi",
        "",
    ]
    return "\n".join(lines)


def readme(scenario: Scenario, name: str) -> str:
    return f"""# Gazebo harness for `{scenario.meta.name}`

Generated by tiltlab; not run on the machine that generated it (Gazebo needs a Linux host with
PX4 v1.17 and gz sim Harmonic). Frames: gz body is FLU; tiltlab's FRD (x, y, z) maps to (x, -y, -z).
Rotor links sit at the foil pressure points and point along the deflected thrust; each
MulticopterMotorModel maps full command (maxRotVelocity {MAX_ROT_VELOCITY:.0f} rad/s) onto the fan's
effective CT, so the thrust seen by gz equals the CA_ROTOR*_CT in the airframe file.

## Geometry in this export
Exports of the same scenario overwrite this directory, so confirm this is the deflection set you
meant to fly (the airframe file carries the same list in its header). A set that tiltlab reports
as "no level-attitude hover trim" or "Fz unattainable" will sit still in Gazebo: PX4's allocator
finds no motor combination that gives pure vertical thrust and commands zero.
{chr(10).join("- " + line for line in geometry_summary(scenario))}

## Files
- `models/{name}/model.sdf`, `model.config`, `meshes/airframe.stl` (the CAD converted to the gz
  body frame FLU; the blue discs are the rotor links at the foil pressure points, not the ducts)
- `worlds/{name}.sdf`
- `px4/airframes/{airframe_id(name)}_gz_{name}` (posix airframe with the CA_ROTOR* geometry)

## Install into a PX4 checkout (v1.17)
```bash
cp -r models/{name} $PX4/Tools/simulation/gz/models/
cp worlds/{name}.sdf $PX4/Tools/simulation/gz/worlds/
cp px4/airframes/{airframe_id(name)}_gz_{name} $PX4/ROMFS/px4fmu_common/init.d-posix/airframes/
# add {airframe_id(name)}_gz_{name} to $PX4/ROMFS/px4fmu_common/init.d-posix/airframes/
#   CMakeLists.txt
cd $PX4 && make px4_sitl gz_{name}
```
PX4's gz bridge (src/modules/simulation/gz_bridge) publishes `/{name}/command/motor_speed` and reads
odometry, IMU, barometer, magnetometer and GPS from the model; QGroundControl connects on UDP 14550.
PX4 spawns the vehicle itself as `{name}_0` (px4-rc.gzsim), so the world must not include it too:
a second copy would sit inside the first. The Gazebo entity tree should show one `{name}_0`.

## Flying
1. QGroundControl must be connected (PX4 refuses to arm with "No connection to the GCS"). PX4 SITL
   talks to localhost only, and inside WSL2 (default NAT networking) UDP broadcasts never reach
   Windows while unicast in both directions does. The airframe therefore starts the GCS MAVLink
   link aimed at the Windows host (WSL's default gateway) on UDP 14550, so QGC's default
   auto-connect link picks the vehicle up with no configuration. `PX4_GCS_HOST` and
   `PX4_GCS_PORT` override the target; the console prints `tiltlab: MAVLink GCS link to ...`.
   Fallback by hand: QGC > Application Settings > Comm Links > Add, type UDP, listening port
   14551 (not 14550, which QGC's auto-connect already holds), server `<WSL IP>:18570`
   (`hostname -I` in WSL). QGC sends the first packet and PX4 answers that address directly.
2. Wait for "Ready to fly" in QGC (the estimator needs GPS, baro and IMU; the log shows
   `attitude_invalid` and `global_position_invalid` until then).
3. In the PX4 console: `commander takeoff`, later `commander land`. Or use QGC's Takeoff slider,
   then the virtual joystick (Application Settings > General > Virtual Joystick) or a USB gamepad.
4. `commander status` and `listener actuator_motors` show the mode and the ten motor commands.

## PX4's gz bridge has only 8 ESC channels
`src/modules/simulation/gz_bridge/module.yaml` declares `__max_num_servos: &max_num_servos 8`, so
`SIM_GZ_EC_FUNC9` and `SIM_GZ_EC_FUNC10` do not exist and PX4 publishes at most 8 motor speeds.
Gazebo then prints `MulticopterMotorModel ... You tried to access index N of the Actuator velocity
array which is of size M` for every motor beyond M. Before building, raise the limit:
```bash
sed -i 's/^__max_num_servos: &max_num_servos 8/__max_num_servos: \\&max_num_servos 12/' \\
  $PX4/src/modules/simulation/gz_bridge/module.yaml
```
The velocity array PX4 sends has as many entries as there are *consecutive* configured functions
from `SIM_GZ_EC_FUNC1`; a size of 4 means `SIM_GZ_EC_FUNC5` is unset (airframe not applied, or a
saved parameter file from an earlier run of this autostart id under
`build/px4_sitl_default/rootfs`). Check with `param show SIM_GZ_EC_FUNC*` in the PX4 shell.

A second, hidden limit follows: `GZMixingInterfaceESC::motorSpeedCallback` copies every motor
speed into `esc_status.esc[]`, which has `CONNECTED_ESC_MAX = 8` entries. With ten motors the
write overruns the stack and glibc aborts PX4 (`__stack_chk_fail`, SIGABRT) a moment after
`INFO [gz_bridge] world: ..., model: ..._0`; QGroundControl then never sees a vehicle. Clamp the
copy to 8 entries before building (the esc_status message is telemetry only):
```bash
F=$PX4/src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp
sed -i 's/esc_status.esc_count = actuators.velocity_size();/'\\
'esc_status.esc_count = actuators.velocity_size() < esc_status_s::CONNECTED_ESC_MAX'\\
' ? actuators.velocity_size() : esc_status_s::CONNECTED_ESC_MAX;/' $F
sed -i 's/for (int i = 0; i < actuators.velocity_size(); i++) {{/'\\
'for (int i = 0; i < esc_status.esc_count; i++) {{/' $F
```
The WSL helper script applies both patches when the model has more than 8 motors.

## Line endings
PX4's `sh` sources the airframe line by line. If the file arrives with Windows CRLF endings every
`param set-default` fails silently and PX4 runs on defaults (`CA_ROTOR_COUNT 4`, four ESC outputs).
tiltlab writes LF; if you edit or copy the files on Windows, run `sed -i 's/\\r$//'` on them in WSL.

## What to check first
1. `gz topic -l` shows `/model/{name}/command/motor_speed`; PX4 prints `INFO [gz_bridge] connected`.
2. In Stabilized on the ground, raising the throttle spins rotors 8 and 9 (centreline) and the
   wing rotors according to the allocator; compare with tiltlab's hover `u`.
3. Hover in Position mode. If the vehicle drifts fore-aft at level attitude, the geometry has a
   net Fx, which tiltlab reports as "no level-attitude hover trim".

## PX4 controller sizing written into the airframe
PX4's default MC_* gains fit a small quad (about 130 rad/s^2 of angular acceleration per unit of
normalised torque, 10 to 25 ms motor spool). This airframe gives far less angular authority per
unit and its fans spool in {fan_lag_s(scenario) * 1000:.0f} ms, so with the defaults the rate
loop is slower than the attitude loop above it and the vehicle flips within two seconds of
lift-off (seen in gz before this sizing was added). The airframe therefore sets, per axis:
rate-loop crossover w_c = min(4 rad/s, 1 / (3.5 x fan lag)), rate P = w_c / (torque authority /
inertia), I = 0.6 P (yaw 0.5 P), D = 0.05 P, integrator limit 0.15, attitude P = w_c / 2.5,
autotune disabled (MC_AT_EN 0), plus THR_MDL_FAC 1 with a
zero idle command so the gz thrust (motorConstant x omega^2) is linear in PX4's command, and
MPC_THR_HOVER at tiltlab's hover collective. Axes tiltlab marks unattainable keep PX4 defaults.
{chr(10).join(f"- {k} = {v:g}" for k, v in px4_tuning(scenario).items())}

With the per-pair foil set [135, 60, 120, 60] deg this flew in gz sim (2026-09-10): `commander
takeoff` to 2.5 m, 30 s hover with roll and pitch standard deviation under 3 deg and altitude
within 2.4 cm, `commander land`; the hover motor commands matched tiltlab's prediction (mean
0.449) on every motor. See docs/gazebo_flight_2026-09-10.md in the tiltlab repository.

## Limits
No aerodynamics of the foil or wing, no jet interaction, no ground effect; the Coanda turning is
baked into the rotor axes (effective deflection), not simulated. Mass and inertia are the
scenario's values (placeholders until the CoG dashboard weights are imported).
"""


def export_gazebo(
    scenario: Scenario, out_dir: str | Path, name: str | None = None
) -> dict[str, Any]:
    """Write the harness under out_dir/<name>/ and return the file paths."""
    name = name or scenario.meta.name
    root = Path(out_dir) / name
    model_dir = root / "models" / name
    (model_dir / "meshes").mkdir(parents=True, exist_ok=True)
    (root / "worlds").mkdir(parents=True, exist_ok=True)
    (root / "px4" / "airframes").mkdir(parents=True, exist_ok=True)

    mesh_uri = None
    if scenario.meta.cad_model:
        src = Path(__file__).resolve().parents[3] / "scenarios" / scenario.meta.cad_model
        if src.is_file() and airframe_stl(src, model_dir / "meshes" / "airframe.stl"):
            mesh_uri = f"model://{name}/meshes/airframe.stl"

    (model_dir / "model.sdf").write_text(
        model_sdf(scenario, name, mesh_uri), encoding="utf-8", newline="\n"
    )
    (model_dir / "model.config").write_text(
        f'<?xml version="1.0"?>\n<model>\n  <name>{escape(name)}</name>\n  <version>1.0</version>\n'
        f'  <sdf version="1.9">model.sdf</sdf>\n  <author><name>tiltlab</name></author>\n'
        f"  <description>{escape(scenario.meta.name)} generated by tiltlab</description>\n"
        "</model>\n",
        encoding="utf-8",
    )
    (root / "worlds" / f"{name}.sdf").write_text(world_sdf(name), encoding="utf-8", newline="\n")
    airframe = root / "px4" / "airframes" / f"{airframe_id(name)}_gz_{name}"
    airframe.write_text(px4_airframe(scenario, name), encoding="utf-8", newline="\n")
    (root / "README.md").write_text(readme(scenario, name), encoding="utf-8", newline="\n")
    return {
        "root": str(root),
        "model_sdf": str(model_dir / "model.sdf"),
        "world_sdf": str(root / "worlds" / f"{name}.sdf"),
        "airframe": str(airframe),
        "readme": str(root / "README.md"),
        "mesh": mesh_uri,
    }
