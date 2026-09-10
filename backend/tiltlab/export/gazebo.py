"""Gazebo (gz sim) harness generated from a Scenario.

Writes a model (SDF 1.9) with the airframe body and one rotor link per fan placed at the
effective force point and pointed along the effective thrust direction (foil-aware), each driven
by the gz MulticopterMotorModel plugin with the fan's effective thrust constant; a world that
includes the model with the sensor systems PX4's gz bridge expects; a PX4 posix airframe file
carrying the same CA_ROTOR* geometry so the allocator in SITL matches the exported params; and a
README with the launch steps. Frames: FRD (x forward, y right, z down) becomes gz body FLU
(x forward, y left, z up): (x, y, z) -> (x, -y, -z).

Generated, not run here (Gazebo needs a Linux host); see the README for the checklist.
"""

from __future__ import annotations

import math
import shutil
from pathlib import Path
from typing import Any
from xml.sax.saxutils import escape

import numpy as np

from tiltlab.export.params import ca_geometry_params
from tiltlab.scenario import Scenario

MAX_ROT_VELOCITY = 1000.0  # rad/s at full command; motorConstant maps it onto the fan CT
ROTOR_MASS_KG = 0.32  # XFly 80 mm EDF unit
AIRFRAME_ID = 4010


def frd_to_flu(v: tuple[float, float, float] | np.ndarray) -> tuple[float, float, float]:
    x, y, z = (float(c) for c in v)
    return (x, -y, -z)


def axis_to_rpy(axis_flu: tuple[float, float, float]) -> tuple[float, float, float]:
    """Roll, pitch, yaw (rad) that rotate the link +z onto axis_flu (zero roll)."""
    x, y, z = axis_flu
    # pitch about y then roll about x: z' = (sin p cos r, -sin r, cos p cos r)
    pitch = math.atan2(x, math.hypot(y, z) if (y or z) else 1e-12)
    roll = math.atan2(-y, z)
    return (roll, pitch, 0.0)


def _inertia(scenario: Scenario) -> dict[str, float]:
    m = scenario.mass.total_kg
    ix = np.asarray(scenario.mass.inertia_frd_kgm2, dtype=float)
    if np.allclose(ix, 0.0):
        # placeholder: uniform box 1.2 m long, 0.9 m wide, 0.3 m tall
        a, b, c = 1.2, 0.9, 0.3
        ix = np.diag([m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12])
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
    ca = ca_geometry_params(scenario)
    body_visual = (
        f'<visual name="airframe_visual"><geometry><mesh><uri>{escape(mesh_uri)}</uri></mesh>'
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
        f"        <mass>{scenario.mass.total_kg:.4f}</mass>",
        "        <inertia>"
        + "".join(f"<{k}>{v:.6f}</{k}>" for k, v in inertia.items())
        + "</inertia>",
        "      </inertial>",
        f"      {body_visual}",
        '      <collision name="airframe_collision"><geometry><box><size>1.2 0.9 0.3</size></box>'
        "</geometry></collision>",
        '      <sensor name="imu_sensor" type="imu"><always_on>1</always_on>'
        "<update_rate>250</update_rate></sensor>",
        '      <sensor name="air_pressure_sensor" type="air_pressure"><always_on>1</always_on>'
        '<update_rate>50</update_rate><air_pressure><pressure><noise type="gaussian">'
        "<mean>0</mean><stddev>0.01</stddev></noise></pressure></air_pressure></sensor>",
        '      <sensor name="magnetometer_sensor" type="magnetometer"><always_on>1</always_on>'
        "<update_rate>100</update_rate></sensor>",
        "    </link>",
    ]
    for fan in scenario.fans_sorted():
        i = fan.id
        pos = frd_to_flu(scenario.effective_pos(fan) - np.asarray(scenario.mass.cg_frd_m))
        axis = frd_to_flu(scenario.effective_axis(fan))
        r, p, y = axis_to_rpy(axis)
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
            "      <timeConstantUp>0.15</timeConstantUp><timeConstantDown>0.15</timeConstantDown>",
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
    <include><uri>model://{escape(name)}</uri><pose>0 0 0.3 0 0 0</pose></include>
  </world>
</sdf>
"""


def px4_airframe(scenario: Scenario, name: str) -> str:
    ca = ca_geometry_params(scenario)
    lines = [
        "#!/bin/sh",
        f"# @name tiltlab {name} (gz)",
        "# @type Multirotor",
        "# @class Copter",
        "# Generated by tiltlab from the scenario; CA_ROTOR* match the exported .params.",
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
        lines.append(f"param set-default SIM_GZ_EC_MIN{i + 1} 150")
        lines.append(f"param set-default SIM_GZ_EC_MAX{i + 1} 1000")
    lines += ["", "param set-default MPC_THR_HOVER 0.35", ""]
    return "\n".join(lines)


def readme(scenario: Scenario, name: str) -> str:
    return f"""# Gazebo harness for `{scenario.meta.name}`

Generated by tiltlab; not run on the machine that generated it (Gazebo needs a Linux host with
PX4 v1.17 and gz sim Harmonic). Frames: gz body is FLU; tiltlab's FRD (x, y, z) maps to (x, -y, -z).
Rotor links sit at the foil pressure points and point along the deflected thrust; each
MulticopterMotorModel maps full command (maxRotVelocity {MAX_ROT_VELOCITY:.0f} rad/s) onto the fan's
effective CT, so the thrust seen by gz equals the CA_ROTOR*_CT in the airframe file.

## Files
- `models/{name}/model.sdf`, `model.config`, `meshes/` (airframe glTF when the scenario has one)
- `worlds/{name}.sdf`
- `px4/airframes/{AIRFRAME_ID}_gz_{name}` (posix airframe with the CA_ROTOR* geometry)

## Install into a PX4 checkout (v1.17)
```bash
cp -r models/{name} $PX4/Tools/simulation/gz/models/
cp worlds/{name}.sdf $PX4/Tools/simulation/gz/worlds/
cp px4/airframes/{AIRFRAME_ID}_gz_{name} $PX4/ROMFS/px4fmu_common/init.d-posix/airframes/
# add {AIRFRAME_ID}_gz_{name} to $PX4/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
cd $PX4 && make px4_sitl gz_{name}
```
PX4's gz bridge (src/modules/simulation/gz_bridge) publishes `/{name}/command/motor_speed` and reads
odometry, IMU, barometer and magnetometer from the model; QGroundControl connects on UDP 14550.

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
        if src.is_file():
            shutil.copy(src, model_dir / "meshes" / src.name)
            mesh_uri = f"model://{name}/meshes/{src.name}"

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
    airframe = root / "px4" / "airframes" / f"{AIRFRAME_ID}_gz_{name}"
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
