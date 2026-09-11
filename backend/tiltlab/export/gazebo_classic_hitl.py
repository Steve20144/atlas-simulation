"""Gazebo Classic HITL harness: the real Pixhawk flies a Gazebo Classic model over USB.

PX4 v1.17 supports hardware-in-the-loop only through the MAVLink HIL_* messages, which the
Gazebo Classic ``libgazebo_mavlink_interface`` plugin speaks over a serial port (the new gz sim
bridge does not). This module writes a Gazebo Classic (SDF 1.6) model of the scenario mirroring
PX4's reference ``iris_hitl`` model: one rotor per fan at the effective force point along the
effective thrust direction with ``libgazebo_motor_model`` scaled to the fan's effective CT, the
IMU, GPS, magnetometer and barometer plugins, and the MAVLink interface in HIL serial mode with
one control channel per motor. It also writes a world, a QGroundControl-loadable parameter file
that switches the Pixhawk into HITL with this geometry, and a step-by-step README.

Frames: FRD (x forward, y right, z down) becomes Gazebo body FLU: (x, y, z) -> (x, -y, -z).
Generated, not run here (needs Ubuntu with Gazebo Classic 11 and the PX4 sitl_gazebo-classic
plugins).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
from xml.sax.saxutils import escape

import numpy as np

from tiltlab.core.params_px4 import (
    PARAM_TYPE_INT32,
    default_header,
    params_file_from_dict,
    write_params_file,
)
from tiltlab.export.gazebo import (
    _inertia,
    airframe_stl,
    axis_to_rpy,
    body_mass_kg,
    fan_lag_s,
    frd_to_flu,
    px4_tuning,
)
from tiltlab.export.params import ca_geometry_params
from tiltlab.scenario import Scenario

HIL_AIRFRAME = 1001  # PX4 "HIL Quadcopter X": sets SYS_HITL 1 and starts pwm_out_sim -m hil
INPUT_SCALING = 1000.0  # rad/s at full command in the mavlink_interface control channels
MAX_ROT_VELOCITY = 1100.0
ROTOR_MASS_KG = 0.32
DEFAULT_SERIAL = "/dev/ttyACM0"


def _rotor_block(i: int, pos: tuple[float, float, float], rpy: tuple[float, float, float]) -> str:
    r, p, y = rpy
    return f"""    <link name='rotor_{i}'>
      <pose>{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} {r:.5f} {p:.5f} {y:.5f}</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>{ROTOR_MASS_KG}</mass>
        <inertia>
          <ixx>0.0003</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0003</iyy>
          <iyz>0</iyz>
          <izz>0.0005</izz>
          </inertia>
      </inertial>
      <collision name='rotor_{i}_collision'>
        <geometry><cylinder><length>0.02</length><radius>0.042</radius></cylinder></geometry>
        <surface><contact><ode/></contact><friction><ode/></friction></surface>
      </collision>
      <visual name='rotor_{i}_visual'>
        <geometry><cylinder><length>0.02</length><radius>0.042</radius></cylinder></geometry>
        <material><script><name>Gazebo/Blue</name>
          <uri>file://media/materials/scripts/gazebo.material</uri></script></material>
      </visual>
      <gravity>1</gravity>
      <velocity_decay/>
    </link>
    <joint name='rotor_{i}_joint' type='revolute'>
      <child>rotor_{i}</child>
      <parent>base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
          </dynamics>
        <use_parent_model_frame>0</use_parent_model_frame>
      </axis>
    </joint>
"""


def _motor_plugin(i: int, turning: str, ct: float, km: float, lag_s: float) -> str:
    return f"""    <plugin name='motor_{i}_model' filename='libgazebo_motor_model.so'>
      <robotNamespace/>
      <jointName>rotor_{i}_joint</jointName>
      <linkName>rotor_{i}</linkName>
      <turningDirection>{turning}</turningDirection>
      <timeConstantUp>{lag_s:.3f}</timeConstantUp>
      <timeConstantDown>{lag_s:.3f}</timeConstantDown>
      <maxRotVelocity>{MAX_ROT_VELOCITY:.0f}</maxRotVelocity>
      <motorConstant>{ct / INPUT_SCALING**2:.10e}</motorConstant>
      <momentConstant>{abs(km):.6f}</momentConstant>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>{i}</motorNumber>
      <rotorDragCoefficient>0</rotorDragCoefficient>
      <rollingMomentCoefficient>0</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/{i}</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
"""


def _channel(i: int) -> str:
    return f"""        <channel name='rotor{i + 1}'>
          <input_index>{i}</input_index>
          <input_offset>0</input_offset>
          <input_scaling>{INPUT_SCALING:.0f}</input_scaling>
          <zero_position_disarmed>0</zero_position_disarmed>
          <zero_position_armed>0</zero_position_armed>
          <joint_control_type>velocity</joint_control_type>
        </channel>
"""


def model_sdf(scenario: Scenario, name: str, mesh_uri: str | None, serial: str) -> str:
    inertia = _inertia(scenario)
    ca = ca_geometry_params(scenario)
    n = len(scenario.fans)
    body_visual = (
        f"<geometry><mesh><scale>1 1 1</scale><uri>{escape(mesh_uri)}</uri></mesh></geometry>"
        if mesh_uri
        else "<geometry><box><size>1.2 0.9 0.3</size></box></geometry>"
    )
    out = [
        f"<!-- Generated by tiltlab from scenario {escape(scenario.meta.name)} for HITL "
        "(mirrors PX4-SITL_gazebo-classic models/iris_hitl) -->",
        "<sdf version='1.6'>",
        f"  <model name='{escape(name)}'>",
        "    <link name='base_link'>",
        "      <pose>0 0 0 0 0 0</pose>",
        "      <inertial>",
        "        <pose>0 0 0 0 0 0</pose>",
        f"        <mass>{body_mass_kg(scenario, ROTOR_MASS_KG):.4f}</mass>",
        "        <inertia>"
        + "".join(f"<{k}>{v:.6f}</{k}>" for k, v in inertia.items())
        + "</inertia>",
        "      </inertial>",
        "      <collision name='base_link_collision'>",
        "        <geometry><box><size>1.2 0.9 0.3</size></box></geometry>",
        "        <surface><contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode>"
        "</contact><friction><ode/></friction></surface>",
        "      </collision>",
        f"      <visual name='base_link_visual'>{body_visual}",
        "        <material><script><name>Gazebo/DarkGrey</name>"
        "<uri>file://media/materials/scripts/gazebo.material</uri></script></material></visual>",
        "      <gravity>1</gravity>",
        "      <velocity_decay/>",
        "    </link>",
        "    <link name='/imu_link'>",
        "      <pose>0 0 0 0 0 0</pose>",
        "      <inertial><pose>0 0 0 0 0 0</pose><mass>0.015</mass>",
        "        <inertia><ixx>1e-05</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-05</iyy><iyz>0</iyz>"
        "<izz>1e-05</izz></inertia></inertial>",
        "    </link>",
        "    <joint name='/imu_joint' type='revolute'>",
        "      <child>/imu_link</child><parent>base_link</parent>",
        "      <axis><xyz>1 0 0</xyz><limit><lower>0</lower><upper>0</upper><effort>0</effort>"
        "<velocity>0</velocity></limit><dynamics><spring_reference>0</spring_reference>"
        "<spring_stiffness>0</spring_stiffness></dynamics><use_parent_model_frame>1"
        "</use_parent_model_frame></axis>",
        "    </joint>",
    ]
    cg = np.asarray(scenario.mass.cg_frd_m)
    for fan in scenario.fans_sorted():
        pos = frd_to_flu(scenario.effective_pos(fan) - cg)
        rpy = axis_to_rpy(frd_to_flu(scenario.effective_axis(fan)))
        out.append(_rotor_block(fan.id, pos, rpy))
    out.append(
        "    <plugin name='rosbag' filename='libgazebo_multirotor_base_plugin.so'>\n"
        "      <robotNamespace/>\n      <linkName>base_link</linkName>\n"
        "      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>\n    </plugin>\n"
    )
    for fan in scenario.fans_sorted():
        i = fan.id
        out.append(
            _motor_plugin(
                i,
                "ccw" if fan.spin == "CCW" else "cw",
                float(ca[f"CA_ROTOR{i}_CT"]),
                float(ca[f"CA_ROTOR{i}_KM"]),
                fan_lag_s(scenario),
            )
        )
    out.append(
        """    <model name='gps0'>
      <link name='link'>
        <pose>0 0 0 0 0 0</pose>
        <inertial><pose>0 0 0 0 0 0</pose><mass>0.01</mass>
          <inertia>
            <ixx>2.1733e-06</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.1733e-06</iyy>
            <iyz>0</iyz>
            <izz>1.8e-07</izz>
            </inertia>
        </inertial>
        <sensor name='gps' type='gps'>
          <pose>0 0 0 0 0 0</pose>
          <plugin name='gps_plugin' filename='libgazebo_gps_plugin.so'>
            <robotNamespace/>
            <gpsNoise>1</gpsNoise>
            <gpsXYRandomWalk>2.0</gpsXYRandomWalk>
            <gpsZRandomWalk>4.0</gpsZRandomWalk>
            <gpsXYNoiseDensity>0.0002</gpsXYNoiseDensity>
            <gpsZNoiseDensity>0.0004</gpsZNoiseDensity>
            <gpsVXYNoiseDensity>0.2</gpsVXYNoiseDensity>
            <gpsVZNoiseDensity>0.4</gpsVZNoiseDensity>
          </plugin>
        </sensor>
      </link>
    </model>
    <joint name='gps0_joint' type='fixed'>
      <parent>base_link</parent>
      <child>gps0::link</child>
    </joint>
    <plugin name='groundtruth_plugin' filename='libgazebo_groundtruth_plugin.so'>
      <robotNamespace/>
    </plugin>
    <plugin name='magnetometer_plugin' filename='libgazebo_magnetometer_plugin.so'>
      <robotNamespace/>
      <pubRate>100</pubRate>
      <noiseDensity>0.0004</noiseDensity>
      <randomWalk>6.4e-06</randomWalk>
      <biasCorrelationTime>600</biasCorrelationTime>
      <magTopic>/mag</magTopic>
    </plugin>
    <plugin name='barometer_plugin' filename='libgazebo_barometer_plugin.so'>
      <robotNamespace/>
      <pubRate>50</pubRate>
      <baroTopic>/baro</baroTopic>
      <baroDriftPaPerSec>0</baroDriftPaPerSec>
    </plugin>
"""
    )
    out.append(
        f"""    <plugin name='mavlink_interface' filename='libgazebo_mavlink_interface.so'>
      <robotNamespace/>
      <imuSubTopic>/imu</imuSubTopic>
      <magSubTopic>/mag</magSubTopic>
      <baroSubTopic>/baro</baroSubTopic>
      <mavlink_addr>INADDR_ANY</mavlink_addr>
      <mavlink_tcp_port>4560</mavlink_tcp_port>
      <mavlink_udp_port>14560</mavlink_udp_port>
      <serialEnabled>1</serialEnabled>
      <serialDevice>{escape(serial)}</serialDevice>
      <baudRate>921600</baudRate>
      <qgc_addr>INADDR_ANY</qgc_addr>
      <qgc_udp_port>14550</qgc_udp_port>
      <sdk_addr>INADDR_ANY</sdk_addr>
      <sdk_udp_port>14540</sdk_udp_port>
      <hil_mode>1</hil_mode>
      <hil_state_level>0</hil_state_level>
      <send_vision_estimation>0</send_vision_estimation>
      <send_odometry>1</send_odometry>
      <enable_lockstep>0</enable_lockstep>
      <use_tcp>0</use_tcp>
      <motorSpeedCommandPubTopic>/gazebo/command/motor_speed</motorSpeedCommandPubTopic>
      <control_channels>
{"".join(_channel(i) for i in range(n))}      </control_channels>
    </plugin>
    <plugin name='rotors_gazebo_imu_plugin' filename='libgazebo_imu_plugin.so'>
      <robotNamespace/>
      <linkName>/imu_link</linkName>
      <imuTopic>/imu</imuTopic>
      <gyroscopeNoiseDensity>0.0003394</gyroscopeNoiseDensity>
      <gyroscopeRandomWalk>3.8785e-05</gyroscopeRandomWalk>
      <gyroscopeBiasCorrelationTime>1000.0</gyroscopeBiasCorrelationTime>
      <gyroscopeTurnOnBiasSigma>0.0087</gyroscopeTurnOnBiasSigma>
      <accelerometerNoiseDensity>0.004</accelerometerNoiseDensity>
      <accelerometerRandomWalk>0.006</accelerometerRandomWalk>
      <accelerometerBiasCorrelationTime>300.0</accelerometerBiasCorrelationTime>
      <accelerometerTurnOnBiasSigma>0.196</accelerometerTurnOnBiasSigma>
    </plugin>
  </model>
</sdf>
"""
    )
    return "\n".join(out)


def world_sdf(name: str) -> str:
    return f"""<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="hitl_{escape(name)}_world">
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
      </scene>
    <gui fullscreen='0'><camera name='user_camera'><pose>-6.0 -3.0 4.0 0 0.5 0.4</pose>
      <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
        </camera>
        </gui>
    <include><uri>model://sun</uri></include>
    <physics name='default_physics' default='0' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <ode><solver><type>quick</type><iters>10</iters><sor>1.3</sor>
        <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling></solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer></constraints></ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://{escape(name)}</uri><pose>0 0 0.5 0 0 0</pose></include>
  </world>
</sdf>
"""


def hitl_params(scenario: Scenario) -> dict[str, int | float]:
    """Parameters QGroundControl loads onto the Pixhawk for HITL with this geometry."""
    ca = ca_geometry_params(scenario)
    params: dict[str, int | float] = {
        "SYS_AUTOSTART": HIL_AIRFRAME,
        "SYS_HITL": 1,
        "CA_AIRFRAME": 0,
        "CA_METHOD": int(scenario.control.ca_method),
        "UAVCAN_ENABLE": 0,
    }
    params.update({k: v for k, v in ca.items() if k != "CA_ROTOR_COUNT"})
    params["CA_ROTOR_COUNT"] = int(ca["CA_ROTOR_COUNT"])
    for i in range(int(ca["CA_ROTOR_COUNT"])):
        params[f"HIL_ACT_FUNC{i + 1}"] = 101 + i
    # same controller sizing that flew the gz SITL harness: the Classic motor model is also
    # thrust = k * omega^2 with omega = command * input_scaling (zero_position_armed 0), so
    # THR_MDL_FAC 1 linearises it and the gains sized from authority, inertia and fan lag apply
    params.update(px4_tuning(scenario))
    return params


def readme(scenario: Scenario, name: str, serial: str) -> str:
    return f"""# Gazebo Classic HITL harness for `{scenario.meta.name}`

The Pixhawk runs the real PX4 firmware and flies this Gazebo Classic model over USB
(MAVLink HIL_SENSOR / HIL_GPS in, HIL_ACTUATOR_CONTROLS out). Generated by tiltlab, not run here.

Files: `models/{name}/model.sdf` + `model.config` (+ `meshes/airframe.stl`),
`worlds/hitl_{name}.world`,
`px4/{name}_hitl.params` (load with QGroundControl), this README.

## Read first
- Stock PX4 v1.17 `px4_fmu-v6x` firmware CANNOT run HITL: with SYS_HITL set the boot script runs
  `pwm_out_sim start -m hil` (ROMFS/px4fmu_common/init.d/rcS:466-470) and no fmu-v6x board
  configuration compiles that module. Build once with the module enabled (step 2).
- HITL works with Gazebo Classic (gazebo 11) and PX4's sitl_gazebo-classic plugins on Ubuntu
  20.04/22.04. The new "gz sim" bridge has no HITL mode. Windows: use Ubuntu 22.04 in WSL2 with
  usbipd-win to pass the Pixhawk USB port through, or a Linux machine.
- Fans and ESCs must be unpowered for every HITL session. PX4 will command the motors as if flying.

## 1. Ubuntu prerequisites
```bash
git clone --recursive -b v1.17.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot && bash ./Tools/setup/ubuntu.sh        # toolchains, Gazebo Classic, dependencies
sudo apt install gazebo libgazebo-dev                       # if the script did not install Gazebo
Classic
sudo usermod -aG dialout $USER && newgrp dialout            # access to /dev/ttyACM*
```

## 2. Firmware with the HITL output module
```bash
cd ~/PX4-Autopilot
echo 'CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y' >> boards/px4/fmu-v6x/default.px4board
make px4_fmu-v6x_default
make px4_fmu-v6x_default upload        # Pixhawk on USB, no other program holding the port
```
The build target is the same for Pixhawk 6X and 6X Pro.

## 3. Parameters
1. Connect QGroundControl (USB), Vehicle Setup > Parameters > Tools > Load from file:
   `px4/{name}_hitl.params`. It sets SYS_AUTOSTART {HIL_AIRFRAME} (HIL airframe), SYS_HITL 1,
   the {len(scenario.fans)} rotor geometry (CA_ROTOR*), CA_ROTOR_COUNT and
   HIL_ACT_FUNC1..{len(scenario.fans)} = 101..{100 + len(scenario.fans)}.
2. Reboot the Pixhawk. The console should show `pwm_out_sim` running (`pwm_out_sim status` in
   Analyze Tools > MAVLink Console) and the sensors reported as simulated.
3. Close QGroundControl before starting Gazebo (only one program can own the serial port); Gazebo
   forwards MAVLink to QGC on UDP 14550, reconnect QGC after step 5. The plugin sends that to
   localhost (it binds its QGC socket to `qgc_addr`, so leave it INADDR_ANY), which under WSL2
   never reaches QGC on Windows: run `wsl/qgc_udp_relay.py` in WSL alongside Gazebo (the helper
   script starts it), which unicasts the telemetry to the Windows host and returns QGC's replies.

## 4. Build the Gazebo Classic plugins and install this model
```bash
cd ~/PX4-Autopilot && DONT_RUN=1 make px4_sitl_default gazebo-classic
cp -r models/{name} ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
cp worlds/hitl_{name}.world
~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot
~/PX4-Autopilot/build/px4_sitl_default
```

## 5. Run
```bash
ls /dev/ttyACM*                                              # the Pixhawk; edit serialDevice in
model.sdf if it is not {serial}
cd ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
gazebo --verbose worlds/hitl_{name}.world
```
Expect `Opened serial device {serial}` in the terminal and, in QGC (UDP), GPS lock and a level
attitude within a few seconds. Arm from QGC or a joystick (`COM_RC_IN_MODE 1` for joystick
only) and raise the throttle: rotors 8 and 9 (centreline) and the wing rotors spin as the
allocator dictates. A vehicle that pitches or slides at level attitude has a net moment or force
in this geometry, which tiltlab reports as "no level-attitude hover trim".

## What this model does not do
No foil aerodynamics (the Coanda turning is baked into the rotor directions), no jet
interaction, no ground effect, box collision only. Mass and inertia are the scenario's
placeholders until the CoG dashboard weights are imported.
"""


def export_gazebo_classic_hitl(
    scenario: Scenario, out_dir: str | Path, name: str | None = None, serial: str = DEFAULT_SERIAL
) -> dict[str, Any]:
    name = name or f"{scenario.meta.name}_hitl"
    root = Path(out_dir) / name
    model_dir = root / "models" / name
    (model_dir / "meshes").mkdir(parents=True, exist_ok=True)
    (root / "worlds").mkdir(parents=True, exist_ok=True)
    (root / "px4").mkdir(parents=True, exist_ok=True)

    mesh_uri = None
    if scenario.meta.cad_model:
        src = Path(__file__).resolve().parents[3] / "scenarios" / scenario.meta.cad_model
        if src.is_file():
            if airframe_stl(src, model_dir / "meshes" / "airframe.stl"):
                mesh_uri = f"model://{name}/meshes/airframe.stl"

    (model_dir / "model.sdf").write_text(
        model_sdf(scenario, name, mesh_uri, serial), encoding="utf-8", newline="\n"
    )
    (model_dir / "model.config").write_text(
        f'<?xml version="1.0"?>\n<model>\n  <name>{escape(name)}</name>\n  <version>1.0</version>\n'
        f"  <sdf version='1.6'>model.sdf</sdf>\n  <author><name>tiltlab</name></author>\n"
        f"  <description>{escape(scenario.meta.name)} HITL model generated by tiltlab"
        "</description>\n"
        "</model>\n",
        encoding="utf-8",
    )
    (root / "worlds" / f"hitl_{name}.world").write_text(
        world_sdf(name), encoding="utf-8", newline="\n"
    )
    params = hitl_params(scenario)
    types = {k: PARAM_TYPE_INT32 for k in params if isinstance(params[k], int)}
    pf = params_file_from_dict(
        params,
        types,
        header=default_header(
            note=f"tiltlab HITL set for {scenario.meta.name}: HIL airframe, SYS_HITL 1, "
            f"{len(scenario.fans)} rotors, HIL_ACT_FUNC1..{len(scenario.fans)}"
        ),
    )
    params_path = root / "px4" / f"{name}.params"
    write_params_file(pf, params_path)
    (root / "README.md").write_text(readme(scenario, name, serial), encoding="utf-8", newline="\n")
    return {
        "root": str(root),
        "model_sdf": str(model_dir / "model.sdf"),
        "world": str(root / "worlds" / f"hitl_{name}.world"),
        "params": str(params_path),
        "readme": str(root / "README.md"),
        "mesh": mesh_uri,
    }
