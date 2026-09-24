# Autonomous Claw Bulldozer ROS 2 Workspace

An autonomous manipulation and earth-moving platform featuring a multi-axis articulating high-torque claw and bucket mechanism, 4WD skid-steer chassis, ESP32 dual-core real-time control, onboard IMU pose tracking, dual camera vision systems, and ROS 2 Jazzy integration.

> 📦 **Hardware Heritage & Lineage Notice:** Core electronics (ESP32 DevKitC, TC1508 H-bridge, GY-BMI160 6-DOF IMU, buck converters) and the baseline micro-ROS architecture were harvested and evolved from the decommissioned [Mihaii2/esp32_bulldozer](https://github.com/Mihaii2/esp32_bulldozer).

<div align="center">
  <img src="video/claw_demo.gif" width="75%" alt="Autonomous Claw Bulldozer Actuation & Teleoperation Demo" /><br/>
  <em>Autonomous Claw Bulldozer in action: Synchronized multi-axis lift, bucket articulation, and payload capture.</em>
</div>

<br/>

<div align="center">
  <table>
    <tr>
      <td align="center" width="33%">
        <img src="photos/v1_prototype.jpg" width="100%" alt="Revision 1: Skinny Skeleton Prototype" /><br/>
        <b>Revision 1: Organic Lattice</b><br/>
        <em>3D round skeleton claw; discarded due to massive support waste and print cost.</em>
      </td>
      <td align="center" width="33%">
        <img src="photos/v2_axis_error.jpg" width="100%" alt="Revision 2: Off-Axis Pivot" /><br/>
        <b>Revision 2: Split Axis Failure</b><br/>
        <em>Cheaper multi-part claw, but non-concentric pivot axes and horn bolt misalignments failed grip lock.</em>
      </td>
      <td align="center" width="33%">
        <img src="photos/v3_thin_beams.jpg" width="100%" alt="Revision 3: Concentric Thin Beams" /><br/>
        <b>Revision 3: Compliant Beams</b><br/>
        <em>Concentric claw/cup axis resolved kinematics, but thin support arms caused excessive structural flex.</em>
      </td>
    </tr>
    <tr>
      <td align="center" width="33%">
        <img src="photos/v4_final_claw.jpg" width="100%" alt="Revision 4: Rigid Reinforced Claw" /><br/>
        <b>Revision 4: Rigid Production</b><br/>
        <em>Reinforced structural beam geometry; eliminates claw deflection and locks payload into collection cup.</em>
      </td>
      <td align="center" width="33%">
        <img src="photos/v5_dual_vision.jpg" width="100%" alt="Revision 5: Dual Vision & Phone Tilter Rig" /><br/>
        <b>Revision 5: Dual Vision & Enclosure Rig</b><br/>
        <em>Beam-enclosed ESP32-CAM, 5th-servo phone tilter mast, and custom electronics protective cover.</em>
      </td>
      <td align="center" width="33%">
        <!-- Empty slot for layout balance / future CAD revision -->
      </td>
    </tr>
  </table>
</div>

> **🛠 Engineering Revision History & CAD Iteration Log:**
> * **v1.0 (Organic Monolithic Claw):** Claw designed as a continuous 3D organic round skeleton. While lightweight on paper, slicer analysis proved it unviable for FDM printing: massive support structures consumed more filament than the part itself, driving print failure rates and material cost excessively high.
> * **v2.0 (Modular Shell & The Pivot Misalignment Post-Mortem):** Redesigned the claw into flat-laying, easily printable structural components, as well as redesigning the chassis goussets to get rid of a lot of waste support plastic. However, rapid CAD prototyping introduced two major mechanical flaws:
>   * **Axial Offset:** The bucket/cup swing axis and the claw rotation axis were placed on disparate coordinate planes rather than a single concentric shaft. The mechanical offset degraded clamping reach, making object acquisition unreliable.
>   * **Horn Geometry Error:** Servo horn mounting hole pitch and bolt clearances were mismeasured, preventing secure mechanical torque transfer without binding.
> * **v3.0 (Concentric Axis & Structural Compliance Investigation):** Redesigned the mounting hub to align the cup and claw along a shared, concentric pivot axis. The mechanical geometry functioned correctly, but the claw extension beams were undersized in thickness. Clamping torque from the MG995 servos flexed the beams outward rather than transferring grip pressure into the target object.
> * **v4.0 (Rigid Structural Reinforcement):** Completely re-engineered the claw support beams with increased cross-sectional area and internal ribbing. Flex is eliminated, permitting high-torque payload capture and retraction directly into the retention cup.
> * **v4.1 (Locomotion Tuning & Track/Wheel Calibration):**
>   * **Wheel Dimensional Scaling:** Replaced oversized 45mm-radius wheel collisions with realistic 20mm-radius (40mm diameter) hubs recessed directly into the chassis axle slots for accurate Gazebo physics and ground clearance.
>   * **Pivot & Scrubbing Firmware Mitigation:** Tuned the differential drive turning profile across all speed gears to supply 100% PWM (255) to the outside wheels and 0% PWM to the inside wheels during turns, breaking lateral static friction without stalling the motors.
>   * **Drivetrain Upgrade Roadmap:** Acquired **all-metal TT 1:90 6V reduction gearboxes** to replace the stock yellow plastic units, delivering substantial low-end torque for heavy earth-moving without risk of gear-tooth shearing.
> * **v5.0 (Perception, High-Current Power & Electronics Shielding — Current Hardware):**
>   * **Initial ESP32-CAM Prototype:** Early tests yielded severe frame-drop bottlenecks, latency, and poor visual fidelity under default configurations.
>   * **Smartphone Mount Pivot:** Designed a dedicated smartphone holder actuated by a 5th MG995 servo on `GPIO 4` to handle primary teleoperation and high-resolution spatial streaming via a mobile device.
>   * **Dual-Camera Convergence & Custom Enclosure:** Subsequent firmware optimizations, clock adjustments, and memory pipeline tuning substantially stabilized ESP32-CAM stream throughput. Attempting to mount the bare board to the pre-existing overhead beam directly above the claw proved mechanically unviable and structurally awkward. Consequently, engineered a custom dedicated 3D-printed plastic enclosure tailored specifically to house the ESP32-CAM and lock rigidly onto the upper beam assembly. The platform now implements a **dual-vision stack**: an actuated smartphone tilter for primary broad-field telemetry and a ruggedized, enclosed ESP32-CAM for direct manipulation monitoring, alongside an upcoming **8×8 Time-of-Flight (ToF)** distance matrix.
>   * **2S2P High-Current Battery Array (Brownout Mitigation):** In the 2-cell configuration, intense multi-joint dynamic actuation (simultaneous 5-servo manipulation alongside aggressive 4WD skid-steer scrubbing) drew more peak current than the pack could sustain. The severe voltage sag triggered brownout resets on the ESP32 logic rail. To resolve this, added two additional 18650 cells wired in parallel (**2S2P topology**). The battery pack maintains nominal **7.4V** while unlocking a massive **20A instantaneous discharge headroom**, ensuring complete electrical stability under full-load operation.
>   * **RF Range Expansion:** Added dedicated 3dBi external dipole antennas to both the main motherboard ESP32 and the secondary ESP32-CAM to prevent telemetry drops and feed degradation across Wi-Fi.
>   * **Electronics Enclosure / Cover:** Designed and printed a custom top-shell protective cover shielding the modular quad-perfboard assembly, buck converters, and wiring harness from earth, debris, and direct mechanical impacts during aggressive digging operations.

---

## 📐 Circuit Schematic

<div align="center">
  <img src="photos/schematic.png" width="95%" alt="KiCad Electronics Schematic" /><br/>
  <em>KiCad schematic detailing ESP32 pin mappings, sensor buses, power filtering, and motor/servo actuation stages.</em>
</div>

---

## 🖧 Custom Perfboard Motherboard & Hardware Protection

The primary motherboard is built on a modular composite deck created by joining **four 5cm × 7cm standard perfboards** mechanically using **M2 nuts and bolts**. This bolted multi-board array provides a rigid, vibration-resistant foundation for point-to-point power rails, sensor buses, RF transceivers, and fault isolation:

<div align="center">
  <table>
    <tr>
      <td align="center" width="50%">
        <img src="photos/perfboard_front.jpg" width="95%" alt="Perfboard Front / Component Placement" /><br/>
        <b>Component Plane (Top Side)</b><br/>
        <em>M2 bolted 4-tile perfboard array with ESP32 DevKit, MP1584EN & XL4015 bucks, heatsinked TC1508, screw terminals, JST-XH headers, 5A fuse, and servo bank.</em>
      </td>
      <td align="center" width="50%">
        <img src="photos/perfboard_back.jpg" width="95%" alt="Perfboard Back / Solder Traces" /><br/>
        <b>Trace & Solder Plane (Bottom Side)</b><br/>
        <em>Reinforced copper ground plane, point-to-point interconnect jumpers across the bolted boards, and logic decoupling.</em>
      </td>
    </tr>
  </table>
</div>

### Modular Wiring, Interconnects & Thermal Hardening
* **2S2P High-Current Bus Integration:** 4× 18650 cells wired in a 2-series 2-parallel configuration feed nominal 7.4V with 20A continuous discharge potential into the motherboard, fully eliminating inductive voltage dips and preventing the ESP32 from brownout resetting during heavy maneuvers.
* **Bolted Multi-Board Substrate (M2 Hardware):** Four individual 5cm × 7cm perfboards are mechanically fastened together with M2 nuts and bolts, forming a wide and rigid chassis electronics platform without fragile solder joints bridging structural seams.
* **Modular JST-XH Connectors:**
  * **IMU Interconnect:** Male JST-XH pin headers soldered directly to the perfboard carry the 3.3V logic/I2C lines to a mating female connector harness on the GY-BMI160 sensor.
  * **Auxiliary ESP32-CAM Power:** A dedicated perfboard-soldered male JST-XH connector delivers clean 5.0V logic rail power directly from the MP1584EN buck converter to the camera enclosure harness.
* **Screw Terminal Power & Motor Distribution:**
  * **Battery Bus:** 2× 2-pin PCB screw terminal blocks securely anchor the high-current 7.4V battery input and master return paths without relying on fragile jumper pins.
  * **DC Locomotion:** 4× 2-pin PCB screw terminal blocks connect the left and right drive motor leads directly to the TC1508 motor driver outputs.
* **Active Driver Heat Dissipation:** Under sustained skid-steer scrubbing, the TC1508 motor driver frequently entered internal thermal shutdown. An extruded aluminum heatsink/radiator was thermal-bonded to the IC package, eliminating thermal throttling during stall torque scenarios.
* **MCU Pin Isolation ($4\times 1\,\text{k}\Omega$ Resistors):** Dedicated $1\,\text{k}\Omega$ series current-limiting resistors were installed on every logic trace connecting the ESP32 GPIOs to the TC1508 inputs (`IN1`–`IN4`). If the H-bridge suffers catastrophic shoot-through or internal shorting to the motor supply rail, these resistors prevent high voltage from back-feeding and frying the ESP32 silicon.
* **5A Automotive Blade Fuse:** A standard 5A car blade fuse was soldered directly to the perfboard immediately downstream of the main power switch on the positive line feeding the TC1508 motor rail, isolating full-system shorts and protecting the battery pack from catastrophic damage.

---

## 🧩 Bill of Materials (BOM)

### Current Build (v5.0 Autonomous Dual-Vision Platform)

| Component / Module | Specification / Model | Qty | Description / Role |
|---|---|---|---|
| **Microcontroller** | ESP32 DevKitC V4 (SuooTci / USB) | 1 | Core logic, FreeRTOS tasks, PWM Generation, I2C IMU polling & Micro-ROS bridge |
| **RF Transceivers** | 2.4GHz 3dBi Omni Antennas | 2 | High-gain external antennas on U.FL/SMA pig-tails (1x DevKitC, 1x ESP32-CAM) |
| **IMU Sensor** | Bosch GY-BMI160 (6-DOF) | 1 | 3-axis gyro + 3-axis accelerometer for chassis orientation and terrain profiling |
| **Motor Driver** | TC1508 / MX1508 Dual H-Bridge | 1 | 4-channel DC driver ($2.0\text{V} - 9.6\text{V}$, $1.5\text{A}$ peak/ch) retrofitted with an extruded aluminum radiator |
| **DC Drive Motors** | TT Gearbox DC Motors (3V–9V) | 4 | Chassis locomotion (2x Left, 2x Right in parallel; *all-metal 1:90 6V reduction gearboxes purchased for swap*) |
| **High-Torque Servos** | TowerPro MG995 Metal-Gear | 5 | Actuation: Dual synced arm lift, bucket tilt, claw grip, and phone tilter mechanism |
| **Motherboard Substrate** | 5cm × 7cm Single-Sided Perfboards | 4 | Mechanically bolted together with M2 nuts & bolts into a unified composite deck |
| **Board Assembly Fasteners**| M2 Screws & Hex Nuts | Multi | Mechanical inter-board anchoring linking the four perfboards rigidly |
| **Logic Step-Down** | MP1584EN Buck Converter | 1 | Regulates 7.4V battery pack down to stable 5.0V for ESP32 VIN & ESP32-CAM |
| **Servo Step-Down** | XL4015 High-Current Buck Converter | 1 | High-capacity DC-DC step down (7.4V to 6.0V, $\ge 5\text{A}$) dedicated to MG995 servo array |
| **Logic Isolation** | $1\,\text{k}\Omega$ Resistors (Through-Hole) | 4 | Inline current-limiting protection on MCU-to-TC1508 logic control lines |
| **Circuit Fuse** | Standard Automotive Blade Fuse (5A) | 1 | Car blade fuse soldered directly to perfboard on the TC1508 7.4V battery input rail |
| **Signal Interconnects** | JST-XH 2.54mm Headers & Plugs | 2 | Motherboard male headers + wire plugs for modular IMU bus and ESP32-CAM 5V supply |
| **Power Interconnects** | 2-Pin Screw Terminal Blocks (5.08mm) | 6 | High-current connections (2x Battery bus input, 4x TT DC Motor outputs) |
| **Power Decoupling Caps**| $470\,\mu\text{F}$ Electrolytic ($\ge 16\text{V}$) | 2 | Inductive spike and brownout protection (1x on 7.4V battery rail, 1x on 5.0V logic rail) |
| **Main Battery Pack** | 18650 Li-ion Cells (2S2P / 7.4V Nominal) | 4 | High-discharge power source delivering up to 20A continuous at 7.4V to eliminate MCU resets |
| **Main Power Switch** | Mini 3-Pin SPDT Toggle Switch | 1 | Master battery circuit cutoff switch |
| **Electronics Shell** | 3D-Printed Top Bay Enclosure | 1 | Protective cover shielding motherboard, buck regulators, and wiring harness |
| **Secondary Vision** | ESP32-CAM in Custom 3D Enclosure | 1 | Close-proximity claw inspection; housed in a custom enclosure overcoming beam-mounting limits |
| **Primary Vision** | Articulated Smartphone Tilter Mount | 1 | Dynamic tilting mobile phone rig actuated via 5th MG995 servo on `GPIO 4` |
| **Depth Sensor (Roadmap)** | 8x8 Multizone ToF Matrix Sensor | 1 | Surface profiling and short-range claw docking validation |

---

### Legacy Hardware Donor: [Mihaii2/esp32_bulldozer](https://github.com/Mihaii2/esp32_bulldozer)

| Component / Module | Specification / Model | Qty | Status in Current Project |
|---|---|---|---|
| **Microcontroller** | ESP32 DevKitC V4 | 1 | **Migrated:** Re-flashed with manipulator kinematics and 5-channel servo control. |
| **Motor Driver** | TC1508 Dual H-Bridge | 1 | **Migrated:** Retained with aluminum heatsink and resistor isolation for chassis drive. |
| **DC Motors** | TT Gearbox DC Motors (3V–9V) | 4 | **Migrated:** Reused for chassis drive train (*all-metal 1:90 6V units already purchased for swap*). |
| **IMU Sensor** | Bosch GY-BMI160 (6-DOF) | 1 | **Migrated:** Shifted from balance control to kinematic leveling and terrain detection. |
| **Filter Capacitors** | $470\,\mu\text{F}$ Electrolytic ($\ge 16\text{V}$) | 2 | **Migrated:** Retained for dual-rail bus filtering. |
| **Buck Converter** | MP1584EN | 1 | **Migrated:** Retained as dedicated logic rail regulator. |
| **Tilt Switch** | SW-520D Ball Switch | 1 | **Deprecated:** Retired in favor of 6-DOF IMU telemetry. |
| **Dynamic Inversion Logic**| Auto-Flip Firmware Controller | — | **Deprecated:** Replaced by kinematic trajectory control for arm and bucket. |

---

## 🛠 Features
- **Articulated High-Torque Clamping:** 4× MG995 servos providing dual-arm lifting, cup tilting, and active payload grabbing.
- **Dynamic Perception Mast:** 5th MG995 servo driving an adjustable-pitch smartphone cradle alongside a dedicated, encased ESP32-CAM for dual-angle perception.
- **2S2P Anti-Brownout Power Bus:** 4× 18650 cell array supplying 20A continuous headroom at 7.4V, resolving previous ESP32 logic resets during high-strain claw and track maneuvers.
- **Reinforced M2-Bolted Backbone:** 4-piece perfboard array rigidly locked with M2 hardware, integrated with screw terminals and keyed JST-XH wiring harnesses for modular serviceability.
- **High-Gain RF Connectivity:** Dual 3dBi external antennas eliminating packet drop on high-bandwidth ROS 2 telemetry and camera streams.
- **Debris & Impact Shielding:** Custom 3D-printed electronics housing protecting motherboards, buck converters, and connections against soil fallout.
- **Fail-Safe Electrical Design:** Perfboard-soldered 5A car blade fuse, $4\times 1\,\text{k}\Omega$ MCU gate isolation resistors, and an aluminum thermal radiator preventing driver burnout.
- **Dual-Rail Isolated Power Distribution:** High-current XL4015 buck converter isolates heavy servo inductive back-EMF from the ESP32 logic rail.
- **Calibrated Skid-Steer Kinematics:** Asymmetrical 100/0 PWM steering regime breaks track scrubbing, supplemented by 20mm scale-matched wheel collisions.
- **Concentric Axis Kinematics:** Revision 4 unibody geometry maintains zero rotational shear between retention cup and claw swing arcs.
- **Native Micro-ROS Integration:** ESP32 communicates deterministic low-latency telemetry and joint states natively to ROS 2 Jazzy over XRCE-DDS.
- **Dual-Core FreeRTOS Execution:** Decoupled architecture isolating high-speed I2C sensor/PWM routines from network communication.

---

## 🔌 Hardware Architecture & Power Routing

```mermaid
graph TD
    Batt["4x 18650 Battery Array<br/>(2S2P Topology, 7.4V Nominal, 20A Headroom)"] --> Switch["3-Pin Toggle Switch"]
    Switch --> ScrewBatt["2x Screw Terminals<br/>(Main 7.4V Bus)"]
    
    subgraph PowerDistribution["Modular Deck (4x 5x7cm Perfboards Bolted with M2 Hardware)"]
        ScrewBatt --> Cap1["470uF Bulk Filter"]
        Cap1 --> XL4015["XL4015 Buck Converter<br/>(Steps down to 6.0V, 5A Peak)"]
        Cap1 --> MP1584["MP1584EN Buck Converter<br/>(Steps down to 5.0V)"]
        ScrewBatt --> Fuse["5A Car Blade Fuse<br/>(Soldered to Perfboard)"]
        MP1584 --> Cap2["470uF Logic Filter"]
        MP1584 --> JST_Cam["JST-XH 5V Power Connector"]
    end

    subgraph Actuators["High-Current Actuation"]
        XL4015 -->|"6.0V Rail"| Servos["5x MG995 Servos<br/>(Arm Sync, Cup, Claw, Phone Tilter)"]
        Fuse -->|"Protected 7.4V Rail"| TC1508["TC1508 H-Bridge<br/>(+ Radiator / Heatsink)"]
        TC1508 --> ScrewMotors["4x Screw Terminals"]
        ScrewMotors --> TT_Motors["4x TT Motors<br/>(Skid-Steer Locomotion)"]
    end

    subgraph LogicAndSensors["Logic & Processing Plane (Cover Enclosure)"]
        Cap2 -->|"5.0V VIN"| ESP32["ESP32 DevKitC V4<br/>(+ 3dBi Antenna)"]
        ESP32 -->|"JST-XH Harness (3.3V/I2C)"| BMI160["Bosch GY-BMI160 (6-DOF IMU)"]
        ESP32 -->|"PWM Control (LEDC)"| Servos
        ESP32 -->|"4x 1k Series Resistors"| TC1508
        ESP32 -.->|"Future Expansion"| ToF["8x8 ToF Matrix Sensor"]
        JST_Cam --> ESP32Cam["ESP32-CAM in Custom Enclosure<br/>(+ 3dBi Antenna)"]
        ESP32Cam -.->|"WLAN Stream"| ROS2["ROS 2 Ecosystem"]
        Phone["Smartphone (Tilter Mount)"] -.->|"WebRTC / RTSP"| ROS2
    end
```

---

## ⚡ Hardware Pinout (ESP32)

Unified GPIO allocation across motor drive stages, servo PWM channels, and sensor buses:

| Subsystem | Component Pin | ESP32 GPIO / Rail | Peripheral / Channel | Function / Calibration Limits |
|---|---|---|---|---|
| **DC Drive (TC1508)** | `IN1` | `GPIO 23` | GPIO Output via $1\,\text{k}\Omega$ Resistor | Right Track Motor (Direction A) |
| **DC Drive (TC1508)** | `IN2` | `GPIO 22` | GPIO Output via $1\,\text{k}\Omega$ Resistor | Right Track Motor (Direction B) |
| **DC Drive (TC1508)** | `IN3` | `GPIO 21` | GPIO Output via $1\,\text{k}\Omega$ Resistor | Left Track Motor (Direction A) |
| **DC Drive (TC1508)** | `IN4` | `GPIO 19` | GPIO Output via $1\,\text{k}\Omega$ Resistor | Left Track Motor (Direction B) |
| **Servos (MG995)** | Arm Base Left | `GPIO 18` | `LEDC_CHANNEL_0` | `132.0°` (Down / 0%) to `44.0°` (Up / 100%) |
| **Servos (MG995)** | Arm Base Right | `GPIO 5` | `LEDC_CHANNEL_1` | `40.0°` (Down / 0%) to `124.0°` (Up / 100%) *(Mirrored sync)* |
| **Servos (MG995)** | Cup / Wrist Tilt | `GPIO 17` | `LEDC_CHANNEL_2` | Independent Pitch Mechanism |
| **Servos (MG995)** | Claw Gripper | `GPIO 16` | `LEDC_CHANNEL_3` | Independent Grip / Release Mechanism |
| **Servos (MG995)** | Phone Tilter | `GPIO 4` | `LEDC_CHANNEL_4` | Actuated Camera / Phone Pitch Control |
| **IMU (BMI160)** | `SCL` | `GPIO 32` | `I2C_NUM_0` SCL via JST-XH | Hardware I2C Clock Line[cite: 1] |
| **IMU (BMI160)** | `SDA` | `GPIO 25` | `I2C_NUM_0` SDA via JST-XH | Hardware I2C Data Line[cite: 1] |
| **IMU (BMI160)** | `CS` / `SA0` | `3.3V Rail` | Logic High via JST-XH | Forces I2C Mode / Sets Address to `0x69`[cite: 1] |

> *Note: All MG995 servos utilize `LEDC_TIMER_0` configured for 50 Hz PWM with 14-bit resolution.*

---

## 🚀 Getting Started

### Prerequisites
- Ubuntu 22.04 / 24.04
- ROS 2 (Jazzy / Humble)
- ESP-IDF v5.2+
- Docker (for micro-ROS agent)

```bash
### Build & Run
# Clone and enter workspace
cd ~/ROS_projects/claw_bulldozer_ws

# Build the ROS 2 packages
colcon build --symlink-install

# Source the overlay
source install/setup.bash

# Run main bringup script
./run.sh

# Source ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Navigate to firmware project
cd ~/ROS_projects/claw_bulldozer_ws/firmware/flipper_firmware

# Build, flash to ESP32, and launch UART monitor
idf.py build flash monitor

# Start micro-ROS Agent (in separate terminal)
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
```