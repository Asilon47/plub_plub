# **plub_plub – A Modular Hardware/Software Stack for Mobile Robotic Research**

> **Author:** Asil Arnous  
> **Institution:** *Universidad de Deusto*  
> **License:** *Proprietary Software License v1.0* (see full text at the end of this document)

---

## 1. Overview and Research Motivation
`plub_plub` aggregates the mechanical, electronic, and cyber-physical artefacts required to study advanced feedback control, sensor fusion, and on-board autonomy for differential-drive mobile robots. The repository is intentionally partitioned so that each subsystem can be analysed, verified, and reproduced in isolation before being integrated into the complete ROS 2 ecosystem.

---

## 2. Repository Topology

| Directory / File                              | Purpose (concise)                                                                                                                                                 |
|-----------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **`MCU/`**                                    | Bare-metal firmware for the *STM32F4* micro-controller unit (MCU). Written in C 17, organised as a conventional *STM32CubeMX* project with `Core/` and `Drivers/`.|
| **`plub_plub_ws/`**                           | Colcon-driven ROS 2 workspace (*Humble* LTS), including all message definitions, nodes, launch files, and unit tests (`src/`, `launch/`, `test/`).                |
| **`motors_system_analysis/`**                 | MATLAB\* measurements (🄫 MATLAB R2024b) for parameter identification and closed-loop eigen-analysis of the traction motors.                                       |
| **`cad/`** *(or similar, e.g. `robot_cad/`)*  | Parametric CAD assemblies (STEP & Parasolid) plus 2-D technical drawings of the chassis, sensor mounts, and battery housing.                                     |
| **`.gitignore`, `LICENSE`, `README.md`**      | Conventional project metadata.                                                                                                                                    |

\*The MATLAB and CAD artefacts are **not** elaborated further in this README; they are retained for reproducibility of the mechanical and system-identification studies.

---

## 3. MCU Firmware: Build and Flash Workflow

### 3.1 Toolchain Prerequisites

| Component | Minimum Version | Rationale |
|-----------|-----------------|-----------|
| **GNU Arm Embedded Toolchain** | 12.3-Rel1 | Deterministic cross-compilation |
| **CMake** | ≥ 3.25 | Out-of-source multi-config build |
| **OpenOCD** *or* **STM32CubeProgrammer** | 0.12.0 / 2.15 | SWD flashing and debugging |
| **stlink-tools** (optional) | 1.7 | Flash via `st-flash` |

Install on Ubuntu 22.04:

```bash
sudo apt update
sudo apt install gcc-arm-none-eabi cmake make openocd stlink-tools
```

### 3.2 Building

```bash
cd MCU
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

The resulting ELF (`plub_plub.elf`) and binary (`plub_plub.bin`) images appear in `MCU/build/`.

### 3.3 Flashing

**Option A — OpenOCD session**

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/plub_plub.elf verify reset exit"
```

**Option B — ST-LINK command-line**

```bash
st-flash write build/plub_plub.bin 0x08000000
```

Confirm successful programming via:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "mdw 0x08000000 4"
```

---

## 4. ROS 2 Workspace Configuration

### 4.1 System Dependencies

```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions \
                 ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-rosbridge-suite can-utils
```

### 4.2 Workspace Build

```bash
cd plub_plub_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4.3 Launching the Full Stack

```bash
ros2 launch plub_bringup base.launch.py use_can:=true mcu_baud:=115200
```

The launch file orchestrates:

1. **`mcu_bridge_node`** – DDS ↔ UART gateway to the STM32 running `rosserial_transport`.  
2. **`base_controller`** – `diff_drive_controller` from `ros2_control`.  
3. **Sensor drivers** – IMU, wheel encoders, and optional depth camera.

---

## 5. Operating-System Level Configuration

### 5.1 Serial and USB Permissions

| Immediate (non-persistent) | Persistent (`udev` rule) |
|---------------------------|---------------------------|
| `sudo chmod 777 /dev/ttyUSB0` | Create `/etc/udev/rules.d/99-plub.rules` with: <br>`SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"` <br>then reload rules: <br>`sudo udevadm control --reload-rules && sudo udevadm trigger` |

### 5.2 CAN FD Channel Initialisation

```bash
sudo ip link set can1 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can1 up
```

Validate with:

```bash
candump can1
```

---

## 6. Experimental Reproduction and Scholarly Use

All experiments reported in the companion thesis can be reproduced by following these sequential steps:

1. **Mechatronic Assembly:** Refer to the CAD drawings for exact hole patterns and material specifications (Section 2).  
2. **Controller Deployment:** Flash the MCU as per Section 3.3, ensuring the firmware commit hash matches the one cited in the manuscript.  
3. **Middleware Bring-up:** Build and source the ROS 2 workspace (Section 4.2) on an Ubuntu 22.04 host.  
4. **Communication Sanity Checks:**  
   * `ros2 topic echo /joint_states` — confirms UART bridge integrity.  
   * `cansend can1 …` / `candump can1` — validates CAN timing budget.  
5. **Closed-Loop Trials:** Launch the full stack (Section 4.3) and execute the provided ROS bags for benchmark trajectories (`ros2 bag play bags/figure8`).  

---

## 7. Citing This Repository

> **A. Arnous**, “**plub_plub: A Modular ROS 2 Platform for Investigating Embedded Motor Control and Sensor Fusion**,” *Trabajo Fin de Grado*, Universidad ___, 2025. GitHub: <https://github.com/Asilon47/plub_plub>

---

## 8. License

```
Proprietary Software License v1.0
--------------------------------------------------
This software is licensed, not sold. You have no rights to use, copy, modify, or distribute this software in any form. All rights are reserved by Asil Arnous, 2025.
```

*For academic evaluation purposes only. Commercial or open-source redistribution is strictly prohibited.*
