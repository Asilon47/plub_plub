@@ -1,4 +1,4 @@
# **plub_plub – A Modular Hardware/Software Stack for Mobile Robotic Research**
# **plub_plub – A Modular Hardware/Software Stack for Mobile Robotic**

> **Author:** Asil Arnous  
> **Institution:** *Universidad de Deusto*  
> **License:** *Proprietary Software License v1.0* (see full text at the end of this document)
---
## 1. Overview
`plub_plub` aggregates the mechanical, electronic, and software artefacts required to study feedback control, sensor fusion, and on-board autonomy for mecanum wheeled mobile robots. The repository is intentionally partitioned so that each subsystem can be analysed, verified, and reproduced in isolation before being integrated into the complete ecosystem.
---
## 2. Repository Topology
| Directory / File                              | Purpose (concise)                                                                                                                                                 |
|-----------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **`MCU/`**                                    | Bare-metal firmware for the *STM32F4* micro-controller unit (MCU). Written in C 17, organised as a conventional *STM32CubeMX* project with `Core/` and `Drivers/`.|
| **`plub_plub_ws/`**                           | Colcon-driven ROS 2 workspace (*Humble* LTS), including all message definitions, nodes and launch files.                |
| **`motors_system_analysis/`**                 | MATLAB\* measurements (🄫 MATLAB R2024b) for parameter identification and closed-loop analysis of the  motors.                                       |
| **`cad/`**   | CAD assemblies.                                     |
                                                                                                                                  |
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
colcon build --symlink-install
source ./install/setup.bash
```
### 4.3 Launching the Full Stack
```bash
ros2 launch run run.launch.py
```
---
## 5. Operating-System Level Configuration
### 5.1 Serial and USB Permissions
```bash
sudo chmod 777 /dev/ttyUSB0`
lsusb \
  | awk '/03e7/ { gsub(/:$/,"",$4); printf "/dev/bus/usb/%03d/%03d\n", $2, $4 }' \
  | xargs sudo chmod 666
```
### 5.2 CAN FD Channel Initialisation
```bash
sudo ip link set can1 type can bitrate 500000
sudo ip link set can1 up
```
Validate with:
```bash
candump can1
```
---
## 6. License
```
Proprietary Software License v1.0
--------------------------------------------------
This software is licensed, not sold. You have no rights to use, copy, modify, or distribute this software in any form. All rights are reserved by Asil Arnous, 2025.
```
*For academic evaluation purposes only. Commercial or open-source redistribution is strictly prohibited.*