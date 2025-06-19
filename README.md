<<<<<<< HEAD
# plub_plub

---

© 2025 Asil Arnous. All Rights Reserved.

No part of this code may be used, reproduced, or distributed in any form without express written permission.

<!--
  ╔════════════════════════════════════════════════════════════════╗
  ║                                                                ║
  ║      🌟 Welcome to **Project Aurora**: The Dawn of Innovation   wohoo       ║
  ║                                                                          ║
  ╚════════════════════════════════════════════════════════════════╝
-->

<p align="center">
  <img src="https://raw.githubusercontent.com/username/repo/main/assets/banner.png" alt="Project Aurora" width="800"/>
</p>

<p align="center">
  <a href="#rocket-features">🚀 Features</a> •
  <a href="#installation">⚙️ Installation</a> •
  <a href="#usage">💡 Usage</a> •
  <a href="#roadmap">🗺️ Roadmap</a> •
  <a href="#contributing">🤝 Contributing</a> •
  <a href="#license">📜 License</a>
</p>

<p align="center">
  <img alt="Version" src="https://img.shields.io/badge/version-1.0.0-00bfff.svg" />
  <img alt="License: MIT" src="https://img.shields.io/badge/license-MIT-green.svg" />
  <img alt="PRs Welcome" src="https://img.shields.io/badge/PRs-welcome-brightgreen.svg" />
  <img alt="Stars" src="https://img.shields.io/github/stars/username/repo?style=social" />
</p>

---

## 🎬 Live Demo

<p align="center">
  <a href="https://demo.project-aurora.com"><strong>View it in your browser »</strong></a>
  <br/><br/>
  <img src="https://media.giphy.com/media/3o7aCTfyhYawdOXcFW/giphy.gif" alt="Demo GIF" width="600"/>
</p>

---

## 📋 Table of Contents

1. [🚀 Features](#rocket-features)  
2. [⚙️ Installation](#installation)  
3. [💡 Usage](#usage)  
4. [🗺️ Roadmap](#roadmap)  
5. [🤝 Contributing](#contributing)  
6. [📜 License](#license)  
7. [✉️ Contact](#contact)  

---

## 🚀 Features

- **Blazing Performance** — Optimized algorithms delivering sub-millisecond responses.  
- **Modular Architecture** — Plug-and-play modules for every use case.  
- **Responsive UI** — Pixel-perfect layouts across all devices.  
- **Robust Security** — Industry-grade encryption and continuous vulnerability scanning.  
- **Extensive Documentation** — Step-by-step guides, code samples, and FAQs.  

> 💡 **Tip:** Tailor each feature above to highlight your project’s unique selling points!

---

## ⚙️ Installation

1. **Clone the repo**  
   ```bash
   git clone https://github.com/username/repo.git
   cd repo
=======
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
>>>>>>> 3c11c9864d18f234fb81bf437f364af2d23b27f3
