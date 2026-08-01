# EuroBot 2026 - TicTacBot

![PlatformIO](https://img.shields.io/badge/PlatformIO-2.0+-orange.svg)
![ESP32](https://img.shields.io/badge/ESP32-S3-blue.svg)
![Eurobot](https://img.shields.io/badge/Eurobot-2026-red.svg)

> **⚠️ Project migrated to**: [this repo](https://github.com/Ibtihal-mrn/TicTac).

This repository contains the embedded firmware for our fully autonomous robot, built for the **Eurobot 2026 competition** held in Belgium.

Eurobot is an international amateur robotics contest where two robots face off in a themed arena to autonomously collect, place, and manipulate objects within a 90-second match. No remote controls—just pure onboard intelligence.

## 🧠 The Stack
- **MCU**: ESP32 (master controller) – handles real-time decision making, sensor fusion, and motor/servo actuation.
- **Framework**: PlatformIO (VS Code) – for cross-platform build management and library dependency handling.
- **Language**: C++ (Arduino framework / ESP-IDF mix) – optimized for low-latency control loops.
- **CI/CD**: GitHub Actions (`.github/agents`) – automated builds and tests on every push to keep the firmware battle-ready.

## ⚙️ Key Challenges Addressed
- **State-machine architecture** to handle the multi-phase match timeline (start, positioning, action sequences, endgame).
- **Sensor integration** (line-following, distance/IR, color detection, and encoders) for precise localization and object interaction.
- **Real-time motor control** – smooth PID loops to navigate the arena at speed without overshooting.

## 🏗️ Repository Structure
- **`esp32_master/`** – Main firmware for the onboard master controller. Contains all core logic, sensor drivers, and motion control.
- **`.github/agents/`** – GitHub Actions workflows for continuous integration (building, linting, and automated firmware validation).

## 🚀 Getting Started (for local development)
1. Clone the repo:
```bash
git clone https://github.com/Muten-Roshi-Sama/EuroBot_2026.git
```
2. Open the esp32_master folder in VS Code with the PlatformIO extension installed.
3. Run pio run to build the firmware.
4. Upload to your ESP32 with pio run --target upload.

---
Made with ❤️, sleepless nights, and a lot of Belgian chocolate by our EuroBot Team.
