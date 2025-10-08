# 🤖 Eurobot 2026 – Team Robot Project

## Overview
This repository contains the complete source code, hardware assets, and collaborative work for our **Eurobot 2026** robot.  
The project is built entirely in **C++ (Arduino)** and structured to support modular development for each subsystem.

Our goal is to design, build, and program a fully autonomous robot capable of completing the Eurobot challenge efficiently and reliably.

---

## 🧩 Project Structure
```
eurobot-2026/
│
├── 3d_files         # 3D printable parts and mechanical designs
│
├── src/
│   ├── main.ino      # Arduino IDE
│   └── main.cpp      # PlatformIO
├── lib/
│   ├── movement/          # Controlled Movement System
│   ├── servo_mechanism/   # Grabbing Mechanism
│   ├── thermometer/       # Cursor Movement Mechanism
│   ├── detection/         # Sensors and color detection
│   ├── mapping/           # Board Matrix Mapping
│   ├── fsm/               # Finite State Machine
│   └── hardware/          # Hardware setup, pins, assembly
│
└── README.md
```

Each library contains its own `.cpp` and `.h` files, making it easy to test and integrate independently.

---

## 🧠 Development Workflow

### Branching Model
- `main` → stable, competition-ready robot code  
- `dev` → integration branch for tested features  
- `feature/...` → one branch per subsystem (owned by its team)

Example branches:
- feature/movement
- feature/servo
- feature/thermometer
- feature/detection
- feature/mapping
- feature/fsm
- feature/hardware


### 🧰 Setup Instructions for Collaborators

**1. Clone the repository**
```bash
# Clone the repository
git clone https://github.com/Muten-Roshi-Sama/EuroBot_2026
cd EuroBot_2026
```
**2. Switch to your assigned branch**
```
git checkout -b feature/<your-feature> origin/feature/<your-feature>
# Example
git checkout -b feature/movement origin/feature/movement
```
**3. Work in your module folder**
Example:
```
lib/movement/
├── Movement.h
└── Movement.cpp
```
**4. Commit and push regularly**
```bash
git add .
git commit -m "Implement PID control for movement"
git push origin feature/movement
```
**5. Merge process**
Open a Pull Request (PR) to merge into dev
Felix (and Vass) will review and test integration
Once stable → merged from dev to main

### 👥 Team Responsibilities

#### 🏃 Actions
| Task | Main Responsible | Contributors |
|------|------------------|--------------|
| Controlled movement system (Encoder) | Ibtihal | Adam |
| Servo-powered plow / grabber mechanism | Ayoub | Ibrahim |
| Thermometer cursor | Ibrahim | Ayoub |

#### 🧭 Detection
| Task | Main Responsible | Contributor |
|------|------------------|--------------|
| Obstacle detection and avoidance (IR, Ultrasonic) | Chaimae | – |
| Object color detection | Chaimae | – |
| Board mapping | Ibtihal | – |

#### 🧰 Robot Hardware
| Task | Main Responsible | Contributor |
|------|------------------|--------------|
| 3D printing and mechanical design | Vassily | – |
| Assembly (buttons, switches, start system) | Adam | – |
| FSM (Finite State Machine in C++) | Felix | Vassily |
| Git merge management | Vassily | Adam |



### ⚙️ Tools & Environment
- Language: C++ (Arduino)
- Platform: Arduino IDE / PlatformIO
- Version Control: Git + GitHub
- 3D Design: Fusion 360 / SolidWorks (under 3d_Files/)
- Collaboration: Branch-based workflow with PR reviews


### 🚀 Future Additions
- Add unit testing folder (/tests/)
- Document serial communication and pin layout
- Upload mechanical drawings and wiring schematics
- Add CI workflow to check C++ build status on PRs
---

Maintainer, backup and review: Vass
FSM integration lead: Felix
Hardware coordination: Adam

“Think modular, build stable, test continuously — then dominate Eurobot.” ⚙️🤖

