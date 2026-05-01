# 🦀 CrabBot — Quadruped Walking Robot

A crab-inspired quadruped robot built with a Raspberry Pi 4 and LX-16A serial bus servos. The robot features 8 motors (2 per leg), actuator feedback-based gait optimization, and supports forward, backward, and sideways (crab) walking.

---

## 📸 Robot Overview

| Spec | Value |
|------|-------|
| Platform | Raspberry Pi 4 |
| Motors | 8x LewanSoul LX-16A Serial Bus Servos |
| DOF | 2 per leg (yaw + pitch) |
| Communication | BusLinker (CH340 USB-Serial) |
| Power | 12V LiPo battery + buck converter |
| Control | Python 3 over SSH / VS Code Remote |

---

## 🦵 Leg Layout

```
    FL ──── FR
    │        │
    BL ──── BR

Each leg:
  YAW   motor = swings leg forward/backward or sideways
  PITCH motor = lifts/lowers leg
```

## 🔌 Motor ID Map

| ID | Name | Function |
|----|------|----------|
| 1 | FL_PITCH | Front-left lift |
| 2 | FL_YAW | Front-left swing |
| 3 | FR_YAW | Front-right swing |
| 4 | FR_PITCH | Front-right lift |
| 5 | BL_PITCH | Back-left lift |
| 6 | BL_YAW | Back-left swing |
| 7 | BR_YAW | Back-right swing |
| 8 | BR_PITCH | Back-right lift |

---

## 📁 Repository Structure

```
crabbot/
│
├── crab_gait.py          # Main gait controller (run this to walk the bot)
│
├── gait_iterations/      # Earlier versions showing tuning progression
│   ├── gait_v1_initial.py
│   ├── gait_v2_stability.py
│   ├── gait_v3_crabwalk.py
│   ├── gait_v4_smooth.py
│   └── README.md
│
├── scripts/
│   ├── motor_setup.py    # Motor calibration and torque control
│   ├── level.py          # Interactive position setter for leveling
│   ├── read_positions.py # Read all motor positions on demand
│   └── keepalive.py      # SSH session keepalive
│
├── plots/
│   └── plot_gait.py      # Generate performance plots from gait_log.csv
│
├── docs/
│   ├── wiring.md         # Wiring guide
│   ├── calibration.md    # Calibration procedure
│   └── citations.md      # References and attribution
│
└── README.md
```

---

## ⚙️ Installation

### 1. Clone the repo on your Pi
```bash
git clone https://github.com/YOUR_USERNAME/crabbot.git
cd crabbot
```

### 2. Create and activate virtual environment
```bash
python3 -m venv ~/robotenv
source ~/robotenv/bin/activate
```

### 3. Install dependencies
```bash
pip install git+https://github.com/ethanlipson/PyLX-16A.git
pip install matplotlib pandas
```

### 4. Check serial port
```bash
ls /dev/ttyUSB*
# Should show /dev/ttyUSB0
```

---

## 🚀 Running the Robot

```bash
source ~/robotenv/bin/activate
python3 crab_gait.py
```

### Controls

| Key | Action |
|-----|--------|
| `w` | Walk forward (3 steps) |
| `s` | Walk backward (3 steps) |
| `a` | Crab walk left (3 steps) |
| `d` | Crab walk right (3 steps) |
| `n` | Return to neutral position |
| `f` | Print live motor feedback |
| `q` | Safe shutdown |

---

## 🧠 Gait Design

### Trot Gait (Forward/Backward)
Diagonal pairs move together:
- **Pair A**: FL + BR
- **Pair B**: FR + BL

Each step has 3 phases:
1. Pair A lifts and swings forward, Pair B pushes back
2. Pair A plants, Pair B lifts and swings forward
3. Both pairs return toward neutral

### Crab Walk (Sideways)
Left or right side lifts and shifts laterally while the other side supports:
1. FL + BL lift and swing sideways
2. FL + BL plant
3. FR + BR lift and catch up
4. FR + BR plant

### Actuator Feedback
The controller reads motor position and load after each phase:
- **Load > 200** = leg is planted on ground
- **Load < 200** = leg is in the air
- Timing adjusts dynamically based on ground contact confirmation
- All data logged to `gait_log.csv`

---

## 📊 Performance Plots

After walking the robot, generate performance plots:

```bash
python3 plots/plot_gait.py
```

Generates `gait_analysis.png` with:
- Pitch/Yaw motor positions over time
- Motor load (torque proxy) over time
- Step cycle timing (speed proxy)
- Ground contact heatmap per phase

---

## 🔧 Calibration

See [docs/calibration.md](docs/calibration.md) for full calibration procedure.

Quick start:
```bash
python3 scripts/motor_setup.py
```

---

## ⚡ Wiring

See [docs/wiring.md](docs/wiring.md) for full wiring diagram and instructions.

---

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| `No such file or directory: /dev/ttyUSB0` | Replug BusLinker, run `lsusb` to verify CH340 detected |
| Motor TIMEOUT | Check 12V battery is connected and powered |
| Bot leaning to one side | Run `python3 scripts/level.py` and adjust pitch neutrals |
| Negative motor positions | Horn needs remounting — see calibration guide |
| SSH disconnecting | Run `python3 scripts/keepalive.py` in second terminal |

---

## 📚 Citations and Attribution

### Hardware

[1] LewanSoul, "LX-16A Serial Bus Servo User Manual," LewanSoul Inc., 2019.
Available: https://www.lewansoul.com/product/detail-17.html

[2] Raspberry Pi Foundation, "Raspberry Pi 4 Model B Datasheet," 2019.
Available: https://www.raspberrypi.com/products/raspberry-pi-4-model-b/

[3] QinHeng Electronics, "CH340 USB to Serial Converter Datasheet," WCH, 2019.
Available: http://www.wch-ic.com/products/CH340.html

### Software Libraries

[4] E. Lipson, "PyLX-16A: Python library for LewanSoul LX-16A Serial Bus Servos," GitHub, 2021.
Available: https://github.com/ethanlipson/PyLX-16A

[5] Python Software Foundation, "Python 3 Language Reference," 2023.
Available: https://www.python.org/

[6] J. D. Hunter, "Matplotlib: A 2D Graphics Environment," Computing in Science & Engineering, vol. 9, no. 3, pp. 90-95, 2007.
Available: https://matplotlib.org/

[7] W. McKinney, "Data Structures for Statistical Computing in Python," Proceedings of the 9th Python in Science Conference, pp. 56-61, 2010.
Available: https://pandas.pydata.org/

[8] Microsoft, "Visual Studio Code Remote SSH Extension," 2023.
Available: https://code.visualstudio.com/docs/remote/ssh

[9] Raspberry Pi Foundation, "Raspberry Pi OS," 2023.
Available: https://www.raspberrypi.com/software/

### Robotics & Gait Research

[10] H. Kimura, Y. Fukuoka, and A. H. Cohen, "Adaptive dynamic walking of a quadruped robot on natural ground based on biological concepts," The International Journal of Robotics Research, vol. 26, no. 5, pp. 475-490, 2007.

[11] C. D. Bellicoso et al., "Advances in real-world applications for legged robots," Journal of Field Robotics, vol. 35, no. 8, pp. 1311-1326, 2018.

[12] M. H. Raibert, "Legged robots that balance," MIT Press, Cambridge, MA, 1986.

[13] R. M. Alexander, "Principles of Animal Locomotion," Princeton University Press, 2003.

[14] D. Wooden et al., "Autonomous navigation for BigDog," IEEE International Conference on Robotics and Automation (ICRA), 2010.

### Tools

[15] GitHub, "GitHub — Code hosting platform," 2023.
Available: https://github.com/

[16] Git, "Git — Distributed version control system," 2023.
Available: https://git-scm.com/

---

## 📄 License

MIT License — free to use and modify.
