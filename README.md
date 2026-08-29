# SynMech MoBot

Differential drive mobile robot platform based on ROS Noetic, STM32 Blue Pill, and Orange Pi PC+. Designed for indoor SLAM and autonomous navigation.

## Hardware

| Component | Model | Interface |
|---|---|---|
| SBC | Orange Pi PC+ (ARM) | Host |
| MCU | STM32F103C8T6 (Blue Pill) | USB-Serial to OPi |
| Lidar | RPLidar A1M8 | USB `/dev/rplidar` |
| IMU | MPU6050 | I2C (0x68) |
| Motor Driver | TB6612FNG | GPIO + PWM |
| Controller | PS2 Wireless (optional) | SPI to STM32 |
| Wheels | Differential drive | Track width: 168mm, Wheel radius: 21.5mm |

## Repository Structure

```
synmech_mobot/
├── docker/                     # Docker deployment for Orange Pi
│   ├── Dockerfile.opi
│   ├── docker-compose.yml
│   └── entrypoint.sh
├── synmech_bringup/            # ROS launch files, controllers, configs
│   ├── launch/
│   │   ├── bringup.launch      # Core: URDF + Lidar + Odometry
│   │   ├── slam.launch         # bringup + Gmapping SLAM
│   │   ├── slam_view.launch    # Laptop: RViz + Teleop keyboard
│   │   └── robot_base.launch   # Standalone motor test
│   ├── scripts/
│   │   ├── fake_odom.py        # Phase 1: Fake odometry (no hardware)
│   │   └── calibrate_pwm.py   # PWM-to-velocity calibration tool
│   ├── src/
│   │   └── base_controller.py  # Phase 2: Real STM32 + IMU odometry
│   └── config/
│       ├── calibration.yaml    # PWM↔velocity mapping data
│       ├── slam.rviz           # RViz layout for SLAM
│       └── 99-synmech-usb.rules # Udev rules for USB devices
├── synmech_description/        # URDF/Xacro robot model
├── synmech_navigation/         # Navigation stack (move_base, AMCL)
├── synmech_sim/                # Gazebo simulation
└── synmech_stm32/              # PlatformIO firmware (Arduino framework)
    ├── platformio.ini
    ├── src/main.cpp
    └── lib/
        ├── Config/             # Hardware pin definitions
        ├── MotorDriver/        # TB6612FNG control
        ├── IMUSensor/          # MPU6050 with anti-drift filtering
        ├── SerialComm/         # UART protocol (OPi ↔ STM32)
        ├── PS2Controller/      # PS2 wireless gamepad
        └── Encoder/            # (Optional) wheel encoders
```

## Quick Start

### 1. Deploy to Orange Pi

```bash
# SSH into Orange Pi
ssh huywros@<OPI_IP>

# Clone and launch
cd ~/catkin_ws/src/synmech_mobot/docker
docker compose up
```

### 2. Connect from Laptop

```bash
source ~/catkin_ws/src/synmech_mobot/network.env
export ROS_MASTER_URI=http://${OPI_IP}:${ROS_PORT}
export ROS_IP=${LAPTOP_IP}
roslaunch synmech_bringup remote_view.launch
```

### 3. Flash STM32 Firmware

```bash
# On development machine with PlatformIO
cd synmech_stm32
pio run --target upload
```

---

## Development Phases

### Phase 1 — Fake Odometry (No STM32 required)

Uses `fake_odom.py` to provide odometry from `/cmd_vel` integration. Suitable for testing SLAM with manual robot movement.

**bringup.launch** uses: `fake_odom.py`

**slam.launch** Gmapping config:

| Parameter | Value | Description |
|---|---|---|
| `temporalUpdate` | **1.0** | Force scan processing every 1s (compensates for fake odom not knowing about manual movement) |
| `minimumScore` | **50** | Permissive scan matching threshold |
| `particles` | **30** | Number of particle filter hypotheses |

> **Tip**: Move the robot slowly (< 10°/sec rotation) to allow scan matching to converge between updates.

---

### Phase 2 — STM32 + MPU6050 (Current)

Uses `base_controller.py` connected to STM32 via Serial. MPU6050 provides accurate heading (yaw), linear velocity is estimated from commanded speed + calibration table.

**bringup.launch** uses: `base_controller.py`

**To switch from Phase 1 to Phase 2:**
1. In `bringup.launch`: Comment out `fake_odom.py`, uncomment `base_controller.py`
2. In `slam.launch`: Change these parameters:

| Parameter | Phase 1 | Phase 2 | Why |
|---|---|---|---|
| `temporalUpdate` | 1.0 | **-1.0** | Real odom triggers updates automatically |
| `minimumScore` | 50 | **200** | Stricter matching — real odom gives good initial estimates |
| `particles` | 30 | **30** | Can remain the same, increase to 60 if map quality is poor |

---

### Phase 3 — Navigation (Planned)

Uses saved map + AMCL localization + move_base for autonomous path planning.

---

## UART Protocol (OPi ↔ STM32)

### Commands: OPi → STM32

| Command | Format | Example | Description |
|---|---|---|---|
| Motor | `M,<pwmL>,<pwmR>\n` | `M,150,-150\n` | Set wheel PWM (-255 to 255) |
| Calibrate IMU | `C\n` | `C\n` | Recalibrate MPU6050 gyro offset |
| Emergency Stop | `S\n` | `S\n` | Stop both motors immediately |
| PS2 Mode | `PS2\n` | `PS2\n` | Switch motor input to PS2 controller |
| Serial Mode | `SER\n` | `SER\n` | Switch motor input to serial commands |

### Telemetry: STM32 → OPi

| Message | Format | Example | Description |
|---|---|---|---|
| IMU Data | `D,<yaw_deg>\n` | `D,-45.25\n` | Yaw angle in degrees (50Hz) |
| Calibrate ACK | `A\n` | `A\n` | IMU calibration completed |
| Error | `E,<code>\n` | `E,1\n` | 1=IMU fail, 2=Motor fault |

> **Note**: Telemetry is always sent regardless of input mode. PS2 mode only changes where motor commands come from — IMU data continues flowing to OPi via serial.

## PS2 Controller

| Input | Action |
|---|---|
| Left Stick Y | Forward / Backward |
| Left Stick X | Turn Left / Right |
| L1 (hold) | Slow mode (50% speed) |
| R1 (hold) | Boost mode (200% speed) |
| SELECT | Emergency Stop |

Joystick uses **quadratic mapping** for fine control: small stick deflections produce very small velocities, while full deflection reaches max speed. Dead zone of ±30 around center prevents drift.

## PWM Calibration

Since there are no wheel encoders, velocity estimation relies on a pre-measured PWM-to-velocity mapping.

```bash
# Run calibration tool (connect STM32 first)
python3 synmech_bringup/scripts/calibrate_pwm.py --port /dev/stm32

# The script will:
# 1. Drive at various PWM values (50, 80, 100, ... 255)
# 2. Ask you to measure distance traveled
# 3. Compute linear regression: velocity = slope * PWM + intercept
# 4. Save results to config/calibration.yaml
```

## Gmapping SLAM Parameters Reference

Key parameters in `slam.launch` that affect map quality:

| Parameter | Default | Range | Effect |
|---|---|---|---|
| `linearUpdate` | 0.1 | 0.05–0.5 | Min distance (m) before processing a scan |
| `angularUpdate` | 0.1 | 0.05–0.5 | Min rotation (rad) before processing a scan |
| `temporalUpdate` | 1.0 | -1.0 to 5.0 | Force update every N seconds (-1.0 = disabled) |
| `minimumScore` | 50 | 0–1000 | Min scan match score to accept. Higher = stricter |
| `particles` | 30 | 10–100 | More = better accuracy, more CPU/RAM |
| `lstep` | 0.05 | 0.01–0.2 | Scan matcher linear search step (m) |
| `astep` | 0.05 | 0.01–0.2 | Scan matcher angular search step (rad) |
| `maxUrange` | 6.0 | 1.0–12.0 | Max usable laser range for matching (m) |
| `maxRange` | 8.0 | 1.0–12.0 | Max laser range for map insertion (m) |

## Udev Rules

To create stable device names that persist across reboots:

```bash
# Copy rules to system
sudo cp synmech_bringup/config/99-synmech-usb.rules /etc/udev/rules.d/

# Reload and trigger
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify
ls -la /dev/rplidar /dev/stm32
```

Find your device IDs with: `lsusb` or `udevadm info --name=/dev/ttyUSB0 --attribute-walk`

## Docker Architecture

```
docker compose up
  ├── synmech_roscore    → roscore (port 2710)
  └── synmech_bringup    → slam.launch (Lidar + Odom + Gmapping)
       ├── /dev:/dev mounted (privileged) for hardware access
       └── Source code volume-mounted for live editing
```

Debug shell: `docker compose run --rm shell`

## License

MIT
