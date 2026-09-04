# SynMech MoBot

Differential drive mobile robot platform based on **ROS Noetic**, **STM32 Blue Pill**, and **Orange Pi PC+**. Designed for indoor SLAM and autonomous navigation.

---

## Hardware

| Component | Model | Interface |
|---|---|---|
| SBC | Orange Pi PC+ (ARM) | Host — chạy ROS Noetic trong Docker |
| MCU | STM32F103C8T6 (Blue Pill) | USB CDC (Virtual COM Port) tới OPi |
| Lidar | RPLidar A1M8 | USB `/dev/rplidar` (chip CP2102) |
| IMU | MPU6050 | I2C (0x68) — nối vào STM32 (PB10/PB11) |
| Motor Driver | TB6612FNG | GPIO + PWM từ STM32 |
| Controller | PS2 Wireless (optional) | SPI tới STM32 |
| Programmer | ST-Link V2 | SWD — chỉ dùng để nạp code |
| Wheels | Differential drive | Track width: 168mm, Wheel radius: 21.5mm |

---

## Repository Structure

```
synmech_mobot/
├── network.env                 # ← Cấu hình IP mạng (OPi, Laptop, ROS port)
├── docker/                     # Docker deployment cho Orange Pi
│   ├── Dockerfile.opi          # Image ROS Noetic cho ARM
│   ├── docker-compose.yml      # Services: roscore, bringup, shell
│   ├── entrypoint.sh           # Script khởi động container
│   ├── run_teleop.sh           # Chạy teleop trong container đang chạy
│   └── setup_opi.sh            # Script cài đặt ban đầu cho OPi
├── synmech_bringup/            # ROS launch files, controllers, configs
│   ├── launch/
│   │   ├── bringup.launch      # Core: URDF + Lidar + Odometry node
│   │   ├── slam.launch         # bringup + Gmapping SLAM
│   │   ├── slam_view.launch    # Laptop: RViz + Teleop keyboard
│   │   ├── remote_view.launch  # Laptop: RViz + Teleop (legacy)
│   │   ├── robot_base.launch   # Standalone base_controller test
│   │   └── lidar.launch        # Chỉ Lidar (debug)
│   ├── scripts/
│   │   ├── fake_odom.py        # Phase 1: Fake odometry (không cần STM32)
│   │   ├── calibrate_pwm.py    # Công cụ hiệu chuẩn PWM ↔ velocity
│   │   ├── test_serial_motor.py # Test giao tiếp serial + motor
│   │   └── lidar_locker.py     # Khóa Lidar khi không dùng
│   ├── src/
│   │   └── base_controller.py  # Phase 2: Real STM32 + IMU odometry
│   └── config/
│       ├── calibration.yaml    # Bảng PWM ↔ velocity
│       ├── slam.rviz           # Layout RViz cho SLAM
│       └── 99-synmech-usb.rules # Udev rules cho USB devices
├── synmech_description/        # URDF/Xacro robot model
├── synmech_navigation/         # Navigation stack (Phase 3 — planned)
├── synmech_sim/                # Gazebo simulation
└── synmech_stm32/              # Firmware STM32 (Arduino framework)
    ├── synmech_stm32.ino       # ← File chính (dùng chung Arduino IDE + PlatformIO)
    ├── platformio.ini           # Config PlatformIO
    ├── upload_to_opi.sh         # Script build + flash qua OPi (HID bootloader)
    └── src/                     # Thư viện nguồn
        ├── Config.h             # ← CẤU HÌNH CHÂN, bật/tắt module
        ├── MotorDriver.cpp/h    # Điều khiển TB6612FNG
        ├── ImuSensor.cpp/h      # MPU6050 với bộ lọc chống drift
        ├── SerialBridge.cpp/h   # Giao thức UART (OPi ↔ STM32)
        ├── Kinematics.cpp/h     # Chuyển đổi vận tốc ↔ PWM
        ├── Encoder.cpp/h        # ⚠️ Chưa triển khai — chỉ là ý tưởng
        ├── PS2X_lib.cpp/h       # Tay cầm PS2 wireless
        └── main.cpp             # Proxy file cho PlatformIO (không sửa)
```

---

## Cấu hình `Config.h` — Bật/tắt module

File `synmech_stm32/src/Config.h` là nơi **duy nhất** bạn cần sửa để bật/tắt các module phần cứng. Mỗi module được điều khiển bởi một `#define`:

```c
// ============ BẬT/TẮT MODULE ============

// ENCODER — ⚠️ CHƯA TRIỂN KHAI, CHỈ LÀ Ý TƯỞNG
// Nếu sau này gắn encoder vào bánh xe, uncomment dòng dưới.
// Hiện tại PHẢI ĐỂ COMMENT vì chưa có phần cứng.
// #define USE_ENCODER

// DUAL IMU — Dùng 2 con MPU6050 (một AD0=LOW 0x68, một AD0=HIGH 0x69)
// Hiện tại PHẢI ĐỂ COMMENT vì chỉ có 1 con MPU6050.
// #define USE_DUAL_IMU

// PS2 CONTROLLER — Tay cầm PS2 wireless
// Uncomment nếu bạn có gắn module PS2 receiver vào STM32.
// ⚠️ LƯU Ý: Các chân PS2 (PB12-PB15) trùng với Motor phải!
//    Nếu bật PS2, cần đấu lại dây motor sang chân khác.
// #define USE_PS2
```

### Cấu hình hiện tại (Phase 2 — Motor + IMU, không encoder, không PS2)

| Module | Trạng thái | Lý do |
|---|---|---|
| IMU (MPU6050) | ✅ Luôn bật | Cung cấp Yaw cho odometry |
| Motor (TB6612FNG) | ✅ Luôn bật | Điều khiển 2 bánh xe |
| Encoder | ❌ Comment out | ⚠️ **Chỉ là ý tưởng**, chưa có phần cứng encoder. Code đã viết sẵn nhưng chưa test. |
| Dual IMU | ❌ Comment out | Chỉ có 1 con MPU6050 |
| PS2 | ❌ Comment out | Chân SPI trùng chân motor. Cần remap chân nếu muốn dùng. |

### Bảng chân hiện tại

| Chức năng | Chân STM32 | Ghi chú |
|---|---|---|
| Motor trái PWM | PA15 | Cần `AFIO_REMAP_SWJ_NOJTAG` (đã có trong code) |
| Motor trái IN1 | PA9 | |
| Motor trái IN2 | PA10 | |
| Motor phải PWM | PB13 | |
| Motor phải IN1 | PB15 | |
| Motor phải IN2 | PB14 | |
| Motor STBY | PA8 | Phải kéo HIGH để motor hoạt động |
| IMU SDA | PB11 | I2C2 |
| IMU SCL | PB10 | I2C2 |

> [!CAUTION]
> **KHÔNG dùng PA13, PA14** cho motor hoặc bất kỳ GPIO nào! Đây là chân SWD (nạp code). Nếu bạn cấu hình nhầm 2 chân này, mạch sẽ bị khóa và **không thể nạp code được nữa** (ST-Link báo `Failed to read core_id`). Phải dùng thủ thuật BOOT0 để recovery (xem phần Troubleshooting).

---

## Firmware STM32 — Arduino IDE + PlatformIO song song

Thư mục `synmech_stm32/` được cấu trúc để **cả Arduino IDE lẫn PlatformIO đều biên dịch được từ cùng một bộ source code**, không cần copy hay di chuyển file.

### Cách hoạt động

| IDE | File chính | Tìm thư viện ở đâu |
|---|---|---|
| Arduino IDE | `synmech_stm32.ino` | `src/` (qua `#include "src/Config.h"`) |
| PlatformIO | `src/main.cpp` → include `../synmech_stm32.ino` | `src/` (thư mục mặc định) |

File `src/main.cpp` chỉ là proxy 3 dòng, **không bao giờ cần sửa**:
```cpp
#ifdef PLATFORMIO
#include "../synmech_stm32.ino"
#endif
```

### Nạp code bằng Arduino IDE

**Cấu hình Board Manager** (đã test thành công):

| Setting | Value |
|---|---|
| Board | Generic STM32F1 series |
| Board part number | BluePill F103C8 |
| U(S)ART support | Enabled (generic 'Serial') |
| USB support | **CDC (generic 'Serial' supersede U(S)ART)** |
| Upload method | STM32CubeProgrammer (SWD) |

> [!IMPORTANT]
> **USB support phải là CDC** — đây là tính năng cho phép STM32 tạo cổng COM ảo qua cổng Micro-USB, để giao tiếp serial với OPi mà **không cần module USB-to-UART rời** (CH340, CP2102...).
>
> ST-Link **chỉ dùng để nạp code** (SWD). Sau khi nạp xong, tháo ST-Link ra. Giao tiếp runtime giữa STM32 ↔ OPi hoàn toàn qua cổng **Micro-USB** trên Blue Pill.

### Nạp code bằng PlatformIO (VSCode)

```bash
cd synmech_stm32
pio run --target upload
```

PlatformIO dùng `st-flash` qua ST-Link (SWD). Build flags đã cấu hình sẵn CDC trong `platformio.ini`.

---

## Cài đặt Udev Rules (trên Orange Pi)

Để thiết bị USB có tên cố định (không bị đổi thứ tự mỗi lần cắm):

```bash
# Copy rules
sudo cp synmech_bringup/config/99-synmech-usb.rules /etc/udev/rules.d/

# Reload
sudo udevadm control --reload-rules
sudo udevadm trigger

# Rút cắm lại USB, rồi kiểm tra
ls -la /dev/rplidar /dev/stm32
```

### Tìm Vendor/Product ID

```bash
lsusb
```

Kết quả mẫu:
```
Bus 001 Device 004: ID 10c4:ea60 Silicon Labs CP210x   ← RPLidar
Bus 001 Device 005: ID 0483:5740 STMicroelectronics    ← STM32 CDC
```

| Thiết bị | idVendor | idProduct | Symlink |
|---|---|---|---|
| RPLidar A1M8 (CP2102) | `10c4` | `ea60` | `/dev/rplidar` |
| STM32 CDC (Virtual COM) | `0483` | `5740` | `/dev/stm32` |

---

## Network Configuration (`network.env`)

```env
OPI_IP=192.168.1.99        # IP của Orange Pi trên mạng LAN
LAPTOP_IP=192.168.1.12     # IP của Laptop
OPI_USER=huy               # User SSH vào OPi
ROS_PORT=2710               # Port của roscore (mặc định ROS là 11311)
ROS_MASTER_URI=http://192.168.1.99:2710
ROS_IP=192.168.1.99
```

> [!NOTE]
> **Về ROS_IP**: Biến này mang nghĩa *"Tôi đang chạy trên máy nào"*. Khi chạy container trên OPi, `ROS_IP` phải là IP của OPi. Khi chạy RViz trên Laptop, `ROS_IP` phải là IP của Laptop. File `entrypoint.sh` trong Docker sẽ tự set `ROS_IP=${OPI_IP}` cho các container trên OPi.

Trước khi bắt đầu, hãy kiểm tra IP thực tế:
```bash
# Trên OPi
hostname -I    # Phải trùng với OPI_IP trong network.env

# Trên Laptop
hostname -I    # Phải trùng với LAPTOP_IP trong network.env
```

Nếu IP thay đổi (router cấp DHCP mới), sửa lại `network.env` và rebuild container.

---

# Development Phases

## Cấu hình X11 Forwarding cho giao diện ROS (RViz)

Môi trường Docker theo mặc định được cô lập (isolated) hoàn toàn khỏi X-server của máy host. Do đó, để khởi chạy các ứng dụng đồ họa (như RViz) từ bên trong container và hiển thị lên màn hình laptop, cần thực hiện cấu hình X11 forwarding theo 2 bước dưới đây:

### 1. Phân quyền truy cập X-server (Trên máy Host)
Mở terminal trên máy host (Laptop) và thực thi lệnh cấp quyền cho các tiến trình local (bao gồm Docker container) kết nối tới X-server:
```bash
xhost +local: > /dev/null 2>&1
```
> **Khuyến nghị:** Thêm lệnh này vào cuối file `~/.bashrc` của máy host để tự động cấp quyền mỗi khi khởi tạo phiên terminal mới. (Hậu tố `> /dev/null 2>&1` được sử dụng để ẩn các standard output logs không cần thiết).

### 2. Định tuyến biến môi trường DISPLAY (Trong Container)
Kiểm tra định danh của display hiện tại trên máy host bằng lệnh:
```bash
echo $DISPLAY
```
*(Kết quả phổ biến thường là `:0`, `:1` đối với X11, hoặc `:2` đối với hệ thống dùng Wayland/WSL).*

Sau khi truy cập vào môi trường bash của container, cần khai báo biến định tuyến tương ứng trước khi gọi các ứng dụng ROS có GUI. Thay giá trị `:2` bằng ID thực tế thu được ở bước trên:
```bash
export DISPLAY=:2
```

---

## Phase 1 — Fake Odometry (Không cần STM32)

**Mục đích**: Test SLAM chỉ với Lidar, không cần phần cứng motor/IMU. Robot được di chuyển bằng tay, odometry được tính giả (fake) từ `/cmd_vel`.

### Cấu hình

#### Trên `bringup.launch`:
- **Uncomment** node `fake_odom.py`
- **Comment out** node `base_controller.py`

```xml
<!-- BẬT cái này cho Phase 1 -->
<node pkg="synmech_bringup" type="fake_odom.py" name="fake_odom" output="screen">
    <param name="track_width"  value="0.168"/>
    <param name="wheel_radius" value="0.0215"/>
</node>

<!-- TẮT cái này cho Phase 1 -->
<!--
<node pkg="synmech_bringup" type="base_controller.py" name="base_controller" output="screen">
    ...
</node>
-->
```

#### Trên `slam.launch`:
| Parameter | Giá trị Phase 1 | Lý do |
|---|---|---|
| `temporalUpdate` | **1.0** | Ép Gmapping xử lý scan mỗi 1 giây (vì fake odom không biết robot đang di chuyển bằng tay) |
| `minimumScore` | **50** | Dễ dãi — scan matching phải linh hoạt vì odom không chính xác |

#### STM32 Config.h:
Không cần thay đổi gì — Phase 1 không dùng STM32.

### Quy trình chạy Phase 1

**Terminal 1 — SSH vào OPi, khởi động ROS + SLAM:**
```bash
ssh huy@192.168.1.99
cd ~/catkin_ws/src/synmech_mobot/docker
docker compose up
```

Chờ đến khi thấy:
```
[synmech] Launching SLAM (bringup + gmapping)...
```

**Terminal 2 — Laptop Host (Mở giao diện RViz):**
```bash
# Yêu cầu: Đã thực thi `xhost +local:` trên host terminal
export DISPLAY=:2    # Thay :2 bằng định danh thực tế thu được từ lệnh `echo $DISPLAY` trên host

source ~/catkin_ws/src/synmech_mobot/network.env
export ROS_MASTER_URI=http://${OPI_IP}:${ROS_PORT}
export ROS_IP=${LAPTOP_IP}
roslaunch synmech_bringup slam_view.launch
```

**Thao tác**: Di chuyển robot bằng tay, từ từ (< 10°/s xoay). Bản đồ sẽ dần được tạo trên RViz.

---

## Phase 2 — STM32 + MPU6050 (Hiện tại)

**Mục đích**: Robot tự chạy bằng motor, IMU cung cấp góc Yaw chính xác cho odometry. Điều khiển qua bàn phím (teleop) hoặc tay cầm PS2.

### Cấu hình

#### 1. STM32 `Config.h` — Bật/tắt module:

```c
// #define USE_ENCODER    ← ĐỂ COMMENT (chưa có encoder)
// #define USE_DUAL_IMU   ← ĐỂ COMMENT (chỉ có 1 IMU)
// #define USE_PS2        ← ĐỂ COMMENT (trừ khi có tay cầm PS2 VÀ đã remap chân)
```

#### 2. `bringup.launch` — Đổi sang base_controller:

```xml
<!-- TẮT cái này cho Phase 2 -->
<!--
<node pkg="synmech_bringup" type="fake_odom.py" name="fake_odom" ...>
    ...
</node>
-->

<!-- BẬT cái này cho Phase 2 -->
<node pkg="synmech_bringup" type="base_controller.py" name="base_controller" output="screen">
    <param name="port"        value="/dev/stm32"/>
    <param name="baudrate"    value="115200"/>
    <param name="vel_to_pwm"  value="200.0"/>
    <param name="track_width" value="0.168"/>
</node>
```

#### 3. `slam.launch` — Điều chỉnh Gmapping:

| Parameter | Phase 1 | → Phase 2 | Lý do |
|---|---|---|---|
| `temporalUpdate` | 1.0 | **-1.0** | Real odom tự trigger update, không cần ép theo thời gian |
| `minimumScore` | 50 | **200** | Odom chính xác hơn → yêu cầu scan matching chặt hơn |
| `particles` | 30 | **30** | Giữ nguyên, tăng lên 60 nếu bản đồ kém |

### Quy trình chạy Phase 2

#### Bước 0 — Nạp firmware STM32

Nạp code bằng Arduino IDE hoặc PlatformIO (xem phần **"Firmware STM32"** ở trên).

Sau khi nạp xong:
1. Tháo ST-Link ra
2. Cắm cổng **Micro-USB** trên Blue Pill vào **Orange Pi**

#### Bước 1 — Kiểm tra kết nối USB trên OPi

```bash
ssh huy@192.168.1.99

# Kiểm tra thiết bị USB đã nhận chưa
lsusb
```

**Kết quả mong đợi:**
```
Bus 001 Device 00X: ID 0483:5740 STMicroelectronics Virtual COM Port   ← STM32
Bus 001 Device 00Y: ID 10c4:ea60 Silicon Labs CP210x UART Bridge       ← RPLidar
```

> [!WARNING]
> Nếu **không thấy** dòng `0483:5740` (STM32), kiểm tra:
> 1. Cáp Micro-USB có hỗ trợ data không? (Nhiều cáp chỉ sạc, không có dây data)
> 2. STM32 đã được nạp firmware với **USB CDC** chưa? (Arduino IDE: USB support = CDC)
> 3. Thử rút cắm lại cáp Micro-USB
> 4. Chạy `dmesg | tail -20` ngay sau khi cắm để xem kernel có nhận thiết bị không

Kiểm tra symlink udev:
```bash
ls -la /dev/stm32 /dev/rplidar
```

Nếu symlink chưa có, cài udev rules (xem phần **"Cài đặt Udev Rules"**).

Kiểm tra cổng serial thực tế:
```bash
ls /dev/ttyACM*    # STM32 CDC thường xuất hiện ở đây
ls /dev/ttyUSB*    # RPLidar thường xuất hiện ở đây
```

#### Bước 2 — Test giao tiếp serial (tùy chọn nhưng khuyến khích)

```bash
# Trên OPi, mở serial monitor
# (Thay /dev/stm32 bằng /dev/ttyACM0 nếu chưa cài udev)
screen /dev/stm32 115200
```

Bạn sẽ thấy dòng telemetry chạy liên tục:
```
D,-0.12
D,-0.15
D,-0.18
...
```

Thử gõ lệnh:
```
M,150,150    ← Cả 2 bánh quay tiến
M,-150,150   ← Xoay tại chỗ
S            ← Dừng khẩn cấp
C            ← Hiệu chuẩn IMU
```

> [!IMPORTANT]
> Nếu IMU luôn trả về `D,0.00` (không thay đổi khi xoay mạch):
> - Kiểm tra dây I2C (SDA → PB11, SCL → PB10) đã nối đúng chưa
> - Kiểm tra nguồn 3.3V cho MPU6050
> - Chạy `i2cdetect -y 1` trên STM32 (nếu có) hoặc kiểm tra bằng Arduino I2C Scanner
> - Thử gõ `C` để recalibrate

Thoát screen: nhấn `Ctrl+A` rồi `K` rồi `Y`.

#### Bước 3 — Khởi động ROS + SLAM trên OPi

**Terminal 1 — SSH vào OPi:**
```bash
ssh huy@192.168.1.99
cd ~/catkin_ws/src/synmech_mobot/docker
docker compose up
```

Chờ log hiện:
```
============================================
 SynMech Robot — ROS Noetic (Docker)
 ROS_MASTER_URI: http://192.168.1.99:2710
 ROS_IP:         192.168.1.99
============================================
[synmech] Launching SLAM (bringup + gmapping)...
```

> [!NOTE]
> Nếu gặp lỗi `Permission denied: '/dev/stm32'` hoặc `could not open port`:
> - Kiểm tra udev rules đã set `MODE="0666"` chưa
> - Hoặc chạy `sudo chmod 666 /dev/ttyACM0` trên OPi trước khi khởi động Docker

#### Bước 4 — Mở RViz + Teleop trên Laptop

**Terminal 2 — Laptop Host (Mở giao diện RViz):**
```bash
# Yêu cầu: Đã thực thi `xhost +local:` trên host terminal
export DISPLAY=:2    # Thay :2 bằng định danh thực tế thu được từ lệnh `echo $DISPLAY` trên host

source ~/catkin_ws/src/synmech_mobot/network.env
export ROS_MASTER_URI=http://${OPI_IP}:${ROS_PORT}
export ROS_IP=${LAPTOP_IP}
roslaunch synmech_bringup slam_view.launch
```

Cửa sổ xterm sẽ mở ra cho teleop keyboard. Dùng phím `i/j/k/l` để lái robot.

#### Bước 5 — Debug shell (tùy chọn)

Mở thêm terminal trong Docker container để debug:

**Terminal 3 — SSH vào OPi:**
```bash
ssh huy@192.168.1.99
cd ~/catkin_ws/src/synmech_mobot/docker
docker compose run --rm shell
```

Trong shell container, bạn có thể:
```bash
# Xem danh sách topic
rostopic list

# Monitor IMU data từ base_controller
rostopic echo /odom

# Gửi lệnh motor trực tiếp
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -1

# Xem TF tree
rosrun tf view_frames
```

---

## UART Protocol (OPi ↔ STM32)

### Commands: OPi → STM32

| Command | Format | Example | Description |
|---|---|---|---|
| Motor | `M,<pwmL>,<pwmR>\n` | `M,150,-150\n` | Set PWM từng bánh (-255 đến 255) |
| Calibrate IMU | `C\n` | `C\n` | Recalibrate gyro offset MPU6050 |
| Emergency Stop | `S\n` | `S\n` | Dừng cả 2 motor ngay lập tức |
| PS2 Mode | `PS2\n` | `PS2\n` | Chuyển input motor sang tay cầm PS2 |
| Serial Mode | `SER\n` | `SER\n` | Chuyển input motor về serial (OPi) |

### Telemetry: STM32 → OPi

| Message | Format | Example | Description |
|---|---|---|---|
| IMU Data | `D,<yaw_deg>\n` | `D,-45.25\n` | Góc Yaw (độ), gửi liên tục 50Hz |
| Calibrate ACK | `A\n` | `A\n` | Báo hiệu calibrate xong |
| Error | `E,<code>\n` | `E,1\n` | 1=IMU fail, 2=Motor fault |

> Telemetry **luôn gửi** bất kể đang ở mode Serial hay PS2. PS2 mode chỉ thay đổi nguồn lệnh motor.

---

## Encoder — Ý tưởng chưa triển khai

> [!WARNING]
> **Encoder hiện tại CHỈ LÀ Ý TƯỞNG**. Code đã được viết sẵn trong `Encoder.cpp/h` nhưng:
> - Chưa có phần cứng encoder trên robot
> - Chưa được test thực tế
> - Các chân encoder trong `Config.h` (PB4, PB5, PB8, PB9) chỉ là dự kiến
> - `#define USE_ENCODER` phải **luôn để comment**
>
> Nếu sau này gắn encoder, uncomment `USE_ENCODER` trong `Config.h`. Khi đó telemetry sẽ đổi format sang: `YAW:<yaw>,V_LEFT:<vL>,V_RIGHT:<vR>` và `base_controller.py` cần được cập nhật để parse format mới.

---

## PS2 Controller

| Input | Action |
|---|---|
| Left Stick Y | Forward / Backward |
| Left Stick X | Turn Left / Right |
| L1 (hold) | Slow mode (50% speed) |
| R1 (hold) | Boost mode (200% speed) |
| SELECT | Emergency Stop |

Joystick dùng **quadratic mapping** cho điều khiển mịn. Dead zone ±30.

> [!WARNING]
> Các chân PS2 (PB12–PB15) **trùng với chân motor phải** (PB13, PB14, PB15). Nếu muốn dùng đồng thời PS2 + Motor, phải remap chân motor sang chân khác rồi cập nhật `Config.h`.

---

## Gmapping SLAM Parameters

| Parameter | Default | Mô tả |
|---|---|---|
| `linearUpdate` | 0.1 | Khoảng cách tối thiểu (m) trước khi xử lý scan |
| `angularUpdate` | 0.1 | Góc xoay tối thiểu (rad) trước khi xử lý scan |
| `temporalUpdate` | 1.0 / -1.0 | Ép update theo thời gian (-1.0 = tắt) |
| `minimumScore` | 50 / 200 | Ngưỡng scan matching (cao = chặt hơn) |
| `particles` | 30 | Số particle filter (nhiều = chính xác hơn, tốn CPU) |
| `maxUrange` | 6.0 | Tầm laser dùng cho matching (m) |
| `maxRange` | 8.0 | Tầm laser dùng cho map (m) |

---

## Docker Architecture

```
docker compose up
  ├── synmech_roscore     → roscore (port 2710)
  └── synmech_bringup     → slam.launch
       ├── robot_state_publisher (URDF → TF)
       ├── rplidarNode (/dev/rplidar → /scan)
       ├── base_controller.py (/dev/stm32 → /odom, /tf)
       └── slam_gmapping (/scan + /odom → /map)
```

Debug shell: `docker compose run --rm shell`

---

## Troubleshooting

### ST-Link báo `Failed to read core_id` — không nạp được code

**Nguyên nhân**: Firmware cũ đã chiếm chân PA13/PA14 (chân SWD), nên ST-Link không thể kết nối.

**Giải pháp — Recovery bằng BOOT0:**

1. **Rút** ST-Link ra khỏi USB máy tính (tắt nguồn hoàn toàn)
2. Trên Blue Pill, cắm jumper **BOOT0 → 1** (cắm sang phía chân `1`)
3. **Cắm** ST-Link lại vào USB (chip vào chế độ bootloader, firmware cũ không chạy)
4. Nạp code mới bằng Arduino IDE hoặc PlatformIO
5. Sau khi nạp thành công, **rút** ST-Link ra
6. Cắm jumper **BOOT0 → 0** (về vị trí bình thường)
7. **Cắm** ST-Link lại — code mới chạy bình thường

### STM32 cắm Micro-USB vào OPi nhưng không thấy `/dev/ttyACM*`

1. Kiểm tra cáp Micro-USB có phải loại data (không phải loại chỉ sạc)
2. Firmware phải được nạp với **USB CDC** bật (Arduino IDE: USB support = CDC)
3. Chạy `dmesg | tail -20` ngay sau khi cắm để xem kernel log
4. Chạy `lsusb` — phải thấy `ID 0483:5740 STMicroelectronics`

### IMU luôn trả về `D,0.00`

1. Kiểm tra dây I2C: SDA → PB11, SCL → PB10
2. Kiểm tra nguồn 3.3V cho MPU6050
3. Gửi lệnh `C` qua serial để recalibrate
4. Nếu vẫn không được, thử dùng Arduino I2C Scanner để kiểm tra MPU6050 có trả lời trên bus I2C không

### Motor không quay khi gõ `M,150,150`

1. Kiểm tra chân **STBY (PA8)** — phải được kéo HIGH (code đã `digitalWrite(STBY, 1)`)
2. Kiểm tra nguồn motor (pin/adapter riêng cho TB6612FNG VM)
3. Kiểm tra đấu dây motor đúng chân trong `Config.h`
4. Thử tăng PWM: `M,255,255` — nếu motor vẫn không quay, lỗi phần cứng

### Docker container báo lỗi serial port

```
Serial connection failed: [Errno 13] Permission denied: '/dev/stm32'
```

**Giải pháp:**
```bash
sudo chmod 666 /dev/ttyACM0    # hoặc /dev/stm32
# Hoặc cài udev rules với MODE="0666" (khuyến khích)
```

---

## Phase 3 — Navigation (Planned)

Sử dụng bản đồ đã lưu + AMCL localization + move_base cho autonomous path planning. Chưa triển khai.

---

## License

MIT
