#!/usr/bin/env python3
"""
Test giao tiếp Serial giữa Orange Pi và STM32.
Gửi lệnh PWM trực tiếp, không qua ROS.

Cách dùng (trong Docker container):
  python3 /catkin_ws/src/synmech_mobot/synmech_bringup/scripts/test_serial_motor.py

Giao thức UART:
  OPi → STM32:  M,<pwmL>,<pwmR>\n   (điều khiển motor)
                 S\n                   (dừng khẩn cấp)
                 C\n                   (calibrate IMU)
  STM32 → OPi:  D,<yaw_deg>\n        (dữ liệu IMU)
                 A\n                   (ACK calibrate)
                 E,<code>\n            (lỗi)
"""

import serial
import time
import sys
import threading

PORT = '/dev/stm32'
BAUD = 115200


def serial_reader(ser, stop_event):
    """Thread đọc dữ liệu từ STM32 liên tục."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"  ← STM32: {line}")
        except Exception as e:
            print(f"  [!] Lỗi đọc serial: {e}")
            break
        time.sleep(0.01)


def send(ser, cmd):
    """Gửi lệnh và in log."""
    print(f"  → Gửi: {cmd.strip()}")
    ser.write(cmd.encode('utf-8'))
    ser.flush()


def main():
    print("=" * 50)
    print(" TEST SERIAL: Orange Pi ↔ STM32")
    print("=" * 50)

    # --- Bước 1: Kết nối ---
    print(f"\n[1] Kết nối {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
        print(f"    ✓ Kết nối thành công!")
    except Exception as e:
        print(f"    ✗ THẤT BẠI: {e}")
        print(f"    → Kiểm tra: STM32 có cắm USB không? /dev/stm32 có tồn tại không?")
        sys.exit(1)

    # Bật thread đọc phản hồi từ STM32
    stop_event = threading.Event()
    reader = threading.Thread(target=serial_reader, args=(ser, stop_event))
    reader.daemon = True
    reader.start()

    time.sleep(0.5)  # Chờ STM32 sẵn sàng

    # --- Bước 2: Gửi lệnh Stop trước ---
    print(f"\n[2] Gửi lệnh STOP (S)...")
    send(ser, "S\n")
    time.sleep(1)

    # --- Bước 3: Test motor trái ---
    print(f"\n[3] Test MOTOR TRÁI — PWM 100 (3 giây)...")
    print(f"    Lệnh: M,100,0")
    send(ser, "M,100,0\n")
    time.sleep(3)
    send(ser, "M,0,0\n")
    time.sleep(1)

    # --- Bước 4: Test motor phải ---
    print(f"\n[4] Test MOTOR PHẢI — PWM 100 (3 giây)...")
    print(f"    Lệnh: M,0,100")
    send(ser, "M,0,100\n")
    time.sleep(3)
    send(ser, "M,0,0\n")
    time.sleep(1)

    # --- Bước 5: Test cả hai ---
    print(f"\n[5] Test CẢ HAI MOTOR — PWM 100 (3 giây)...")
    print(f"    Lệnh: M,100,100")
    send(ser, "M,100,100\n")
    time.sleep(3)

    # --- Bước 6: Stop ---
    print(f"\n[6] STOP...")
    send(ser, "S\n")
    time.sleep(1)

    # --- Bước 7: Test chiều ngược ---
    print(f"\n[7] Test CHIỀU NGƯỢC — PWM -100,-100 (3 giây)...")
    send(ser, "M,-100,-100\n")
    time.sleep(3)
    send(ser, "S\n")
    time.sleep(1)

    # --- Kết thúc ---
    print(f"\n{'=' * 50}")
    print(f" KẾT QUẢ:")
    print(f"  - Nếu bánh quay ở bước 3,4,5,7 → Serial + Firmware OK")
    print(f"  - Nếu không quay → Kiểm tra firmware STM32 / dây motor")
    print(f"  - Nếu quay sai chiều → Đổi dây motor hoặc sửa firmware")
    print(f"{'=' * 50}")

    stop_event.set()
    ser.close()


if __name__ == '__main__':
    main()
