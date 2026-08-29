#!/usr/bin/env python3
"""
RPLidar A1 Motor Locker
This script prevents the RPLidar A1 from auto-spinning when the USB port is closed by Linux.
It holds the port open with DTR=True, which turns the motor OFF.
When ROS starts, it can concurrently open the port and override DTR to spin the motor.
"""

import serial
import time
import sys

def main():
    print("Starting Lidar Locker daemon...")
    while True:
        try:
            # Open the serial port to prevent Linux from dropping DTR
            s = serial.Serial("/dev/rplidar")
            # Set DTR High (True). On RPLidar A1, this cuts power to the motor
            s.dtr = True
            
            # Sleep forever. Consumes 0% CPU and doesn't read data (won't conflict with ROS)
            while True: 
                time.sleep(3600)
                
        except Exception as e:
            # Port might be completely unavailble (e.g., unplugged)
            # Sleep briefly to avoid CPU spin, then retry
            time.sleep(5)

if __name__ == "__main__":
    main()
