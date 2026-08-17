#!/usr/bin/env python3
import serial
import time
import yaml
import sys

# Default port - user can change via argument or just edit
PORT = '/dev/stm32'
BAUDRATE = 115200

def main():
    print("=== PWM to Velocity Calibration Tool ===")
    print("Ensure the base_controller ROS node is NOT running to avoid serial conflicts.")
    
    port = PORT
    if len(sys.argv) > 1:
        port = sys.argv[1]
        
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=1)
        print(f"Connected to {port}")
    except Exception as e:
        print(f"Failed to connect to {port}: {e}")
        sys.exit(1)
        
    # Ensure serial mode
    ser.write(b"SER\n")
    time.sleep(0.1)

    points = []
    points.append([0, 0.0]) # Base point: 0 PWM = 0 m/s
    
    try:
        while True:
            pwm_input = input("\nEnter PWM value (0-255) to test, or 'q' to finish & save: ")
            if pwm_input.lower() == 'q':
                break
                
            try:
                pwm = int(pwm_input)
            except ValueError:
                print("Invalid input.")
                continue
                
            if pwm < 0 or pwm > 255:
                print("PWM must be between 0 and 255.")
                continue
                
            duration_input = input("Enter duration to run (seconds) [default 2.0]: ")
            duration = float(duration_input) if duration_input else 2.0
            
            print(f"Running robot forward at PWM {pwm} for {duration} seconds...")
            ser.write(f"M,{pwm},{pwm}\n".encode('utf-8'))
            time.sleep(duration)
            ser.write(b"M,0,0\n") # Stop
            
            dist_input = input("Enter the actual distance traveled in meters: ")
            try:
                dist = float(dist_input)
                vel = dist / duration
                print(f"Calculated Velocity: {vel:.3f} m/s")
                points.append([pwm, round(vel, 3)])
            except ValueError:
                print("Invalid distance. Discarding this run.")
                
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ser.write(b"M,0,0\n")
        ser.close()
        
    print("\n--- Calibration Results ---")
    points.sort(key=lambda x: x[0]) # Sort by PWM
    for p in points:
        print(f"PWM: {p[0]} -> {p[1]:.3f} m/s")
        
    save = input("\nSave to calibration.yaml? (y/n): ")
    if save.lower() == 'y':
        data = {
            'calibration_points': points
        }
        with open('calibration.yaml', 'w') as f:
            yaml.dump(data, f)
        print("Saved to calibration.yaml in the current directory.")
        print("Remember to move it to synmech_bringup/config/ and point to it in your launch files.")

if __name__ == '__main__':
    main()
