#!/usr/bin/env python3
"""
Motor Testing Script
Tests motor movements: forward, reverse, right, left with 2-second stops between each
"""

import sys
import time
from motorControl.controller import MotorController

def main():
    # Initialize motor controller
    print("Initializing motor controller...")
    motor = MotorController(port='/dev/ttyACM0', baudrate=115200)
    
    if motor.arduino is None:
        print("Failed to connect to Arduino. Exiting.")
        return
    
    print("Motor controller initialized successfully!")
    print("\nStarting motor test sequence...\n")
    
    try:
        # Forward movement - both motors at 90 RPM
        print("Moving FORWARD at 30 RPM for 3 seconds...")
        d1, d2 = motor.setBothMotors(30, 30, 3, 3)
        print(f"Distance traveled - Motor1: {d1} cm, Motor2: {d2} cm\n")
        
        # Stop for 2 seconds
        print("STOPPING for 2 seconds...")
        motor.stop(duration=2)
        
        # Reverse movement - both motors at -90 RPM
        print("Moving REVERSE at -30 RPM for 3 seconds...")
        d1, d2 = motor.setBothMotors(-30, -30, 3, 3)
        print(f"Distance traveled - Motor1: {d1} cm, Motor2: {d2} cm\n")
        
        # Stop for 2 seconds
        print("STOPPING for 2 seconds...")
        motor.stop(duration=2)
        
        # Right turn - motor1 at 30 RPM, motor2 at -30 RPM (differential drive)
        print("Turning RIGHT at 30 RPM for 3 seconds...")
        d1, d2 = motor.setBothMotors(15, -15, 3, 3)
        print(f"Distance traveled - Motor1: {d1} cm, Motor2: {d2} cm\n")
        
        # Stop for 2 seconds
        print("STOPPING for 2 seconds...")
        motor.stop(duration=2)
        
        # Left turn - motor1 at -30 RPM, motor2 at 30 RPM (differential drive)
        print("Turning LEFT at 30 RPM for 3 seconds...")
        d1, d2 = motor.setBothMotors(-15, 15, 3, 3)
        print(f"Distance traveled - Motor1: {d1} cm, Motor2: {d2} cm\n")
        
        # Final stop
        print("STOPPING - Test sequence complete!")
        motor.stop()
        
        print("\nMotor test sequence completed successfully!")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        motor.stop()
    except Exception as e:
        print(f"\nError during test: {e}")
        motor.stop()
    finally:
        motor.close()

if __name__ == "__main__":
    main()
