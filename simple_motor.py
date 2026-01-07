#!/usr/bin/env python3
"""
Simple motor control - set RPM and get distance in real-time
"""

import serial
import time

# Connect to Arduino
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
time.sleep(2)

# Clear initial buffer
arduino.flushInput()

# Set RPM
def setRPM(rpm1, rpm2):
    arduino.write(f"{rpm1},{rpm2}\n".encode())

# Get distance
def getDist():
    if arduino.in_waiting > 0:
        line = arduino.readline().decode().strip()
        if "Dist1(cm):" in line:
            parts = line.split(',')
            for part in parts:
                if "Dist1(cm):" in part:
                    d1 = float(part.split(':')[1])
                elif "Dist2(cm):" in part:
                    d2 = float(part.split(':')[1])
            return d1, d2
    return None, None

# Main
try:
    # Forward 5 seconds
    print("Forward 15 RPM for 5 seconds...")
    setRPM(15, 15)
    time.sleep(0.2)  # Wait for Arduino response
    arduino.flushInput()  # Clear the response
    
    start = time.time()
    while time.time() - start < 5:
        d1, d2 = getDist()
        if d1 is not None:
            print(f"D1: {d1:.2f} cm, D2: {d2:.2f} cm")
    
    # Stop 2 seconds
    print("\nStopping for 2 seconds...")
    setRPM(0, 0)
    time.sleep(0.2)  # Wait for Arduino response
    arduino.flushInput()  # Clear the response
    time.sleep(1.8)  # Rest of 2 seconds
    
    # Reverse 5 seconds
    print("\nReverse -15 RPM for 5 seconds...")
    setRPM(-15, -15)
    time.sleep(0.2)  # Wait for Arduino response
    arduino.flushInput()  # Clear the response
    
    start = time.time()
    while time.time() - start < 5:
        d1, d2 = getDist()
        if d1 is not None:
            print(f"D1: {d1:.2f} cm, D2: {d2:.2f} cm")
    
    # Final stop
    print("\nFinal stop...")
    setRPM(0, 0)
    time.sleep(0.2)
    
    print("\nDone!")
    
except KeyboardInterrupt:
    print("\nInterrupted!")
    setRPM(0, 0)
except Exception as e:
    print(f"\nError: {e}")
    setRPM(0, 0)
finally:
    arduino.close()
    print("Connection closed")
