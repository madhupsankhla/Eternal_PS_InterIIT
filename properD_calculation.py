#!/usr/bin/env python3
"""
Motor control with movement functions
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
    time.sleep(0.2)
    arduino.flushInput()

# Get distance
def getDist():
    if arduino.in_waiting > 0:
        line = arduino.readline().decode().strip()
        if "Dist1(cm):" in line:
            parts = line.split(',')
            d1, d2 = None, None
            for part in parts:
                if "Dist1(cm):" in part:
                    d1 = float(part.split(':')[1])
                elif "Dist2(cm):" in part:
                    d2 = float(part.split(':')[1])
            return d1, d2
    return None, None

def forward(rpm, duration):
    """
    Move forward at specified RPM for given duration
    Returns: (distance1, distance2) in cm
    """
    setRPM(rpm, rpm)
    
    start = time.time()
    final_d1, final_d2 = None, None
    
    while time.time() - start < duration:
        d1, d2 = getDist()
        if d1 is not None:
            final_d1, final_d2 = d1, d2
    
    return final_d1, final_d2

def backward(rpm, duration):
    """
    Move backward at specified RPM for given duration
    Returns: (distance1, distance2) in cm
    """
    setRPM(-rpm, -rpm)
    
    start = time.time()
    final_d1, final_d2 = None, None
    
    while time.time() - start < duration:
        d1, d2 = getDist()
        if d1 is not None:
            final_d1, final_d2 = d1, d2
    
    return final_d1, final_d2

def right(rpm, duration):
    """
    Turn right - Motor1 forward, Motor2 backward
    Returns: (distance1, distance2) in cm
    """
    setRPM(rpm, -rpm)
    
    start = time.time()
    final_d1, final_d2 = None, None
    
    while time.time() - start < duration:
        d1, d2 = getDist()
        if d1 is not None:
            final_d1, final_d2 = d1, d2
    
    return final_d1, final_d2

def left(rpm, duration):
    """
    Turn left - Motor1 backward, Motor2 forward
    Returns: (distance1, distance2) in cm
    """
    setRPM(-rpm, rpm)
    
    start = time.time()
    final_d1, final_d2 = None, None
    
    while time.time() - start < duration:
        d1, d2 = getDist()
        if d1 is not None:
            final_d1, final_d2 = d1, d2
    
    return final_d1, final_d2

def stop(duration=None):
    """
    Stop both motors
    Args:
        duration: Time in seconds to keep motors stopped (None = indefinite)
    """
    setRPM(0, 0)
    if duration is not None:
        time.sleep(duration)
    else:
        time.sleep(0.2)

# Main - Example usage
try:
    # Test forward
    d1, d2 = forward(60, 3)
    print(f"Forward complete - Final distances: D1={d1:.2f} cm, D2={d2:.2f} cm\n")
    
    stop(1)
    
    # Test right turn
    d1, d2 = right(20, 2)
    print(f"Right turn complete - Final distances: D1={d1:.2f} cm, D2={d2:.2f} cm\n")
    
    stop(1)
    
    # Test backward
    d1, d2 = backward(60, 3)
    print(f"Backward complete - Final distances: D1={d1:.2f} cm, D2={d2:.2f} cm\n")
    
    stop(1)
    
    # Test left turn
    d1, d2 = left(20, 2)
    print(f"Left turn complete - Final distances: D1={d1:.2f} cm, D2={d2:.2f} cm\n")
    
    stop()
    
    print("\nAll tests completed!")
    
except KeyboardInterrupt:
    print("\nInterrupted!")
    stop()
except Exception as e:
    print(f"\nError: {e}")
    stop()
finally:
    arduino.close()
    print("Connection closed")
