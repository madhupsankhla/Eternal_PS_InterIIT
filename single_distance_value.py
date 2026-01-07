#!/usr/bin/env python3
"""
Raspberry Pi code for USB communication with Arduino
Sends two values to Arduino and receives the result
"""

import serial
import time

def connect_arduino(port='/dev/ttyACM0', baudrate=115200, timeout=1):
    """
    Establish connection with Arduino
    
    Args:
        port: Serial port (usually /dev/ttyACM0 or /dev/ttyUSB0)
        baudrate: Communication speed (must match Arduino - 115200)
        timeout: Read timeout in seconds
    
    Returns:
        serial.Serial object
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for Arduino to reset after connection
        
        # Read the "Arduino Ready" message
        if ser.in_waiting > 0:
            print(ser.readline().decode('utf-8').strip())
        
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to Arduino: {e}")
        print("Make sure Arduino is connected and check the port name.")
        print("Common ports: /dev/ttyACM0, /dev/ttyACM1, /dev/ttyUSB0")
        return None

def send_rpm_get_distance(ser, targetRpm1, targetRpm2):
    """
    Send target RPM values to Arduino and receive distance measurements
    
    Args:
        ser: Serial connection object
        targetRpm1: Target RPM for motor 1 (float)
        targetRpm2: Target RPM for motor 2 (float)
    
    Returns:
        tuple: (distanceCm1, distanceCm2) or (None, None) if error
    """
    if ser is None:
        print("No serial connection available")
        return None, None
    
    try:
        # Send RPM values as comma-separated string with newline
        message = f"{targetRpm1},{targetRpm2}\n"
        ser.write(message.encode('utf-8'))
        print(f"Sent to Arduino - RPM1: {targetRpm1}, RPM2: {targetRpm2}")
        
        # Wait for response
        time.sleep(0.15)  # Small delay for Arduino to process
        
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            # Parse the response (distanceCm1,distanceCm2)
            distances = response.split(',')
            if len(distances) == 2:
                distanceCm1 = float(distances[0])
                distanceCm2 = float(distances[1])
                print(f"Received from Arduino - Distance1: {distanceCm1:.2f} cm, Distance2: {distanceCm2:.2f} cm")
                return distanceCm1, distanceCm2
            else:
                print(f"Unexpected response format: {response}")
                return None, None
        else:
            print("No response from Arduino")
            return None, None
            
    except Exception as e:
        print(f"Error during communication: {e}")
        return None, None

def main():
    """Main function to demonstrate Arduino motor control communication"""
    print("=== Raspberry Pi - Arduino Motor Control Communication ===\n")
    
    # Connect to Arduino
    arduino = connect_arduino()
    
    if arduino is None:
        return
    
    try:
        # Clear any initial serial buffer
        time.sleep(1)
        arduino.flushInput()
        
        # Get RPM values from user
        rpm1 = float(input("Enter target RPM for Motor 1: "))
        rpm2 = float(input("Enter target RPM for Motor 2: "))
        
        # Send RPM values and get distances
        dist1, dist2 = send_rpm_get_distance(arduino, rpm1, rpm2)
        
    finally:
        # Clean up
        if arduino and arduino.is_open:
            arduino.close()
            print("\nArduino connection closed")

if __name__ == "__main__":
    main()
