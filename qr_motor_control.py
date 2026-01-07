#!/usr/bin/env python3
"""
QR Motor Control
Moves to specified stages (in cm) and scans QR codes at each position
Integrates Z-axis motor control with QR code scanning
"""

import cv2
from pyzbar.pyzbar import decode
import numpy as np
import csv
import re
import os
import time
import json
import threading
from z_module import init_z_axis, z_axis, cleanup_z_axis

def parse_qr_data(qr_text):
    """
    Parse QR code format: {"qr_raw_data": "R{rack}_S{shelf}_ITM{item}"}
    or directly: R{rack}_S{shelf}_ITM{item}
    Returns: (rack_number, shelf_number, item_number) or None if invalid
    """
    # Try to parse as JSON first
    try:
        data = json.loads(qr_text)
        if isinstance(data, dict) and 'qr_raw_data' in data:
            qr_text = data['qr_raw_data']
    except (json.JSONDecodeError, ValueError):
        # Not JSON, use original text
        pass
    
    # Parse the actual QR code pattern
    pattern = r'R(\d+)_S(\d+)_ITM(\d+)'
    match = re.match(pattern, qr_text)
    if match:
        rack = int(match.group(1))
        shelf = int(match.group(2))
        item = int(match.group(3))
        return (rack, shelf, item)
    return None

def load_grid_from_csv(csv_file='inventory_grid.csv'):
    """
    Load existing grid from CSV file in matrix format.
    Returns: dict with (rack, shelf) as key and item number as value
    Ignores 'NIL' entries.
    """
    grid = {}
    if os.path.exists(csv_file):
        with open(csv_file, 'r', newline='') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            
            if header and len(header) > 1:
                shelves = []
                for col in header[1:]:
                    match = re.match(r'Shelf_(\d+)', col)
                    if match:
                        shelves.append(int(match.group(1)))
                
                for row in reader:
                    if len(row) > 0:
                        rack_match = re.match(r'Rack_(\d+)', row[0])
                        if rack_match:
                            rack = int(rack_match.group(1))
                            
                            for i, shelf in enumerate(shelves):
                                if i + 1 < len(row) and row[i + 1].strip() and row[i + 1].strip() != 'NIL':
                                    try:
                                        item = int(row[i + 1])
                                        grid[(rack, shelf)] = item
                                    except ValueError:
                                        continue
    return grid

def save_grid_to_csv(grid, csv_file='inventory_grid.csv'):
    """
    Save grid to CSV file in matrix format.
    """
    if not grid:
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Database'])
        return
    
    max_rack = max(rack for rack, shelf in grid.keys())
    max_shelf = max(shelf for rack, shelf in grid.keys())
    
    racks = list(range(1, max_rack + 1))
    shelves = list(range(1, max_shelf + 1))
    
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        
        header = ['Database'] + [f'Shelf_{shelf}' for shelf in shelves]
        writer.writerow(header)
        
        for rack in racks:
            row = [f'Rack_{rack}']
            for shelf in shelves:
                item = grid.get((rack, shelf), 'NIL')
                row.append(item)
            writer.writerow(row)
    
    print(f"Grid saved to {csv_file}")

def find_available_camera(max_index=10):
    """
    Check camera indices from 0 to max_index and return the first available one.
    """
    print("Searching for available cameras...")
    for index in range(max_index):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                print(f"✓ Found working camera at index {index}")
                return index
    print("✗ No working camera found")
    return None

def qr_scanner_thread(cap, grid, csv_file, running_flag, previous_data):
    """QR scanner thread - runs continuously like qr_scanner.py"""
    while running_flag[0]:
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to capture frame")
            break
        
        # Decode QR codes in the frame
        decoded_objects = decode(frame)
        
        # Process each detected QR code
        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')
            qr_type = obj.type
            
            # Get the bounding box coordinates
            points = obj.polygon
            
            # If the points do not form a quad, find convex hull
            if len(points) > 4:
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                points = hull
            
            # Draw the bounding box around the QR code
            n = len(points)
            for j in range(n):
                cv2.line(frame, tuple(points[j]), tuple(points[(j+1) % n]), (0, 255, 0), 3)
            
            # Draw a rectangle for the text background
            x = obj.rect.left
            y = obj.rect.top
            w = obj.rect.width
            h = obj.rect.height
            
            # Display the QR code type and data on the frame
            cv2.rectangle(frame, (x, y - 30), (x + w, y), (0, 255, 0), -1)
            cv2.putText(frame, f'{qr_type}: {qr_data}', (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
            # Print to console only when new code is detected
            if qr_data != previous_data[0]:
                print(f"\n{'='*50}")
                print(f"QR Code Type: {qr_type}")
                print(f"Data: {qr_data}")
                
                # Parse and update grid
                parsed = parse_qr_data(qr_data)
                if parsed:
                    rack, shelf, item = parsed
                    grid[(rack, shelf)] = item
                    print(f"✓ Parsed: Rack {rack}, Shelf {shelf}, Item {item}")
                    
                    # Save to CSV after each successful scan
                    save_grid_to_csv(grid, csv_file)
                else:
                    print(f"✗ Invalid format. Expected: R{{rack}}_S{{shelf}}_ITM{{item}}")
                
                print(f"{'='*50}")
                previous_data[0] = qr_data
        
        # Display the resulting frame
        cv2.imshow('QR Code Scanner - Press Q to Quit', frame)
        
        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running_flag[0] = False
            break
    
    cv2.destroyAllWindows()

def get_stage_positions():
    """
    Get stage positions (in cm) from user input until 'stop' is entered.
    If user types 'stop' immediately, load from saved JSON file.
    Returns: list of positions in cm
    """
    json_file = 'stage_positions.json'
    positions = []
    
    print("\n" + "="*50)
    print("Enter stage positions in cm (31-185 cm)")
    print("Type 'stop' when done")
    print("="*50)
    
    first_input = True
    
    while True:
        user_input = input("Enter position (cm): ").strip().lower()
        
        if user_input == 'stop':
            # If first input is 'stop', load from saved file
            if first_input and os.path.exists(json_file):
                try:
                    with open(json_file, 'r') as f:
                        saved_data = json.load(f)
                        positions = saved_data.get('positions', [])
                        print(f"✓ Loaded {len(positions)} saved positions from {json_file}")
                        print(f"Positions: {positions}")
                except Exception as e:
                    print(f"✗ Error loading saved positions: {e}")
            break
        
        first_input = False
        
        try:
            position = float(user_input)
            if 31 <= position <= 185:
                positions.append(position)
                print(f"✓ Added position: {position} cm")
            else:
                print("✗ Position must be between 31 and 185 cm")
        except ValueError:
            print("✗ Invalid input. Enter a number or 'stop'")
    
    # Save positions to JSON file if new positions were entered
    if positions and not first_input:
        try:
            with open(json_file, 'w') as f:
                json.dump({'positions': positions}, f, indent=2)
            print(f"✓ Saved {len(positions)} positions to {json_file}")
        except Exception as e:
            print(f"✗ Error saving positions: {e}")
    
    return positions

def main():
    csv_file = 'inventory_grid.csv'
    OFFSET = 31  # cm (home position)
    
    # Get stage positions from user
    positions = get_stage_positions()
    
    if not positions:
        print("No positions entered. Exiting.")
        return
    
    print(f"\nStage positions: {positions}")
    print(f"Total stages: {len(positions)}\n")
    
    # Initialize Z-axis
    print("Initializing Z-axis...")
    init_z_axis()
    
    # Load existing grid
    grid = load_grid_from_csv(csv_file)
    print(f"Loaded {len(grid)} entries from {csv_file}")
    
    # Find and initialize camera
    camera_index = find_available_camera()
    if camera_index is None:
        print("Error: Could not find any working camera")
        cleanup_z_axis()
        return
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print("Error: Could not open webcam")
        cleanup_z_axis()
        return
    
    # QR scanner state
    running_flag = [True]
    previous_data = [None]
    
    # Start QR scanner thread (exactly like qr_scanner.py)
    scanner_thread = threading.Thread(
        target=qr_scanner_thread, 
        args=(cap, grid, csv_file, running_flag, previous_data), 
        daemon=True
    )
    scanner_thread.start()
    
    print("QR Code Scanner Started")
    print("Press 'q' in camera window to quit")
    print("Scanning for format: {\"qr_raw_data\": \"R{rack}_S{shelf}_ITM{item}\"}")
    print("Also accepts: R{rack}_S{shelf}_ITM{item}")
    
    try:
        current_position = OFFSET  # Start at home position (31cm absolute)
        
        # Move through all positions
        for i, target_position in enumerate(positions, 1):
            if not running_flag[0]:  # Stop if scanner was quit
                break
                
            print(f"\n{'='*50}")
            print(f"Stage {i}/{len(positions)}: Moving to {target_position} cm")
            print(f"{'='*50}")
            
            # Calculate relative distance from offset (31cm)
            # If target > current: move UP, if target < current: move DOWN
            distance = abs(target_position - current_position)
            
            if distance < 0.5:
                print("Already at target position")
            else:
                # Direction: 1 = UP (away from offset), 0 = DOWN (toward offset)
                if target_position > current_position:
                    direction = 1  # Moving UP
                else:
                    direction = 0  # Moving DOWN
                
                direction_text = "UP" if direction == 1 else "DOWN"
                
                print(f"Moving {direction_text} {distance:.1f} cm...")
                z_axis(distance, direction)
                current_position = target_position
                print(f"✓ Reached position: {current_position} cm")
            
            # Wait 6 seconds at this position for scanning
            print(f"\nWaiting 6 seconds at position {current_position} cm for QR scanning...")
            time.sleep(6)
        
        # Return to home position (offset = 31cm)
        if current_position != OFFSET:
            print(f"\n{'='*50}")
            print(f"All stages completed! Returning to home position ({OFFSET} cm)...")
            print(f"{'='*50}")
            
            distance = abs(current_position - OFFSET)
            
            # If current > offset: move DOWN, if current < offset: move UP
            if current_position > OFFSET:
                direction = 0  # Moving DOWN toward offset
                direction_text = "DOWN"
            else:
                direction = 1  # Moving UP toward offset
                direction_text = "UP"
            
            print(f"Moving {direction_text} {distance:.1f} cm...")
            z_axis(distance, direction)
            current_position = OFFSET
            print(f"✓ Returned to home position: {OFFSET} cm")
        else:
            print(f"\nAlready at home position: {OFFSET} cm")
        
        print(f"\nAll movements complete! Final grid contains {len(grid)} entries")
        print("QR scanner still running. Press 'q' in camera window to quit.")
        
        # Keep running until user quits scanner
        while running_flag[0]:
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nOperation interrupted by user")
        running_flag[0] = False
    except Exception as e:
        print(f"\nError: {e}")
        running_flag[0] = False
    finally:
        # Cleanup
        running_flag[0] = False
        time.sleep(0.5)
        cap.release()
        cv2.destroyAllWindows()
        cleanup_z_axis()
        
        # Final save
        save_grid_to_csv(grid, csv_file)
        print("\nQR Code Scanner Stopped")
        print(f"Final grid contains {len(grid)} entries")
        print("Cleanup complete")

if __name__ == "__main__":
    main()
