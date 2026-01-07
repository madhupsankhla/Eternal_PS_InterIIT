import cv2
from pyzbar.pyzbar import decode
import numpy as np
import csv
import re
import os
import json
from collections import defaultdict

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
            header = next(reader, None)  # Read header: Database, Shelf_1, Shelf_2, ...
            
            if header and len(header) > 1:
                # Parse shelf numbers from header
                shelves = []
                for col in header[1:]:
                    # Extract number from "Shelf_X"
                    match = re.match(r'Shelf_(\d+)', col)
                    if match:
                        shelves.append(int(match.group(1)))
                
                # Read each rack row
                for row in reader:
                    if len(row) > 0:
                        # Extract rack number from "Rack_X"
                        rack_match = re.match(r'Rack_(\d+)', row[0])
                        if rack_match:
                            rack = int(rack_match.group(1))
                            
                            # Read items for each shelf
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
    Save grid to CSV file in matrix format:
    Database, Shelf_1, Shelf_2, Shelf_3, ...
    Rack_1, item, item, item, ...
    Rack_2, item, item, item, ...
    
    Fills in missing racks/shelves with NIL values.
    """
    if not grid:
        # Create empty file with just header
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Database'])
        return
    
    # Find max rack and shelf to create complete grid
    max_rack = max(rack for rack, shelf in grid.keys())
    max_shelf = max(shelf for rack, shelf in grid.keys())
    
    # Create complete range from 1 to max (fills in missing numbers)
    racks = list(range(1, max_rack + 1))
    shelves = list(range(1, max_shelf + 1))
    
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Write header row: Database, Shelf_1, Shelf_2, ...
        header = ['Database'] + [f'Shelf_{shelf}' for shelf in shelves]
        writer.writerow(header)
        
        # Write each rack row
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
    Returns: camera index or None if no camera found
    """
    print("Searching for available cameras...")
    for index in range(max_index):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            # Try to read a frame to verify it actually works
            ret, _ = cap.read()
            cap.release()
            if ret:
                print(f"✓ Found working camera at index {index}")
                return index
    print("✗ No working camera found")
    return None

def scan_qr_codes():
    """
    Real-time QR code scanner using webcam with OpenCV.
    Scans QR codes in format R{rack}_S{shelf}_ITM{item} and updates CSV grid.
    Press 'q' to quit the application.
    """
    csv_file = 'inventory_grid.csv'
    
    # Load existing grid
    grid = load_grid_from_csv(csv_file)
    print(f"Loaded {len(grid)} entries from {csv_file}")
    
    # Find available camera
    camera_index = find_available_camera()
    if camera_index is None:
        print("Error: Could not find any working camera")
        return
    
    # Initialize webcam with the found index
    cap = cv2.VideoCapture(camera_index)
    
    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam")
        return
    
    print("QR Code Scanner Started")
    print("Press 'q' to quit")
    print("Scanning for format: {\"qr_raw_data\": \"R{rack}_S{shelf}_ITM{item}\"}")
    print("Also accepts: R{rack}_S{shelf}_ITM{item}")
    
    # Store previously detected codes to avoid repeated detections
    previous_data = None
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to capture frame")
            break
        
        # Decode QR codes in the frame
        decoded_objects = decode(frame)
        
        # Process each detected QR code
        for obj in decoded_objects:
            # Get QR code data
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
            if qr_data != previous_data:
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
                previous_data = qr_data
        
        # Display the resulting frame
        cv2.imshow('QR Code Scanner - Press Q to Quit', frame)
        
        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()
    
    # Final save
    save_grid_to_csv(grid, csv_file)
    print("\nQR Code Scanner Stopped")
    print(f"Final grid contains {len(grid)} entries")

if __name__ == "__main__":
    scan_qr_codes()
