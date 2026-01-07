#!/usr/bin/env python3
"""
Z-Axis Controller with Camera Feed
Controls Z-axis position from 31 cm to 185 cm with live camera view
31 cm is the offset (home position)
"""

import cv2
from z_module import init_z_axis, z_axis, cleanup_z_axis
import threading

class ZAxisController:
    def __init__(self):
        # Offset and range
        self.OFFSET = 31  # cm
        self.MIN_POS = 31  # cm (minimum)
        self.MAX_POS = 185  # cm (maximum)
        
        # Current position (starts at offset)
        self.current_position = self.OFFSET
        
        # Initialize Z-axis
        print("Initializing Z-axis...")
        init_z_axis()
        
        # Initialize camera
        self.cap = None
        self.running = True
        self.init_camera()
        
    def init_camera(self):
        """Initialize camera"""
        print("Searching for camera...")
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    self.cap = cap
                    print(f"✓ Camera initialized at index {i}")
                    return
                cap.release()
        print("✗ No camera found")
    
    def camera_feed(self):
        """Display camera feed in separate window"""
        if self.cap is None:
            print("No camera available")
            return
        
        cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
        
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                cv2.imshow('Camera Feed', frame)
            
            # Break on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()
    
    def move_to_position(self, target_position):
        """Move to the specified position"""
        # Validate range
        if target_position < self.MIN_POS or target_position > self.MAX_POS:
            print(f"✗ Error: Position must be between {self.MIN_POS} and {self.MAX_POS} cm")
            return False
        
        # Calculate difference
        difference = target_position - self.current_position
        distance = abs(difference)
        
        if distance < 0.5:
            print("Already at target position")
            return True
        
        # Determine direction based on difference
        if difference > 0:
            direction = 1  # Positive difference = move UP
            direction_text = "UP"
        else:
            direction = 0  # Negative difference = move DOWN
            direction_text = "DOWN"
        
        # Move Z-axis
        print(f"Moving {direction_text} {distance:.1f} cm...")
        z_axis(distance, direction)
        
        # Update current position
        self.current_position = target_position
        print(f"✓ Moved {direction_text} {distance:.1f} cm - Now at {self.current_position:.1f} cm")
        return True
    
    def run(self):
        """Main control loop"""
        # Start camera feed in separate thread
        if self.cap is not None:
            camera_thread = threading.Thread(target=self.camera_feed, daemon=True)
            camera_thread.start()
        
        print("\n" + "="*50)
        print("Z-Axis Position Controller")
        print("="*50)
        print(f"Current offset: {self.OFFSET} cm")
        print(f"Current position: {self.current_position} cm")
        print(f"Valid range: {self.MIN_POS} - {self.MAX_POS} cm")
        print("Type 'quit' or 'exit' to stop")
        print("="*50 + "\n")
        
        try:
            while True:
                user_input = input(f"Enter target position (cm) [Current: {self.current_position} cm]: ").strip().lower()
                
                if user_input in ['quit', 'exit', 'q']:
                    break
                
                try:
                    target = float(user_input)
                    self.move_to_position(target)
                except ValueError:
                    print("✗ Invalid input. Please enter a number.")
        
        except KeyboardInterrupt:
            print("\n\nStopped by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        cleanup_z_axis()
        print("Cleanup complete")

def main():
    controller = ZAxisController()
    controller.run()

if __name__ == "__main__":
    main()
