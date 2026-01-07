#!/usr/bin/env python3
"""
Integrated GUI for Motor Control with QR Scanning
Combines horizontal motor control with Z-axis scanning
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import csv
import re
import os

from motorControl.controller import MotorController
from z_module import init_z_axis, z_axis, cleanup_z_axis


class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control System")
        self.root.geometry("800x700")
        
        # Robot parameters
        self.WHEEL_DIAMETER = 20.32  # cm
        self.WHEEL_SEPARATION = 52.0  # cm
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.Z_OFFSET = 31  # cm (home position)
        self.SCAN_WAIT_TIME = 6  # seconds
        
        # Control variables
        self.motor = None
        self.running = False
        self.stop_requested = False
        self.current_z_position = self.Z_OFFSET
        
        # QR Scanner variables
        self.cap = None
        self.grid = {}
        self.csv_file = 'inventory_grid.csv'
        self.scanner_running = False
        self.scanner_thread = None
        self.qr_previous_data = [None]
        
        # Movement state
        self.in_scanning_mode = False
        self.scan_mode_active = False
        
        self.create_widgets()
        
    def create_widgets(self):
        """Create all GUI widgets"""
        
        # Main container with padding
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # ===== Configuration Section =====
        config_frame = ttk.LabelFrame(main_frame, text="Configuration", padding="10")
        config_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Stage positions (5 stages)
        ttk.Label(config_frame, text="Z-Axis Stage Positions (cm):").grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=5)
        
        self.stage_entries = []
        for i in range(5):
            ttk.Label(config_frame, text=f"Stage {i+1}:").grid(row=1+i//3, column=(i%3)*2, sticky=tk.W, padx=5)
            entry = ttk.Entry(config_frame, width=10)
            entry.grid(row=1+i//3, column=(i%3)*2+1, padx=5)
            self.stage_entries.append(entry)
        
        # Linear distance
        ttk.Label(config_frame, text="Linear Distance (cm):").grid(row=3, column=0, sticky=tk.W, padx=5, pady=5)
        self.l_dist_entry = ttk.Entry(config_frame, width=10)
        self.l_dist_entry.grid(row=3, column=1, padx=5, pady=5)
        
        # Linear speed (RPM)
        ttk.Label(config_frame, text="Linear Speed (RPM):").grid(row=3, column=2, sticky=tk.W, padx=5, pady=5)
        self.l_rpm_entry = ttk.Entry(config_frame, width=10)
        self.l_rpm_entry.grid(row=3, column=3, padx=5, pady=5)
        
        # Turn speed (RPM)
        ttk.Label(config_frame, text="Turn Speed (RPM):").grid(row=4, column=0, sticky=tk.W, padx=5, pady=5)
        self.t_rpm_entry = ttk.Entry(config_frame, width=10)
        self.t_rpm_entry.grid(row=4, column=1, padx=5, pady=5)
        
        # Turn time input (manual or calculated)
        ttk.Label(config_frame, text="Turn Time (sec):").grid(row=4, column=2, sticky=tk.W, padx=5, pady=5)
        self.t_time_entry = ttk.Entry(config_frame, width=10)
        self.t_time_entry.grid(row=4, column=3, padx=5, pady=5)
        
        # Linear time input (manual or calculated)
        ttk.Label(config_frame, text="Linear Time (sec):").grid(row=5, column=0, sticky=tk.W, padx=5, pady=5)
        self.l_time_entry = ttk.Entry(config_frame, width=10)
        self.l_time_entry.grid(row=5, column=1, padx=5, pady=5)
        
        # Calculate button
        calc_btn = ttk.Button(config_frame, text="Calculate Times", command=self.calculate_times)
        calc_btn.grid(row=6, column=0, columnspan=2, pady=10)
        
        # ===== Control Buttons Section =====
        control_frame = ttk.LabelFrame(main_frame, text="Control", padding="10")
        control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Start/Stop buttons
        btn_frame1 = ttk.Frame(control_frame)
        btn_frame1.grid(row=0, column=0, columnspan=4, pady=5)
        
        self.start_btn = ttk.Button(btn_frame1, text="START", command=self.start_system, width=15)
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(btn_frame1, text="STOP", command=self.stop_system, width=15, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        self.scan_btn = ttk.Button(btn_frame1, text="SCAN", command=self.start_scan_sequence, width=15, state=tk.DISABLED)
        self.scan_btn.pack(side=tk.LEFT, padx=5)
        
        self.refresh_btn = ttk.Button(btn_frame1, text="REFRESH", command=self.refresh_motors, width=15, state=tk.DISABLED)
        self.refresh_btn.pack(side=tk.LEFT, padx=5)
        
        # Direction buttons
        btn_frame2 = ttk.Frame(control_frame)
        btn_frame2.grid(row=1, column=0, columnspan=4, pady=10)
        
        # Top row - Forward
        self.forward_btn = ttk.Button(btn_frame2, text="↑ Forward", command=lambda: self.manual_move('forward'), width=12, state=tk.DISABLED)
        self.forward_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # Middle row - Left and Right
        self.left_btn = ttk.Button(btn_frame2, text="← Left", command=lambda: self.manual_move('left'), width=12, state=tk.DISABLED)
        self.left_btn.grid(row=1, column=0, padx=5, pady=5)
        
        self.right_btn = ttk.Button(btn_frame2, text="Right →", command=lambda: self.manual_move('right'), width=12, state=tk.DISABLED)
        self.right_btn.grid(row=1, column=2, padx=5, pady=5)
        
        # Bottom row - Backward
        self.backward_btn = ttk.Button(btn_frame2, text="↓ Backward", command=lambda: self.manual_move('backward'), width=12, state=tk.DISABLED)
        self.backward_btn.grid(row=2, column=1, padx=5, pady=5)
        
        # ===== Obstacle Avoidance Section =====
        obstacle_frame = ttk.LabelFrame(main_frame, text="Obstacle Avoidance", padding="10")
        obstacle_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.obstacle_left_btn = ttk.Button(obstacle_frame, text="Obstacle LEFT", command=lambda: self.obstacle_avoidance('left'), width=20, state=tk.DISABLED)
        self.obstacle_left_btn.pack(side=tk.LEFT, padx=10, pady=5)
        
        self.obstacle_right_btn = ttk.Button(obstacle_frame, text="Obstacle RIGHT", command=lambda: self.obstacle_avoidance('right'), width=20, state=tk.DISABLED)
        self.obstacle_right_btn.pack(side=tk.LEFT, padx=10, pady=5)
        
        # ===== Status Section =====
        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="10")
        status_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.status_text = tk.Text(status_frame, height=15, width=80, wrap=tk.WORD)
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_text.config(yscrollcommand=scrollbar.set)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(3, weight=1)
        
    def log(self, message):
        """Add message to status text box"""
        self.status_text.insert(tk.END, message + "\n")
        self.status_text.see(tk.END)
        print(message)
        
    def calculate_times(self):
        """Calculate time required for turn and linear movements"""
        try:
            # Calculate turn time
            t_rpm = float(self.t_rpm_entry.get())
            if t_rpm <= 0:
                raise ValueError("Turn speed must be positive")
            
            # For 90 degree turn with differential drive:
            # Arc length = (π/2) * wheel_separation
            # Distance per wheel = arc_length
            arc_length = (math.pi / 2) * self.WHEEL_SEPARATION
            
            # Speed of each wheel (cm/min) = RPM * wheel_circumference
            wheel_speed_cm_per_min = t_rpm * self.WHEEL_CIRCUMFERENCE
            
            # Time (minutes) = distance / speed
            time_minutes = arc_length / wheel_speed_cm_per_min
            
            # Convert to seconds
            t_time = time_minutes * 60
            
            self.t_time_entry.delete(0, tk.END)
            self.t_time_entry.insert(0, f"{t_time:.2f}")
            self.log(f"✓ Turn time calculated: {t_time:.2f} seconds for 90° turn")
            
            # Calculate linear time
            l_rpm = float(self.l_rpm_entry.get())
            l_dist = float(self.l_dist_entry.get())
            
            if l_rpm <= 0:
                raise ValueError("Linear speed must be positive")
            if l_dist <= 0:
                raise ValueError("Linear distance must be positive")
            
            # Linear movement time
            wheel_speed_linear = l_rpm * self.WHEEL_CIRCUMFERENCE  # cm/min
            l_time = (l_dist / wheel_speed_linear) * 60  # seconds
            
            self.l_time_entry.delete(0, tk.END)
            self.l_time_entry.insert(0, f"{l_time:.2f}")
            self.log(f"✓ Linear time calculated: {l_time:.2f} seconds for {l_dist} cm at {l_rpm} RPM")
            
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")
            
    def get_stage_positions(self):
        """Get stage positions from entry fields"""
        positions = []
        for i, entry in enumerate(self.stage_entries):
            val = entry.get().strip()
            if val:
                try:
                    pos = float(val)
                    if 31 <= pos <= 185:
                        positions.append(pos)
                    else:
                        raise ValueError(f"Stage {i+1} position must be between 31-185 cm")
                except ValueError as e:
                    raise ValueError(f"Invalid stage {i+1} position: {e}")
        return positions
        
    def validate_inputs(self):
        """Validate all input fields"""
        try:
            # Get stage positions
            positions = self.get_stage_positions()
            if not positions:
                raise ValueError("At least one stage position is required")
            
            # Get linear distance
            l_dist = float(self.l_dist_entry.get())
            if l_dist <= 0:
                raise ValueError("Linear distance must be positive")
            
            # Get linear RPM
            l_rpm = float(self.l_rpm_entry.get())
            if l_rpm <= 0:
                raise ValueError("Linear speed must be positive")
            
            # Get turn RPM
            t_rpm = float(self.t_rpm_entry.get())
            if t_rpm <= 0:
                raise ValueError("Turn speed must be positive")
            
            # Get times from entry fields (user input or calculated)
            t_time_str = self.t_time_entry.get().strip()
            l_time_str = self.l_time_entry.get().strip()
            
            if not t_time_str or not l_time_str:
                raise ValueError("Please provide Turn Time and Linear Time (calculate or enter manually)")
            
            t_time = float(t_time_str)
            l_time = float(l_time_str)
            
            if t_time <= 0 or l_time <= 0:
                raise ValueError("Times must be positive values")
            
            return positions, l_dist, l_rpm, t_rpm, t_time, l_time
            
        except ValueError as e:
            messagebox.showerror("Validation Error", str(e))
            return None
            
    def start_system(self):
        """Initialize and start the robot system"""
        # Validate inputs
        validation = self.validate_inputs()
        if validation is None:
            return
        
        positions, l_dist, l_rpm, t_rpm, t_time, l_time = validation
        
        self.log("="*50)
        self.log("Initializing Robot System...")
        self.log("="*50)
        
        # Store configuration
        self.positions = positions
        self.l_dist = l_dist
        self.l_rpm = l_rpm
        self.t_rpm = t_rpm
        self.t_time = t_time
        self.l_time = l_time
        
        # Calculate movement times
        self.calculate_movement_times()
        
        # Initialize motor controller
        try:
            self.log("Connecting to Arduino...")
            self.motor = MotorController(port='/dev/ttyACM0', baudrate=115200)
            if self.motor.arduino is None:
                raise Exception("Failed to connect to Arduino")
            self.log("✓ Motor controller initialized")
        except Exception as e:
            self.log(f"✗ Error: {e}")
            messagebox.showerror("Connection Error", str(e))
            return
        
        # Initialize Z-axis
        try:
            self.log("Initializing Z-axis...")
            init_z_axis()
            self.current_z_position = self.Z_OFFSET
            self.log("✓ Z-axis initialized")
        except Exception as e:
            self.log(f"✗ Error: {e}")
            messagebox.showerror("Z-axis Error", str(e))
            return
        
        # Initialize camera and QR scanner
        try:
            self.log("Initializing camera...")
            camera_index = self.find_available_camera()
            if camera_index is None:
                raise Exception("No camera found")
            
            self.cap = cv2.VideoCapture(camera_index)
            if not self.cap.isOpened():
                raise Exception("Could not open camera")
            
            self.log(f"✓ Camera initialized (index {camera_index})")
            
            # Load existing grid
            self.grid = self.load_grid_from_csv()
            self.log(f"✓ Loaded {len(self.grid)} entries from {self.csv_file}")
            
            # Start QR scanner thread
            self.scanner_running = True
            self.qr_previous_data = [None]
            self.scanner_thread = threading.Thread(
                target=self.qr_scanner_thread,
                daemon=True
            )
            self.scanner_thread.start()
            self.log("✓ QR scanner started")
            
        except Exception as e:
            self.log(f"✗ Camera/QR Error: {e}")
            messagebox.showerror("Camera Error", str(e))
            return
        
        # Update UI
        self.running = True
        self.stop_requested = False
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.scan_btn.config(state=tk.NORMAL)
        self.refresh_btn.config(state=tk.NORMAL)
        self.forward_btn.config(state=tk.NORMAL)
        self.left_btn.config(state=tk.NORMAL)
        self.right_btn.config(state=tk.NORMAL)
        self.backward_btn.config(state=tk.NORMAL)
        self.obstacle_left_btn.config(state=tk.NORMAL)
        self.obstacle_right_btn.config(state=tk.NORMAL)
        
        self.log("\n✓ System ready! Use direction buttons or press SCAN for scanning sequence")
        self.log("="*50)
        
    def stop_system(self):
        """Stop the robot system"""
        self.log("\n" + "="*50)
        self.log("Stopping system...")
        self.log("="*50)
        
        self.stop_requested = True
        self.running = False
        
        # Stop motors
        if self.motor:
            try:
                self.motor.setRPM(0, 0)
                time.sleep(0.2)
                self.motor.close()
                self.log("✓ Motors stopped")
            except Exception as e:
                self.log(f"✗ Motor stop error: {e}")
        
        # Stop QR scanner
        self.scanner_running = False
        if self.scanner_thread and self.scanner_thread.is_alive():
            self.log("Waiting for scanner thread to stop...")
            self.scanner_thread.join(timeout=2.0)
        
        if self.cap:
            time.sleep(0.3)
            self.cap.release()
            self.cap = None
            cv2.destroyAllWindows()
            time.sleep(0.2)
            self.log("✓ Camera released")
        
        # Save final grid
        try:
            self.save_grid_to_csv(self.grid)
            self.log(f"✓ Saved {len(self.grid)} entries to {self.csv_file}")
        except Exception as e:
            self.log(f"✗ Save error: {e}")
        
        # Cleanup Z-axis
        try:
            cleanup_z_axis()
            self.log("✓ Z-axis cleanup complete")
        except Exception as e:
            self.log(f"✗ Z-axis cleanup error: {e}")
        
        # Reset scanner thread reference
        self.scanner_thread = None
        
        # Update UI
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.scan_btn.config(state=tk.DISABLED)
        self.refresh_btn.config(state=tk.DISABLED)
        self.forward_btn.config(state=tk.DISABLED)
        self.left_btn.config(state=tk.DISABLED)
        self.right_btn.config(state=tk.DISABLED)
        self.backward_btn.config(state=tk.DISABLED)
        self.obstacle_left_btn.config(state=tk.DISABLED)
        self.obstacle_right_btn.config(state=tk.DISABLED)
        
        self.log("\n✓ System stopped successfully")
        
    def calculate_movement_times(self):
        """Log the pre-calculated movement times"""
        self.log(f"Movement times:")
        self.log(f"  - Linear: {self.l_time:.2f} sec for {self.l_dist} cm at {self.l_rpm} RPM")
        self.log(f"  - Turn: {self.t_time:.2f} sec for 90° at {self.t_rpm} RPM")
    
    def refresh_motors(self):
        """Refresh motor connection and update times from entry fields"""
        if not self.running:
            return
        
        try:
            self.log("\n→ Refreshing motors...")
            
            # Update times from entry fields
            self.update_times_from_entries()
            
            # Send stop command to reset motor state
            self.motor.setRPM(0, 0)
            time.sleep(0.1)
            
            # Send a brief pulse to wake up motors
            self.motor.setRPM(5, 5)
            time.sleep(0.1)
            self.motor.setRPM(0, 0)
            time.sleep(0.1)
            
            self.log("✓ Motors refreshed and ready")
            self.log(f"  - Turn Time: {self.t_time:.2f} sec")
            self.log(f"  - Linear Time: {self.l_time:.2f} sec")
            
        except Exception as e:
            self.log(f"✗ Refresh error: {e}")
    
    def update_times_from_entries(self):
        """Update time values from entry fields before each action"""
        try:
            t_time_str = self.t_time_entry.get().strip()
            l_time_str = self.l_time_entry.get().strip()
            
            if t_time_str:
                self.t_time = float(t_time_str)
            if l_time_str:
                self.l_time = float(l_time_str)
                
        except ValueError:
            pass  # Keep existing values if input is invalid
        
    def manual_move(self, direction):
        """Handle manual movement buttons"""
        if not self.running or self.stop_requested:
            return
        
        # Run in separate thread to not block GUI
        thread = threading.Thread(target=self._execute_manual_move, args=(direction,), daemon=True)
        thread.start()
        
    def _execute_manual_move(self, direction):
        """Execute manual movement in background thread"""
        try:
            # Update times from entry fields before action
            self.update_times_from_entries()
            
            if direction == 'forward':
                self.log(f"\n→ Moving FORWARD {self.l_dist} cm at {self.l_rpm} RPM for {self.l_time:.2f}s...")
                self.log(f"[DEBUG] Sending: setRPM({self.l_rpm}, {self.l_rpm})")
                result = self.motor.setRPM(self.l_rpm, self.l_rpm)
                self.log(f"[DEBUG] setRPM returned: {result}")
                time.sleep(self.l_time)
                self.motor.setRPM(0, 0)
                self.log("✓ Forward movement complete")
                
            elif direction == 'backward':
                self.log(f"\n→ Moving BACKWARD {self.l_dist} cm at {self.l_rpm} RPM for {self.l_time:.2f}s...")
                self.log(f"[DEBUG] Sending: setRPM({-self.l_rpm}, {-self.l_rpm})")
                result = self.motor.setRPM(-self.l_rpm, -self.l_rpm)
                self.log(f"[DEBUG] setRPM returned: {result}")
                time.sleep(self.l_time)
                self.motor.setRPM(0, 0)
                self.log("✓ Backward movement complete")
                
            elif direction == 'left':
                self.log(f"\n→ Turning LEFT 90° at {self.t_rpm} RPM for {self.t_time:.2f}s...")
                left_rpm = float(self.t_rpm)
                right_rpm = -float(self.t_rpm)
                self.log(f"[DEBUG] Calculated: left_rpm={left_rpm}, right_rpm={right_rpm}")
                self.log(f"[DEBUG] Sending: setRPM({left_rpm}, {right_rpm})")
                result = self.motor.setRPM(left_rpm, right_rpm)
                self.log(f"[DEBUG] setRPM returned: {result}")
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                self.log("✓ Left turn complete")
                
            elif direction == 'right':
                self.log(f"\n→ Turning RIGHT 90° at {self.t_rpm} RPM for {self.t_time:.2f}s...")
                left_rpm = -float(self.t_rpm)
                right_rpm = float(self.t_rpm)
                self.log(f"[DEBUG] Calculated: left_rpm={left_rpm}, right_rpm={right_rpm}")
                self.log(f"[DEBUG] Sending: setRPM({left_rpm}, {right_rpm})")
                result = self.motor.setRPM(left_rpm, right_rpm)
                self.log(f"[DEBUG] setRPM returned: {result}")
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                self.log("✓ Right turn complete")
                
        except Exception as e:
            self.log(f"✗ Movement error: {e}")
            import traceback
            self.log(f"[DEBUG] Traceback: {traceback.format_exc()}")
            
    def start_scan_sequence(self):
        """Start the scan sequence in a background thread"""
        if not self.running or self.stop_requested:
            return
        
        thread = threading.Thread(target=self.execute_z_axis_scan, daemon=True)
        thread.start()
        
    def execute_z_axis_scan(self):
        """Execute full Z-axis scanning through all stages"""
        if self.stop_requested:
            return
        
        try:
            self.log("\n" + "="*50)
            self.log("Z-AXIS SCANNING SEQUENCE")
            self.log("="*50)
            
            # Move through all stage positions
            for i, target_position in enumerate(self.positions, 1):
                if self.stop_requested:
                    break
                
                self.log(f"\nStage {i}/{len(self.positions)}: Moving to {target_position} cm")
                
                distance = abs(target_position - self.current_z_position)
                
                if distance < 0.5:
                    self.log("Already at target position")
                else:
                    # Determine direction
                    if target_position > self.current_z_position:
                        direction = 1  # UP
                        direction_text = "UP"
                    else:
                        direction = 0  # DOWN
                        direction_text = "DOWN"
                    
                    self.log(f"Moving {direction_text} {distance:.1f} cm...")
                    z_axis(distance, direction)
                    self.current_z_position = target_position
                    self.log(f"✓ Reached position: {self.current_z_position} cm")
                
                # Wait for QR scanning
                self.log(f"Scanning for {self.SCAN_WAIT_TIME} seconds...")
                time.sleep(self.SCAN_WAIT_TIME)
            
            # Return to home position
            if self.current_z_position != self.Z_OFFSET and not self.stop_requested:
                self.log(f"\nReturning to home position ({self.Z_OFFSET} cm)...")
                distance = abs(self.current_z_position - self.Z_OFFSET)
                
                # Compensate for 14.5 cm offset issue
                if distance > 14.5:
                    distance = distance - 14.5
                    self.log(f"[Compensation] Adjusted distance by -14.5 cm")
                
                if self.current_z_position > self.Z_OFFSET:
                    direction = 0  # DOWN
                    direction_text = "DOWN"
                else:
                    direction = 1  # UP
                    direction_text = "UP"
                
                self.log(f"Moving {direction_text} {distance:.1f} cm...")
                z_axis(distance, direction)
                self.current_z_position = self.Z_OFFSET
                self.log(f"✓ Returned to home position")
            
            self.log("\n✓ Z-axis scanning sequence complete!")
            self.log("="*50)
            
        except Exception as e:
            self.log(f"✗ Z-axis scan error: {e}")
            
    def obstacle_avoidance(self, side):
        """Execute obstacle avoidance maneuver"""
        if not self.running or self.stop_requested:
            return
        
        thread = threading.Thread(target=self._execute_obstacle_avoidance, args=(side,), daemon=True)
        thread.start()
        
    def _execute_obstacle_avoidance(self, side):
        """Execute obstacle avoidance in background thread"""
        try:
            self.log("\n" + "="*50)
            self.log(f"OBSTACLE AVOIDANCE - {side.upper()}")
            self.log("="*50)
            
            # Stop current movement
            self.motor.setRPM(0, 0)
            time.sleep(0.5)
            
            if side == 'left':
                # Left rectangle path
                self.log("1. Turn LEFT 90°")
                self.motor.setRPM(self.t_rpm, -self.t_rpm)
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log(f"2. Forward {self.l_dist} cm")
                self.motor.setRPM(self.t_rpm, self.t_rpm)
                time.sleep(self.l_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log("3. Turn RIGHT 90°")
                self.motor.setRPM(-self.t_rpm, self.t_rpm)
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log(f"4. Forward {self.l_dist} cm")
                self.motor.setRPM(self.t_rpm, self.t_rpm)
                time.sleep(self.l_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log("5. Turn RIGHT 90°")
                self.motor.setRPM(-self.t_rpm, self.t_rpm)
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log(f"6. Forward {self.l_dist} cm")
                self.motor.setRPM(self.t_rpm, self.t_rpm)
                time.sleep(self.l_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log("7. Turn LEFT 90°")
                self.motor.setRPM(self.t_rpm, -self.t_rpm)
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
            else:  # right
                # Right rectangle path
                self.log("1. Turn RIGHT 90°")
                self.motor.setRPM(-self.t_rpm, self.t_rpm)
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log(f"2. Forward {self.l_dist} cm")
                self.motor.setRPM(self.t_rpm, self.t_rpm)
                time.sleep(self.l_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log("3. Turn LEFT 90°")
                self.motor.setRPM(self.t_rpm, -self.t_rpm)
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log(f"4. Forward {self.l_dist} cm")
                self.motor.setRPM(self.t_rpm, self.t_rpm)
                time.sleep(self.l_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log("5. Turn LEFT 90°")
                self.motor.setRPM(self.t_rpm, -self.t_rpm)
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log(f"6. Forward {self.l_dist} cm")
                self.motor.setRPM(self.t_rpm, self.t_rpm)
                time.sleep(self.l_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
                
                self.log("7. Turn RIGHT 90°")
                self.motor.setRPM(-self.t_rpm, self.t_rpm)
                time.sleep(self.t_time)
                self.motor.setRPM(0, 0)
                time.sleep(0.5)
            self.log("✓ Obstacle avoidance complete!")
            self.log("="*50)
            
        except Exception as e:
            self.log(f"✗ Obstacle avoidance error: {e}")
            
    # ===== QR Scanner Functions =====
    
    def find_available_camera(self, max_index=10):
        """Find first available camera"""
        for index in range(max_index):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                ret, _ = cap.read()
                cap.release()
                if ret:
                    return index
        return None
        
    def qr_scanner_thread(self):
        """QR scanner thread - runs continuously"""
        while self.scanner_running:
            try:
                ret, frame = self.cap.read()
                
                if not ret:
                    continue
                
                # Decode QR codes
                decoded_objects = decode(frame)
                
                for obj in decoded_objects:
                    qr_data = obj.data.decode('utf-8')
                    qr_type = obj.type
                    
                    # Draw bounding box
                    points = obj.polygon
                    if len(points) > 4:
                        hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                        points = hull
                    
                    n = len(points)
                    for j in range(n):
                        cv2.line(frame, tuple(points[j]), tuple(points[(j+1) % n]), (0, 255, 0), 3)
                    
                    # Display text
                    x, y = obj.rect.left, obj.rect.top
                    w, h = obj.rect.width, obj.rect.height
                    cv2.rectangle(frame, (x, y - 30), (x + w, y), (0, 255, 0), -1)
                    cv2.putText(frame, f'{qr_type}: {qr_data}', (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    
                    # Process new QR code
                    if qr_data != self.qr_previous_data[0]:
                        self.log(f"\n[QR] Scanned: {qr_data}")
                        
                        parsed = self.parse_qr_data(qr_data)
                        if parsed:
                            rack, shelf, item = parsed
                            self.grid[(rack, shelf)] = item
                            self.log(f"[QR] ✓ Rack {rack}, Shelf {shelf}, Item {item}")
                            self.save_grid_to_csv(self.grid)
                        else:
                            self.log(f"[QR] ✗ Invalid format")
                        
                        self.qr_previous_data[0] = qr_data
                
                # Display frame
                cv2.imshow('QR Code Scanner', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
            except Exception as e:
                self.log(f"[QR] Error: {e}")
                break
        
        cv2.destroyAllWindows()
        
    def parse_qr_data(self, qr_text):
        """
        Parse QR code format: {"qr_raw_data": "R{rack}_S{shelf}_ITM{item}"}
        or directly: R{rack}_S{shelf}_ITM{item}
        """
        # Try to parse as JSON first
        try:
            import json
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
            return (int(match.group(1)), int(match.group(2)), int(match.group(3)))
        return None
        
    def load_grid_from_csv(self):
        """Load grid from CSV file"""
        grid = {}
        if os.path.exists(self.csv_file):
            try:
                with open(self.csv_file, 'r', newline='') as f:
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
            except Exception as e:
                self.log(f"Error loading CSV: {e}")
        return grid
        
    def save_grid_to_csv(self, grid):
        """Save grid to CSV file"""
        if not grid:
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Database'])
            return
        
        max_rack = max(rack for rack, shelf in grid.keys())
        max_shelf = max(shelf for rack, shelf in grid.keys())
        
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['Database'] + [f'Shelf_{shelf}' for shelf in range(1, max_shelf + 1)]
            writer.writerow(header)
            
            for rack in range(1, max_rack + 1):
                row = [f'Rack_{rack}']
                for shelf in range(1, max_shelf + 1):
                    item = grid.get((rack, shelf), 'NIL')
                    row.append(item)
                writer.writerow(row)


def main():
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
