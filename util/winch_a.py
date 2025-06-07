#!/usr/bin/env python3
"""
Interactive sail control CLI with position tracking
Usage: python3 winch_a.py
"""

import serial
import struct
import math
import time
import sys
import json
import os

# Serial configuration
SERIAL_PORT = '/dev/arduino_control'
BAUD_RATE = 115200

# Command codes
CMD_WINCH_CW_STEPS = 0x12   # Let sail out
CMD_WINCH_CCW_STEPS = 0x13  # Bring sail in

# Sail parameters
BOOM_LENGTH = 48
WINCH_TO_MAST = 44
SPOOL_RADIUS = 1.5
GEAR_RATIO = 5
STEPS_PER_REVOLUTION = 1600

# State file
STATE_FILE = os.path.expanduser("~/.sail_position.json")

class SailController:
    def __init__(self):
        self.current_angle = self.load_position()
        self.ser = None
        
    def load_position(self):
        """Load saved position from file"""
        try:
            with open(STATE_FILE, 'r') as f:
                data = json.load(f)
                return data.get('angle', 0.0)
        except:
            return 0.0
    
    def save_position(self):
        """Save current position to file"""
        with open(STATE_FILE, 'w') as f:
            json.dump({'angle': self.current_angle}, f)
    
    def connect(self):
        """Connect to Arduino"""
        if not self.ser:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Arduino reset
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.ser:
            self.ser.close()
            self.ser = None
    
    def angle_to_steps(self, angle):
        """Convert angle to absolute steps from 0"""
        angle = abs(angle)
        angle = max(0.0, min(88.0, angle))
        
        # Calculate the rope length at current angle
        length = math.sqrt(
            BOOM_LENGTH**2 + WINCH_TO_MAST**2 - 
            2 * BOOM_LENGTH * WINCH_TO_MAST * math.cos(math.radians(angle))
        )
        
        # Calculate the minimum rope length when sail is at 0 degrees (fully in)
        min_length = math.sqrt(
            BOOM_LENGTH**2 + WINCH_TO_MAST**2 - 
            2 * BOOM_LENGTH * WINCH_TO_MAST * math.cos(math.radians(0))
        )
        
        # The actual rope to let out is the difference from minimum
        rope_out = length - min_length
        
        return int((rope_out * GEAR_RATIO * STEPS_PER_REVOLUTION) / (2 * math.pi * SPOOL_RADIUS))
    
    def move_to_angle(self, target_angle):
        """Move sail from current position to target angle"""
        target_angle = max(0.0, min(88.0, target_angle))
        
        # Calculate steps for both positions
        current_steps = self.angle_to_steps(self.current_angle)
        target_steps = self.angle_to_steps(target_angle)
        
        # Calculate relative movement
        steps_diff = target_steps - current_steps
        
        if steps_diff == 0:
            print(f"Already at {target_angle}°")
            return
        
        # Connect and send command
        self.connect()
        
        if steps_diff > 0:
            # Let sail out (CW)
            print(f"Moving from {self.current_angle}° to {target_angle}° (letting out {steps_diff} steps)")
            command = bytes([CMD_WINCH_CW_STEPS]) + struct.pack('>I', abs(steps_diff))
        else:
            # Bring sail in (CCW)
            print(f"Moving from {self.current_angle}° to {target_angle}° (bringing in {abs(steps_diff)} steps)")
            command = bytes([CMD_WINCH_CCW_STEPS]) + struct.pack('>I', abs(steps_diff))
        
        self.ser.write(command)
        
        # Read response
        time.sleep(0.5)
        while self.ser.in_waiting > 0:
            print(f"Arduino: {self.ser.readline().decode('utf-8').strip()}")
        
        # Update position
        self.current_angle = target_angle
        self.save_position()
        
        print(f"Sail now at {self.current_angle}°")
    
    def status(self):
        """Display current status"""
        print(f"\nCurrent sail angle: {self.current_angle}°")
        print(f"Steps from 0°: {self.angle_to_steps(self.current_angle)}")
        print("\nExample movements:")
        for angle in [0, 15, 30, 45, 60, 75, 88]:
            steps_diff = self.angle_to_steps(angle) - self.angle_to_steps(self.current_angle)
            direction = "out" if steps_diff > 0 else "in"
            print(f"  To {angle:2}°: {abs(steps_diff):5} steps {direction}")
    
    def run_cli(self):
        """Run interactive CLI"""
        print("Sail Control CLI")
        print("================")
        print(f"Current position: {self.current_angle}°")
        print("\nCommands:")
        print("  <angle>  - Move to angle (0-88)")
        print("  status   - Show current position and distances")
        print("  reset    - Reset position to 0°")
        print("  quit     - Exit")
        
        while True:
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'quit' or cmd == 'q':
                    break
                elif cmd == 'status' or cmd == 's':
                    self.status()
                elif cmd == 'reset':
                    self.current_angle = 0.0
                    self.save_position()
                    print("Position reset to 0°")
                else:
                    # Try to parse as angle
                    try:
                        angle = float(cmd)
                        if 0 <= angle <= 88:
                            self.move_to_angle(angle)
                        else:
                            print("Angle must be between 0 and 88 degrees")
                    except ValueError:
                        print("Unknown command. Enter angle (0-88) or 'help'")
                        
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")
        
        self.disconnect()
        print("Goodbye!")

if __name__ == "__main__":
    controller = SailController()
    controller.run_cli()
