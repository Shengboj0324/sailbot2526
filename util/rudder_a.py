#!/usr/bin/env python3
"""
Simple rudder control via serial communication
Usage: python3 rudder_control.py [angle]
"""

import serial
import time
import sys

# Serial configuration
SERIAL_PORT = '/dev/arduino_control'
BAUD_RATE = 115200

# Command codes
SERVO_CMD = 0x20

# Rudder configuration
NEUTRAL_SERVO_ANGLE = 55  # True 0 position for rudder
MIN_RUDDER_ANGLE = -21    # Maximum left
MAX_RUDDER_ANGLE = 21     # Maximum right

def send_rudder_angle(rudder_angle):
    """
    Send rudder angle to Arduino via serial.

    Args:
        rudder_angle: Rudder angle in degrees (-21 to +21)
                     Negative = left, Positive = right, 0 = neutral
    """
    # Constrain rudder angle to safe range
    rudder_angle = max(MIN_RUDDER_ANGLE, min(MAX_RUDDER_ANGLE, float(rudder_angle)))

    # Convert rudder angle to servo angle
    # Neutral (0°) = 57° servo, add rudder angle to move left/right
    servo_angle = int(NEUTRAL_SERVO_ANGLE + rudder_angle)

    # Connect and send command to Arduino
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        time.sleep(2)  # Wait for Arduino reset

        # Send command: SERVO_CMD followed by servo angle
        command = bytes([SERVO_CMD, servo_angle])
        ser.write(command)

        # Read and display response
        time.sleep(0.2)
        while ser.in_waiting > 0:
            print(f"Arduino: {ser.readline().decode('utf-8').strip()}")

        print(f"Rudder angle: {rudder_angle:+.1f}° → Servo angle: {servo_angle}°")

def initialize_rudder():
    """Set rudder to neutral position (0°)"""
    send_rudder_angle(0)
    print("Rudder initialized to neutral position")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        # No arguments - initialize to neutral
        initialize_rudder()
    elif len(sys.argv) == 2:
        try:
            angle = float(sys.argv[1])
            send_rudder_angle(angle)
        except ValueError:
            print("Error: Angle must be a number")
            sys.exit(1)
    else:
        print("Usage: python3 rudder_control.py [angle]")
        print("  angle: Rudder angle in degrees (-21 to +21)")
        print("         Negative = left, Positive = right")
        print("  No angle: Initialize to neutral (0°)")
        sys.exit(1)
