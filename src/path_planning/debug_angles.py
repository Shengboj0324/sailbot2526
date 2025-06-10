#!/usr/bin/env python3
"""Debug angle calculations between implementations"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import math
import path_planning.leg
from path_planning.leg import Leg

# Check polar data
print("Checking Polar Data:")
print("-" * 40)

# Python implementation
path_planning.leg.MODERN_FORTRAN_AVAILABLE = False
path_planning.leg.F2PY_FORTRAN_AVAILABLE = False
leg_python = Leg()

print(f"Python polar data:")
print(f"  Upwind VMG: {leg_python.polar_data.upwind_vmg}°")
print(f"  Downwind VMG: {leg_python.polar_data.downwind_vmg}°")

# Check angle calculations for tacking scenario
print("\nTacking Calculation Debug:")
print("-" * 40)

# Test case: upwind
start = (42.0, -71.0)
end = (42.1, -71.0)
wind_angle = 0.0
boat_heading = 0.0

# Global wind angle
global_wind_angle = (boat_heading + wind_angle) % 360
print(f"Global wind angle: {global_wind_angle}°")

# Tacking angles
upwind_vmg = leg_python.polar_data.upwind_vmg
k_angle = global_wind_angle + 180 + upwind_vmg  # Starboard tack
j_angle = global_wind_angle + 180 - upwind_vmg  # Port tack

print(f"K angle (starboard): {k_angle}°")
print(f"J angle (port): {j_angle}°")

# Convert to radians and unit vectors
k_rad = math.radians(k_angle)
j_rad = math.radians(j_angle)

k_x = math.cos(k_rad)
k_y = math.sin(k_rad)
j_x = math.cos(j_rad)
j_y = math.sin(j_rad)

print(f"\nUnit vectors:")
print(f"  K vector: ({k_x:.6f}, {k_y:.6f})")
print(f"  J vector: ({j_x:.6f}, {j_y:.6f})")

# Target vector
target_x = end[1] - start[1]  # lon difference
target_y = end[0] - start[0]  # lat difference

print(f"\nTarget vector: ({target_x:.6f}, {target_y:.6f})")

# Matrix determinant
det = k_x * j_y - k_y * j_x
print(f"\nDeterminant: {det:.6f}")

# Solve for scalar (using starboard first)
det_scalar1 = target_x * j_y - j_x * target_y
scalar1 = det_scalar1 / det

print(f"Scalar1: {scalar1:.6f}")

# Calculate tack point
tack_lon = start[1] + k_x * scalar1
tack_lat = start[0] + k_y * scalar1

print(f"\nCalculated tack point: ({tack_lat:.10f}, {tack_lon:.10f})")

# Now let's check what Modern Fortran gives
path_planning.leg.MODERN_FORTRAN_AVAILABLE = True
leg_modern = Leg()
modern_result = leg_modern.calculate_path(start, end, wind_angle, boat_heading)
print(f"Modern Fortran result: {modern_result[0]}")