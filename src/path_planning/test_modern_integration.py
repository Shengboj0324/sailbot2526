#!/usr/bin/env python3
"""Test script to verify modern Fortran integration"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from path_planning.leg import Leg, MODERN_FORTRAN_AVAILABLE, F2PY_FORTRAN_AVAILABLE

print("=" * 60)
print("Modern Fortran Integration Test")
print("=" * 60)

# Check availability
print(f"Modern Fortran Available: {MODERN_FORTRAN_AVAILABLE}")
print(f"F2py Fortran Available: {F2PY_FORTRAN_AVAILABLE}")

# If modern is available, check library details
if MODERN_FORTRAN_AVAILABLE:
    from path_planning.leg import leg_modern_lib
    print(f"Modern library loaded from: {leg_modern_lib._name}")

# Create leg instance
print("\nCreating Leg instance...")
leg = Leg()

# Test calculations
test_cases = [
    # (start_lat, start_lon, end_lat, end_lon, wind_angle, boat_heading, name)
    ((42.0, -71.0), (42.1, -71.0), 0.0, 0.0, "Upwind (Tacking)"),
    ((42.0, -71.0), (42.1, -71.0), 180.0, 0.0, "Downwind (Jibing)"),
    ((42.0, -71.0), (42.1, -71.0), 90.0, 0.0, "Beam reach (Direct)"),
]

print("\nRunning test calculations...")
for start, end, wind_angle, boat_heading, name in test_cases:
    print(f"\n{name}:")
    print(f"  Start: {start}")
    print(f"  End: {end}")
    print(f"  Wind angle: {wind_angle}°")
    print(f"  Boat heading: {boat_heading}°")
    
    waypoints = leg.calculate_path(start, end, wind_angle, boat_heading)
    
    print(f"  Waypoints: {len(waypoints)}")
    for i, wp in enumerate(waypoints):
        print(f"    {i+1}: ({wp[0]:.6f}, {wp[1]:.6f})")

# Performance comparison if both are available
if MODERN_FORTRAN_AVAILABLE and F2PY_FORTRAN_AVAILABLE:
    print("\n" + "=" * 60)
    print("Performance Comparison")
    print("=" * 60)
    
    import time
    
    # Force modern implementation
    leg_modern = Leg()
    SAVED_F2PY = F2PY_FORTRAN_AVAILABLE
    path_planning.leg.F2PY_FORTRAN_AVAILABLE = False
    
    # Benchmark modern
    start_time = time.perf_counter()
    for _ in range(1000):
        leg_modern.calculate_path((42.0, -71.0), (42.1, -71.0), 0.0, 0.0)
    modern_time = time.perf_counter() - start_time
    
    # Force f2py implementation
    path_planning.leg.F2PY_FORTRAN_AVAILABLE = SAVED_F2PY
    path_planning.leg.MODERN_FORTRAN_AVAILABLE = False
    leg_f2py = Leg()
    
    # Benchmark f2py
    start_time = time.perf_counter()
    for _ in range(1000):
        leg_f2py.calculate_path((42.0, -71.0), (42.1, -71.0), 0.0, 0.0)
    f2py_time = time.perf_counter() - start_time
    
    # Restore settings
    path_planning.leg.MODERN_FORTRAN_AVAILABLE = True
    
    print(f"Modern Fortran: {modern_time:.3f} seconds for 1000 calculations")
    print(f"F2py Fortran: {f2py_time:.3f} seconds for 1000 calculations")
    print(f"Modern is {f2py_time/modern_time:.1f}x faster")

print("\n✓ Test complete!")