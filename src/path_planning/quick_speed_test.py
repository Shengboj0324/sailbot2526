#!/usr/bin/env python3
"""
Quick and simple speed test between Python and Fortran implementations
"""

import sys
import os
import time

# Add path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the module
import path_planning.leg
from path_planning.leg import Leg

def test_implementation(name, force_python=False, force_modern=False, force_f2py=False):
    """Test a specific implementation"""
    # Save original state
    original_modern = path_planning.leg.MODERN_FORTRAN_AVAILABLE
    original_f2py = path_planning.leg.F2PY_FORTRAN_AVAILABLE
    
    # Force implementation
    if force_python:
        path_planning.leg.MODERN_FORTRAN_AVAILABLE = False
        path_planning.leg.F2PY_FORTRAN_AVAILABLE = False
    elif force_modern:
        path_planning.leg.MODERN_FORTRAN_AVAILABLE = original_modern
        path_planning.leg.F2PY_FORTRAN_AVAILABLE = False
    elif force_f2py:
        path_planning.leg.MODERN_FORTRAN_AVAILABLE = False
        path_planning.leg.F2PY_FORTRAN_AVAILABLE = original_f2py
    
    # Create instance
    leg = Leg()
    
    # Test case (upwind/tacking scenario)
    start = (42.0, -71.0)
    end = (42.1, -71.0)
    wind_angle = 0.0
    boat_heading = 0.0
    
    # Warmup
    for _ in range(100):
        leg.calculate_path(start, end, wind_angle, boat_heading)
    
    # Benchmark
    iterations = 10000
    start_time = time.perf_counter()
    
    for _ in range(iterations):
        waypoints = leg.calculate_path(start, end, wind_angle, boat_heading)
    
    end_time = time.perf_counter()
    total_time = end_time - start_time
    
    # Restore original state
    path_planning.leg.MODERN_FORTRAN_AVAILABLE = original_modern
    path_planning.leg.F2PY_FORTRAN_AVAILABLE = original_f2py
    
    # Results
    calcs_per_second = iterations / total_time
    time_per_calc_ms = (total_time / iterations) * 1000
    
    print(f"\n{name}:")
    print(f"  Total time: {total_time:.3f} seconds")
    print(f"  Calculations/second: {calcs_per_second:,.0f}")
    print(f"  Time per calculation: {time_per_calc_ms:.3f} ms")
    print(f"  Result: {len(waypoints)} waypoints")
    
    return calcs_per_second

print("=" * 60)
print("Quick Speed Comparison Test")
print("=" * 60)

# Check availability
print("\nChecking implementations:")
print(f"  Modern Fortran: {'Available' if path_planning.leg.MODERN_FORTRAN_AVAILABLE else 'Not Available'}")
print(f"  F2py Fortran: {'Available' if path_planning.leg.F2PY_FORTRAN_AVAILABLE else 'Not Available'}")

print(f"\nRunning {10000} calculations for each implementation...")

# Test each implementation
python_speed = test_implementation("Pure Python", force_python=True)

modern_speed = None
if path_planning.leg.MODERN_FORTRAN_AVAILABLE:
    modern_speed = test_implementation("Modern Fortran", force_modern=True)

f2py_speed = None
if path_planning.leg.F2PY_FORTRAN_AVAILABLE:
    f2py_speed = test_implementation("F2py Fortran", force_f2py=True)

# Summary
print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)

if modern_speed:
    speedup = modern_speed / python_speed
    print(f"\nModern Fortran vs Python:")
    print(f"  Speedup: {speedup:.1f}x faster")
    print(f"  Python: {python_speed:,.0f} calcs/sec")
    print(f"  Modern: {modern_speed:,.0f} calcs/sec")

if f2py_speed:
    speedup = f2py_speed / python_speed
    print(f"\nF2py Fortran vs Python:")
    print(f"  Speedup: {speedup:.1f}x faster")
    print(f"  Python: {python_speed:,.0f} calcs/sec")
    print(f"  F2py: {f2py_speed:,.0f} calcs/sec")

if modern_speed and f2py_speed:
    improvement = modern_speed / f2py_speed
    print(f"\nModern vs F2py:")
    print(f"  Modern is {improvement:.2f}x faster than F2py")

print("\n✓ Test complete!")