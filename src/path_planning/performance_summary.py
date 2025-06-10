#!/usr/bin/env python3
"""
Final Performance Summary: Python vs Modern Fortran
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import path_planning.leg
from path_planning.leg import Leg

print("=" * 70)
print("SAILBOAT PATH PLANNING PERFORMANCE SUMMARY")
print("=" * 70)

# Check what's available
print("\nImplementation Status:")
print(f"  ✓ Pure Python: Always available (baseline)")
print(f"  {'✓' if path_planning.leg.MODERN_FORTRAN_AVAILABLE else '✗'} Modern Fortran (ISO_C_BINDING): {'Available' if path_planning.leg.MODERN_FORTRAN_AVAILABLE else 'Not available'}")
print(f"  {'✓' if path_planning.leg.F2PY_FORTRAN_AVAILABLE else '✗'} F2py Fortran: {'Available' if path_planning.leg.F2PY_FORTRAN_AVAILABLE else 'Not available'}")

if not path_planning.leg.MODERN_FORTRAN_AVAILABLE:
    print("\nModern Fortran not available. Please run:")
    print("  cd /home/anton/ros2_ws/src/sailbot2425/src/path_planning")
    print("  ./build_modern_fortran.sh")
    sys.exit(1)

print("\nBenchmark Configuration:")
print("  Calculations per test: 50,000")
print("  Test scenario: Upwind sailing (tacking required)")

# Run benchmarks
def benchmark(force_python=False):
    original_modern = path_planning.leg.MODERN_FORTRAN_AVAILABLE
    if force_python:
        path_planning.leg.MODERN_FORTRAN_AVAILABLE = False
    
    leg = Leg()
    
    # Warmup
    for _ in range(1000):
        leg.calculate_path((42.0, -71.0), (42.1, -71.0), 0.0, 0.0)
    
    # Actual benchmark
    iterations = 50000
    start_time = time.perf_counter()
    
    for _ in range(iterations):
        leg.calculate_path((42.0, -71.0), (42.1, -71.0), 0.0, 0.0)
    
    end_time = time.perf_counter()
    
    path_planning.leg.MODERN_FORTRAN_AVAILABLE = original_modern
    
    return end_time - start_time, iterations

print("\nRunning benchmarks...")

# Python benchmark
print("  Testing Pure Python...", end='', flush=True)
python_time, iterations = benchmark(force_python=True)
python_speed = iterations / python_time
print(f" {python_speed:,.0f} calcs/sec")

# Modern Fortran benchmark
print("  Testing Modern Fortran...", end='', flush=True)
fortran_time, iterations = benchmark(force_python=False)
fortran_speed = iterations / fortran_time
print(f" {fortran_speed:,.0f} calcs/sec")

# Results
speedup = fortran_speed / python_speed

print("\n" + "=" * 70)
print("RESULTS")
print("=" * 70)

print(f"\nPure Python Performance:")
print(f"  Speed: {python_speed:,.0f} calculations/second")
print(f"  Time per calculation: {(python_time/iterations)*1000:.3f} ms")

print(f"\nModern Fortran Performance:")
print(f"  Speed: {fortran_speed:,.0f} calculations/second")
print(f"  Time per calculation: {(fortran_time/iterations)*1000:.3f} ms")

print(f"\nSPEEDUP: {speedup:.1f}x faster")

print("\n" + "=" * 70)
print("CONCLUSION")
print("=" * 70)
print(f"The Modern Fortran implementation provides a {speedup:.1f}x performance")
print("improvement over pure Python while maintaining the same API.")
print("\nThis translates to:")
print(f"  • {(speedup-1)*100:.0f}% reduction in computation time")
print(f"  • {fortran_speed-python_speed:,.0f} more calculations per second")
print("\nThe Fortran implementation is automatically used by the ROS system")
print("when available, providing significant performance benefits for")
print("real-time navigation calculations.")

print("\n✓ Performance test complete!")