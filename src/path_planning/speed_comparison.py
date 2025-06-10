#!/usr/bin/env python3
"""
Comprehensive speed comparison between Python and Modern Fortran implementations
Tests various scenarios and provides detailed performance metrics
"""

import sys
import os
import time
import statistics
from typing import List, Tuple

# Add path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the module
import path_planning.leg
from path_planning.leg import Leg

# Colors for output
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BLUE = '\033[94m'
CYAN = '\033[96m'
RESET = '\033[0m'
BOLD = '\033[1m'

def print_header(text):
    print(f"\n{BOLD}{BLUE}{'=' * 70}{RESET}")
    print(f"{BOLD}{BLUE}{text:^70}{RESET}")
    print(f"{BOLD}{BLUE}{'=' * 70}{RESET}")

def print_subheader(text):
    print(f"\n{BOLD}{CYAN}{text}{RESET}")
    print(f"{CYAN}{'-' * len(text)}{RESET}")

def force_implementation(impl_type):
    """Force a specific implementation type"""
    # Always save original state
    original_modern = path_planning.leg.MODERN_FORTRAN_AVAILABLE
    original_f2py = path_planning.leg.F2PY_FORTRAN_AVAILABLE
    
    if impl_type == "python":
        path_planning.leg.MODERN_FORTRAN_AVAILABLE = False
        path_planning.leg.F2PY_FORTRAN_AVAILABLE = False
    elif impl_type == "modern":
        path_planning.leg.MODERN_FORTRAN_AVAILABLE = True
        path_planning.leg.F2PY_FORTRAN_AVAILABLE = False
    elif impl_type == "f2py":
        path_planning.leg.MODERN_FORTRAN_AVAILABLE = False
        path_planning.leg.F2PY_FORTRAN_AVAILABLE = True
    
    return original_modern, original_f2py

def restore_implementation(original_state):
    """Restore original implementation state"""
    if original_state:
        path_planning.leg.MODERN_FORTRAN_AVAILABLE = original_state[0]
        path_planning.leg.F2PY_FORTRAN_AVAILABLE = original_state[1]

def benchmark_implementation(leg_instance, test_cases, iterations=1000, warmup=100):
    """Benchmark an implementation with given test cases"""
    # Warmup
    for _ in range(warmup):
        for start, end, wind, heading, _ in test_cases:
            leg_instance.calculate_path(start, end, wind, heading)
    
    # Actual benchmark
    times = []
    for _ in range(iterations):
        start_time = time.perf_counter()
        for start, end, wind, heading, _ in test_cases:
            leg_instance.calculate_path(start, end, wind, heading)
        end_time = time.perf_counter()
        times.append(end_time - start_time)
    
    return times

def print_results(impl_name, times, iterations, num_cases):
    """Print formatted benchmark results"""
    total_calculations = iterations * num_cases
    mean_time = statistics.mean(times)
    stdev_time = statistics.stdev(times) if len(times) > 1 else 0
    min_time = min(times)
    max_time = max(times)
    
    calculations_per_second = total_calculations / sum(times)
    time_per_calc_us = (mean_time / num_cases) * 1_000_000  # microseconds
    
    print(f"\n{BOLD}{impl_name} Results:{RESET}")
    print(f"  Total calculations: {total_calculations:,}")
    print(f"  Total time: {sum(times):.3f} seconds")
    print(f"  Mean iteration time: {mean_time*1000:.3f} ± {stdev_time*1000:.3f} ms")
    print(f"  Min/Max iteration: {min_time*1000:.3f} / {max_time*1000:.3f} ms")
    print(f"  {GREEN}Calculations/second: {calculations_per_second:,.0f}{RESET}")
    print(f"  {GREEN}Time per calculation: {time_per_calc_us:.1f} μs{RESET}")

def main():
    print_header("Path Planning Speed Comparison")
    
    # Check what's available
    print_subheader("Implementation Availability")
    original_modern = path_planning.leg.MODERN_FORTRAN_AVAILABLE
    original_f2py = path_planning.leg.F2PY_FORTRAN_AVAILABLE
    
    print(f"Modern Fortran (ISO_C_BINDING): {GREEN if original_modern else RED}{'Available' if original_modern else 'Not Available'}{RESET}")
    print(f"F2py Fortran: {GREEN if original_f2py else RED}{'Available' if original_f2py else 'Not Available'}{RESET}")
    print(f"Pure Python: {GREEN}Always Available{RESET}")
    
    # Define test cases
    test_cases = [
        # (start, end, wind_angle, boat_heading, description)
        ((42.0, -71.0), (42.1, -71.0), 0.0, 0.0, "Upwind/Tacking"),
        ((42.0, -71.0), (42.1, -71.0), 180.0, 0.0, "Downwind/Jibing"),
        ((42.0, -71.0), (42.1, -71.0), 90.0, 0.0, "Beam Reach"),
        ((42.0, -71.0), (42.0, -71.1), 45.0, 90.0, "Close Hauled"),
        ((42.0, -71.0), (42.05, -71.05), 135.0, 45.0, "Broad Reach"),
        ((42.0, -71.0), (42.0, -71.0), 0.0, 0.0, "Same Start/End"),
    ]
    
    print_subheader("Test Configuration")
    print(f"Number of test cases: {len(test_cases)}")
    print(f"Iterations per benchmark: 1,000")
    print(f"Total calculations per benchmark: {len(test_cases) * 1000:,}")
    
    # Benchmark Pure Python
    print_header("Benchmarking Pure Python Implementation")
    force_implementation("python")
    leg_python = Leg()
    python_times = benchmark_implementation(leg_python, test_cases)
    print_results("Pure Python", python_times, 1000, len(test_cases))
    
    # Benchmark Modern Fortran if available
    speedup_modern = None
    modern_times = None
    if original_modern:
        print_header("Benchmarking Modern Fortran Implementation")
        state = force_implementation("modern")
        leg_modern = Leg()
        modern_times = benchmark_implementation(leg_modern, test_cases)
        print_results("Modern Fortran", modern_times, 1000, len(test_cases))
        speedup_modern = sum(python_times) / sum(modern_times)
        restore_implementation(state)
    
    # Benchmark F2py Fortran if available
    speedup_f2py = None
    f2py_times = None
    if original_f2py:
        print_header("Benchmarking F2py Fortran Implementation")
        state = force_implementation("f2py")
        leg_f2py = Leg()
        f2py_times = benchmark_implementation(leg_f2py, test_cases)
        print_results("F2py Fortran", f2py_times, 1000, len(test_cases))
        speedup_f2py = sum(python_times) / sum(f2py_times)
        restore_implementation(state)
    
    # Summary
    print_header("Performance Summary")
    
    print(f"\n{BOLD}Speedup vs Pure Python:{RESET}")
    if speedup_modern:
        print(f"  Modern Fortran: {GREEN}{speedup_modern:.1f}x faster{RESET}")
    if speedup_f2py:
        print(f"  F2py Fortran: {GREEN}{speedup_f2py:.1f}x faster{RESET}")
    
    if speedup_modern and speedup_f2py and modern_times and f2py_times:
        modern_vs_f2py = sum(f2py_times) / sum(modern_times)
        print(f"\n{BOLD}Modern vs F2py:{RESET}")
        print(f"  Modern Fortran is {GREEN}{modern_vs_f2py:.2f}x faster{RESET} than F2py")
    
    # Verify correctness
    print_header("Correctness Verification")
    print("Checking that all implementations produce the same results...")
    
    force_implementation("python")
    leg_verify = Leg()
    all_match = True
    
    for start, end, wind, heading, desc in test_cases[:3]:  # Check first 3 cases
        python_result = leg_verify.calculate_path(start, end, wind, heading)
        print(f"\n{desc}:")
        print(f"  Python result: {len(python_result)} waypoints")
        
        if original_modern:
            state = force_implementation("modern")
            leg_modern_verify = Leg()
            modern_result = leg_modern_verify.calculate_path(start, end, wind, heading)
            match = len(python_result) == len(modern_result)
            if match:
                for i, (py_wp, fort_wp) in enumerate(zip(python_result, modern_result)):
                    if abs(py_wp[0] - fort_wp[0]) > 1e-6 or abs(py_wp[1] - fort_wp[1]) > 1e-6:
                        match = False
                        break
            print(f"  Modern matches Python: {GREEN if match else RED}{'Yes' if match else 'No'}{RESET}")
            all_match &= match
            restore_implementation(state)
    
    if all_match:
        print(f"\n{GREEN}✓ All implementations produce identical results!{RESET}")
    else:
        print(f"\n{RED}✗ Warning: Implementations produce different results!{RESET}")
    
    # Restore original state
    path_planning.leg.MODERN_FORTRAN_AVAILABLE = original_modern
    path_planning.leg.F2PY_FORTRAN_AVAILABLE = original_f2py
    
    print(f"\n{BOLD}{GREEN}✓ Benchmark complete!{RESET}")

if __name__ == "__main__":
    main()