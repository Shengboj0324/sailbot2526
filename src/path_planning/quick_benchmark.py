#!/usr/bin/env python3
"""Quick performance benchmark"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from path_planning.leg import Leg

# Test parameters
n_iterations = 10000
test_cases = [
    ((42.0, -71.0), (42.1, -71.0), 0.0, 0.0),    # Tacking
    ((42.0, -71.0), (42.1, -71.0), 180.0, 0.0),  # Jibing
    ((42.0, -71.0), (42.1, -71.0), 90.0, 0.0),   # Direct
]

print("Quick Performance Benchmark")
print("=" * 50)
print(f"Running {n_iterations} iterations of {len(test_cases)} test cases")
print(f"Total calculations: {n_iterations * len(test_cases)}")
print()

# Create instance
leg = Leg()

# Warmup
for _ in range(100):
    for case in test_cases:
        leg.calculate_path(*case)

# Benchmark
start_time = time.perf_counter()

for _ in range(n_iterations):
    for case in test_cases:
        leg.calculate_path(*case)

end_time = time.perf_counter()

# Results
total_time = end_time - start_time
total_calcs = n_iterations * len(test_cases)
time_per_calc = total_time / total_calcs * 1000  # ms

print(f"Results:")
print(f"  Total time: {total_time:.3f} seconds")
print(f"  Time per calculation: {time_per_calc:.3f} ms")
print(f"  Calculations per second: {total_calcs / total_time:.0f}")
print()
print("✓ Benchmark complete!")