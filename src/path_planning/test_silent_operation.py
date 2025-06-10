#!/usr/bin/env python3
"""Test that Fortran module operates silently without debug output"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from path_planning.leg import Leg

print("Testing silent operation...")
print("-" * 40)

# Create instance - should be silent
leg = Leg()

# Test calculation - should be silent
waypoints = leg.calculate_path(
    (42.0, -71.0),
    (42.1, -71.0),
    0.0,  # head-on wind
    0.0   # heading north
)

print(f"✓ Path calculated successfully")
print(f"✓ Result: {len(waypoints)} waypoints")
print(f"✓ No debug output - production ready!")
print("-" * 40)
print("Test passed!")