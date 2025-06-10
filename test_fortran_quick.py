#!/usr/bin/env python3
"""Quick test to verify Fortran module is working in ROS environment"""

import sys
import os

# Source the ROS workspace
sys.path.insert(0, '/home/anton/ros2_ws/src/sailbot2425/install/path_planning/lib/python3.12/site-packages')

try:
    from path_planning.path_planning.leg import Leg
    print("Successfully imported Leg module")
    
    # Create instance
    leg = Leg()
    
    # Test a simple calculation
    print("\nTesting path calculation...")
    waypoints = leg.calculate_path(
        (42.0, -71.0),
        (42.1, -71.0),
        0.0,  # head-on wind
        0.0   # heading north
    )
    
    print(f"Result: {len(waypoints)} waypoints")
    for i, wp in enumerate(waypoints):
        print(f"  Waypoint {i+1}: {wp}")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()