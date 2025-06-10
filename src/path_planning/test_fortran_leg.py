#!/usr/bin/env python3
"""
Test script for Fortran leg module implementation
Tests various path planning scenarios and compares with expected results
"""

import sys
import os

# Add the path_planning module to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the leg module
try:
    from path_planning.leg import Leg
    print("[TEST] Successfully imported Leg module")
except ImportError as e:
    print(f"[TEST] ERROR: Failed to import Leg module: {e}")
    sys.exit(1)

def test_scenario(name, start, end, wind_angle, boat_heading, first_starboard=True):
    """Test a specific scenario"""
    print(f"\n[TEST] ========== Test Scenario: {name} ==========")
    print(f"[TEST] Start: {start}")
    print(f"[TEST] End: {end}")
    print(f"[TEST] Wind angle (rel to boat): {wind_angle}°")
    print(f"[TEST] Boat heading: {boat_heading}°")
    print(f"[TEST] First maneuver starboard: {first_starboard}")
    
    try:
        leg_calc = Leg()
        waypoints = leg_calc.calculate_path(start, end, wind_angle, boat_heading, first_starboard)
        
        print(f"[TEST] Result: {len(waypoints)} waypoints")
        for i, wp in enumerate(waypoints):
            print(f"[TEST]   Waypoint {i+1}: ({wp[0]:.6f}, {wp[1]:.6f})")
        
        return True
    except Exception as e:
        print(f"[TEST] ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all test scenarios"""
    print("[TEST] Starting Fortran leg module tests...")
    
    # Test scenarios
    scenarios = [
        # Direct path (no tacking/jibing needed)
        {
            "name": "Direct Path - Beam Reach",
            "start": (42.0, -71.0),
            "end": (42.1, -71.0),
            "wind_angle": 90.0,  # Wind from starboard
            "boat_heading": 0.0,  # Heading north
            "first_starboard": True
        },
        
        # Upwind sailing - need to tack
        {
            "name": "Upwind - Need to Tack",
            "start": (42.0, -71.0),
            "end": (42.1, -71.0),
            "wind_angle": 0.0,   # Head-on wind
            "boat_heading": 0.0,  # Heading north
            "first_starboard": True
        },
        
        # Downwind sailing - need to jibe
        {
            "name": "Downwind - Need to Jibe",
            "start": (42.0, -71.0),
            "end": (42.1, -71.0),
            "wind_angle": 180.0,  # Wind from behind
            "boat_heading": 0.0,  # Heading north
            "first_starboard": True
        },
        
        # Close hauled but not in no-sail zone
        {
            "name": "Close Hauled - No Tack Needed",
            "start": (42.0, -71.0),
            "end": (42.05, -71.05),
            "wind_angle": 45.0,   # Wind 45° off bow
            "boat_heading": 45.0,  # Heading northeast
            "first_starboard": True
        },
        
        # Port tack first
        {
            "name": "Upwind - Port Tack First",
            "start": (42.0, -71.0),
            "end": (42.1, -71.0),
            "wind_angle": 0.0,   # Head-on wind
            "boat_heading": 0.0,  # Heading north
            "first_starboard": False
        },
        
        # Different boat heading
        {
            "name": "Complex Heading - Tacking",
            "start": (42.0, -71.0),
            "end": (42.0, -71.1),
            "wind_angle": 315.0,  # Wind from port bow
            "boat_heading": 270.0,  # Heading west
            "first_starboard": True
        }
    ]
    
    passed = 0
    failed = 0
    
    for scenario in scenarios:
        success = test_scenario(
            scenario["name"],
            scenario["start"],
            scenario["end"],
            scenario["wind_angle"],
            scenario["boat_heading"],
            scenario.get("first_starboard", True)
        )
        
        if success:
            passed += 1
        else:
            failed += 1
    
    print(f"\n[TEST] ========== Test Summary ==========")
    print(f"[TEST] Total tests: {len(scenarios)}")
    print(f"[TEST] Passed: {passed}")
    print(f"[TEST] Failed: {failed}")
    
    if failed == 0:
        print("[TEST] All tests PASSED! ✓")
    else:
        print(f"[TEST] Some tests FAILED! ✗")
        sys.exit(1)

if __name__ == "__main__":
    main()