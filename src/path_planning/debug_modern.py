#!/usr/bin/env python3
"""Debug why modern Fortran isn't loading in speed_comparison.py"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# First check direct import
print("1. Direct import test:")
try:
    import path_planning.leg
    print(f"   MODERN_FORTRAN_AVAILABLE = {path_planning.leg.MODERN_FORTRAN_AVAILABLE}")
    print(f"   F2PY_FORTRAN_AVAILABLE = {path_planning.leg.F2PY_FORTRAN_AVAILABLE}")
    
    if hasattr(path_planning.leg, 'leg_modern_lib'):
        print(f"   leg_modern_lib exists: {path_planning.leg.leg_modern_lib is not None}")
except Exception as e:
    print(f"   Error: {e}")

print("\n2. Force reload test:")
# Try forcing a reload
if 'path_planning.leg' in sys.modules:
    del sys.modules['path_planning.leg']
    
import path_planning.leg
print(f"   After reload: MODERN = {path_planning.leg.MODERN_FORTRAN_AVAILABLE}")

print("\n3. Create Leg instance:")
from path_planning.leg import Leg
leg = Leg()
result = leg.calculate_path((42.0, -71.0), (42.1, -71.0), 0.0, 0.0)
print(f"   Result: {len(result)} waypoints")

print("\n4. Check state manipulation:")
original = path_planning.leg.MODERN_FORTRAN_AVAILABLE
print(f"   Original state: {original}")
path_planning.leg.MODERN_FORTRAN_AVAILABLE = False
print(f"   After setting False: {path_planning.leg.MODERN_FORTRAN_AVAILABLE}")
path_planning.leg.MODERN_FORTRAN_AVAILABLE = True
print(f"   After setting True: {path_planning.leg.MODERN_FORTRAN_AVAILABLE}")
path_planning.leg.MODERN_FORTRAN_AVAILABLE = original
print(f"   Restored to: {path_planning.leg.MODERN_FORTRAN_AVAILABLE}")