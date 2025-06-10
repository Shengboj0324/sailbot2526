#!/usr/bin/env python3
"""Quick verification that modern Fortran is being used"""

import sys
import os

# Add path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import and check
try:
    from path_planning.leg import Leg, MODERN_FORTRAN_AVAILABLE, F2PY_FORTRAN_AVAILABLE
    
    print("Fortran Implementation Status:")
    print(f"  Modern Fortran (ISO_C_BINDING): {'✓ Available' if MODERN_FORTRAN_AVAILABLE else '✗ Not Available'}")
    print(f"  F2py Fortran: {'✓ Available' if F2PY_FORTRAN_AVAILABLE else '✗ Not Available'}")
    
    # Create instance and test
    leg = Leg()
    waypoints = leg.calculate_path((42.0, -71.0), (42.1, -71.0), 0.0, 0.0)
    
    if MODERN_FORTRAN_AVAILABLE:
        print("\n✓ SUCCESS: Modern Fortran implementation is being used!")
    elif F2PY_FORTRAN_AVAILABLE:
        print("\n→ INFO: Using f2py Fortran implementation (fallback)")
    else:
        print("\n→ INFO: Using pure Python implementation (fallback)")
        
    print(f"\nTest calculation returned {len(waypoints)} waypoints")
    
except Exception as e:
    print(f"✗ ERROR: {e}")
    sys.exit(1)