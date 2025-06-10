"""
Modern Python wrapper for high-performance Fortran path planning
Uses ctypes for clean interface with ISO_C_BINDING Fortran module
"""

import ctypes
import numpy as np
import os
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import IntEnum

# Error codes matching Fortran
class PathPlanningError(IntEnum):
    SUCCESS = 0
    ERROR_INVALID_INPUT = -1
    ERROR_COLLINEAR_VECTORS = -2
    ERROR_POLAR_NOT_LOADED = -3

# Load the Fortran library
def load_fortran_library():
    """Load the compiled Fortran shared library"""
    lib_dir = os.path.dirname(os.path.abspath(__file__))
    lib_patterns = [
        'leg_modern.so',
        'leg_modern.cpython*.so',
        'libleg_modern.so'
    ]
    
    for pattern in lib_patterns:
        lib_path = os.path.join(lib_dir, pattern)
        if os.path.exists(lib_path):
            return ctypes.CDLL(lib_path)
        
        # Check with glob
        import glob
        matches = glob.glob(os.path.join(lib_dir, pattern))
        if matches:
            return ctypes.CDLL(matches[0])
    
    raise ImportError("Could not find compiled Fortran library leg_modern.so")

# Global library instance
try:
    _fortran_lib = load_fortran_library()
    
    # Define C function signatures
    _fortran_lib.initialize_polar_data.argtypes = []
    _fortran_lib.initialize_polar_data.restype = ctypes.c_int
    
    _fortran_lib.calculate_path.argtypes = [
        ctypes.c_double,  # start_lat
        ctypes.c_double,  # start_lon
        ctypes.c_double,  # end_lat
        ctypes.c_double,  # end_lon
        ctypes.c_double,  # wind_angle
        ctypes.c_double,  # boat_heading
        ctypes.c_bool,    # first_maneuver_is_starboard
        ctypes.POINTER(ctypes.c_double),  # waypoints_lat
        ctypes.POINTER(ctypes.c_double),  # waypoints_lon
        ctypes.POINTER(ctypes.c_int)      # n_waypoints
    ]
    _fortran_lib.calculate_path.restype = ctypes.c_int
    
    FORTRAN_AVAILABLE = True
except Exception as e:
    FORTRAN_AVAILABLE = False
    _fortran_lib = None

class ModernLeg:
    """High-performance path calculator using modern Fortran backend"""
    
    def __init__(self):
        if not FORTRAN_AVAILABLE:
            raise RuntimeError("Fortran library not available")
        
        # Initialize polar data
        status = _fortran_lib.initialize_polar_data()
        if status != PathPlanningError.SUCCESS:
            raise RuntimeError(f"Failed to initialize polar data: {status}")
        
        self._max_waypoints = 10
    
    def calculate_path(self, 
                      start_point: Tuple[float, float],
                      end_point: Tuple[float, float],
                      wind_angle: float,
                      boat_heading: float,
                      first_maneuver_is_starboard: bool = True) -> List[Tuple[float, float]]:
        """
        Calculate optimal path between points using high-performance Fortran backend
        
        Args:
            start_point: (lat, lon) starting position
            end_point: (lat, lon) ending position
            wind_angle: wind angle relative to boat (degrees)
            boat_heading: boat heading (degrees, 0=North)
            first_maneuver_is_starboard: first tack/jibe direction
            
        Returns:
            List of waypoints [(lat, lon), ...]
            
        Raises:
            ValueError: Invalid input parameters
            RuntimeError: Calculation errors
        """
        # Prepare output arrays
        waypoints_lat = (ctypes.c_double * self._max_waypoints)()
        waypoints_lon = (ctypes.c_double * self._max_waypoints)()
        n_waypoints = ctypes.c_int()
        
        # Call Fortran function
        status = _fortran_lib.calculate_path(
            ctypes.c_double(start_point[0]),
            ctypes.c_double(start_point[1]),
            ctypes.c_double(end_point[0]),
            ctypes.c_double(end_point[1]),
            ctypes.c_double(wind_angle),
            ctypes.c_double(boat_heading),
            ctypes.c_bool(first_maneuver_is_starboard),
            waypoints_lat,
            waypoints_lon,
            ctypes.byref(n_waypoints)
        )
        
        # Check status
        if status == PathPlanningError.ERROR_INVALID_INPUT:
            raise ValueError("Invalid input parameters (NaN or Inf values)")
        elif status == PathPlanningError.ERROR_COLLINEAR_VECTORS:
            # This is actually okay - it means direct path
            pass
        elif status == PathPlanningError.ERROR_POLAR_NOT_LOADED:
            raise RuntimeError("Polar data not loaded")
        elif status != PathPlanningError.SUCCESS:
            raise RuntimeError(f"Unknown error: {status}")
        
        # Convert to Python list
        waypoints = []
        for i in range(n_waypoints.value):
            waypoints.append((waypoints_lat[i], waypoints_lon[i]))
        
        return waypoints

# Drop-in replacement for existing Leg class
class Leg(ModernLeg):
    """Backward compatible wrapper for modern implementation"""
    
    def __init__(self):
        # Try modern implementation first
        self._use_modern = False
        try:
            super().__init__()
            self._use_modern = True
        except:
            # Fall back to Python implementation
            from .leg_original import Leg as LegOriginal
            self._fallback = LegOriginal()
    
    def calculate_path(self, 
                      start_point: Tuple[float, float],
                      end_point: Tuple[float, float],
                      wind_angle: float,
                      boat_heading: float,
                      first_maneuver_is_starboard: bool = True) -> List[Tuple[float, float]]:
        if self._use_modern:
            return super().calculate_path(start_point, end_point, wind_angle, 
                                        boat_heading, first_maneuver_is_starboard)
        else:
            # Use original implementation
            return self._fallback.calculate_path(start_point, end_point, wind_angle,
                                               boat_heading, first_maneuver_is_starboard)