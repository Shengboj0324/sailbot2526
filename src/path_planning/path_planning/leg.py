import math
import numpy as np
import os
from dataclasses import dataclass
from typing import List, Tuple, Optional
import csv
import ctypes
from .geometry_utils import Angle, Vector

# Try to load modern Fortran implementation first
MODERN_FORTRAN_AVAILABLE = False
F2PY_FORTRAN_AVAILABLE = False

try:
    # Try to load modern Fortran implementation with ISO_C_BINDING
    import os.path
    import sys
    
    # Try multiple possible locations for the library
    lib_paths = [
        os.path.join(os.path.dirname(__file__), 'leg_modern.so'),
        os.path.join(os.path.dirname(__file__), '../leg_modern.so'),
        'leg_modern.so'
    ]
    
    leg_modern_lib = None
    for lib_path in lib_paths:
        if os.path.exists(lib_path):
            try:
                leg_modern_lib = ctypes.CDLL(lib_path)
                break
            except:
                pass
    
    if leg_modern_lib is not None:
        # Define function signatures
        # initialize_polar_data() -> int
        leg_modern_lib.initialize_polar_data.restype = ctypes.c_int
        leg_modern_lib.initialize_polar_data.argtypes = []
        
        # calculate_path(...) -> int
        leg_modern_lib.calculate_path.restype = ctypes.c_int
        leg_modern_lib.calculate_path.argtypes = [
            ctypes.c_double,  # start_lat
            ctypes.c_double,  # start_lon
            ctypes.c_double,  # end_lat
            ctypes.c_double,  # end_lon
            ctypes.c_double,  # wind_angle
            ctypes.c_double,  # boat_heading
            ctypes.c_bool,    # first_maneuver_is_starboard
            ctypes.POINTER(ctypes.c_double),  # waypoints_lat array
            ctypes.POINTER(ctypes.c_double),  # waypoints_lon array
            ctypes.POINTER(ctypes.c_int)      # n_waypoints
        ]
        
        MODERN_FORTRAN_AVAILABLE = True
        pass  # Modern Fortran module loaded successfully
except Exception as e:
    pass  # Modern Fortran module not available

# Fall back to f2py implementation if modern not available
if not MODERN_FORTRAN_AVAILABLE:
    try:
        from . import leg_fortran_module
        F2PY_FORTRAN_AVAILABLE = True
        pass  # F2py Fortran module imported successfully
    except ImportError:
        F2PY_FORTRAN_AVAILABLE = False
        pass  # F2py Fortran module not available, using Python fallback

class PolarData:
    """manages boat performance data from polar diagram

    Fixed to use 8 mph (3.58 m/s) wind speed only. The polar data is extracted
    for this specific wind speed to simplify calculations.
    """
    def __init__(self):
        self.twa_values = []  # true wind angles
        self.boat_speeds = []  # boat speeds for 8 mph wind
        self.upwind_vmg = None  # maxvmg upwind angle
        self.downwind_vmg = None  # maxvmg downwind angle

        current_dir = os.path.dirname(__file__)
        pol_path = os.path.join(current_dir, 'data', 'test.pol')
        self._load_from_file(pol_path)
        
        # PolarData initialized successfully

    def _load_from_file(self, filepath: str):
        """load and parse polar data from file"""
        # Loading polar data
        
        with open(filepath, 'r') as f:
            lines = f.readlines()

        # parse header for true wind speeds
        header = lines[0].strip().split(';')
        tws_values = [float(x) for x in header[1:]]
        
        # Find the column index for 8 mph (3.58 m/s) - closest is 3.5 m/s at index 7
        # 8 mph = 3.58 m/s, so we use the 3.5 m/s column
        wind_speed_idx = 7  # 3.5 m/s column

        # parse boat speeds for 8 mph wind
        for line in lines[1:-1]:  # exclude last line which contains MAXVMG
            values = line.strip().split(';')
            self.twa_values.append(float(values[0]))
            self.boat_speeds.append(float(values[wind_speed_idx + 1]))  # +1 because first column is TWA

        # parse max vmg angles
        maxvmg = lines[-1].strip().split(';')[1:]
        self.upwind_vmg = float(maxvmg[0])
        self.downwind_vmg = float(maxvmg[1])
        
        # Polar data loaded

    def get_boat_speed(self, twa: float) -> float:
        """get boat speed for given true wind angle at 8 mph wind"""
        # find nearest angle index
        twa_idx = min(range(len(self.twa_values)), key=lambda i: abs(self.twa_values[i] - abs(twa)))
        return self.boat_speeds[twa_idx]

class Leg:
    """calculates optimal path between waypoints considering wind conditions"""
    def __init__(self):
        # Initializing Leg calculator
        self.polar_data = PolarData()
        # Define the upwind and downwind no-sail zones (relative to boat heading)
        self.upwind_zone = (315, 45)   # 45 degrees on either side of bow
        self.downwind_zone = (135, 225)  # 45 degrees on either side of stern
        
        # Initialize Fortran module if available
        if MODERN_FORTRAN_AVAILABLE:
            # Initialize modern Fortran polar data
            status = leg_modern_lib.initialize_polar_data()
            if status != 0:
                print(f"Warning: Modern Fortran initialization returned status {status}")
        elif F2PY_FORTRAN_AVAILABLE:
            # Initialize f2py Fortran polar data
            leg_fortran_module.leg_fortran.load_polar_data()

    def calculate_path(self, start_point: Tuple[float, float],
                      end_point: Tuple[float, float],
                      wind_angle: float,  # Wind angle relative to boat
                      boat_heading: float,
                      first_maneuver_is_starboard: bool = True) -> List[Tuple[float, float]]:
        """
        calculate optimal path between points considering wind relative to boat

        args:
            start_point: (lat, lon) of starting position
            end_point: (lat, lon) of ending position
            wind_angle: wind angle relative to boat heading in degrees (0 = head-on)
            boat_heading: current boat heading in degrees (0 = North, clockwise)
            first_maneuver_is_starboard: If True, the first tack/jibe will be on a starboard
                                         maneuver. If False, it will be port. Defaults to True.

        returns:
            list of waypoints including any intermediate tacking/jibing points
        """
        # Calculate optimal path
        
        if MODERN_FORTRAN_AVAILABLE:
            # Using modern FORTRAN implementation with ISO_C_BINDING
            
            # Prepare output arrays
            waypoints_lat = (ctypes.c_double * 10)()
            waypoints_lon = (ctypes.c_double * 10)()
            n_waypoints = ctypes.c_int()
            
            # Call modern Fortran function
            status = leg_modern_lib.calculate_path(
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
            
            if status != 0:
                # Modern Fortran returned error, fall back to Python
                return self._calculate_path_python(start_point, end_point, wind_angle, 
                                                  boat_heading, first_maneuver_is_starboard)
            
            # Convert to list of tuples
            waypoints = [(waypoints_lat[i], waypoints_lon[i]) for i in range(n_waypoints.value)]
            
            # Path calculated successfully using modern Fortran
            
            return waypoints
            
        elif F2PY_FORTRAN_AVAILABLE:
            # Using f2py FORTRAN implementation
            
            # Prepare output arrays
            waypoints_lat = np.zeros(10, dtype=np.float64)
            waypoints_lon = np.zeros(10, dtype=np.float64)
            
            # Call Fortran function
            waypoints_lat, waypoints_lon, n_waypoints = leg_fortran_module.leg_fortran.calculate_path(
                start_point[0], start_point[1],
                end_point[0], end_point[1],
                wind_angle,
                boat_heading,
                first_maneuver_is_starboard
            )
            
            # Convert to list of tuples
            waypoints = [(waypoints_lat[i], waypoints_lon[i]) for i in range(n_waypoints)]
            
            # Path calculated successfully using f2py Fortran
            
            return waypoints
        else:
            # Using Python fallback implementation
            return self._calculate_path_python(start_point, end_point, wind_angle, 
                                              boat_heading, first_maneuver_is_starboard)
    
    def _calculate_path_python(self, start_point: Tuple[float, float],
                              end_point: Tuple[float, float],
                              wind_angle: float,
                              boat_heading: float,
                              first_maneuver_is_starboard: bool = True) -> List[Tuple[float, float]]:
        """Python fallback implementation"""
        # Calculate desired course angle (global reference frame)
        course_angle = math.degrees(math.atan2(
            end_point[1] - start_point[1],
            end_point[0] - start_point[0]
        ))

        # Calculate angle between course and boat heading (relative to boat)
        course_relative_to_boat = (course_angle - boat_heading) % 360
        if course_relative_to_boat > 180:
            course_relative_to_boat -= 360

        # Check if the target direction is in a no-sail zone relative to the wind
        # First, normalize wind angle to 0-360 range
        wind_angle_normalized = wind_angle % 360

        # Determine if upwind or downwind sailing based on relative wind angle
        upwind_sailing = (wind_angle_normalized >= self.upwind_zone[0] or
                           wind_angle_normalized <= self.upwind_zone[1])

        downwind_sailing = (wind_angle_normalized >= self.downwind_zone[0] and
                             wind_angle_normalized <= self.downwind_zone[1])

        # Calculate the angle between our desired course and the wind
        target_to_wind_angle = (course_relative_to_boat - wind_angle_normalized) % 360
        if target_to_wind_angle > 180:
            target_to_wind_angle -= 360

        # For vector calculations, we need to convert to a global reference frame
        global_wind_angle = (boat_heading + wind_angle_normalized) % 360
        normalized_wind = Angle(1, global_wind_angle)

        # Determine if we need to tack or jibe based on wind zones and polar data
        if upwind_sailing and abs(target_to_wind_angle) < self.polar_data.upwind_vmg:
            # We're trying to sail too close to the wind - need to tack
            return self._calculate_upwind_path(
                start_point, end_point, normalized_wind, boat_heading, first_maneuver_is_starboard)

        elif downwind_sailing and abs(target_to_wind_angle) > self.polar_data.downwind_vmg:
            # We're trying to sail directly downwind - need to jibe
            return self._calculate_downwind_path(
                start_point, end_point, normalized_wind, boat_heading, first_maneuver_is_starboard)

        else:
            # Direct path is possible - no need for tacking or jibing
            return [end_point]

    def _calculate_upwind_path(self, start: Tuple[float, float],
                             end: Tuple[float, float],
                             wind: Angle,
                             boat_heading: float,
                             first_maneuver_is_starboard: bool) -> List[Tuple[float, float]]:
        """calculate tacking path for upwind sailing

        Args:
            first_maneuver_is_starboard: If True, the first tack will be starboard. Else, port.
        """
        # Create vector from start to end
        v_target = Vector(
            Angle(1, math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))),
            math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2))

        # Calculate optimal tacking vectors using the polar data
        # Wind is already converted to global reference frame
        # k_starboard: starboard tack vector (wind over boat's starboard/right side)
        # j_port: port tack vector (wind over boat's port/left side)
        k_starboard = Vector(wind + Angle(1, 180 + self.polar_data.upwind_vmg), 1)
        j_port = Vector(wind + Angle(1, 180 - self.polar_data.upwind_vmg), 1)

        if first_maneuver_is_starboard:
            leg1_vec = k_starboard
            leg2_vec = j_port
        else:
            leg1_vec = j_port
            leg2_vec = k_starboard

        # Solve system of equations to find optimal tacking point
        # v_target = scalar1 * leg1_vec + scalar2 * leg2_vec
        D_det = np.linalg.det([[leg1_vec.xcomp(), leg2_vec.xcomp()],
                               [leg1_vec.ycomp(), leg2_vec.ycomp()]])

        if abs(D_det) < 1e-9: # Avoid division by zero if vectors are collinear
            # This shouldn't happen with valid VMG angles but as a fallback:
            # WARNING: Collinear tacking vectors, defaulting to direct path
            return [end]

        D_scalar1 = np.linalg.det([[v_target.xcomp(), leg2_vec.xcomp()],
                                   [v_target.ycomp(), leg2_vec.ycomp()]])
        # D_scalar2 = np.linalg.det([[leg1_vec.xcomp(), v_target.xcomp()], # Not strictly needed for tack point
        #                            [leg1_vec.ycomp(), v_target.ycomp()]])

        scalar1 = D_scalar1 / D_det
        # scalar2 = D_scalar2 / D_det # Length along second leg

        # Calculate intermediate tacking point: start + leg1_vec * scalar1
        tack_point_x = start[0] + leg1_vec.xcomp() * scalar1
        tack_point_y = start[1] + leg1_vec.ycomp() * scalar1

        tack_point = (tack_point_x, tack_point_y)

        return [tack_point, end]

    def _calculate_downwind_path(self, start: Tuple[float, float],
                               end: Tuple[float, float],
                               wind: Angle,
                               boat_heading: float,
                               first_maneuver_is_starboard: bool) -> List[Tuple[float, float]]:
        """calculate jibing path for downwind sailing

        Args:
            first_maneuver_is_starboard: If True, the first jibe will be starboard. Else, port.
        """
        # Similar to upwind but using downwind vmg angles
        v_target = Vector(
            Angle(1, math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))),
            math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2))

        # Wind is already converted to global reference frame
        # k_starboard: starboard jibe vector (wind over boat's starboard/right quarter)
        # j_port: port jibe vector (wind over boat's port/left quarter)
        k_starboard = Vector(wind + Angle(1, 180 + self.polar_data.downwind_vmg), 1)
        j_port = Vector(wind + Angle(1, 180 - self.polar_data.downwind_vmg), 1)

        if first_maneuver_is_starboard:
            leg1_vec = k_starboard
            leg2_vec = j_port
        else:
            leg1_vec = j_port
            leg2_vec = k_starboard

        # Solve system of equations for optimal jibing point
        # v_target = scalar1 * leg1_vec + scalar2 * leg2_vec
        D_det = np.linalg.det([[leg1_vec.xcomp(), leg2_vec.xcomp()],
                               [leg1_vec.ycomp(), leg2_vec.ycomp()]])

        if abs(D_det) < 1e-9: # Avoid division by zero if vectors are collinear
            # WARNING: Collinear jibing vectors, defaulting to direct path
            return [end]

        D_scalar1 = np.linalg.det([[v_target.xcomp(), leg2_vec.xcomp()],
                                   [v_target.ycomp(), leg2_vec.ycomp()]])
        # D_scalar2 = np.linalg.det([[leg1_vec.xcomp(), v_target.xcomp()], # Not strictly needed
        #                            [leg1_vec.ycomp(), v_target.ycomp()]])

        scalar1 = D_scalar1 / D_det
        # scalar2 = D_scalar2 / D_det

        # Calculate intermediate jibing point: start + leg1_vec * scalar1
        jibe_point_x = start[0] + leg1_vec.xcomp() * scalar1
        jibe_point_y = start[1] + leg1_vec.ycomp() * scalar1

        jibe_point = (jibe_point_x, jibe_point_y)

        return [jibe_point, end]