#!/usr/bin/env python3
"""
Path Smoother using Bezier curves
Smooths waypoint paths for better sailing performance
"""
import numpy as np
from typing import List, Tuple

class PathSmoother:
    """
    Smooth waypoint paths using cubic Bezier curves
    """
    
    def __init__(self, smoothing_distance=5.0):
        """
        Args:
            smoothing_distance: Distance before/after waypoint to start curve (meters)
        """
        self.smoothing_distance = smoothing_distance
    
    def smooth_path(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smooth a path through waypoints using Bezier curves
        
        Args:
            waypoints: List of (lat, lon) tuples
        
        Returns:
            Smoothed path as list of (lat, lon) tuples
        """
        if len(waypoints) < 3:
            return waypoints
        
        smoothed_path = []
        
        # Add first waypoint
        smoothed_path.append(waypoints[0])
        
        # Smooth intermediate waypoints
        for i in range(1, len(waypoints) - 1):
            prev_wp = waypoints[i-1]
            curr_wp = waypoints[i]
            next_wp = waypoints[i+1]
            
            # Generate Bezier curve around this waypoint
            curve_points = self.generate_bezier_curve(prev_wp, curr_wp, next_wp)
            smoothed_path.extend(curve_points)
        
        # Add last waypoint
        smoothed_path.append(waypoints[-1])
        
        return smoothed_path
    
    def generate_bezier_curve(self, p0: Tuple[float, float], 
                             p1: Tuple[float, float], 
                             p2: Tuple[float, float],
                             num_points: int = 10) -> List[Tuple[float, float]]:
        """
        Generate cubic Bezier curve around waypoint p1
        
        Args:
            p0: Previous waypoint (lat, lon)
            p1: Current waypoint (lat, lon)
            p2: Next waypoint (lat, lon)
            num_points: Number of points to generate
        
        Returns:
            List of points on the curve
        """
        # Convert to local coordinates (approximate)
        def latlon_to_xy(lat, lon, ref_lat, ref_lon):
            R = 6371000  # Earth radius
            x = R * np.radians(lon - ref_lon) * np.cos(np.radians(ref_lat))
            y = R * np.radians(lat - ref_lat)
            return x, y
        
        def xy_to_latlon(x, y, ref_lat, ref_lon):
            R = 6371000
            lat = ref_lat + np.degrees(y / R)
            lon = ref_lon + np.degrees(x / (R * np.cos(np.radians(ref_lat))))
            return lat, lon
        
        # Use p1 as reference
        x0, y0 = latlon_to_xy(p0[0], p0[1], p1[0], p1[1])
        x1, y1 = 0, 0  # p1 is at origin
        x2, y2 = latlon_to_xy(p2[0], p2[1], p1[0], p1[1])
        
        # Calculate control points
        # Direction vectors
        d01 = np.array([x1 - x0, y1 - y0])
        d12 = np.array([x2 - x1, y2 - y1])
        
        # Normalize
        len01 = np.linalg.norm(d01)
        len12 = np.linalg.norm(d12)
        
        if len01 > 0:
            d01 = d01 / len01
        if len12 > 0:
            d12 = d12 / len12
        
        # Control points at smoothing_distance from waypoint
        smooth_dist = min(self.smoothing_distance, len01 * 0.3, len12 * 0.3)
        
        cp0_x = x1 - d01[0] * smooth_dist
        cp0_y = y1 - d01[1] * smooth_dist
        
        cp1_x = x1 + d12[0] * smooth_dist
        cp1_y = y1 + d12[1] * smooth_dist
        
        # Generate Bezier curve points
        curve_points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # Cubic Bezier formula
            x = (1-t)**3 * cp0_x + 3*(1-t)**2*t * x1 + 3*(1-t)*t**2 * x1 + t**3 * cp1_x
            y = (1-t)**3 * cp0_y + 3*(1-t)**2*t * y1 + 3*(1-t)*t**2 * y1 + t**3 * cp1_y
            
            # Convert back to lat/lon
            lat, lon = xy_to_latlon(x, y, p1[0], p1[1])
            curve_points.append((lat, lon))
        
        return curve_points
    
    def get_lookahead_point(self, current_pos: Tuple[float, float],
                           path: List[Tuple[float, float]],
                           lookahead_distance: float = 10.0) -> Tuple[float, float]:
        """
        Get lookahead point on path for pure pursuit control
        
        Args:
            current_pos: Current position (lat, lon)
            path: Smoothed path
            lookahead_distance: Lookahead distance in meters
        
        Returns:
            Lookahead point (lat, lon)
        """
        if not path:
            return current_pos
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        
        for i, point in enumerate(path):
            dist = self.haversine_distance(current_pos, point)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Find lookahead point
        for i in range(closest_idx, len(path)):
            dist = self.haversine_distance(current_pos, path[i])
            if dist >= lookahead_distance:
                return path[i]
        
        # Return last point if no point found
        return path[-1]
    
    def haversine_distance(self, pos1: Tuple[float, float], 
                          pos2: Tuple[float, float]) -> float:
        """Calculate distance between two lat/lon points in meters"""
        R = 6371000  # Earth radius in meters
        lat1, lon1 = np.radians(pos1[0]), np.radians(pos1[1])
        lat2, lon2 = np.radians(pos2[0]), np.radians(pos2[1])
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        
        return R * c

