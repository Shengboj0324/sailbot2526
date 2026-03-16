#!/usr/bin/env python3
"""
Comprehensive Position Tracking and Navigation Test Suite
Tests boat position tracking, waypoint navigation, path following accuracy
Extremely detailed validation for academic presentation
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'sailboat_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'path_planning'))

import numpy as np
import unittest
import json
from datetime import datetime

class PositionTracker:
    """Tracks boat position with detailed metrics"""
    
    def __init__(self):
        self.positions = []
        self.timestamps = []
        self.headings = []
        self.speeds = []
        self.errors = []
        
    def add_position(self, lat, lon, heading, speed, timestamp, target_lat=None, target_lon=None):
        """Record position with optional target for error calculation"""
        self.positions.append((lat, lon))
        self.timestamps.append(timestamp)
        self.headings.append(heading)
        self.speeds.append(speed)
        
        if target_lat is not None and target_lon is not None:
            error = self.haversine_distance(lat, lon, target_lat, target_lon)
            self.errors.append(error)
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two points in meters"""
        R = 6371000
        lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        return R * c
    
    def get_statistics(self):
        """Calculate comprehensive tracking statistics"""
        if len(self.positions) < 2:
            return {}
        
        # Calculate total distance traveled
        total_distance = 0
        for i in range(len(self.positions) - 1):
            dist = self.haversine_distance(
                self.positions[i][0], self.positions[i][1],
                self.positions[i+1][0], self.positions[i+1][1]
            )
            total_distance += dist
        
        # Calculate average speed
        if len(self.timestamps) > 1:
            time_elapsed = self.timestamps[-1] - self.timestamps[0]
            avg_speed = total_distance / time_elapsed if time_elapsed > 0 else 0
        else:
            avg_speed = 0
        
        # Calculate heading changes
        heading_changes = []
        for i in range(len(self.headings) - 1):
            change = abs(self.headings[i+1] - self.headings[i])
            if change > 180:
                change = 360 - change
            heading_changes.append(change)
        
        stats = {
            'total_distance_m': total_distance,
            'num_positions': len(self.positions),
            'avg_speed_ms': avg_speed,
            'max_speed_ms': max(self.speeds) if self.speeds else 0,
            'min_speed_ms': min(self.speeds) if self.speeds else 0,
            'avg_heading_change_deg': np.mean(heading_changes) if heading_changes else 0,
            'max_heading_change_deg': max(heading_changes) if heading_changes else 0,
        }
        
        if self.errors:
            stats.update({
                'avg_cross_track_error_m': np.mean(self.errors),
                'max_cross_track_error_m': max(self.errors),
                'rms_cross_track_error_m': np.sqrt(np.mean(np.array(self.errors)**2)),
            })
        
        return stats


class TestPositionTrackingHarsh(unittest.TestCase):
    """Harsh tests for position tracking accuracy"""
    
    def setUp(self):
        from sailboat_control.state_estimator import ExtendedKalmanFilter
        self.ekf = ExtendedKalmanFilter()
        self.tracker = PositionTracker()
        self.test_results = []
    
    def test_straight_line_tracking_accuracy(self):
        """Test position tracking along straight line path"""
        # Define straight line path: north for 100m
        start_lat, start_lon = 37.0, -122.0
        target_lat = start_lat + 100 / 111000  # 100m north
        target_lon = start_lon
        
        # Simulate boat moving along path
        lat, lon = start_lat, start_lon
        velocity = 2.0  # m/s north
        dt = 0.1
        
        for i in range(100):
            # Update position
            lat += (velocity * dt) / 111000
            
            # Update EKF
            self.ekf.predict(dt)
            self.ekf.update_gps_position(lat, lon)
            
            # Track position
            est_lat, est_lon = self.ekf.xy_to_latlon(self.ekf.state[0], self.ekf.state[1])
            heading = 0.0  # North
            speed = np.sqrt(self.ekf.state[3]**2 + self.ekf.state[4]**2)
            
            self.tracker.add_position(est_lat, est_lon, heading, speed, i * dt,
                                     target_lat=lat, target_lon=lon)
        
        # Validate tracking accuracy
        stats = self.tracker.get_statistics()
        
        # Cross-track error should be < 1m
        self.assertLess(stats['rms_cross_track_error_m'], 1.0,
                       f"RMS cross-track error {stats['rms_cross_track_error_m']:.3f}m exceeds 1m")
        
        # Maximum error should be < 2m
        self.assertLess(stats['max_cross_track_error_m'], 2.0,
                       f"Max cross-track error {stats['max_cross_track_error_m']:.3f}m exceeds 2m")
        
        self.test_results.append({
            'test': 'straight_line_tracking',
            'stats': stats,
            'pass': stats['rms_cross_track_error_m'] < 1.0
        })
    
    def test_circular_path_tracking_accuracy(self):
        """Test position tracking along circular path"""
        # Define circular path: 50m radius
        center_lat, center_lon = 37.0, -122.0
        radius_m = 50.0
        angular_velocity = 0.1  # rad/s
        linear_velocity = radius_m * angular_velocity
        
        dt = 0.1
        angle = 0.0
        
        for i in range(200):
            # Calculate position on circle
            dx = radius_m * np.cos(angle)
            dy = radius_m * np.sin(angle)
            
            lat = center_lat + dy / 111000
            lon = center_lon + dx / (111000 * np.cos(np.radians(center_lat)))
            
            # Update EKF
            self.ekf.predict(dt)
            self.ekf.update_gps_position(lat, lon)
            
            # Calculate heading (tangent to circle)
            heading = np.degrees(angle + np.pi/2) % 360
            
            # Track position
            est_lat, est_lon = self.ekf.xy_to_latlon(self.ekf.state[0], self.ekf.state[1])
            speed = np.sqrt(self.ekf.state[3]**2 + self.ekf.state[4]**2)
            
            self.tracker.add_position(est_lat, est_lon, heading, speed, i * dt,
                                     target_lat=lat, target_lon=lon)
            
            angle += angular_velocity * dt
        
        stats = self.tracker.get_statistics()
        
        # Circular path is harder - allow 2m RMS error
        self.assertLess(stats['rms_cross_track_error_m'], 2.0,
                       f"Circular path RMS error {stats['rms_cross_track_error_m']:.3f}m exceeds 2m")
        
        self.test_results.append({
            'test': 'circular_path_tracking',
            'stats': stats,
            'pass': stats['rms_cross_track_error_m'] < 2.0
        })

    def test_zigzag_maneuver_tracking(self):
        """Test position tracking during aggressive zigzag maneuvers"""
        # Simulate zigzag pattern: alternating 45° turns
        lat, lon = 37.0, -122.0
        heading = 0.0
        speed = 2.0  # m/s
        dt = 0.1

        turn_interval = 50  # Change direction every 5 seconds

        for i in range(400):
            # Change heading every turn_interval steps
            if i % turn_interval == 0:
                heading = (heading + 90) % 360

            # Update position based on heading
            heading_rad = np.radians(heading)
            vx = speed * np.sin(heading_rad)
            vy = speed * np.cos(heading_rad)

            lat += (vy * dt) / 111000
            lon += (vx * dt) / (111000 * np.cos(np.radians(lat)))

            # Update EKF
            self.ekf.predict(dt)
            self.ekf.update_gps_position(lat, lon)
            self.ekf.update_imu_heading(heading_rad)

            # Track position
            est_lat, est_lon = self.ekf.xy_to_latlon(self.ekf.state[0], self.ekf.state[1])
            est_speed = np.sqrt(self.ekf.state[3]**2 + self.ekf.state[4]**2)

            self.tracker.add_position(est_lat, est_lon, heading, est_speed, i * dt,
                                     target_lat=lat, target_lon=lon)

        stats = self.tracker.get_statistics()

        # Zigzag is challenging - allow 3m RMS error
        self.assertLess(stats['rms_cross_track_error_m'], 3.0,
                       f"Zigzag RMS error {stats['rms_cross_track_error_m']:.3f}m exceeds 3m")

        # Verify heading changes are tracked
        self.assertGreater(stats['max_heading_change_deg'], 80.0,
                         "Heading changes not properly tracked")

        self.test_results.append({
            'test': 'zigzag_maneuver',
            'stats': stats,
            'pass': stats['rms_cross_track_error_m'] < 3.0
        })

    def test_gps_dropout_recovery(self):
        """Test position tracking recovers from GPS dropout"""
        lat, lon = 37.0, -122.0
        heading = 45.0
        speed = 2.0
        dt = 0.1

        for i in range(300):
            # Simulate GPS dropout from step 100-150
            if i < 100 or i > 150:
                # Normal GPS updates
                heading_rad = np.radians(heading)
                vx = speed * np.sin(heading_rad)
                vy = speed * np.cos(heading_rad)

                lat += (vy * dt) / 111000
                lon += (vx * dt) / (111000 * np.cos(np.radians(lat)))

                self.ekf.predict(dt)
                self.ekf.update_gps_position(lat, lon)
            else:
                # GPS dropout - prediction only
                self.ekf.predict(dt)

            # Always update IMU
            self.ekf.update_imu_heading(np.radians(heading))

            est_lat, est_lon = self.ekf.xy_to_latlon(self.ekf.state[0], self.ekf.state[1])
            est_speed = np.sqrt(self.ekf.state[3]**2 + self.ekf.state[4]**2)

            self.tracker.add_position(est_lat, est_lon, heading, est_speed, i * dt)

        stats = self.tracker.get_statistics()

        # Should maintain reasonable tracking despite dropout
        self.assertGreater(stats['total_distance_m'], 50.0,
                         "Position tracking failed during GPS dropout")

        self.test_results.append({
            'test': 'gps_dropout_recovery',
            'stats': stats,
            'pass': stats['total_distance_m'] > 50.0
        })


class TestWaypointNavigationHarsh(unittest.TestCase):
    """Harsh tests for waypoint navigation accuracy"""

    def setUp(self):
        from path_planning.path_planning.leg import Leg
        from path_planning.path_planning.waypoint import Waypoint
        self.Leg = Leg
        self.Waypoint = Waypoint
        self.tracker = PositionTracker()
        self.test_results = []

    def test_direct_waypoint_approach_accuracy(self):
        """Test boat reaches waypoint within threshold"""
        # Start and end positions
        start = self.Waypoint(42.0, -71.0)
        end = self.Waypoint(42.01, -71.0)  # ~1.1 km north

        # Create leg with beam reach (90° wind)
        leg = self.Leg(start, end, wind_angle=90.0, boat_heading=0.0, first_maneuver_starboard=True)
        waypoints = leg.get_waypoints()

        # Should be direct path (1 waypoint)
        self.assertEqual(len(waypoints), 1, "Direct path should have 1 waypoint")

        # Verify waypoint is at target
        wp = waypoints[0]
        distance_error = self.haversine_distance(wp.lat, wp.lon, end.lat, end.lon)

        self.assertLess(distance_error, 1.0,
                       f"Waypoint error {distance_error:.2f}m exceeds 1m threshold")

        self.test_results.append({
            'test': 'direct_waypoint',
            'waypoints': len(waypoints),
            'error_m': distance_error,
            'pass': distance_error < 1.0
        })

    def test_upwind_tacking_waypoint_accuracy(self):
        """Test tacking waypoints are geometrically correct"""
        start = self.Waypoint(42.0, -71.0)
        end = self.Waypoint(42.1, -71.0)  # ~11 km north

        # Upwind - requires tacking
        leg = self.Leg(start, end, wind_angle=0.0, boat_heading=0.0, first_maneuver_starboard=True)
        waypoints = leg.get_waypoints()

        # Should have 2 waypoints (tack point + destination)
        self.assertEqual(len(waypoints), 2, "Upwind should require tacking")

        # Verify tack point geometry
        tack_wp = waypoints[0]

        # Tack point should be at VMG angle (49.3°)
        bearing_to_tack = self.calculate_bearing(start.lat, start.lon, tack_wp.lat, tack_wp.lon)

        # Should be approximately 49.3° or 310.7° (starboard tack)
        expected_bearing = 49.3
        bearing_error = min(abs(bearing_to_tack - expected_bearing),
                          abs(bearing_to_tack - (360 - expected_bearing)))

        self.assertLess(bearing_error, 5.0,
                       f"Tack bearing error {bearing_error:.1f}° exceeds 5°")

        # Final waypoint should be at destination
        final_wp = waypoints[1]
        final_error = self.haversine_distance(final_wp.lat, final_wp.lon, end.lat, end.lon)

        self.assertLess(final_error, 1.0,
                       f"Final waypoint error {final_error:.2f}m exceeds 1m")

        self.test_results.append({
            'test': 'upwind_tacking',
            'waypoints': len(waypoints),
            'tack_bearing_error_deg': bearing_error,
            'final_error_m': final_error,
            'pass': bearing_error < 5.0 and final_error < 1.0
        })

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two points"""
        R = 6371000
        lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing from point 1 to point 2"""
        lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = np.sin(dlon) * np.cos(lat2)
        y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
        bearing = np.degrees(np.arctan2(x, y))
        return (bearing + 360) % 360
