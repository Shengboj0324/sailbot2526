#!/usr/bin/env python3
"""
Drift/Current Estimator
Estimates water current by comparing GPS track to heading
"""
import numpy as np
from collections import deque

class DriftEstimator:
    """
    Estimates drift/current by comparing GPS velocity to boat heading
    """
    
    def __init__(self, window_size=20):
        self.window_size = window_size
        self.drift_estimates = deque(maxlen=window_size)
        
        # Current estimate
        self.drift_speed = 0.0  # m/s
        self.drift_direction = 0.0  # degrees
        
        # Confidence
        self.confidence = 0.0
    
    def update(self, gps_vx, gps_vy, boat_heading, boat_speed):
        """
        Update drift estimate
        
        Args:
            gps_vx: GPS velocity in x (east) direction (m/s)
            gps_vy: GPS velocity in y (north) direction (m/s)
            boat_heading: Boat heading in degrees
            boat_speed: Boat speed through water (m/s)
        """
        # Calculate expected velocity based on heading and speed
        heading_rad = np.radians(boat_heading)
        expected_vx = boat_speed * np.sin(heading_rad)
        expected_vy = boat_speed * np.cos(heading_rad)
        
        # Drift is the difference between GPS velocity and expected velocity
        drift_vx = gps_vx - expected_vx
        drift_vy = gps_vy - expected_vy
        
        # Convert to speed and direction
        drift_speed = np.sqrt(drift_vx**2 + drift_vy**2)
        drift_direction = np.degrees(np.arctan2(drift_vx, drift_vy)) % 360
        
        # Store estimate
        self.drift_estimates.append({
            'speed': drift_speed,
            'direction': drift_direction,
            'vx': drift_vx,
            'vy': drift_vy
        })
        
        # Calculate average drift
        if len(self.drift_estimates) >= 5:
            # Average drift velocity components
            avg_vx = np.mean([d['vx'] for d in self.drift_estimates])
            avg_vy = np.mean([d['vy'] for d in self.drift_estimates])
            
            self.drift_speed = np.sqrt(avg_vx**2 + avg_vy**2)
            self.drift_direction = np.degrees(np.arctan2(avg_vx, avg_vy)) % 360
            
            # Confidence based on consistency
            speeds = [d['speed'] for d in self.drift_estimates]
            speed_std = np.std(speeds)
            self.confidence = 1.0 / (1.0 + speed_std)
        else:
            self.confidence = 0.0
    
    def get_drift(self):
        """Get current drift estimate"""
        return {
            'speed': self.drift_speed,
            'direction': self.drift_direction,
            'confidence': self.confidence
        }
    
    def compensate_heading(self, target_heading, boat_speed):
        """
        Calculate heading compensation for drift
        
        Args:
            target_heading: Desired ground track heading (degrees)
            boat_speed: Boat speed through water (m/s)
        
        Returns:
            Compensated heading to steer (degrees)
        """
        if self.confidence < 0.3 or boat_speed < 0.5:
            return target_heading
        
        # Convert to vectors
        target_rad = np.radians(target_heading)
        drift_rad = np.radians(self.drift_direction)
        
        # Desired velocity over ground
        desired_vx = boat_speed * np.sin(target_rad)
        desired_vy = boat_speed * np.cos(target_rad)
        
        # Subtract drift to get required velocity through water
        drift_vx = self.drift_speed * np.sin(drift_rad)
        drift_vy = self.drift_speed * np.cos(drift_rad)
        
        required_vx = desired_vx - drift_vx
        required_vy = desired_vy - drift_vy
        
        # Convert back to heading
        compensated_heading = np.degrees(np.arctan2(required_vx, required_vy)) % 360
        
        # Limit compensation to ±30 degrees
        heading_change = compensated_heading - target_heading
        if heading_change > 180:
            heading_change -= 360
        elif heading_change < -180:
            heading_change += 360
        
        heading_change = np.clip(heading_change, -30, 30)
        compensated_heading = (target_heading + heading_change) % 360
        
        return compensated_heading

