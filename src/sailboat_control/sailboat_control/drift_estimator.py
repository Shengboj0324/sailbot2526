#!/usr/bin/env python3
"""
Drift / Current Estimator — Phase-5 Enhanced.

Two estimation modes:
  1. Classic:  GPS ground-track velocity minus heading-based velocity
  2. UKF-aware:  Uses UKF surge (u) and sway (v) in the body frame
     to directly extract leeway and current without heading-error bias.

The UKF path is strictly superior because it already fuses IMU gyro, so
the heading used to compute "expected" velocity is much more accurate.
"""
import numpy as np
from collections import deque


class DriftEstimator:
    """Estimates drift/current from GPS track vs boat motion."""

    def __init__(self, window_size=20):
        self.window_size = window_size
        self.drift_estimates = deque(maxlen=window_size)

        # Current estimate
        self.drift_speed = 0.0      # m/s
        self.drift_direction = 0.0  # degrees (0 = North)

        # Confidence
        self.confidence = 0.0

    # ── Classic update (Phase-4 path) ────────────────────────────────
    def update(self, gps_vx, gps_vy, boat_heading, boat_speed):
        """Update drift from GPS velocity vs heading-based velocity."""
        heading_rad = np.radians(boat_heading)
        expected_vx = boat_speed * np.sin(heading_rad)
        expected_vy = boat_speed * np.cos(heading_rad)

        drift_vx = gps_vx - expected_vx
        drift_vy = gps_vy - expected_vy

        self._store_sample(drift_vx, drift_vy)

    # ── UKF-aware update (Phase-5 path) ──────────────────────────────
    def update_from_ukf(self, gps_vx, gps_vy, ukf_heading_rad,
                         ukf_surge, ukf_sway):
        """Update drift using UKF body-frame velocities.

        The UKF provides surge (u, forward) and sway (v, sideways) in the
        body frame.  Convert to earth frame and subtract from GPS velocity
        to isolate the current.

        Args:
            gps_vx:  GPS velocity east  (m/s)
            gps_vy:  GPS velocity north (m/s)
            ukf_heading_rad:  UKF heading (radians, 0=East math convention)
            ukf_surge: body-frame forward velocity (m/s)
            ukf_sway:  body-frame lateral velocity (m/s, + = starboard)
        """
        cpsi = np.cos(ukf_heading_rad)
        spsi = np.sin(ukf_heading_rad)

        # Body → Earth
        boat_vx = ukf_surge * cpsi - ukf_sway * spsi
        boat_vy = ukf_surge * spsi + ukf_sway * cpsi

        drift_vx = gps_vx - boat_vx
        drift_vy = gps_vy - boat_vy

        self._store_sample(drift_vx, drift_vy)

    # ── Internal ─────────────────────────────────────────────────────
    def _store_sample(self, drift_vx, drift_vy):
        drift_speed = np.sqrt(drift_vx**2 + drift_vy**2)
        drift_dir   = np.degrees(np.arctan2(drift_vx, drift_vy)) % 360

        self.drift_estimates.append({
            'speed': drift_speed, 'direction': drift_dir,
            'vx': drift_vx, 'vy': drift_vy
        })

        if len(self.drift_estimates) >= 5:
            avg_vx = np.mean([d['vx'] for d in self.drift_estimates])
            avg_vy = np.mean([d['vy'] for d in self.drift_estimates])

            self.drift_speed     = np.sqrt(avg_vx**2 + avg_vy**2)
            self.drift_direction = np.degrees(np.arctan2(avg_vx, avg_vy)) % 360

            speeds = [d['speed'] for d in self.drift_estimates]
            speed_std = np.std(speeds)
            self.confidence = 1.0 / (1.0 + speed_std)
        else:
            self.confidence = 0.0

    def get_drift(self):
        """Get current drift estimate."""
        return {
            'speed': self.drift_speed,
            'direction': self.drift_direction,
            'confidence': self.confidence
        }

    def compensate_heading(self, target_heading, boat_speed):
        """Calculate heading compensation for drift.

        Returns compensated heading to steer (degrees).
        """
        if self.confidence < 0.3 or boat_speed < 0.5:
            return target_heading

        target_rad = np.radians(target_heading)
        drift_rad  = np.radians(self.drift_direction)

        desired_vx = boat_speed * np.sin(target_rad)
        desired_vy = boat_speed * np.cos(target_rad)

        drift_vx = self.drift_speed * np.sin(drift_rad)
        drift_vy = self.drift_speed * np.cos(drift_rad)

        required_vx = desired_vx - drift_vx
        required_vy = desired_vy - drift_vy

        compensated_heading = np.degrees(np.arctan2(required_vx, required_vy)) % 360

        # Limit compensation to ±30 degrees
        heading_change = compensated_heading - target_heading
        if heading_change > 180:
            heading_change -= 360
        elif heading_change < -180:
            heading_change += 360

        heading_change = np.clip(heading_change, -30, 30)
        return (target_heading + heading_change) % 360

    def get_leeway_correction(self, target_heading, leeway_deg, boat_speed):
        """Apply leeway correction from UKF on top of drift compensation.

        Args:
            target_heading: desired ground track (degrees)
            leeway_deg: leeway angle from UKF (degrees, + = leeward)
            boat_speed: boat speed (m/s)

        Returns:
            Compensated heading accounting for both drift and leeway.
        """
        # First compensate for current
        heading = self.compensate_heading(target_heading, boat_speed)

        # Then compensate for leeway (steer upwind by leeway amount)
        heading = heading - leeway_deg

        # Normalise
        heading = heading % 360
        return heading

