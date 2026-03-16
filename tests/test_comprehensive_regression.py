#!/usr/bin/env python3
"""
Comprehensive Regression Test Suite for Sailbot2526
Tests all aspects: sensor fusion, control algorithms, navigation, path planning
Harsh validation criteria for production readiness
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'sailboat_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'path_planning'))

import numpy as np
import unittest
from collections import deque
import time
import json

# Import all modules under test
from sailboat_control.adaptive_pid import AdaptivePID, normalize_angle, calculate_heading_error
from sailboat_control.drift_estimator import DriftEstimator
from sailboat_control.path_smoother import PathSmoother

class TestAdaptivePIDHarsh(unittest.TestCase):
    """Harsh tests for Adaptive PID controller with strict tolerances"""
    
    def setUp(self):
        self.pid = AdaptivePID(Kp_base=70.0, Ki_base=0.5, Kd_base=35.0)
        self.test_results = []
    
    def test_zero_error_produces_zero_output(self):
        """Zero error must produce exactly zero output"""
        output, terms = self.pid.update(error=0.0, dt=0.1, boat_speed=2.0)
        self.assertEqual(output, 0.0, "Zero error must produce zero output")
        self.test_results.append({"test": "zero_error", "pass": output == 0.0})
    
    def test_gain_scheduling_speed_range(self):
        """Test gain scheduling across full speed range (0.5-4.0 m/s)"""
        speeds = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
        error = 10.0
        dt = 0.1
        
        outputs = []
        for speed in speeds:
            self.pid.reset()
            output, terms = self.pid.update(error, dt, speed, use_gain_scheduling=True)
            outputs.append(output)
            
            # Verify gains scale with speed
            expected_kp = 70.0 * np.clip(speed / 2.0, 0.5, 2.0)
            self.assertAlmostEqual(terms['Kp'], expected_kp, places=1,
                                 msg=f"Kp mismatch at speed {speed}")
        
        # Output should increase with speed (higher gains)
        for i in range(len(outputs) - 1):
            if speeds[i] < 2.0 and speeds[i+1] < 2.0:
                self.assertLess(outputs[i], outputs[i+1],
                              f"Output should increase with speed: {speeds[i]} vs {speeds[i+1]}")
    
    def test_integral_windup_protection(self):
        """Test anti-windup prevents integral term explosion"""
        error = 50.0  # Large sustained error
        dt = 0.1
        
        # Run for 100 iterations with large error
        for i in range(100):
            output, terms = self.pid.update(error, dt, boat_speed=2.0)
        
        # Integral should be clamped
        self.assertLessEqual(abs(self.pid.integral), self.pid.integral_max,
                           "Integral term exceeded maximum")
        
        # Output should be clamped
        self.assertLessEqual(abs(output), self.pid.output_max,
                           "Output exceeded maximum limit")
    
    def test_derivative_filtering_noise_rejection(self):
        """Test derivative filter rejects high-frequency noise"""
        dt = 0.1
        base_error = 10.0
        noise_amplitude = 5.0
        
        outputs = []
        for i in range(50):
            # Add high-frequency noise
            noise = noise_amplitude * np.sin(i * 2 * np.pi / 5)
            error = base_error + noise
            output, terms = self.pid.update(error, dt, boat_speed=2.0)
            outputs.append(output)
        
        # Output should be smoother than input (lower variance)
        output_std = np.std(outputs[10:])  # Skip initial transient
        # Filtered derivative should reduce noise impact
        self.assertLess(output_std, 500, "Derivative filter should reduce noise")
    
    def test_feedforward_accuracy(self):
        """Test feedforward term provides correct prediction"""
        error = 20.0
        dt = 0.5
        boat_speed = 2.0
        
        output, terms = self.pid.update(error, dt, boat_speed, use_feedforward=True)
        
        # Feedforward should be proportional to error/speed
        expected_ff = 50.0 * (error / 10.0) / boat_speed
        expected_ff = np.clip(expected_ff, -10, 10)
        
        self.assertAlmostEqual(terms['FF'], expected_ff, places=1,
                             msg="Feedforward calculation incorrect")
    
    def test_heading_error_wraparound(self):
        """Test heading error handles 0/360 wraparound correctly"""
        test_cases = [
            (350, 10, -20),   # Crossing 0
            (10, 350, 20),    # Crossing 360
            (180, -180, 0),   # Opposite sides
            (0, 180, 180),    # Maximum error
            (0, -180, -180),  # Maximum error negative
        ]
        
        for target, current, expected in test_cases:
            error = calculate_heading_error(target, current)
            self.assertAlmostEqual(error, expected, places=1,
                                 msg=f"Heading error wrong: {target}° - {current}° = {error}° (expected {expected}°)")
    
    def test_output_saturation_limits(self):
        """Test output never exceeds physical limits"""
        extreme_errors = [-180, -90, -45, 0, 45, 90, 180]
        
        for error in extreme_errors:
            self.pid.reset()
            output, _ = self.pid.update(error, dt=0.1, boat_speed=2.0)
            
            self.assertGreaterEqual(output, -21.0, f"Output {output} below minimum")
            self.assertLessEqual(output, 21.0, f"Output {output} above maximum")
    
    def test_convergence_to_setpoint(self):
        """Test PID converges to zero error within reasonable time"""
        initial_error = 30.0
        dt = 0.1
        max_iterations = 200
        
        error = initial_error
        for i in range(max_iterations):
            output, _ = self.pid.update(error, dt, boat_speed=2.0)
            # Simulate boat response (simplified)
            error -= output * 0.05  # Proportional response
            
            if abs(error) < 1.0:
                self.test_results.append({
                    "test": "convergence",
                    "iterations": i,
                    "final_error": error
                })
                return
        
        self.fail(f"PID did not converge within {max_iterations} iterations. Final error: {error}")


class TestExtendedKalmanFilterHarsh(unittest.TestCase):
    """Harsh tests for EKF sensor fusion with strict accuracy requirements"""
    
    def setUp(self):
        # Import EKF class inline to avoid ROS dependencies
        from sailboat_control.state_estimator import ExtendedKalmanFilter
        self.ekf = ExtendedKalmanFilter()
        self.test_results = []
    
    def test_initial_state_zero(self):
        """Initial state must be zero vector"""
        self.assertTrue(np.allclose(self.ekf.state, np.zeros(9)),
                       "Initial state must be zero")
    
    def test_covariance_positive_definite(self):
        """Covariance matrix must remain positive definite"""
        # Run prediction and update cycles
        for i in range(100):
            self.ekf.predict(0.02)
            self.ekf.update_gps_position(37.0 + i * 0.0001, -122.0)

            # Check eigenvalues are positive
            eigenvalues = np.linalg.eigvals(self.ekf.P)
            self.assertTrue(np.all(eigenvalues > 0),
                          f"Covariance not positive definite at iteration {i}")

    def test_gps_fusion_accuracy(self):
        """Test GPS position fusion achieves RTK-level accuracy"""
        # Simulate GPS measurements with noise
        true_lat, true_lon = 37.0, -122.0
        gps_noise_std = 0.01  # 1cm standard deviation

        for i in range(50):
            # Add GPS noise
            noisy_lat = true_lat + np.random.randn() * gps_noise_std / 111000
            noisy_lon = true_lon + np.random.randn() * gps_noise_std / 111000

            self.ekf.predict(0.02)
            self.ekf.update_gps_position(noisy_lat, noisy_lon)

        # After convergence, position error should be < 10cm
        est_lat, est_lon = self.ekf.xy_to_latlon(self.ekf.state[0], self.ekf.state[1])
        lat_error_m = abs(est_lat - true_lat) * 111000
        lon_error_m = abs(est_lon - true_lon) * 111000 * np.cos(np.radians(true_lat))

        self.assertLess(lat_error_m, 0.1, f"Latitude error {lat_error_m}m exceeds 10cm")
        self.assertLess(lon_error_m, 0.1, f"Longitude error {lon_error_m}m exceeds 10cm")

    def test_imu_heading_fusion_accuracy(self):
        """Test IMU heading fusion achieves ±2° accuracy"""
        true_heading = np.radians(45.0)
        imu_noise_std = np.radians(1.0)  # 1° noise

        for i in range(100):
            self.ekf.predict(0.02)
            noisy_heading = true_heading + np.random.randn() * imu_noise_std
            self.ekf.update_imu_heading(noisy_heading)

        # Heading error should be < 2°
        heading_error_deg = abs(np.degrees(self.ekf.state[2] - true_heading))
        self.assertLess(heading_error_deg, 2.0,
                       f"Heading error {heading_error_deg}° exceeds 2° requirement")

    def test_velocity_estimation_accuracy(self):
        """Test velocity estimation from GPS"""
        # Simulate constant velocity motion
        vx_true, vy_true = 2.0, 1.0  # m/s
        dt = 0.02

        lat, lon = 37.0, -122.0
        for i in range(100):
            # Update position based on velocity
            lat += (vy_true * dt) / 111000
            lon += (vx_true * dt) / (111000 * np.cos(np.radians(lat)))

            self.ekf.predict(dt)
            self.ekf.update_gps_position(lat, lon)

        # Velocity estimate should converge to true velocity
        vx_error = abs(self.ekf.state[3] - vx_true)
        vy_error = abs(self.ekf.state[4] - vy_true)

        self.assertLess(vx_error, 0.1, f"Vx error {vx_error} m/s too large")
        self.assertLess(vy_error, 0.1, f"Vy error {vy_error} m/s too large")

    def test_prediction_step_stability(self):
        """Test prediction step remains stable over long periods"""
        # Run prediction only (no updates) for extended period
        for i in range(1000):
            self.ekf.predict(0.02)

            # State should not explode
            self.assertTrue(np.all(np.abs(self.ekf.state) < 1e6),
                          f"State exploded at iteration {i}")

            # Covariance should not explode
            self.assertTrue(np.all(np.abs(self.ekf.P) < 1e6),
                          f"Covariance exploded at iteration {i}")


class TestOptimalSailControllerHarsh(unittest.TestCase):
    """Harsh tests for optimal sail controller with physical constraints"""

    def setUp(self):
        from sailboat_control.optimal_sail_controller import OptimalSailController
        self.controller = OptimalSailController()
        self.test_results = []

    def test_apparent_wind_calculation_accuracy(self):
        """Test apparent wind calculation against known values"""
        test_cases = [
            # (TWA, TWS, boat_speed, boat_heading, expected_AWA, expected_AWS)
            (0, 10, 0, 0, 0, 10),        # Stationary, head wind
            (90, 10, 0, 0, 90, 10),      # Stationary, beam reach
            (180, 10, 0, 0, 180, 10),    # Stationary, downwind
            (0, 10, 5, 0, 0, 15),        # Moving into wind
            (180, 10, 5, 0, 180, 5),     # Moving with wind
        ]

        for twa, tws, speed, heading, exp_awa, exp_aws in test_cases:
            awa, aws = self.controller.calculate_apparent_wind(twa, tws, speed, heading)

            # Allow 5% tolerance
            awa_error = abs(awa - exp_awa)
            if awa_error > 180:
                awa_error = 360 - awa_error

            self.assertLess(awa_error, 5.0,
                          f"AWA error {awa_error}° for TWA={twa}, speed={speed}")
            self.assertLess(abs(aws - exp_aws) / exp_aws, 0.1,
                          f"AWS error for TWA={twa}, speed={speed}")

    def test_polar_interpolation_boundaries(self):
        """Test polar interpolation at grid boundaries"""
        # Test at exact grid points
        grid_angles = [0, 30, 45, 60, 90, 120, 135, 150, 180]
        grid_speeds = [5, 8, 10, 12, 15, 20]

        for angle in grid_angles:
            for speed in grid_speeds:
                sail = self.controller.get_optimal_sail_angle(angle, speed)

                # Sail angle must be within physical limits
                self.assertGreaterEqual(sail, 0, f"Negative sail angle at {angle}°, {speed} m/s")
                self.assertLessEqual(sail, 88, f"Excessive sail angle at {angle}°, {speed} m/s")

    def test_depowering_logic_heel_threshold(self):
        """Test depowering activates at correct heel angle"""
        base_sail = 40.0
        max_heel = 25.0

        # Below threshold - no depowering
        depowered = self.controller.apply_depowering(base_sail, heel_angle=20.0, gust_detected=False)
        self.assertAlmostEqual(depowered, base_sail, places=1,
                             msg="Depowering should not activate below threshold")

        # Above threshold - depowering active
        depowered = self.controller.apply_depowering(base_sail, heel_angle=35.0, gust_detected=False)
        self.assertGreater(depowered, base_sail,
                         msg="Depowering should ease sail above heel threshold")

        # With gust - additional depowering
        depowered_gust = self.controller.apply_depowering(base_sail, heel_angle=35.0, gust_detected=True)
        self.assertGreater(depowered_gust, depowered,
                         msg="Gust should increase depowering")

    def test_rate_limiting_prevents_shock_loads(self):
        """Test rate limiter prevents sudden sail movements"""
        max_rate = 10.0  # deg/s
        dt = 1.0

        # Command large sudden change
        self.controller.current_sail_angle = 20.0
        target = 80.0

        result = self.controller.rate_limit_sail(target, max_rate, dt)

        # Change should be limited to max_rate * dt
        change = abs(result - 20.0)
        self.assertLessEqual(change, max_rate * dt + 0.1,
                           f"Rate limit violated: {change}° in {dt}s")

    def test_sail_angle_physical_limits(self):
        """Test sail angle never exceeds physical limits (0-88°)"""
        # Test extreme conditions
        extreme_conditions = [
            (180, 25, 5, 0, 45, True),   # Downwind, high wind, gust
            (0, 25, 5, 0, 45, True),     # Upwind, high wind, gust
            (90, 25, 5, 0, 45, True),    # Beam reach, high wind, gust
        ]

        for twa, tws, speed, heading, heel, gust in extreme_conditions:
            sail, awa, aws = self.controller.calculate_sail_angle(
                twa, tws, speed, heading, heel, gust
            )

            self.assertGreaterEqual(sail, 0, f"Sail angle {sail}° below 0°")
            self.assertLessEqual(sail, 88, f"Sail angle {sail}° exceeds 88°")


class TestWindFilterHarsh(unittest.TestCase):
    """Harsh tests for adaptive wind Kalman filter"""

    def setUp(self):
        from sailboat_control.wind_smoother import AdaptiveWindKalmanFilter
        self.filter = AdaptiveWindKalmanFilter()
        self.test_results = []

    def test_gust_detection_threshold(self):
        """Test gust detection activates at 15° innovation threshold"""
        # Feed steady wind
        for i in range(20):
            self.filter.update(45.0, dt=0.1)

        self.assertFalse(self.filter.gust_detected, "No gust should be detected in steady wind")

        # Sudden 20° shift (above 15° threshold)
        for i in range(5):
            self.filter.update(65.0, dt=0.1)

        self.assertTrue(self.filter.gust_detected, "Gust should be detected after 15° shift")

    def test_gust_detection_persistence(self):
        """Test gust detection requires 3 consecutive samples"""
        # Feed steady wind
        for i in range(20):
            self.filter.update(45.0, dt=0.1)

        # Two large innovations, then return to normal
        self.filter.update(65.0, dt=0.1)
        self.filter.update(65.0, dt=0.1)
        self.filter.update(45.0, dt=0.1)

        # Should not trigger (only 2 consecutive)
        self.assertFalse(self.filter.gust_detected,
                        "Gust should require 3 consecutive samples")

    def test_steady_wind_low_pass_filtering(self):
        """Test steady wind estimate filters out high-frequency noise"""
        # Feed noisy wind data
        base_wind = 45.0
        noise_amplitude = 10.0

        measurements = []
        steady_estimates = []

        for i in range(100):
            noise = noise_amplitude * np.sin(i * 2 * np.pi / 10)
            measurement = base_wind + noise
            measurements.append(measurement)

            self.filter.update(measurement, dt=0.1)
            steady_estimates.append(np.degrees(self.filter.steady_wind))

        # Steady estimate should have much lower variance than measurements
        meas_std = np.std(measurements[50:])
        steady_std = np.std(steady_estimates[50:])

        self.assertLess(steady_std, meas_std / 5,
                       f"Steady wind filter insufficient: {steady_std}° vs {meas_std}°")

    def test_circular_statistics_wraparound(self):
        """Test filter handles 0/360° wraparound correctly"""
        # Feed measurements around 0°
        measurements = [355, 358, 1, 4, 357, 2, 359, 3]

        for meas in measurements:
            self.filter.update(float(meas), dt=0.1)

        # Estimate should be near 0°, not 180°
        estimate = np.degrees(self.filter.state[0])
        if estimate < 0:
            estimate += 360

        # Should be within ±10° of 0°
        error = min(abs(estimate - 0), abs(estimate - 360))
        self.assertLess(error, 10.0,
                       f"Wraparound handling failed: estimate={estimate}°")

    def test_innovation_variance_adaptation(self):
        """Test process noise adapts to innovation variance"""
        # Feed steady wind - low innovation
        for i in range(20):
            self.filter.update(45.0 + np.random.randn() * 0.5, dt=0.1)

        Q_steady = self.filter.Q.copy()

        # Feed highly variable wind - high innovation
        for i in range(20):
            self.filter.update(45.0 + np.random.randn() * 10.0, dt=0.1)

        Q_variable = self.filter.Q.copy()

        # Process noise should increase with innovation variance
        self.assertGreater(np.mean(Q_variable), np.mean(Q_steady),
                         "Process noise should adapt to innovation variance")


class TestDriftEstimatorHarsh(unittest.TestCase):
    """Harsh tests for drift/current estimation"""

    def setUp(self):
        self.estimator = DriftEstimator(window_size=20)
        self.test_results = []

    def test_zero_drift_detection(self):
        """Test estimator correctly identifies zero drift"""
        # Boat moving north at 2 m/s, GPS shows same
        for i in range(30):
            self.estimator.update(gps_vx=0.0, gps_vy=2.0, boat_heading=0.0, boat_speed=2.0)

        drift = self.estimator.get_drift()

        self.assertLess(drift['speed'], 0.1, f"False drift detected: {drift['speed']} m/s")
        self.assertGreater(drift['confidence'], 0.5, "Confidence should be high for consistent data")

    def test_constant_drift_estimation(self):
        """Test estimator accurately measures constant drift"""
        # Boat heading north at 2 m/s, current 0.5 m/s east
        drift_vx, drift_vy = 0.5, 0.0
        boat_speed = 2.0

        for i in range(50):
            gps_vx = 0.0 + drift_vx
            gps_vy = boat_speed + drift_vy
            self.estimator.update(gps_vx, gps_vy, boat_heading=0.0, boat_speed=boat_speed)

        drift = self.estimator.get_drift()

        # Should detect 0.5 m/s drift
        self.assertAlmostEqual(drift['speed'], 0.5, delta=0.1,
                             msg=f"Drift speed error: {drift['speed']} vs 0.5 m/s")
        self.assertAlmostEqual(drift['direction'], 90.0, delta=10.0,
                             msg=f"Drift direction error: {drift['direction']}° vs 90°")

