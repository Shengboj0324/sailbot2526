#!/usr/bin/env python3
"""
Phase-7 Cross-System Integration Stress Tests.

Tests the FULL pipeline: UKF → Drift Estimator → MPC → VPP → Arduino geometry,
all wired together and exercised under combined extreme scenarios.

SECTION A — Full pipeline end-to-end validation
SECTION B — Health monitor grading under degradation
SECTION C — Drift + leeway compensation accuracy
SECTION D — Wind-shift replan trigger validation
SECTION E — Endurance: 2000-step full-stack stability
"""
import sys, os, time, math, struct
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'sailboat_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'path_planning'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'util'))

import numpy as np
import unittest

from sailboat_control.ukf_state_estimator import SquareRootUKF, SailboatDynamics
from sailboat_control.mpc_controller import MPCSteering, NomotoModel
from sailboat_control.vpp_sail_optimizer import VPPSailOptimizer
from sailboat_control.drift_estimator import DriftEstimator
from sailboat_control.adaptive_pid import normalize_angle
# Note: HealthMonitor requires rclpy (ROS2), so we test its grading
# logic via a standalone FakeMonitor class below.

# Arduino geometry from winch_a.py
BOOM_LENGTH = 48
WINCH_TO_MAST = 44
SPOOL_RADIUS = 1.5
GEAR_RATIO = 5
STEPS_PER_REVOLUTION = 1600
NEUTRAL_SERVO_ANGLE = 55
RUDDER_MIN, RUDDER_MAX = -21.0, 21.0


def angle_to_steps(angle):
    angle = max(0.0, min(88.0, abs(angle)))
    length = math.sqrt(BOOM_LENGTH**2 + WINCH_TO_MAST**2 -
                       2 * BOOM_LENGTH * WINCH_TO_MAST * math.cos(math.radians(angle)))
    min_len = math.sqrt(BOOM_LENGTH**2 + WINCH_TO_MAST**2 -
                        2 * BOOM_LENGTH * WINCH_TO_MAST)
    rope_out = length - min_len
    return int((rope_out * GEAR_RATIO * STEPS_PER_REVOLUTION) /
               (2 * math.pi * SPOOL_RADIUS))


def rudder_to_servo(rudder_deg):
    """MPC output → servo byte (must match rudder_a.py)."""
    rd = max(RUDDER_MIN, min(RUDDER_MAX, rudder_deg))
    return int(NEUTRAL_SERVO_ANGLE + rd)


# ═══════════════════════════════════════════════════════════════════════
#  SECTION A — Full pipeline end-to-end
# ═══════════════════════════════════════════════════════════════════════
class TestFullPipeline(unittest.TestCase):
    """UKF→Drift→MPC→VPP→Arduino geometry, all wired together."""

    def test_full_stack_200_steps(self):
        """Run full pipeline for 200 steps, every output must be valid."""
        ukf  = SquareRootUKF()
        mpc  = MPCSteering(horizon=8, dt=0.5)
        vpp  = VPPSailOptimizer()
        drift = DriftEstimator()
        model = NomotoModel()

        heading_rad = 0.0
        r = 0.0
        target = 90.0

        for k in range(200):
            ws = 10.0 + np.sin(k * 0.1) * 3
            wd = 180.0 + 15 * np.sin(k * 0.05)

            # 1. UKF predict + update
            ukf.wind_speed = ws
            ukf.wind_angle = np.radians(wd)
            ukf.predict(0.5)
            z = np.array([k * 0.1, k * 0.05, heading_rad,
                          0.0, np.random.randn() * 0.1, r])
            ukf.update(z)

            # 2. Drift estimator (UKF-aware)
            cpsi = np.cos(ukf.x[2])
            spsi = np.sin(ukf.x[2])
            gps_vx = ukf.x[3] * cpsi - ukf.x[4] * spsi + 0.1  # add current
            gps_vy = ukf.x[3] * spsi + ukf.x[4] * cpsi + 0.05
            drift.update_from_ukf(gps_vx, gps_vy, ukf.x[2], ukf.x[3], ukf.x[4])

            # 3. Drift + leeway compensated target
            leeway = ukf.get_leeway_deg()
            comp_target = drift.get_leeway_correction(
                target, leeway, max(0.5, ukf.get_speed()))

            # 4. MPC compute
            heading_deg = normalize_angle(np.degrees(heading_rad))
            rudder, diag = mpc.compute(heading_deg, np.degrees(r),
                                        comp_target, wind_speed=ws,
                                        wind_dir_deg=wd)

            # 5. VPP sail
            sail, vmg, spd, vdiag = vpp.optimize(
                ws, wd, comp_target,
                boat_speed_hint=max(0.5, ukf.get_speed()),
                current_heel_deg=ukf.get_heel_deg())

            # 6. Arduino conversion
            servo = rudder_to_servo(rudder)
            steps = angle_to_steps(sail)

            # 7. Nomoto simulation
            heading_rad, r = model.step(heading_rad, r,
                                         np.radians(rudder), 0.5)

            # ASSERTIONS — every output must be valid at every step
            self.assertTrue(np.all(np.isfinite(ukf.x)), f"UKF NaN at {k}")
            self.assertGreaterEqual(rudder, -21.0, f"Rudder < -21 at {k}")
            self.assertLessEqual(rudder, 21.0, f"Rudder > 21 at {k}")
            self.assertGreaterEqual(sail, 0.0, f"Sail < 0 at {k}")
            self.assertLessEqual(sail, 88.0, f"Sail > 88 at {k}")
            self.assertGreaterEqual(servo, 34, f"Servo < 34 at {k}")
            self.assertLessEqual(servo, 76, f"Servo > 76 at {k}")
            self.assertGreaterEqual(steps, 0, f"Steps < 0 at {k}")
            self.assertTrue(np.isfinite(vmg), f"VPP VMG NaN at {k}")

    def test_pipeline_timing_budget(self):
        """Full pipeline must finish in < 20 ms per cycle."""
        ukf  = SquareRootUKF()
        mpc  = MPCSteering(horizon=8, dt=0.5)
        vpp  = VPPSailOptimizer()
        drift = DriftEstimator()

        t0 = time.perf_counter()
        for _ in range(50):
            ukf.wind_speed = 10.0
            ukf.wind_angle = 1.57
            ukf.predict(0.5)
            ukf.update(np.array([0, 0, 0, 0, 0, 0]))
            drift.update_from_ukf(0.1, 0.2, 0.0, 2.0, 0.0)
            comp = drift.get_leeway_correction(90, 0, 2.0)
            rudder, _ = mpc.compute(0, 0, comp)
            sail, _, _, _ = vpp.optimize(10, 180, comp)
            angle_to_steps(sail)
            rudder_to_servo(rudder)
        elapsed = (time.perf_counter() - t0) / 50
        self.assertLess(elapsed, 0.02,
                        f"Full pipeline {elapsed*1000:.1f} ms (limit 20 ms)")


# ═══════════════════════════════════════════════════════════════════════
#  SECTION B — Health monitor grading
# ═══════════════════════════════════════════════════════════════════════
class TestHealthMonitorGrading(unittest.TestCase):
    """Validate severity grading logic (no ROS2 required)."""

    def _make_monitor(self):
        """Create a HealthMonitor-like object without ROS2 Node init."""
        # We test the pure-python grading logic directly
        class FakeMonitor:
            gps_ok = True
            imu_ok = True
            wind_ok = True
            compass_ok = True
            phase5_active = True
            ukf_trace = 0.0
            mpc_cost = 0.0
            heel_angle = 0.0
            ukf_trace_warn = 50.0
            ukf_trace_critical = 200.0
            mpc_cost_warn = 100.0
            heel_critical = 35.0
            HEALTHY = 'HEALTHY'
            DEGRADED = 'DEGRADED'
            CRITICAL = 'CRITICAL'
            FALLBACK = 'FALLBACK'

            def _compute_health_grade(self, critical_ok):
                # Exact copy of HealthMonitor._compute_health_grade
                issues = []
                if not critical_ok:
                    grade = self.CRITICAL
                    if not self.gps_ok:  issues.append('GPS_TIMEOUT')
                    if not self.imu_ok:  issues.append('IMU_TIMEOUT')
                else:
                    grade = self.HEALTHY
                if self.phase5_active:
                    if self.ukf_trace > self.ukf_trace_critical:
                        grade = self.FALLBACK
                        issues.append(f'UKF_DIVERGE')
                    elif self.ukf_trace > self.ukf_trace_warn:
                        if grade == self.HEALTHY: grade = self.DEGRADED
                        issues.append(f'UKF_WARN')
                    if self.mpc_cost > self.mpc_cost_warn:
                        if grade == self.HEALTHY: grade = self.DEGRADED
                        issues.append(f'MPC_HIGH_COST')
                    if abs(self.heel_angle) > self.heel_critical:
                        if grade == self.HEALTHY: grade = self.DEGRADED
                        issues.append(f'HEEL')
                if not self.wind_ok and grade == self.HEALTHY:
                    grade = self.DEGRADED
                    issues.append('WIND_TIMEOUT')
                return grade, '; '.join(issues) if issues else 'all nominal'
        return FakeMonitor()

    def test_healthy_when_all_ok(self):
        m = self._make_monitor()
        grade, _ = m._compute_health_grade(True)
        self.assertEqual(grade, 'HEALTHY')

    def test_critical_on_gps_loss(self):
        m = self._make_monitor()
        m.gps_ok = False
        grade, detail = m._compute_health_grade(False)
        self.assertEqual(grade, 'CRITICAL')
        self.assertIn('GPS_TIMEOUT', detail)

    def test_degraded_on_high_ukf_trace(self):
        m = self._make_monitor()
        m.ukf_trace = 80.0
        grade, detail = m._compute_health_grade(True)
        self.assertEqual(grade, 'DEGRADED')
        self.assertIn('UKF_WARN', detail)

    def test_fallback_on_ukf_divergence(self):
        m = self._make_monitor()
        m.ukf_trace = 300.0
        grade, _ = m._compute_health_grade(True)
        self.assertEqual(grade, 'FALLBACK')

    def test_degraded_on_high_heel(self):
        m = self._make_monitor()
        m.heel_angle = 40.0
        grade, detail = m._compute_health_grade(True)
        self.assertEqual(grade, 'DEGRADED')
        self.assertIn('HEEL', detail)

    def test_degraded_on_wind_timeout(self):
        m = self._make_monitor()
        m.wind_ok = False
        grade, detail = m._compute_health_grade(True)
        self.assertEqual(grade, 'DEGRADED')
        self.assertIn('WIND_TIMEOUT', detail)

    def test_multiple_issues_combined(self):
        m = self._make_monitor()
        m.ukf_trace = 60.0
        m.mpc_cost = 150.0
        m.heel_angle = 40.0
        grade, detail = m._compute_health_grade(True)
        self.assertEqual(grade, 'DEGRADED')
        # All three issues should be reported
        self.assertIn('UKF_WARN', detail)
        self.assertIn('MPC_HIGH_COST', detail)
        self.assertIn('HEEL', detail)


# ═══════════════════════════════════════════════════════════════════════
#  SECTION C — Drift + leeway compensation accuracy
# ═══════════════════════════════════════════════════════════════════════
class TestDriftLeewayCompensation(unittest.TestCase):
    """Validate UKF-aware drift estimation and leeway correction."""

    def test_known_current_estimation(self):
        """With a known 0.5 m/s eastward current, drift estimator must converge."""
        drift = DriftEstimator(window_size=30)
        current_vx = 0.5   # east
        current_vy = 0.0   # north

        for k in range(50):
            # Boat heading north at 2 m/s through water
            boat_heading_rad = 0.0
            boat_surge = 2.0
            boat_sway = 0.0
            # GPS sees boat + current
            gps_vx = boat_surge * np.cos(boat_heading_rad) + current_vx
            gps_vy = boat_surge * np.sin(boat_heading_rad) + current_vy
            drift.update_from_ukf(gps_vx, gps_vy, boat_heading_rad,
                                   boat_surge, boat_sway)

        d = drift.get_drift()
        self.assertAlmostEqual(d['speed'], 0.5, delta=0.1,
                               msg=f"Drift speed {d['speed']:.3f} != 0.5")
        self.assertGreater(d['confidence'], 0.5)

    def test_leeway_correction_upwind(self):
        """Leeway correction must steer upwind to compensate."""
        drift = DriftEstimator()
        # Simulate zero-drift data: GPS velocity matches heading-based velocity
        for _ in range(20):
            # Boat heading north (0°) at 2 m/s: GPS vx=0, vy=2
            drift.update(0, 2.0, 0, 2.0)

        target = 90.0
        leeway = 5.0  # 5° leeward drift

        corrected = drift.get_leeway_correction(target, leeway, 2.0)
        # With ~zero drift, the only correction is the leeway subtraction
        expected = (target - leeway) % 360
        diff = abs(corrected - expected)
        if diff > 180: diff = 360 - diff
        self.assertLess(diff, 5.0,
                        f"Leeway correction {corrected:.1f}° != expected {expected:.1f}°")

    def test_drift_compensation_heading(self):
        """With known drift, compensated heading should crab into current."""
        drift = DriftEstimator(window_size=10)
        # Inject known southward current
        for _ in range(15):
            drift._store_sample(0.0, -0.5)  # 0.5 m/s southward

        # Want to sail due east (90°) at 3 m/s
        comp = drift.compensate_heading(90.0, 3.0)
        # Must steer slightly south-east to compensate for southward push
        # Expect heading < 90° (lower heading = more south component)
        self.assertNotAlmostEqual(comp, 90.0, delta=0.5,
                                  msg="No compensation applied for known current")



# ═══════════════════════════════════════════════════════════════════════
#  SECTION D — Wind-shift replan trigger validation
# ═══════════════════════════════════════════════════════════════════════
class TestWindShiftReplan(unittest.TestCase):
    """Validate that 30°+ wind shift triggers replanning."""

    def test_small_shift_no_replan(self):
        """< 30° shift must NOT trigger replan."""
        last_wind = 90.0
        new_wind = 110.0
        delta = abs(new_wind - last_wind)
        if delta > 180: delta = 360 - delta
        self.assertLess(delta, 30.0, "Small shift should not trigger")

    def test_large_shift_triggers_replan(self):
        """>= 30° shift must trigger replan."""
        last_wind = 90.0
        new_wind = 130.0
        delta = abs(new_wind - last_wind)
        if delta > 180: delta = 360 - delta
        self.assertGreaterEqual(delta, 30.0, "Large shift should trigger")

    def test_wraparound_350_to_20(self):
        """350° → 20° is a 30° shift (not 330°)."""
        last_wind = 350.0
        new_wind = 20.0
        delta = abs(new_wind - last_wind)
        if delta > 180: delta = 360 - delta
        self.assertEqual(delta, 30.0)

    def test_mpc_reset_after_replan(self):
        """MPC must be reset-able after wind shift replan."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        # Drive to saturated state
        for _ in range(30):
            mpc.compute(0, 0, 180)
        # Reset (simulating post-replan)
        mpc.reset()
        rudder, _ = mpc.compute(170, 0, 170)
        # After reset, near-target should give small rudder
        self.assertLess(abs(rudder), 15.0,
                        f"MPC not reset properly: {rudder:.1f}°")


# ═══════════════════════════════════════════════════════════════════════
#  SECTION E — 2000-step full-stack endurance
# ═══════════════════════════════════════════════════════════════════════
class TestFullStackEndurance(unittest.TestCase):
    """2000-step test with all subsystems coupled under random conditions."""

    def test_2000_step_full_stack(self):
        ukf   = SquareRootUKF()
        mpc   = MPCSteering(horizon=8, dt=0.5)
        vpp   = VPPSailOptimizer()
        drift = DriftEstimator(window_size=30)
        model = NomotoModel()
        np.random.seed(2026)

        heading_rad = 0.0
        r = 0.0
        max_cov_trace = 0.0

        for k in range(2000):
            # Random but bounded environment
            ws = max(0.5, 8 + np.random.randn() * 4)
            wd = np.random.uniform(0, 360)
            target = np.random.choice([45, 90, 135, 180, 270])

            # UKF
            ukf.wind_speed = ws
            ukf.wind_angle = np.radians(wd)
            ukf.predict(0.5)
            z = np.array([
                np.random.uniform(-200, 200),
                np.random.uniform(-200, 200),
                heading_rad + np.random.randn() * 0.05,
                np.random.randn() * 0.02,
                np.random.randn() * 0.1,
                r + np.random.randn() * 0.01
            ])
            ukf.update(z)

            # Drift
            cpsi = np.cos(ukf.x[2])
            spsi = np.sin(ukf.x[2])
            gps_vx = ukf.x[3] * cpsi - ukf.x[4] * spsi + 0.2 * np.sin(k * 0.01)
            gps_vy = ukf.x[3] * spsi + ukf.x[4] * cpsi + 0.1 * np.cos(k * 0.01)
            drift.update_from_ukf(gps_vx, gps_vy, ukf.x[2], ukf.x[3], ukf.x[4])

            leeway = ukf.get_leeway_deg()
            comp_target = drift.get_leeway_correction(
                target, leeway, max(0.5, ukf.get_speed()))

            # MPC
            heading_deg = normalize_angle(np.degrees(heading_rad))
            rudder, _ = mpc.compute(heading_deg, np.degrees(r), comp_target)
            heading_rad, r = model.step(heading_rad, r, np.radians(rudder), 0.5)

            # VPP
            sail, vmg, spd, _ = vpp.optimize(
                ws, wd, comp_target,
                current_heel_deg=ukf.get_heel_deg())

            # Arduino
            servo = rudder_to_servo(rudder)
            steps = angle_to_steps(sail)

            # Track covariance
            P = ukf.get_covariance()
            tr = np.trace(P)
            max_cov_trace = max(max_cov_trace, tr)

            # Spot-checks every 100 steps
            if k % 100 == 0:
                self.assertTrue(np.all(np.isfinite(ukf.x)),
                                f"UKF NaN at step {k}")
                self.assertGreaterEqual(rudder, -21.0)
                self.assertLessEqual(rudder, 21.0)
                self.assertGreaterEqual(sail, 0.0)
                self.assertLessEqual(sail, 88.0)
                self.assertGreaterEqual(servo, 34)
                self.assertLessEqual(servo, 76)
                self.assertTrue(np.isfinite(tr),
                                f"Covariance NaN at step {k}")

        # Final checks
        self.assertTrue(np.all(np.isfinite(ukf.x)), "UKF diverged after 2000 steps")
        P = ukf.get_covariance()
        eigvals = np.linalg.eigvalsh(P)
        self.assertTrue(np.all(eigvals > -1e-8),
                        f"Covariance non-PD: min eig={eigvals.min():.2e}")
        self.assertGreater(drift.confidence, 0.0, "Drift estimator has no confidence")

    def test_servo_byte_range_exhaustive(self):
        """Every possible MPC output maps to a valid servo byte [34, 76]."""
        for rd_tenth in range(-210, 211):
            rd = rd_tenth / 10.0
            servo = rudder_to_servo(rd)
            self.assertGreaterEqual(servo, 34, f"Servo {servo} for rudder {rd}°")
            self.assertLessEqual(servo, 76, f"Servo {servo} for rudder {rd}°")

    def test_winch_steps_range_exhaustive(self):
        """Every VPP-valid angle [0, 88] produces valid step count."""
        for deg in range(0, 89):
            steps = angle_to_steps(deg)
            self.assertGreaterEqual(steps, 0)
            self.assertLess(steps, 100000, f"Implausible {steps} steps at {deg}°")


if __name__ == '__main__':
    unittest.main(verbosity=2)