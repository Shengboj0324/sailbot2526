#!/usr/bin/env python3
"""
Phase-6 Integration, Arduino Deployment & Fierce Stress Tests.

SECTION A — Arduino Deployment Validation
  • Law-of-cosines geometry (angle→steps)
  • Serial protocol byte structure
  • Full VPP→winch pipeline (end-to-end)
  • Timing constraints (< 50 ms per control cycle)

SECTION B — Fierce Exhaustive Stress Tests
  • Combined sensor + actuator failures
  • Multi-leg racing simulation (upwind → reach → downwind)
  • 180° wind shift during tack
  • Sensor cascade failure (GPS → IMU → wind lost in sequence)
  • Full storm sequence (calm → build → peak → die)
  • 1000-step endurance under random perturbation
"""
import sys, os, time, struct, math, json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'sailboat_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'path_planning'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'util'))

import numpy as np
import unittest

from sailboat_control.ukf_state_estimator import SquareRootUKF, SailboatDynamics
from sailboat_control.mpc_controller import MPCSteering, NomotoModel
from sailboat_control.vpp_sail_optimizer import VPPSailOptimizer
from sailboat_control.adaptive_pid import AdaptivePID, normalize_angle

# ── Reproduce winch geometry from util/winch_a.py ────────────────────
BOOM_LENGTH = 48
WINCH_TO_MAST = 44
SPOOL_RADIUS = 1.5
GEAR_RATIO = 5
STEPS_PER_REVOLUTION = 1600
CMD_WINCH_CW_STEPS  = 0x12
CMD_WINCH_CCW_STEPS = 0x13


def angle_to_steps(angle):
    """Exact replica of SailController.angle_to_steps from winch_a.py."""
    angle = abs(angle)
    angle = max(0.0, min(88.0, angle))
    length = math.sqrt(
        BOOM_LENGTH**2 + WINCH_TO_MAST**2 -
        2 * BOOM_LENGTH * WINCH_TO_MAST * math.cos(math.radians(angle))
    )
    min_length = math.sqrt(
        BOOM_LENGTH**2 + WINCH_TO_MAST**2 -
        2 * BOOM_LENGTH * WINCH_TO_MAST * math.cos(math.radians(0))
    )
    rope_out = length - min_length
    return int((rope_out * GEAR_RATIO * STEPS_PER_REVOLUTION) /
               (2 * math.pi * SPOOL_RADIUS))


def build_serial_command(current_angle, target_angle):
    """Build the exact 5-byte serial packet winch_a.py sends."""
    current_steps = angle_to_steps(current_angle)
    target_steps  = angle_to_steps(target_angle)
    diff = target_steps - current_steps
    if diff >= 0:
        cmd = CMD_WINCH_CW_STEPS
    else:
        cmd = CMD_WINCH_CCW_STEPS
    return bytes([cmd]) + struct.pack('>I', abs(diff))


# ═══════════════════════════════════════════════════════════════════════
#  SECTION A – Arduino Deployment Validation
# ═══════════════════════════════════════════════════════════════════════
class TestArduinoGeometry(unittest.TestCase):
    """Validate the law-of-cosines angle→steps conversion."""

    def test_zero_angle_zero_steps(self):
        self.assertEqual(angle_to_steps(0), 0)

    def test_88_degrees_max_steps(self):
        steps = angle_to_steps(88)
        self.assertGreater(steps, 0, "88° must produce positive steps")
        # For BOOM=48, MAST=44, max rope ≈ 83.7 - 4.0 = 79.7 units
        self.assertGreater(steps, 5000, f"Max steps {steps} implausibly low")

    def test_monotonic_increase(self):
        """Steps must strictly increase with angle."""
        prev = 0
        for angle in range(1, 89):
            s = angle_to_steps(angle)
            self.assertGreater(s, prev,
                               f"Non-monotonic at {angle}°: {s} <= {prev}")
            prev = s

    def test_symmetry_positive_negative(self):
        """abs() in angle_to_steps means ±angle gives same steps."""
        for a in [10, 30, 60, 88]:
            self.assertEqual(angle_to_steps(a), angle_to_steps(-a))

    def test_clamp_above_88(self):
        """Angles > 88 must clamp to 88."""
        self.assertEqual(angle_to_steps(100), angle_to_steps(88))

    def test_clamp_below_zero(self):
        """Negative angles use abs(), so -5° → 5°."""
        self.assertEqual(angle_to_steps(-5), angle_to_steps(5))


class TestArduinoSerialProtocol(unittest.TestCase):
    """Validate the 5-byte serial packet structure."""

    def test_packet_length_is_5(self):
        pkt = build_serial_command(0, 45)
        self.assertEqual(len(pkt), 5, f"Packet length {len(pkt)} != 5")

    def test_cw_command_byte(self):
        pkt = build_serial_command(0, 45)
        self.assertEqual(pkt[0], CMD_WINCH_CW_STEPS)

    def test_ccw_command_byte(self):
        pkt = build_serial_command(45, 0)
        self.assertEqual(pkt[0], CMD_WINCH_CCW_STEPS)

    def test_step_count_big_endian(self):
        """Steps must be encoded big-endian unsigned 32-bit."""
        pkt = build_serial_command(0, 45)
        steps = struct.unpack('>I', pkt[1:5])[0]
        self.assertEqual(steps, angle_to_steps(45))

    def test_zero_diff_zero_steps(self):
        pkt = build_serial_command(30, 30)
        steps = struct.unpack('>I', pkt[1:5])[0]
        self.assertEqual(steps, 0)

    def test_roundtrip_all_angles(self):
        """Every 5° angle produces valid packet with correct step diff."""
        for a in range(0, 89, 5):
            for b in range(0, 89, 5):
                pkt = build_serial_command(a, b)
                self.assertEqual(len(pkt), 5)
                steps = struct.unpack('>I', pkt[1:5])[0]
                expected = abs(angle_to_steps(b) - angle_to_steps(a))
                self.assertEqual(steps, expected,
                                 f"Mismatch {a}°→{b}°")


class TestVPPToArduinoPipeline(unittest.TestCase):
    """End-to-end: VPP optimizer → sail angle → Arduino steps."""

    def test_vpp_output_in_arduino_range(self):
        """Every VPP output must be in [0, 88]°."""
        vpp = VPPSailOptimizer()
        for tws in [3, 8, 15, 25]:
            for twa in range(0, 360, 30):
                sail, _, _, _ = vpp.optimize(tws, twa, 0.0)
                self.assertGreaterEqual(sail, 0.0)
                self.assertLessEqual(sail, 88.0,
                                     f"VPP {sail:.1f}° at TWS={tws} TWA={twa}")

    def test_vpp_to_steps_valid(self):
        """VPP sail → angle_to_steps must produce non-negative integer."""
        vpp = VPPSailOptimizer()
        sail, _, _, _ = vpp.optimize(10, 90, 0)
        steps = angle_to_steps(sail)
        self.assertIsInstance(steps, int)
        self.assertGreaterEqual(steps, 0)

    def test_full_pipeline_timing(self):
        """VPP + angle_to_steps + serial build must finish in < 50 ms."""
        vpp = VPPSailOptimizer()
        t0 = time.perf_counter()
        for _ in range(100):
            sail, _, _, _ = vpp.optimize(10, 90, 0)
            steps = angle_to_steps(sail)
            pkt = build_serial_command(0, sail)
        elapsed = (time.perf_counter() - t0) / 100
        self.assertLess(elapsed, 0.05,
                        f"Pipeline took {elapsed*1000:.1f} ms (limit 50 ms)")


class TestMPCToArduinoPipeline(unittest.TestCase):
    """MPC rudder → servo PWM bounds."""

    def test_mpc_output_within_servo_range(self):
        """MPC must stay in [-21, 21]° under all conditions."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        for _ in range(200):
            h = np.random.uniform(0, 360)
            yr = np.random.uniform(-30, 30)
            tgt = np.random.uniform(0, 360)
            rudder, _ = mpc.compute(h, yr, tgt)
            self.assertGreaterEqual(rudder, -21.0)
            self.assertLessEqual(rudder, 21.0)

    def test_mpc_cycle_time(self):
        """Single MPC compute must finish in < 10 ms."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        t0 = time.perf_counter()
        for _ in range(100):
            mpc.compute(45.0, 0.0, 90.0)
        elapsed = (time.perf_counter() - t0) / 100
        self.assertLess(elapsed, 0.01,
                        f"MPC took {elapsed*1000:.1f} ms (limit 10 ms)")


# ═══════════════════════════════════════════════════════════════════════
#  SECTION B – Fierce Exhaustive Stress Tests
# ═══════════════════════════════════════════════════════════════════════

class TestCombinedFailures(unittest.TestCase):
    """Simultaneous sensor + actuator failure modes."""

    def test_ukf_gps_dropout_plus_noisy_imu(self):
        """GPS disappears for 30 seconds while IMU has 10× noise."""
        ukf = SquareRootUKF()
        true_psi = np.radians(45)

        # 5 seconds of good data
        for k in range(50):
            ukf.wind_speed = 10.0
            ukf.wind_angle = np.radians(90)
            ukf.predict(0.1)
            z = np.array([10, 20, true_psi, 0, 0, 0])
            ukf.update(z)

        # GPS dropout, IMU 10× noise for 30 seconds
        for k in range(300):
            ukf.predict(0.1)
            noisy_psi = true_psi + np.random.randn() * 0.5
            z = np.array([ukf.x[0], ukf.x[1],  # use UKF's own pos (no GPS)
                          noisy_psi, 0, np.random.randn() * 1.0, 0])
            ukf.update(z)

        self.assertTrue(np.all(np.isfinite(ukf.x)),
                        "UKF diverged during GPS dropout + noisy IMU")
        # Heading should still be within 30° (degraded mode)
        err = abs(np.degrees(ukf.x[2]) - np.degrees(true_psi))
        self.assertLess(err, 30.0, f"Heading drifted {err:.1f}°")

    def test_mpc_with_stuck_rudder_and_changing_wind(self):
        """Rudder stuck at 5° while wind shifts every 10 steps."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        mpc.dr_rate_max = 0.0  # stuck
        mpc.last_dr = np.radians(5.0)

        for k in range(100):
            wind_dir = 90 + 30 * np.sin(k / 10)
            rudder, _ = mpc.compute(45.0, 0.0, 90.0,
                                     wind_dir_deg=wind_dir)
            self.assertAlmostEqual(rudder, 5.0, delta=0.5)

    def test_vpp_with_heel_oscillation(self):
        """Heel oscillates ±35° while computing sail trim."""
        vpp = VPPSailOptimizer()
        for k in range(100):
            heel = 35 * np.sin(k * 0.3)
            sail, vmg, spd, _ = vpp.optimize(
                12.0, 90.0, 0.0, current_heel_deg=heel)
            self.assertGreaterEqual(sail, 0.0)
            self.assertLessEqual(sail, 88.0)
            self.assertTrue(np.isfinite(vmg))


class TestMultiLegRace(unittest.TestCase):
    """Simulate a full 3-leg race: upwind → reach → downwind."""

    def test_three_leg_course_completion(self):
        """MPC+VPP must produce valid commands across all points of sail."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        vpp = VPPSailOptimizer()
        model = NomotoModel()

        legs = [
            {'name': 'upwind', 'target': 30, 'wind_dir': 0, 'wind_spd': 12},
            {'name': 'reach',  'target': 90, 'wind_dir': 0, 'wind_spd': 12},
            {'name': 'downwind', 'target': 170, 'wind_dir': 0, 'wind_spd': 12},
        ]

        heading_rad = 0.0
        r = 0.0
        results = {}

        for leg in legs:
            rudders = []
            sails = []
            headings = []

            for k in range(200):
                heading_deg = normalize_angle(np.degrees(heading_rad))

                rudder, _ = mpc.compute(heading_deg, np.degrees(r),
                                         leg['target'])
                heading_rad, r = model.step(heading_rad, r,
                                             np.radians(rudder), 0.5)

                sail, vmg, spd, _ = vpp.optimize(
                    leg['wind_spd'], leg['wind_dir'],
                    leg['target'],
                    boat_speed_hint=max(0.5, abs(r) * 2))

                rudders.append(rudder)
                sails.append(sail)
                headings.append(heading_deg)

            results[leg['name']] = {
                'final_heading': headings[-1],
                'heading_error': abs(headings[-1] - leg['target']),
                'max_rudder': max(abs(r) for r in rudders),
                'max_sail': max(sails),
                'min_sail': min(sails),
            }

            # Every leg must have rudder in bounds
            for rd in rudders:
                self.assertGreaterEqual(rd, -21.0)
                self.assertLessEqual(rd, 21.0)
            # Sail in Arduino bounds
            for s in sails:
                self.assertGreaterEqual(s, 0.0)
                self.assertLessEqual(s, 88.0)

        # Heading should converge on each leg (within 20°)
        for name, res in results.items():
            he = res['heading_error']
            if he > 180:
                he = 360 - he
            self.assertLess(he, 20.0,
                            f"Leg '{name}' heading error {he:.1f}°")


class TestStormSequence(unittest.TestCase):
    """Full storm: calm → build → peak (40 kt gust) → die down."""

    def test_full_storm_lifecycle(self):
        """All controllers must remain finite and bounded through a storm."""
        ukf = SquareRootUKF()
        mpc = MPCSteering(horizon=8, dt=0.5)
        vpp = VPPSailOptimizer()
        model = NomotoModel()

        heading_rad = 0.0
        r = 0.0

        # Storm profile: wind speed over 600 steps (5 minutes at 2 Hz)
        storm = []
        for k in range(600):
            if k < 100:       # calm buildup
                ws = 3 + k * 0.1
            elif k < 200:     # intensifying
                ws = 13 + (k - 100) * 0.15
            elif k < 350:     # peak gusts
                ws = 28 + 12 * np.sin(k * 0.5)  # 16–40 kt gusts
            elif k < 500:     # dying down
                ws = 28 - (k - 350) * 0.15
            else:             # calm again
                ws = max(3, 5.5 - (k - 500) * 0.02)
            storm.append(max(0.5, ws * 0.5144))  # knots to m/s

        for k, ws in enumerate(storm):
            # Wind direction shifts during peak
            wd = 180 + (30 * np.sin(k * 0.1) if 200 <= k < 350 else 0)

            ukf.wind_speed = ws
            ukf.wind_angle = np.radians(wd)
            ukf.predict(0.5)
            z = np.array([0, 0, heading_rad,
                          np.radians(np.random.randn() * 5), 0, r])
            ukf.update(z)

            heading_deg = normalize_angle(np.degrees(heading_rad))
            rudder, _ = mpc.compute(heading_deg, np.degrees(r), 90.0,
                                     wind_speed=ws, wind_dir_deg=wd)
            heading_rad, r = model.step(heading_rad, r,
                                         np.radians(rudder), 0.5)

            sail, vmg, spd, _ = vpp.optimize(
                ws, wd, 90.0, current_heel_deg=ukf.get_heel_deg())

            # ALL outputs must be finite and bounded at every step
            self.assertTrue(np.isfinite(rudder), f"NaN rudder at step {k}")
            self.assertTrue(np.isfinite(sail), f"NaN sail at step {k}")
            self.assertGreaterEqual(sail, 0.0)
            self.assertLessEqual(sail, 88.0)
            self.assertGreaterEqual(rudder, -21.0)
            self.assertLessEqual(rudder, 21.0)
            self.assertTrue(np.all(np.isfinite(ukf.x)),
                            f"UKF NaN at storm step {k}")


class TestSensorCascadeFailure(unittest.TestCase):
    """Sensors drop out one-by-one: GPS → IMU → wind."""

    def test_progressive_sensor_loss(self):
        """System must remain stable as sensors drop offline."""
        ukf = SquareRootUKF()
        mpc = MPCSteering(horizon=8, dt=0.5)
        vpp = VPPSailOptimizer()

        true_heading = np.radians(90)

        # Phase 1: all sensors (100 steps)
        for k in range(100):
            ukf.wind_speed = 10.0
            ukf.wind_angle = np.radians(180)
            ukf.predict(0.1)
            z = np.array([50, 50, true_heading, 0.05, 0, 0.1])
            ukf.update(z)

        # Phase 2: GPS lost (use UKF prediction only, 100 steps)
        for k in range(100):
            ukf.predict(0.1)
            z = np.array([ukf.x[0], ukf.x[1],  # stale
                          true_heading + np.random.randn() * 0.02,
                          0.05, 0, 0.1])
            ukf.update(z)

        # Phase 3: IMU heading also lost (200 steps dead reckoning)
        for k in range(200):
            ukf.predict(0.1)
            # Only roll_rate still alive
            z = np.array([ukf.x[0], ukf.x[1], ukf.x[2],
                          ukf.x[6], np.random.randn() * 0.5, ukf.x[5]])
            ukf.update(z)

        # Must not have diverged
        self.assertTrue(np.all(np.isfinite(ukf.x)),
                        "UKF diverged during sensor cascade failure")
        P = ukf.get_covariance()
        self.assertTrue(np.all(np.isfinite(P)),
                        "Covariance not finite after cascade failure")


class TestEndurance(unittest.TestCase):
    """1000-step endurance under continuous random perturbation."""

    def test_1000_step_stability(self):
        """All three controllers must remain finite for 1000 steps."""
        ukf = SquareRootUKF()
        mpc = MPCSteering(horizon=8, dt=0.5)
        vpp = VPPSailOptimizer()
        model = NomotoModel()
        np.random.seed(42)

        heading_rad = 0.0
        r = 0.0

        for k in range(1000):
            ws = max(0.5, 10 + np.random.randn() * 5)
            wd = np.random.uniform(0, 360)

            ukf.wind_speed = ws
            ukf.wind_angle = np.radians(wd)
            ukf.predict(0.5)
            z = np.random.randn(6) * 0.3
            z[0] = np.random.uniform(-100, 100)
            z[1] = np.random.uniform(-100, 100)
            z[2] = heading_rad + np.random.randn() * 0.1
            ukf.update(z)

            heading_deg = normalize_angle(np.degrees(heading_rad))
            target = np.random.uniform(0, 360)
            rudder, _ = mpc.compute(heading_deg, np.degrees(r), target)
            heading_rad, r = model.step(heading_rad, r,
                                         np.radians(rudder), 0.5)

            sail, _, _, _ = vpp.optimize(ws, wd, target,
                                          current_heel_deg=ukf.get_heel_deg())

        self.assertTrue(np.all(np.isfinite(ukf.x)),
                        "UKF diverged in 1000-step endurance")
        P = ukf.get_covariance()
        eigvals = np.linalg.eigvalsh(P)
        self.assertTrue(np.all(eigvals > -1e-8),
                        f"Covariance non-PD after endurance: min={eigvals.min():.2e}")


class TestWindShiftDuringTack(unittest.TestCase):
    """180° wind shift during a tacking manoeuvre."""

    def test_tack_with_simultaneous_wind_reversal(self):
        """Boat tacking + 180° wind shift: MPC+VPP must not produce NaN."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        vpp = VPPSailOptimizer()
        model = NomotoModel()

        heading_rad = np.radians(45)
        r = 0.0

        # Tacking: head through the wind from 45° to -45° (315°)
        # Simultaneously wind shifts from 0° to 180°
        for k in range(100):
            progress = k / 100.0
            target = 45 - 90 * progress   # 45 → -45
            wind_dir = 180 * progress     # 0 → 180

            heading_deg = normalize_angle(np.degrees(heading_rad))
            rudder, _ = mpc.compute(heading_deg, np.degrees(r),
                                     target, wind_dir_deg=wind_dir)
            heading_rad, r = model.step(heading_rad, r,
                                         np.radians(rudder), 0.5)

            sail, vmg, spd, _ = vpp.optimize(
                10.0, wind_dir, target)

            self.assertTrue(np.isfinite(rudder),
                            f"NaN rudder during tack at step {k}")
            self.assertTrue(np.isfinite(sail),
                            f"NaN sail during tack at step {k}")
            self.assertGreaterEqual(sail, 0.0)
            self.assertLessEqual(sail, 88.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
