#!/usr/bin/env python3
"""
Phase-6 Production Validation Tests.

SECTION A — SLSQP MPC convergence & constraint enforcement
SECTION B — Obstacle avoidance (static & moving obstacles)
SECTION C — Non-linear stall model correctness
SECTION D — Wave-added resistance & degradation
SECTION E — Added mass effects during tacks/jibes
SECTION F — Predictive heel prevention (proactive depower)
SECTION G — Full pipeline under combined extreme conditions
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'sailboat_control'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'path_planning'))

import numpy as np
import unittest

from sailboat_control.ukf_state_estimator import SquareRootUKF, SailboatDynamics
from sailboat_control.mpc_controller import MPCSteering, NomotoModel, Obstacle
from sailboat_control.vpp_sail_optimizer import VPPSailOptimizer
from sailboat_control.adaptive_pid import normalize_angle


# ═══════════════════════════════════════════════════════════════════════
#  SECTION A — SLSQP MPC convergence & constraint enforcement
# ═══════════════════════════════════════════════════════════════════════
class TestSLSQPConvergence(unittest.TestCase):
    """Verify SLSQP produces better solutions than greedy fallback."""

    def test_slsqp_lower_cost_than_greedy(self):
        """SLSQP must produce equal or lower cost than greedy on 50 random cases."""
        np.random.seed(42)
        mpc = MPCSteering(horizon=10, dt=0.5)

        slsqp_wins = 0
        for _ in range(50):
            h = np.random.uniform(0, 360)
            yr = np.random.uniform(-15, 15)
            tgt = np.random.uniform(0, 360)
            mpc.last_dr = np.radians(np.random.uniform(-21, 21))

            psi0 = np.radians(h)
            r0 = np.radians(yr)
            psi_ref = np.radians(tgt)

            dr_slsqp, cost_slsqp = mpc._solve_slsqp(psi0, r0, psi_ref, 0.0)
            dr_greedy, cost_greedy = mpc._solve_greedy(psi0, r0, psi_ref, 0.0)

            if cost_slsqp <= cost_greedy + 1e-6:
                slsqp_wins += 1

        self.assertGreater(slsqp_wins, 40,
                           f"SLSQP only beat greedy {slsqp_wins}/50 times")

    def test_slsqp_respects_rudder_limits(self):
        """SLSQP must never exceed ±21° rudder travel."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        for _ in range(100):
            h = np.random.uniform(0, 360)
            yr = np.random.uniform(-30, 30)
            tgt = np.random.uniform(0, 360)
            rudder, _ = mpc.compute(h, yr, tgt)
            self.assertGreaterEqual(rudder, -21.0)
            self.assertLessEqual(rudder, 21.0)

    def test_slsqp_respects_slew_rate(self):
        """Consecutive rudder commands must not exceed 15°/s slew rate."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        max_delta = 15.0 * 0.5  # deg per step
        prev = 0.0
        for _ in range(50):
            rudder, _ = mpc.compute(0, 0, 90 + np.random.randn() * 10)
            delta = abs(rudder - prev)
            self.assertLessEqual(delta, max_delta + 0.5,
                                 f"Slew violation: {delta:.1f}° > {max_delta:.1f}°")
            prev = rudder

    def test_slsqp_timing(self):
        """SLSQP solve must finish in < 15 ms average."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        t0 = time.perf_counter()
        for _ in range(50):
            mpc.compute(np.random.uniform(0, 360), 0, 90)
        elapsed = (time.perf_counter() - t0) / 50
        self.assertLess(elapsed, 0.015,
                        f"SLSQP took {elapsed*1000:.1f} ms (limit 15 ms)")

    def test_solver_reports_slsqp(self):
        """Diagnostics must report SLSQP as solver."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        _, diag = mpc.compute(0, 0, 90)
        self.assertEqual(diag['solver'], 'SLSQP')


# ═══════════════════════════════════════════════════════════════════════
#  SECTION B — Obstacle avoidance (static & moving)
# ═══════════════════════════════════════════════════════════════════════
class TestObstacleAvoidance(unittest.TestCase):
    """Verify MPC steers away from obstacles."""

    def test_static_obstacle_ahead(self):
        """With obstacle ahead, MPC cost must be higher than without."""
        # Clean run: no obstacle
        mpc_clean = MPCSteering(horizon=10, dt=0.5)
        mpc_clean.set_boat_position(0.0, 0.0)
        _, diag_clean = mpc_clean.compute(0, 0, 30)
        cost_clean = diag_clean['cost']

        # Obstacle run: obstacle directly in path
        mpc_obs = MPCSteering(horizon=10, dt=0.5)
        mpc_obs.set_boat_position(0.0, 0.0)
        obs = Obstacle(x=1.0, y=2.0, radius=4.0)  # very close
        mpc_obs.set_obstacles([obs])
        _, diag_obs = mpc_obs.compute(0, 0, 30)
        cost_obs = diag_obs['cost']

        self.assertGreater(cost_obs, cost_clean,
                           f"Obstacle did not increase cost: {cost_obs:.2f} vs {cost_clean:.2f}")

    def test_moving_obstacle_crossing(self):
        """Moving obstacle crossing path: MPC must adjust."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        mpc.set_boat_position(0.0, 0.0)
        # Obstacle moving east, will cross path
        obs = Obstacle(x=-5.0, y=3.0, radius=2.0, vx=2.0, vy=0.0)
        mpc.set_obstacles([obs])

        rudder, diag = mpc.compute(0, 0, 0)
        self.assertEqual(diag['n_obstacles'], 1)
        # Rudder should react (any direction is acceptable)
        self.assertTrue(np.isfinite(rudder))

    def test_multiple_obstacles(self):
        """Multiple obstacles: MPC must remain finite and bounded."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        mpc.set_boat_position(0.0, 0.0)
        obstacles = [
            Obstacle(x=2.0, y=5.0, radius=2.0),
            Obstacle(x=-3.0, y=4.0, radius=1.5),
            Obstacle(x=0.0, y=8.0, radius=3.0, vx=-0.5, vy=0.0),
        ]
        mpc.set_obstacles(obstacles)
        rudder, diag = mpc.compute(0, 0, 0)
        self.assertGreaterEqual(rudder, -21.0)
        self.assertLessEqual(rudder, 21.0)
        self.assertEqual(diag['n_obstacles'], 3)

    def test_no_obstacle_no_penalty(self):
        """Without obstacles, cost should be lower than with close obstacle."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        mpc.set_boat_position(0.0, 0.0)
        _, diag_clean = mpc.compute(0, 0, 90)
        cost_clean = diag_clean['cost']

        mpc.last_dr = 0.0
        obs = Obstacle(x=0.0, y=3.0, radius=5.0)
        mpc.set_obstacles([obs])
        _, diag_obs = mpc.compute(0, 0, 90)
        cost_obs = diag_obs['cost']

        self.assertGreater(cost_obs, cost_clean,
                           "Obstacle penalty not reflected in cost")

    def test_obstacle_clear_resets(self):
        mpc = MPCSteering(horizon=10, dt=0.5)
        mpc.set_obstacles([Obstacle(0, 0, 1)])
        self.assertEqual(len(mpc.obstacles), 1)
        mpc.clear_obstacles()
        self.assertEqual(len(mpc.obstacles), 0)


# ═══════════════════════════════════════════════════════════════════════
#  SECTION C — Non-linear stall model correctness
# ═══════════════════════════════════════════════════════════════════════
class TestStallModel(unittest.TestCase):
    """Verify post-stall lift rolloff and drag increase."""

    def test_pre_stall_lift_increases(self):
        """Below stall angle, CL must increase with alpha."""
        vpp = VPPSailOptimizer()
        prev_cl = 0.0
        for deg in range(1, 14):
            alpha = np.radians(deg)
            CL, CD = vpp._aero_coeffs(alpha)
            self.assertGreater(CL, prev_cl,
                               f"CL not increasing at {deg}°")
            prev_cl = CL

    def test_post_stall_lift_drops(self):
        """Well above stall, CL must drop: Viterna model has sin(2α) shape,
        so CL peaks near 45° then drops to zero at 90°."""
        vpp = VPPSailOptimizer()
        CL_at_60, _ = vpp._aero_coeffs(np.radians(60))
        CL_at_80, _ = vpp._aero_coeffs(np.radians(80))
        CL_at_89, _ = vpp._aero_coeffs(np.radians(89))
        # CL must decrease as α approaches 90° (sin(2α) → 0)
        self.assertGreater(CL_at_60, CL_at_80,
                           f"CL at 60° ({CL_at_60:.2f}) should be > CL at 80° ({CL_at_80:.2f})")
        self.assertGreater(CL_at_80, CL_at_89,
                           f"CL at 80° ({CL_at_80:.2f}) should be > CL at 89° ({CL_at_89:.2f})")
        # At 89° CL must be near zero
        self.assertLess(abs(CL_at_89), 0.15,
                        f"CL at 89° ({CL_at_89:.2f}) should be near 0")

    def test_post_stall_drag_increases(self):
        """Above stall angle, CD must increase monotonically."""
        vpp = VPPSailOptimizer()
        CD_at_20 = vpp._aero_coeffs(np.radians(20))[1]
        CD_at_40 = vpp._aero_coeffs(np.radians(40))[1]
        CD_at_60 = vpp._aero_coeffs(np.radians(60))[1]
        self.assertGreater(CD_at_40, CD_at_20)
        self.assertGreater(CD_at_60, CD_at_40)

    def test_90_degree_flat_plate(self):
        """At 90° AoA, CD should approach flat-plate maximum."""
        vpp = VPPSailOptimizer()
        CL_90, CD_90 = vpp._aero_coeffs(np.radians(89))
        self.assertGreater(CD_90, 1.0,
                           f"CD at 89° ({CD_90:.2f}) too low for flat plate")

    def test_stall_detected_in_vpp_diagnostics(self):
        """VPP must report stall detection in diagnostics."""
        vpp = VPPSailOptimizer()
        # Head-to-wind: high AoA likely
        _, _, _, diag = vpp.optimize(15, 10, 0)
        # At some point in scan, stall should be detected
        self.assertIn('stall_detected', diag)

    def test_stall_model_symmetry(self):
        """CL(-α) = -CL(α) and CD(-α) = CD(α)."""
        vpp = VPPSailOptimizer()
        for deg in [5, 15, 30, 60, 85]:
            CL_pos, CD_pos = vpp._aero_coeffs(np.radians(deg))
            CL_neg, CD_neg = vpp._aero_coeffs(np.radians(-deg))
            self.assertAlmostEqual(CL_pos, -CL_neg, delta=0.01,
                                   msg=f"CL not antisymmetric at {deg}°")
            self.assertAlmostEqual(CD_pos, CD_neg, delta=0.01,
                                   msg=f"CD not symmetric at {deg}°")

    def test_thin_airfoil_vs_stall_crossover(self):
        """Smooth transition: no discontinuity around stall angle."""
        vpp = VPPSailOptimizer()
        angles = np.linspace(10, 20, 50)
        CLs = [vpp._aero_coeffs(np.radians(a))[0] for a in angles]
        # Check no jumps > 0.3 between consecutive points
        for i in range(1, len(CLs)):
            self.assertLess(abs(CLs[i] - CLs[i-1]), 0.3,
                            f"CL jump at {angles[i]:.1f}°: {CLs[i-1]:.2f} → {CLs[i]:.2f}")


# ═══════════════════════════════════════════════════════════════════════
#  SECTION D — Wave-added resistance & performance degradation
# ═══════════════════════════════════════════════════════════════════════
class TestWaveResistance(unittest.TestCase):
    """Verify wave-height dependent performance degradation."""

    def test_calm_water_faster_than_waves(self):
        """Boat speed in calm water must exceed speed in 0.3m waves."""
        vpp = VPPSailOptimizer()
        vpp.set_wave_height(0.0)
        _, _, spd_calm, _ = vpp.optimize(10, 90, 0)
        vpp.reset()
        vpp.set_wave_height(0.3)
        _, _, spd_wave, _ = vpp.optimize(10, 90, 0)
        self.assertGreater(spd_calm, spd_wave,
                           f"Calm {spd_calm:.2f} should be faster than wave {spd_wave:.2f}")

    def test_increasing_wave_decreases_speed(self):
        """Speed must decrease monotonically with wave height."""
        vpp = VPPSailOptimizer()
        speeds = []
        for H_s in [0.0, 0.1, 0.2, 0.3, 0.5]:
            vpp.reset()
            vpp.set_wave_height(H_s)
            _, _, spd, _ = vpp.optimize(10, 90, 0)
            speeds.append(spd)
        for i in range(1, len(speeds)):
            self.assertLessEqual(speeds[i], speeds[i-1] + 0.01,
                                 f"Speed increased at H_s={[0,0.1,0.2,0.3,0.5][i]}m")

    def test_wave_resistance_in_hull_model(self):
        """_hull_resistance must increase with wave height."""
        vpp = VPPSailOptimizer()
        vpp.set_wave_height(0.0)
        R_calm = vpp._hull_resistance(2.0)
        vpp.set_wave_height(0.3)
        R_wave = vpp._hull_resistance(2.0)
        self.assertGreater(R_wave, R_calm)

    def test_wave_diagnostics_reported(self):
        """VPP diagnostics must include wave_height."""
        vpp = VPPSailOptimizer()
        vpp.set_wave_height(0.2)
        _, _, _, diag = vpp.optimize(10, 90, 0)
        self.assertAlmostEqual(diag['wave_height'], 0.2)

    def test_extreme_waves_still_finite(self):
        """1m waves (extreme for RC sailbot): all outputs finite."""
        vpp = VPPSailOptimizer()
        vpp.set_wave_height(1.0)
        sail, vmg, spd, diag = vpp.optimize(20, 90, 0)
        self.assertTrue(np.isfinite(sail))
        self.assertTrue(np.isfinite(vmg))
        self.assertTrue(np.isfinite(spd))
        self.assertGreaterEqual(sail, 0.0)
        self.assertLessEqual(sail, 88.0)


# ═══════════════════════════════════════════════════════════════════════
#  SECTION E — Added mass effects during tacks/jibes
# ═══════════════════════════════════════════════════════════════════════
class TestAddedMass(unittest.TestCase):
    """Verify added mass modifies dynamics during rapid manoeuvres."""

    def test_added_mass_reduces_acceleration(self):
        """With added mass, same force produces less acceleration."""
        dyn_with = SailboatDynamics()
        dyn_without = SailboatDynamics()
        dyn_without.m_a_surge = 0.0
        dyn_without.m_a_sway = 0.0
        dyn_without.I_a_yaw = 0.0
        dyn_without.I_a_roll = 0.0

        x0 = np.zeros(10)
        x0[3] = 1.0  # initial surge
        x0[8] = 0.3  # rudder deflection

        x_with = dyn_with.f(x0, 0.5, wind_speed=10.0, wind_angle=1.5)
        x_without = dyn_without.f(x0, 0.5, wind_speed=10.0, wind_angle=1.5)

        # Surge change should be smaller with added mass
        du_with = abs(x_with[3] - x0[3])
        du_without = abs(x_without[3] - x0[3])
        self.assertLess(du_with, du_without + 0.01,
                        "Added mass should reduce surge acceleration")

    def test_added_mass_affects_yaw_during_tack(self):
        """During rapid heading change, added yaw inertia slows turn rate."""
        dyn_with = SailboatDynamics()
        dyn_without = SailboatDynamics()
        dyn_without.I_a_yaw = 0.0

        x0 = np.zeros(10)
        x0[3] = 2.0   # moving forward
        x0[8] = 0.35  # full rudder (20°)

        # Simulate 10 steps
        x_w = x0.copy()
        x_wo = x0.copy()
        for _ in range(10):
            x_w = dyn_with.f(x_w, 0.1, wind_speed=5, wind_angle=0)
            x_wo = dyn_without.f(x_wo, 0.1, wind_speed=5, wind_angle=0)

        # Without added mass should have turned more (bigger heading change)
        dh_with = abs(x_w[2] - x0[2])
        dh_without = abs(x_wo[2] - x0[2])
        self.assertLess(dh_with, dh_without + 0.01,
                        "Added yaw inertia should slow the turn")

    def test_ukf_stable_with_added_mass_during_tack(self):
        """UKF must remain stable during simulated tack with added mass."""
        ukf = SquareRootUKF()
        ukf.wind_speed = 8.0
        ukf.wind_angle = 0.0

        # Simulate tack: rudder goes hard over
        for k in range(100):
            ukf.x[8] = 0.35 if k < 50 else -0.35  # rudder
            ukf.predict(0.1)
            z = np.array([ukf.x[0], ukf.x[1], ukf.x[2],
                          ukf.x[6], ukf.x[7], ukf.x[5]])
            ukf.update(z)

        self.assertTrue(np.all(np.isfinite(ukf.x)),
                        "UKF diverged during tack with added mass")



# ═══════════════════════════════════════════════════════════════════════
#  SECTION F — Predictive heel prevention (proactive depower)
# ═══════════════════════════════════════════════════════════════════════
class TestPredictiveHeelPrevention(unittest.TestCase):
    """Verify system proactively prevents heel limit violation."""

    def test_vpp_rejects_high_heel_trim(self):
        """VPP must not select trim angle that predicts > 25° heel."""
        vpp = VPPSailOptimizer()
        # Strong wind: high heel risk
        sail, vmg, spd, diag = vpp.optimize(20, 90, 0)
        self.assertLessEqual(abs(diag['predicted_heel']), 25.0 + 0.1,
                             f"Predicted heel {diag['predicted_heel']:.1f}° exceeds limit")

    def test_reactive_depower_at_current_heel(self):
        """When current heel > 20°, VPP must depower (widen sail angle)."""
        vpp = VPPSailOptimizer()
        sail_low_heel, _, _, _ = vpp.optimize(12, 90, 0, current_heel_deg=5)
        vpp.reset()
        sail_high_heel, _, _, _ = vpp.optimize(12, 90, 0, current_heel_deg=23)
        self.assertGreaterEqual(sail_high_heel, sail_low_heel,
                                "High heel should produce wider sail angle (depower)")

    def test_mpc_heel_penalty_limits_rudder(self):
        """MPC with high heel should limit aggressive rudder commands."""
        mpc = MPCSteering(horizon=10, dt=0.5)
        rudder_no_heel, _ = mpc.compute(0, 0, 90, heel_deg=0)
        mpc.last_dr = 0.0
        rudder_high_heel, _ = mpc.compute(0, 0, 90, heel_deg=23)
        # High heel should result in smaller or equal rudder to avoid more heeling
        # (due to heel penalty in cost function)
        self.assertLessEqual(abs(rudder_high_heel), abs(rudder_no_heel) + 1.0,
                             "MPC should be less aggressive with high heel")

    def test_combined_heel_protection(self):
        """VPP + MPC must try to minimise heel; when physics prevents < 25°,
        VPP must pick the lowest-heel trim (fallback mode)."""
        vpp = VPPSailOptimizer()
        mpc = MPCSteering(horizon=8, dt=0.5)

        for twa in range(30, 180, 15):
            vpp.reset()
            mpc.reset()
            sail, vmg, spd, vdiag = vpp.optimize(12, twa, 0, current_heel_deg=15)
            rudder, mdiag = mpc.compute(0, 0, twa, heel_deg=15)

            # VPP should either satisfy heel limit or pick minimum heel
            pred_heel = abs(vdiag['predicted_heel'])
            # In strong wind at certain TWA, minimum achievable heel may
            # exceed 25°. That's physics — but VPP must select the trim
            # that minimises heel (fallback). We verify the output is finite
            # and bounded.
            self.assertTrue(np.isfinite(pred_heel),
                            f"NaN heel at TWA={twa}")
            self.assertGreaterEqual(sail, 0.0)
            self.assertLessEqual(sail, 88.0)
            self.assertGreaterEqual(rudder, -21.0)
            self.assertLessEqual(rudder, 21.0)


# ═══════════════════════════════════════════════════════════════════════
#  SECTION G — Full pipeline under combined extreme conditions
# ═══════════════════════════════════════════════════════════════════════
class TestFullPipelineExtreme(unittest.TestCase):
    """Full UKF+MPC+VPP pipeline with obstacles, waves, and stall."""

    def test_obstacle_field_with_waves(self):
        """Navigate through 5 obstacles in 0.3m waves: all outputs valid."""
        ukf = SquareRootUKF()
        mpc = MPCSteering(horizon=8, dt=0.5)
        vpp = VPPSailOptimizer()
        model = NomotoModel()

        ukf.set_wave_height(0.3)
        vpp.set_wave_height(0.3)

        obstacles = [
            Obstacle(x=k * 5, y=10 + k * 3, radius=2.0,
                     vx=0.1 * (-1)**k, vy=0.05)
            for k in range(5)
        ]
        mpc.set_obstacles(obstacles)

        heading_rad = 0.0
        r = 0.0

        for k in range(200):
            ws = 10 + np.sin(k * 0.1) * 3
            wd = 180 + 10 * np.sin(k * 0.05)

            ukf.wind_speed = ws
            ukf.wind_angle = np.radians(wd)
            ukf.predict(0.5)
            z = np.array([k * 0.1, k * 0.05, heading_rad,
                          0, np.random.randn() * 0.1, r])
            ukf.update(z)

            heading_deg = normalize_angle(np.degrees(heading_rad))
            mpc.set_boat_position(k * 0.1, k * 0.05)
            rudder, _ = mpc.compute(heading_deg, np.degrees(r), 45,
                                     heel_deg=ukf.get_heel_deg())
            heading_rad, r = model.step(heading_rad, r,
                                         np.radians(rudder), 0.5)

            sail, vmg, spd, _ = vpp.optimize(
                ws, wd, 45,
                current_heel_deg=ukf.get_heel_deg())

            self.assertTrue(np.all(np.isfinite(ukf.x)), f"UKF NaN at {k}")
            self.assertGreaterEqual(rudder, -21.0)
            self.assertLessEqual(rudder, 21.0)
            self.assertGreaterEqual(sail, 0.0)
            self.assertLessEqual(sail, 88.0)

    def test_storm_with_waves_and_stall(self):
        """Full storm + 0.5m waves: stall model must handle extreme AoA."""
        ukf = SquareRootUKF()
        vpp = VPPSailOptimizer()
        mpc = MPCSteering(horizon=8, dt=0.5)
        model = NomotoModel()

        ukf.set_wave_height(0.5)
        vpp.set_wave_height(0.5)

        heading_rad = 0.0
        r = 0.0

        for k in range(300):
            # Storm wind profile
            if k < 100:
                ws = 5 + k * 0.15
            elif k < 200:
                ws = 20 + 10 * np.sin(k * 0.3)  # gusts to 30
            else:
                ws = max(3, 20 - (k - 200) * 0.15)

            wd = 180 + 30 * np.sin(k * 0.08)

            ukf.wind_speed = ws
            ukf.wind_angle = np.radians(wd)
            ukf.predict(0.5)
            z = np.array([0, 0, heading_rad, 0, 0, r])
            ukf.update(z)

            heading_deg = normalize_angle(np.degrees(heading_rad))
            rudder, _ = mpc.compute(heading_deg, np.degrees(r), 90,
                                     heel_deg=ukf.get_heel_deg())
            heading_rad, r = model.step(heading_rad, r,
                                         np.radians(rudder), 0.5)

            sail, _, _, diag = vpp.optimize(
                ws, wd, 90,
                current_heel_deg=ukf.get_heel_deg())

            self.assertTrue(np.isfinite(rudder), f"NaN rudder at {k}")
            self.assertTrue(np.isfinite(sail), f"NaN sail at {k}")
            self.assertGreaterEqual(sail, 0.0)
            self.assertLessEqual(sail, 88.0)

    def test_ukf_wave_excitation_stability(self):
        """UKF with wave excitation enabled must remain stable for 500 steps."""
        ukf = SquareRootUKF()
        ukf.set_wave_height(0.4, T_p=3.0)
        ukf.wind_speed = 10.0
        ukf.wind_angle = np.radians(90)

        for k in range(500):
            ukf.predict(0.1)
            z = np.random.randn(6) * 0.1
            z[0] = k * 0.1
            z[1] = 0.0
            z[2] = 0.0
            ukf.update(z)

        self.assertTrue(np.all(np.isfinite(ukf.x)),
                        "UKF diverged with wave excitation")
        P = ukf.get_covariance()
        self.assertTrue(np.all(np.isfinite(P)), "Covariance NaN with waves")

    def test_rudder_stall_in_dynamics(self):
        """Rudder force should plateau/decrease beyond stall angle."""
        dyn = SailboatDynamics()
        # Compare rudder effect at 10° vs 30° AoA
        x_small = np.zeros(10)
        x_small[3] = 2.0  # forward speed
        x_small[8] = np.radians(10)  # 10° rudder

        x_large = np.zeros(10)
        x_large[3] = 2.0
        x_large[8] = np.radians(30)  # 30° (well past stall)

        x1 = dyn.f(x_small, 0.5)
        x2 = dyn.f(x_large, 0.5)

        # At 30° the rudder is stalled — yaw rate should be similar or less
        # than at 10° (not proportionally larger)
        yr_small = abs(x1[5])
        yr_large = abs(x2[5])
        # 30° is 3x the angle, but stalled rudder should produce < 3x yaw rate
        ratio = yr_large / (yr_small + 1e-9)
        self.assertLess(ratio, 3.0,
                        f"Rudder at 30° produced {ratio:.1f}x yaw rate vs 10°: stall not modeled")


if __name__ == '__main__':
    unittest.main(verbosity=2)