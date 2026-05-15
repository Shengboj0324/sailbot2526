#!/usr/bin/env python3
"""Regression tests for the mock boat demonstration pipeline."""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "sailboat_control"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "path_planning"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sailboat_control.common import ControlMode
from sailboat_control.events import F
from sailboat_control.mock_boat_simulation import (
    MockBoatConstraints,
    MockBoatSimulator,
    MockWaterEnvironment,
    ascii_map,
    build_summary,
    rate_limit,
    signed_angle,
)
from sailboat_control.mpc_controller import MPCSteering
from sailboat_control.vpp_sail_optimizer import VPPSailOptimizer


class TestMockBoatSimulationPipeline(unittest.TestCase):
    def test_demo_runs_and_moves_toward_course(self):
        sim = MockBoatSimulator()
        initial_distance = np.hypot(sim.waypoints[0][0], sim.waypoints[0][1])
        history = sim.run(duration_s=75.0, dt=0.5)
        summary = build_summary(history)

        self.assertGreater(len(history), 20)
        self.assertLess(history[-1].distance_to_target_m, initial_distance)
        self.assertGreater(summary["max_speed_mps"], 0.4)
        self.assertGreaterEqual(summary["completed_waypoints"], 0)

    def test_actuator_and_physics_constraints_hold_for_full_run(self):
        constraints = MockBoatConstraints()
        sim = MockBoatSimulator(constraints=constraints)
        history = sim.run(duration_s=100.0, dt=0.5)

        for previous, current in zip(history, history[1:]):
            self.assertLessEqual(abs(current.rudder_deg), constraints.max_rudder_deg + 1e-9)
            self.assertLessEqual(abs(current.sail_deg), constraints.max_sail_deg + 1e-9)
            self.assertLessEqual(
                abs(current.rudder_deg - previous.rudder_deg),
                constraints.max_rudder_rate_dps * 0.5 + 1e-6,
            )
            self.assertLessEqual(
                abs(current.sail_deg - previous.sail_deg),
                constraints.max_sail_rate_dps * 0.5 + 1e-6,
            )
            self.assertLessEqual(current.speed_mps, constraints.max_speed_mps + 1e-9)
            self.assertTrue(np.isfinite(current.mpc_cost))

    def test_current_is_learned_by_drift_estimator(self):
        env = MockWaterEnvironment(current_east_mps=0.35, current_north_mps=0.0)
        sim = MockBoatSimulator(environment=env)
        history = sim.run(duration_s=45.0, dt=0.5)

        self.assertGreater(history[-1].drift_confidence, 0.5)
        self.assertAlmostEqual(history[-1].drift_speed_mps, 0.35, delta=0.08)

    def test_ascii_map_marks_start_boat_and_waypoints(self):
        sim = MockBoatSimulator()
        history = sim.run(duration_s=5.0, dt=0.5)
        rendered = ascii_map(history, sim.waypoints)

        self.assertIn("S", rendered)
        self.assertIn("B", rendered)
        self.assertIn("1", rendered)


class TestControllerRegressionCoverage(unittest.TestCase):
    def test_mpc_constraints_under_heading_sweep(self):
        mpc = MPCSteering(horizon=8, dt=0.5)
        previous = 0.0
        for target in range(0, 360, 15):
            rudder, diag = mpc.compute(
                heading_deg=25.0,
                yaw_rate_dps=0.0,
                target_heading_deg=float(target),
                boat_speed=1.8,
                heel_deg=8.0,
            )
            self.assertLessEqual(abs(rudder), 21.0)
            self.assertLessEqual(abs(rudder - previous), 7.5 + 0.6)
            self.assertIn("solver", diag)
            previous = rudder

    def test_vpp_depowers_under_waves_and_heel(self):
        calm = VPPSailOptimizer()
        waves = VPPSailOptimizer()
        waves.set_wave_height(0.35)

        _, _, calm_speed, calm_diag = calm.optimize(12.0, 150.0, 150.0, boat_speed_hint=1.8)
        sail, _, wave_speed, wave_diag = waves.optimize(
            12.0,
            150.0,
            150.0,
            boat_speed_hint=1.8,
            current_heel_deg=28.0,
        )

        self.assertLessEqual(sail, 88.0)
        self.assertLess(wave_speed, calm_speed)
        self.assertEqual(wave_diag["wave_height"], 0.35)
        self.assertEqual(calm_diag["wave_height"], 0.0)

    def test_signed_angle_wraparound_regression(self):
        self.assertEqual(signed_angle(370), 10)
        self.assertEqual(signed_angle(-190), 170)
        self.assertEqual(signed_angle(180), -180)

    def test_rate_limiter_respects_bounds_and_step(self):
        self.assertEqual(rate_limit(100.0, 0.0, 10.0, 0.5, -21.0, 21.0), 5.0)
        self.assertEqual(rate_limit(-100.0, -20.0, 10.0, 0.5, -21.0, 21.0), -21.0)


class TestEventRegressionCoverage(unittest.TestCase):
    def test_fleet_race_event_initializes_rc_mode(self):
        class State:
            autonomous_enabled = True
            control_mode = None

        class Boat:
            state = State()

        boat = Boat()
        F().initialize_event(boat)

        self.assertFalse(boat.state.autonomous_enabled)
        self.assertEqual(boat.state.control_mode, ControlMode.RC)


if __name__ == "__main__":
    unittest.main()
