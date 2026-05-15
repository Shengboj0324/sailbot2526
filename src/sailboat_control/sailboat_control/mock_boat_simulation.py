#!/usr/bin/env python3
"""Terminal-friendly mock sailboat simulation for controller demonstrations.

The simulator is deliberately pure Python and ROS-free. It wraps the production
algorithm classes used by NavigationNode, then applies a compact physics model
with actuator limits, current, wind, heel, no-go-zone loss, and wave resistance.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence, Tuple

import numpy as np

if __package__ in (None, ""):
    package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if package_root not in sys.path:
        sys.path.insert(0, package_root)

try:
    from sailboat_control.drift_estimator import DriftEstimator
    from sailboat_control.mpc_controller import MPCSteering
    from sailboat_control.vpp_sail_optimizer import VPPSailOptimizer
except ImportError:  # Allows direct execution from a source checkout.
    from .drift_estimator import DriftEstimator
    from .mpc_controller import MPCSteering
    from .vpp_sail_optimizer import VPPSailOptimizer


Point = Tuple[float, float]


@dataclass
class MockWaterEnvironment:
    """External conditions for the virtual test pond.

    Angles use navigation convention: 0 degrees is north, 90 is east.
    Wind direction is where the wind is coming from.
    """

    wind_direction_deg: float = 120.0
    wind_speed_mps: float = 8.5
    current_east_mps: float = 0.18
    current_north_mps: float = -0.08
    wave_height_m: float = 0.08
    gust_period_s: float = 18.0
    gust_strength: float = 0.16

    def wind_at(self, t: float) -> Tuple[float, float, bool]:
        gust = math.sin(2.0 * math.pi * t / self.gust_period_s)
        speed = self.wind_speed_mps * (1.0 + self.gust_strength * max(0.0, gust))
        direction = (self.wind_direction_deg + 8.0 * math.sin(t / 11.0)) % 360.0
        return direction, speed, gust > 0.65


@dataclass
class MockBoatConstraints:
    max_rudder_deg: float = 21.0
    max_rudder_rate_dps: float = 15.0
    max_sail_deg: float = 88.0
    max_sail_rate_dps: float = 10.0
    max_speed_mps: float = 3.6
    waypoint_radius_m: float = 4.0
    min_upwind_angle_deg: float = 35.0


@dataclass
class MockBoatState:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_deg: float = 20.0
    yaw_rate_dps: float = 0.0
    speed_mps: float = 0.25
    heel_deg: float = 0.0
    rudder_deg: float = 0.0
    sail_deg: float = 0.0
    completed_waypoints: int = 0


@dataclass
class StepRecord:
    t_s: float
    x_m: float
    y_m: float
    heading_deg: float
    target_heading_deg: float
    rudder_deg: float
    sail_deg: float
    speed_mps: float
    heel_deg: float
    wind_direction_deg: float
    wind_speed_mps: float
    apparent_wind_angle_deg: float
    drift_speed_mps: float
    drift_confidence: float
    distance_to_target_m: float
    waypoint_index: int
    completed_waypoints: int
    mpc_cost: float
    vmg_mps: float
    constraint_notes: Tuple[str, ...] = field(default_factory=tuple)


class MockBoatSimulator:
    """Closed-loop mock boat driven by the production control algorithms."""

    def __init__(
        self,
        waypoints: Sequence[Point] | None = None,
        environment: MockWaterEnvironment | None = None,
        constraints: MockBoatConstraints | None = None,
        state: MockBoatState | None = None,
    ):
        self.environment = environment or MockWaterEnvironment()
        self.constraints = constraints or MockBoatConstraints()
        self.state = state or MockBoatState()
        self.waypoints: List[Point] = list(waypoints or [(35.0, 70.0), (90.0, 55.0), (120.0, 5.0)])
        self.current_waypoint_index = 0
        self.mpc = MPCSteering(horizon=8, dt=0.5)
        self.vpp = VPPSailOptimizer()
        self.vpp.set_wave_height(self.environment.wave_height_m)
        self.drift = DriftEstimator(window_size=12)
        self.history: List[StepRecord] = []

    def current_target(self) -> Point:
        if self.current_waypoint_index >= len(self.waypoints):
            return self.waypoints[-1]
        return self.waypoints[self.current_waypoint_index]

    def run(self, duration_s: float = 90.0, dt: float = 0.5) -> List[StepRecord]:
        steps = int(duration_s / dt)
        for k in range(steps):
            self.step(k * dt, dt)
            if self.state.completed_waypoints >= len(self.waypoints):
                break
        return self.history

    def step(self, t_s: float, dt: float) -> StepRecord:
        target = self.current_target()
        target_heading = bearing_to_target((self.state.x_m, self.state.y_m), target)
        wind_dir, wind_speed, gust = self.environment.wind_at(t_s)

        drift_heading = self.drift.get_leeway_correction(
            target_heading,
            self._estimate_leeway_deg(wind_dir),
            max(self.state.speed_mps, 0.5),
        )
        rudder_cmd, mpc_diag = self.mpc.compute(
            heading_deg=self.state.heading_deg,
            yaw_rate_dps=self.state.yaw_rate_dps,
            target_heading_deg=drift_heading,
            boat_speed=max(self.state.speed_mps, 0.5),
            wind_speed=wind_speed,
            wind_dir_deg=wind_dir,
            heel_deg=self.state.heel_deg,
        )
        self.state.rudder_deg = rate_limit(
            rudder_cmd,
            self.state.rudder_deg,
            self.constraints.max_rudder_rate_dps,
            dt,
            -self.constraints.max_rudder_deg,
            self.constraints.max_rudder_deg,
        )

        sail_cmd, vmg, _, vpp_diag = self.vpp.optimize(
            true_wind_speed=wind_speed,
            true_wind_angle_deg=wind_dir,
            target_bearing_deg=drift_heading,
            boat_speed_hint=max(self.state.speed_mps, 0.5),
            current_heel_deg=self.state.heel_deg,
        )
        self.state.sail_deg = rate_limit(
            sail_cmd,
            self.state.sail_deg,
            self.constraints.max_sail_rate_dps,
            dt,
            0.0,
            self.constraints.max_sail_deg,
        )

        notes = self._advance_physics(wind_dir, wind_speed, gust, dt)
        target = self.current_target()
        distance = distance_between((self.state.x_m, self.state.y_m), target)
        if distance <= self.constraints.waypoint_radius_m and self.current_waypoint_index < len(self.waypoints):
            self.current_waypoint_index += 1
            self.state.completed_waypoints = self.current_waypoint_index
            notes = tuple(list(notes) + [f"waypoint {self.state.completed_waypoints} reached"])
            target = self.current_target()
            distance = distance_between((self.state.x_m, self.state.y_m), target)

        boat_vx, boat_vy = velocity_components(self.state.heading_deg, self.state.speed_mps)
        gps_vx = boat_vx + self.environment.current_east_mps
        gps_vy = boat_vy + self.environment.current_north_mps
        self.drift.update(gps_vx, gps_vy, self.state.heading_deg, self.state.speed_mps)
        drift = self.drift.get_drift()

        record = StepRecord(
            t_s=t_s,
            x_m=self.state.x_m,
            y_m=self.state.y_m,
            heading_deg=self.state.heading_deg % 360.0,
            target_heading_deg=target_heading,
            rudder_deg=self.state.rudder_deg,
            sail_deg=self.state.sail_deg,
            speed_mps=self.state.speed_mps,
            heel_deg=self.state.heel_deg,
            wind_direction_deg=wind_dir,
            wind_speed_mps=wind_speed,
            apparent_wind_angle_deg=signed_angle(wind_dir - self.state.heading_deg),
            drift_speed_mps=drift["speed"],
            drift_confidence=drift["confidence"],
            distance_to_target_m=distance,
            waypoint_index=min(self.current_waypoint_index + 1, len(self.waypoints)),
            completed_waypoints=self.state.completed_waypoints,
            mpc_cost=mpc_diag["cost"],
            vmg_mps=vmg,
            constraint_notes=notes,
        )
        self.history.append(record)
        return record

    def _advance_physics(self, wind_dir: float, wind_speed: float, gust: bool, dt: float) -> Tuple[str, ...]:
        notes = []
        awa = abs(signed_angle(wind_dir - self.state.heading_deg))
        no_go_penalty = 0.22 if awa < self.constraints.min_upwind_angle_deg else 1.0
        if no_go_penalty < 1.0:
            notes.append("inside no-go zone")

        sail_match = max(0.0, 1.0 - abs(self.state.sail_deg - ideal_sail_for_awa(awa)) / 65.0)
        wind_drive = max(0.0, math.sin(math.radians(min(awa, 180.0))))
        wave_drag = 1.0 + 1.8 * self.environment.wave_height_m
        rudder_drag = 1.0 + 0.018 * abs(self.state.rudder_deg)
        target_speed = min(
            self.constraints.max_speed_mps,
            0.25 + 0.34 * wind_speed * wind_drive * sail_match * no_go_penalty / wave_drag,
        )
        accel = (target_speed - self.state.speed_mps) * 0.55
        self.state.speed_mps = float(np.clip(self.state.speed_mps + accel * dt, 0.0, self.constraints.max_speed_mps))
        self.state.speed_mps /= rudder_drag

        yaw_accel = 0.34 * self.state.rudder_deg - 0.72 * self.state.yaw_rate_dps
        self.state.yaw_rate_dps = float(np.clip(self.state.yaw_rate_dps + yaw_accel * dt, -45.0, 45.0))
        self.state.heading_deg = (self.state.heading_deg + self.state.yaw_rate_dps * dt) % 360.0

        heel_target = 0.08 * wind_speed * math.sin(math.radians(awa)) * (1.0 - self.state.sail_deg / 120.0)
        heel_target += 0.12 * self.state.yaw_rate_dps
        if gust:
            heel_target *= 1.18
            notes.append("gust load")
        self.state.heel_deg += (heel_target - self.state.heel_deg) * min(1.0, 0.8 * dt)
        if abs(self.state.heel_deg) > 25.0:
            notes.append("heel constraint active")

        vx, vy = velocity_components(self.state.heading_deg, self.state.speed_mps)
        self.state.x_m += (vx + self.environment.current_east_mps) * dt
        self.state.y_m += (vy + self.environment.current_north_mps) * dt
        return tuple(notes)

    def _estimate_leeway_deg(self, wind_dir: float) -> float:
        awa = signed_angle(wind_dir - self.state.heading_deg)
        return float(np.clip(0.04 * awa * (1.0 + abs(self.state.heel_deg) / 35.0), -12.0, 12.0))


def bearing_to_target(position: Point, target: Point) -> float:
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    return math.degrees(math.atan2(dx, dy)) % 360.0


def distance_between(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def signed_angle(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def velocity_components(heading_deg: float, speed_mps: float) -> Point:
    h = math.radians(heading_deg)
    return speed_mps * math.sin(h), speed_mps * math.cos(h)


def ideal_sail_for_awa(awa_deg: float) -> float:
    awa = abs(awa_deg)
    if awa < 35.0:
        return 0.0
    return float(np.clip(0.52 * awa - 8.0, 0.0, 88.0))


def rate_limit(target: float, current: float, rate: float, dt: float, lower: float, upper: float) -> float:
    delta = np.clip(target - current, -rate * dt, rate * dt)
    return float(np.clip(current + delta, lower, upper))


def ascii_map(history: Sequence[StepRecord], waypoints: Sequence[Point], width: int = 42, height: int = 14) -> str:
    points = [(r.x_m, r.y_m) for r in history] + list(waypoints) + [(0.0, 0.0)]
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs) - 5.0, max(xs) + 5.0
    min_y, max_y = min(ys) - 5.0, max(ys) + 5.0
    grid = [["." for _ in range(width)] for _ in range(height)]

    def project(p: Point) -> Tuple[int, int]:
        x = int((p[0] - min_x) / max(max_x - min_x, 1e-6) * (width - 1))
        y = int((p[1] - min_y) / max(max_y - min_y, 1e-6) * (height - 1))
        return max(0, min(width - 1, x)), height - 1 - max(0, min(height - 1, y))

    x, y = project((0.0, 0.0))
    grid[y][x] = "S"
    for record in history[:: max(1, len(history) // 50 or 1)]:
        x, y = project((record.x_m, record.y_m))
        if grid[y][x] == ".":
            grid[y][x] = "*"
    if history:
        x, y = project((history[-1].x_m, history[-1].y_m))
        grid[y][x] = "B"
    for i, waypoint in enumerate(waypoints, 1):
        x, y = project(waypoint)
        grid[y][x] = str(min(i, 9))
    return "\n".join("".join(row) for row in grid)


def format_record(record: StepRecord) -> str:
    notes = ", ".join(record.constraint_notes) if record.constraint_notes else "nominal"
    return (
        f"{record.t_s:5.1f}s | pos=({record.x_m:6.1f},{record.y_m:6.1f})m "
        f"hdg={record.heading_deg:6.1f}->{record.target_heading_deg:6.1f} "
        f"rud={record.rudder_deg:6.1f} sail={record.sail_deg:5.1f} "
        f"spd={record.speed_mps:4.2f} heel={record.heel_deg:5.1f} "
        f"wind={record.wind_direction_deg:5.1f}/{record.wind_speed_mps:4.1f} "
        f"drift={record.drift_speed_mps:4.2f}@{record.drift_confidence:3.2f} "
        f"wp={record.waypoint_index} dist={record.distance_to_target_m:5.1f} "
        f"cost={record.mpc_cost:6.2f} | {notes}"
    )


def build_summary(history: Sequence[StepRecord]) -> dict:
    if not history:
        return {}
    return {
        "steps": len(history),
        "duration_s": history[-1].t_s,
        "completed_waypoints": history[-1].completed_waypoints,
        "final_position_m": (history[-1].x_m, history[-1].y_m),
        "final_distance_to_target_m": history[-1].distance_to_target_m,
        "max_speed_mps": max(r.speed_mps for r in history),
        "max_abs_heel_deg": max(abs(r.heel_deg) for r in history),
        "max_abs_rudder_deg": max(abs(r.rudder_deg) for r in history),
        "max_sail_deg": max(r.sail_deg for r in history),
        "mean_mpc_cost": float(np.mean([r.mpc_cost for r in history])),
    }


def run_demo(duration_s: float = 90.0, dt: float = 0.5, realtime: bool = False) -> List[StepRecord]:
    sim = MockBoatSimulator()
    print("Mock sailboat control pipeline")
    print("controllers=MPC rudder + VPP sail + drift compensation | constraints=rudder/sail rates, heel, current, waves")
    print(ascii_map([], sim.waypoints))
    print()
    for k in range(int(duration_s / dt)):
        record = sim.step(k * dt, dt)
        if k % max(1, int(2.0 / dt)) == 0 or record.constraint_notes:
            print(format_record(record))
        if realtime:
            time.sleep(dt)
        if record.completed_waypoints >= len(sim.waypoints):
            break
    print()
    print(ascii_map(sim.history, sim.waypoints))
    summary = build_summary(sim.history)
    print("\nSummary")
    for key, value in summary.items():
        print(f"  {key}: {value}")
    return sim.history


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Run the sailbot mock boat terminal demonstration.")
    parser.add_argument("--duration", type=float, default=90.0, help="simulation duration in seconds")
    parser.add_argument("--dt", type=float, default=0.5, help="simulation timestep in seconds")
    parser.add_argument("--realtime", action="store_true", help="sleep between simulation ticks for video capture")
    args = parser.parse_args(list(argv) if argv is not None else None)
    run_demo(duration_s=args.duration, dt=args.dt, realtime=args.realtime)


if __name__ == "__main__":
    main()
