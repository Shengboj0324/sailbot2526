#!/usr/bin/env python3
"""
Model Predictive Controller (MPC) for sailboat steering — Phase-6 Production.

Architecture:
  • SLSQP constrained optimiser (scipy.optimize.minimize) replaces brute-force
    tree search for true gradient-based convergence over the full horizon.
  • Hard constraints on rudder travel (±21°) and slew rate (15°/s) enforced
    via scipy inequality constraints — not post-hoc clamping.
  • Obstacle avoidance via Artificial Potential Field penalty in cost function.
  • Predictive heel constraint: penalises sequences that predict heel > 25°.
  • Falls back to greedy search if scipy is unavailable (Arduino deployment).

Cost function:
  J = Σ_{k=0}^{N-1}  [ Q_ψ·(ψ_k − ψ_ref)² + Q_r·r_k² + R_δ·(δ_k − δ_{k-1})² ]
     + Q_ψf·(ψ_N − ψ_ref)²                         # terminal
     + Σ Q_obs · max(0, d_safe − d_obstacle)²        # obstacle penalty
     + Σ Q_heel · max(0, |heel_k| − heel_lim)²       # heel penalty

Constraints (SLSQP inequality):
  |δ_r|       ≤ 21° (rudder travel)
  |Δδ_r/Δt|   ≤ 15°/s (servo slew rate)

Nomoto first-order yaw model:  T·ṙ + r = K·δ_r
"""
import numpy as np

try:
    from scipy.optimize import minimize as scipy_minimize
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

# ─── Nomoto model defaults ──────────────────────────────────────────
NOMOTO_K = 0.30     # steady-state yaw gain (rad/s per rad rudder)
NOMOTO_T = 2.5      # yaw time-constant (seconds)


class NomotoModel:
    """Nomoto first-order yaw model for MPC prediction."""

    def __init__(self, K=NOMOTO_K, T=NOMOTO_T):
        self.K = K
        self.T = T

    def step(self, psi, r, delta_r, dt):
        """Advance one step.  Returns (psi_new, r_new)."""
        r_dot = (self.K * delta_r - r) / self.T
        r_new   = r + r_dot * dt
        psi_new = psi + r_new * dt
        psi_new = np.arctan2(np.sin(psi_new), np.cos(psi_new))
        return psi_new, r_new


class Obstacle:
    """Represents a static or moving obstacle for collision avoidance."""

    def __init__(self, x, y, radius, vx=0.0, vy=0.0):
        self.x = x
        self.y = y
        self.radius = radius    # safe-distance buffer (m)
        self.vx = vx            # velocity east (m/s) — for moving obstacles
        self.vy = vy            # velocity north (m/s)

    def position_at(self, dt):
        """Predict obstacle position at time dt in the future."""
        return self.x + self.vx * dt, self.y + self.vy * dt


class MPCSteering:
    """SLSQP-based Model Predictive Controller for rudder angle.

    All angles in radians internally; public API accepts/returns degrees.
    """

    def __init__(self, horizon=10, dt=0.5, nomoto_K=NOMOTO_K, nomoto_T=NOMOTO_T):
        self.N  = horizon
        self.dt = dt
        self.model = NomotoModel(nomoto_K, nomoto_T)

        # Cost weights
        self.Q_psi   = 10.0     # heading tracking
        self.Q_r     = 1.0      # yaw rate penalty
        self.R_dr    = 5.0      # rudder rate-of-change penalty
        self.Q_psi_f = 20.0     # terminal heading penalty
        self.Q_obs   = 500.0    # obstacle avoidance penalty
        self.Q_heel  = 50.0     # heel violation penalty

        # Constraints
        self.dr_max      = np.radians(21.0)   # max rudder deflection
        self.dr_rate_max = np.radians(15.0)    # max rudder rate (per second)

        # Heel constraint
        self.heel_limit_rad = np.radians(25.0)

        # Obstacle list (set externally each cycle)
        self.obstacles = []     # list of Obstacle instances

        # Boat state for obstacle distance computation
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.boat_speed = 2.0

        # Internal state
        self.last_dr = 0.0

        # Greedy fallback candidates (used if scipy unavailable)
        self.n_candidates = 7
        self._dr_candidates = np.linspace(-self.dr_rate_max * self.dt,
                                           self.dr_rate_max * self.dt,
                                           self.n_candidates)

    # ── Obstacle management ─────────────────────────────────────────
    def set_obstacles(self, obstacles):
        """Set obstacle list. Each is an Obstacle instance."""
        self.obstacles = list(obstacles)

    def clear_obstacles(self):
        self.obstacles = []

    def set_boat_position(self, x, y):
        self.boat_x = x
        self.boat_y = y

    # ── SLSQP cost function ─────────────────────────────────────────
    def _cost(self, dr_seq, psi0, r0, psi_ref, heel_rad=0.0):
        """Evaluate total cost for a rudder sequence (N,)."""
        psi, r = psi0, r0
        dr_prev = self.last_dr
        cost = 0.0

        # Predicted boat trajectory for obstacle checking
        bx, by = self.boat_x, self.boat_y
        spd = max(self.boat_speed, 0.5)

        for k in range(len(dr_seq)):
            psi, r = self.model.step(psi, r, dr_seq[k], self.dt)

            # Heading error (wrapped)
            e_psi = np.arctan2(np.sin(psi - psi_ref), np.cos(psi - psi_ref))

            # Stage cost
            cost += self.Q_psi * e_psi**2
            cost += self.Q_r   * r**2
            cost += self.R_dr  * (dr_seq[k] - dr_prev)**2
            dr_prev = dr_seq[k]

            # Predicted position for obstacle check
            bx += spd * np.cos(psi) * self.dt
            by += spd * np.sin(psi) * self.dt
            t_future = (k + 1) * self.dt

            # Obstacle avoidance penalty
            for obs in self.obstacles:
                ox, oy = obs.position_at(t_future)
                dist = np.sqrt((bx - ox)**2 + (by - oy)**2)
                violation = max(0.0, obs.radius - dist)
                cost += self.Q_obs * violation**2

            # Predictive heel penalty (simplified: more rudder → more heel)
            # Heel increases with yaw rate due to centripetal force
            pred_heel = abs(heel_rad) + 0.3 * abs(r)
            heel_violation = max(0.0, pred_heel - self.heel_limit_rad)
            cost += self.Q_heel * heel_violation**2

        # Terminal cost
        e_f = np.arctan2(np.sin(psi - psi_ref), np.cos(psi - psi_ref))
        cost += self.Q_psi_f * e_f**2
        return cost

    # ── SLSQP constraint builders ───────────────────────────────────
    def _build_constraints(self):
        """Build scipy constraint dicts for SLSQP."""
        constraints = []
        max_delta = self.dr_rate_max * self.dt

        # Slew rate: |δ[k] - δ[k-1]| ≤ max_delta
        # First step: slew from last_dr
        def slew_first_upper(dr_seq):
            return max_delta - (dr_seq[0] - self.last_dr)

        def slew_first_lower(dr_seq):
            return max_delta + (dr_seq[0] - self.last_dr)

        constraints.append({'type': 'ineq', 'fun': slew_first_upper})
        constraints.append({'type': 'ineq', 'fun': slew_first_lower})

        # Subsequent steps
        for k in range(1, self.N):
            def slew_upper(dr_seq, k=k):
                return max_delta - (dr_seq[k] - dr_seq[k-1])
            def slew_lower(dr_seq, k=k):
                return max_delta + (dr_seq[k] - dr_seq[k-1])
            constraints.append({'type': 'ineq', 'fun': slew_upper})
            constraints.append({'type': 'ineq', 'fun': slew_lower})

        return constraints

    def _build_bounds(self):
        """Rudder travel bounds for each step."""
        return [(-self.dr_max, self.dr_max)] * self.N

    # ── Main compute ────────────────────────────────────────────────
    def compute(self, heading_deg, yaw_rate_dps, target_heading_deg,
                boat_speed=2.0, wind_speed=0.0, wind_dir_deg=0.0,
                heel_deg=0.0):
        """Compute optimal rudder angle (degrees).

        Uses SLSQP for gradient-based constrained optimisation.
        Falls back to greedy search if scipy is unavailable.

        Returns: (rudder_angle_deg, diagnostics_dict)
        """
        psi0    = np.radians(heading_deg)
        r0      = np.radians(yaw_rate_dps)
        psi_ref = np.radians(target_heading_deg)
        heel_rad = np.radians(heel_deg)
        self.boat_speed = boat_speed

        if SCIPY_AVAILABLE and self.dr_rate_max > 0:
            optimal_dr, best_cost = self._solve_slsqp(psi0, r0, psi_ref, heel_rad)
        else:
            optimal_dr, best_cost = self._solve_greedy(psi0, r0, psi_ref, heel_rad)

        self.last_dr = optimal_dr
        rudder_deg = np.degrees(optimal_dr)

        diag = {
            'cost': float(best_cost),
            'horizon': self.N,
            'solver': 'SLSQP' if SCIPY_AVAILABLE else 'greedy',
            'n_obstacles': len(self.obstacles),
            'predicted_heading_deg': float(np.degrees(
                self._rollout_heading(psi0, r0, np.full(self.N, optimal_dr)))),
        }
        return rudder_deg, diag

    def _solve_slsqp(self, psi0, r0, psi_ref, heel_rad):
        """Solve via SLSQP constrained optimisation."""
        # Warm-start: initial guess = hold current rudder
        x0 = np.full(self.N, self.last_dr)

        result = scipy_minimize(
            self._cost,
            x0,
            args=(psi0, r0, psi_ref, heel_rad),
            method='SLSQP',
            bounds=self._build_bounds(),
            constraints=self._build_constraints(),
            options={'maxiter': 50, 'ftol': 1e-6, 'disp': False}
        )

        if result.success or result.fun < self._cost(x0, psi0, r0, psi_ref, heel_rad):
            return float(result.x[0]), float(result.fun)
        else:
            # SLSQP failed to improve — use initial guess
            return float(x0[0]), float(self._cost(x0, psi0, r0, psi_ref, heel_rad))

    def _solve_greedy(self, psi0, r0, psi_ref, heel_rad):
        """Greedy tree-search fallback (no scipy needed)."""
        best_cost = np.inf
        best_dr   = self.last_dr

        for c0 in self._dr_candidates:
            dr0 = self._enforce_constraints(self.last_dr + c0, self.last_dr)
            seq = np.full(self.N, dr0)
            psi, r = self.model.step(psi0, r0, dr0, self.dt)

            for k in range(1, self.N):
                best_local_cost = np.inf
                best_local_dr   = dr0
                for c in self._dr_candidates:
                    dr_try = self._enforce_constraints(seq[k-1] + c, seq[k-1])
                    psi_try, r_try = self.model.step(psi, r, dr_try, self.dt)
                    e = np.arctan2(np.sin(psi_try - psi_ref),
                                   np.cos(psi_try - psi_ref))
                    lc = self.Q_psi * e**2 + self.Q_r * r_try**2
                    if lc < best_local_cost:
                        best_local_cost = lc
                        best_local_dr   = dr_try
                seq[k] = best_local_dr
                psi, r = self.model.step(psi, r, best_local_dr, self.dt)

            total_cost = self._cost(seq, psi0, r0, psi_ref, heel_rad)
            if total_cost < best_cost:
                best_cost = total_cost
                best_dr   = seq[0]

        return float(best_dr), float(best_cost)

    def _enforce_constraints(self, dr, prev_dr):
        """Enforce actuator limits on a single rudder command."""
        delta = dr - prev_dr
        max_delta = self.dr_rate_max * self.dt
        delta = np.clip(delta, -max_delta, max_delta)
        dr = prev_dr + delta
        dr = np.clip(dr, -self.dr_max, self.dr_max)
        return dr

    def _rollout_heading(self, psi0, r0, seq):
        """Return heading at end of sequence."""
        psi, r = psi0, r0
        for dr in seq:
            psi, r = self.model.step(psi, r, dr, self.dt)
        return psi

    def reset(self):
        self.last_dr = 0.0
        self.obstacles = []