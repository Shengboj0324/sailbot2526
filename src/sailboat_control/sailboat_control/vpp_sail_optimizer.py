#!/usr/bin/env python3
"""
Velocity Prediction Program (VPP) Sail Optimizer.

Replaces Phase-4 static polar-table lookup.  Dynamically solves the aero /
hydro equilibrium to find the sail trim that maximises VMG.

Aero model:
  C_L = 2π·sin(α)·η ,  C_D = C_D0 + C_L²/(π·AR·e)
  F_drive = q_air · (C_L·sin|α| - C_D·cos α)

Hydro resistance:
  R = 0.5·ρ_w·S_wet·C_f·V² + k_w·V⁴/L

Equilibrium:  F_drive = R  →  solve for V_s via bisection

VMG = V_s · cos(course - target)

Outputs sail angles in [0, 88]° for Arduino winch deployment.
"""
import numpy as np

RHO_AIR   = 1.225
RHO_WATER = 1025.0
G         = 9.81


class VPPSailOptimizer:
    """Aero/hydro VPP for real-time sail trim optimisation."""

    def __init__(self):
        # Sail geometry
        self.sail_area    = 0.80
        self.aspect_ratio = 3.0
        self.oswald_eff   = 0.85
        self.eta_sail     = 0.75
        self.CD0_sail     = 0.05
        # Hull hydrodynamics
        self.wetted_area  = 0.60
        self.hull_length  = 1.20
        self.Cf           = 0.004
        self.k_wave       = 0.15
        # Limits
        self.max_heel_deg       = 25.0
        self.depower_heel_start = 20.0
        self.max_sail_angle     = 88.0
        self.max_rate_dps       = 10.0
        # State
        self.current_sail_angle = 0.0

    def _aero_coeffs(self, alpha):
        """Lift and drag coefficients at angle of attack α (rad)."""
        CL = np.clip(2.0 * np.pi * np.sin(alpha) * self.eta_sail, -2.0, 2.0)
        CD = self.CD0_sail + CL**2 / (np.pi * self.aspect_ratio * self.oswald_eff)
        return CL, CD

    def _drive_force(self, alpha, V_app):
        CL, CD = self._aero_coeffs(alpha)
        q = 0.5 * RHO_AIR * V_app**2 * self.sail_area
        return q * (CL * np.sin(abs(alpha)) - CD * np.cos(alpha))

    def _heel_force(self, alpha, V_app):
        CL, CD = self._aero_coeffs(alpha)
        q = 0.5 * RHO_AIR * V_app**2 * self.sail_area
        return abs(q * (CL * np.cos(alpha) + CD * np.sin(abs(alpha))))

    def _hull_resistance(self, V):
        R_visc = 0.5 * RHO_WATER * self.wetted_area * self.Cf * V**2
        R_wave = self.k_wave * V**4 / self.hull_length
        return R_visc + R_wave

    def _solve_speed(self, F_drive, tol=0.01):
        """Bisection solver for equilibrium speed (Arduino-friendly)."""
        if F_drive <= 0:
            return 0.0
        lo, hi = 0.0, 6.0
        for _ in range(20):
            mid = 0.5 * (lo + hi)
            if self._hull_resistance(mid) < F_drive:
                lo = mid
            else:
                hi = mid
            if hi - lo < tol:
                break
        return 0.5 * (lo + hi)

    def _apparent_wind(self, tws, twa_rad, V_s):
        aw_x = tws * np.cos(twa_rad) + V_s
        aw_y = tws * np.sin(twa_rad)
        V_app = np.sqrt(aw_x**2 + aw_y**2) + 1e-6
        awa = np.arctan2(aw_y, aw_x)
        return V_app, awa

    def _estimate_heel(self, F_heel, mh=0.40, mass=25.0):
        """Static heel estimate. mh = metacentric height (m).
        Includes keel righting moment for stability."""
        righting = mass * G * mh + 0.5 * RHO_WATER * G * 0.06 * 0.30  # keel contribution
        ratio = np.clip(F_heel / (righting + 1e-6), -1, 1)
        return np.degrees(np.arcsin(ratio))

    def _rate_limit(self, target, dt=1.0):
        change = np.clip(target - self.current_sail_angle,
                         -self.max_rate_dps * dt, self.max_rate_dps * dt)
        result = np.clip(self.current_sail_angle + change, 0, self.max_sail_angle)
        self.current_sail_angle = result
        return result

    def optimize(self, true_wind_speed, true_wind_angle_deg,
                 target_bearing_deg, boat_speed_hint=2.0,
                 current_heel_deg=0.0):
        """Find sail angle maximising VMG toward target bearing.

        Returns (sail_deg, vmg, predicted_speed, diagnostics).
        """
        twa  = np.radians(true_wind_angle_deg)
        tgt  = np.radians(target_bearing_deg)
        best_vmg, best_sail, best_spd, best_heel = -np.inf, 0.0, 0.0, 0.0
        scan = []

        for s_deg in np.arange(0, self.max_sail_angle + 1, 2.0):
            s_rad = np.radians(s_deg)
            V_app, awa = self._apparent_wind(true_wind_speed, twa, boat_speed_hint)
            alpha = awa - s_rad
            V_s = self._solve_speed(self._drive_force(alpha, V_app))
            # Refine once at equilibrium speed
            V_app2, awa2 = self._apparent_wind(true_wind_speed, twa, V_s)
            alpha2 = awa2 - s_rad
            V_s = self._solve_speed(self._drive_force(alpha2, V_app2))
            heel = self._estimate_heel(self._heel_force(alpha2, V_app2))
            vmg  = V_s * np.cos(twa - tgt)
            scan.append({'sail_deg': s_deg, 'speed': V_s,
                         'vmg': vmg, 'heel': heel,
                         'alpha_deg': np.degrees(alpha2)})
            if abs(heel) <= self.max_heel_deg and vmg > best_vmg:
                best_vmg, best_sail, best_spd, best_heel = vmg, s_deg, V_s, heel

        # Fallback: if no sail angle satisfies heel limit, pick lowest heel
        if best_vmg == -np.inf and scan:
            min_heel_pt = min(scan, key=lambda p: abs(p['heel']))
            best_sail = min_heel_pt['sail_deg']
            best_spd  = min_heel_pt['speed']
            best_heel = min_heel_pt['heel']
            best_vmg  = min_heel_pt['vmg']

        # Depowering for excess heel
        if abs(current_heel_deg) > self.depower_heel_start:
            factor = 1.0 + 0.1 * (abs(current_heel_deg) - self.depower_heel_start)
            best_sail = min(best_sail * factor, self.max_sail_angle)
        best_sail = self._rate_limit(best_sail)

        # Best L/D from scan
        best_ld = 0.0
        for pt in scan:
            CL, CD = self._aero_coeffs(np.radians(pt['alpha_deg']))
            if CD > 1e-6:
                best_ld = max(best_ld, abs(CL / CD))

        diag = {'vmg': best_vmg, 'predicted_speed': best_spd,
                'predicted_heel': best_heel, 'scan_points': len(scan),
                'best_lift_to_drag': best_ld}
        return best_sail, best_vmg, best_spd, diag

    def to_arduino_angle(self, sail_deg):
        return np.clip(sail_deg, 0.0, self.max_sail_angle)

    def reset(self):
        self.current_sail_angle = 0.0
