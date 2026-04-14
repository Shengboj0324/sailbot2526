#!/usr/bin/env python3
"""
Velocity Prediction Program (VPP) Sail Optimizer — Phase-6 Production.

High-fidelity aero/hydro equilibrium solver with:
  • Non-linear stall model (post-stall lift rolloff + flow separation drag)
  • Wave-height dependent added resistance (Gerritsma & Beukelman)
  • Proactive heel constraint (rejects trim angles that PREDICT > 25° heel)
  • Calibration-ready hull parameters with defaults for 1.2 m sailbot hull

Aero model (pre-stall):
  C_L = 2π·sin(α)·η     (α < α_stall)
  C_D = C_D0 + C_L² / (π·AR·e)

Aero model (post-stall):
  C_L = C_L_max · sin(2α)  (Viterna extrapolation for flat-plate limit)
  C_D = C_D_max - (C_D_max - C_D(α_stall))·cos²(α - α_stall)

Hydro resistance:
  R_visc = ½·ρ_w·S_wet·C_f·V²           (ITTC-57)
  R_wave = k_w·V⁴/L                      (wave-making)
  R_added = ½·ρ_w·S_wet·C_aw·(H_s/L)²·V² (added resistance in waves)

Outputs sail angles in [0, 88]° for Arduino winch deployment.
"""
import numpy as np

RHO_AIR   = 1.225
RHO_WATER = 1025.0
G         = 9.81


class VPPSailOptimizer:
    """High-fidelity aero/hydro VPP for real-time sail trim optimisation."""

    def __init__(self):
        # ── Sail geometry ────────────────────────────────────────────
        self.sail_area    = 0.80    # m²
        self.aspect_ratio = 3.0
        self.oswald_eff   = 0.85
        self.eta_sail     = 0.75    # sail efficiency factor
        self.CD0_sail     = 0.05    # zero-lift drag

        # ── Stall model parameters ───────────────────────────────────
        self.alpha_stall  = np.radians(15.0)   # stall angle (rad)
        self.CL_max       = 1.2     # max CL at stall
        self.CD_max       = 1.8     # flat-plate drag at 90°
        self.stall_width  = np.radians(5.0)    # transition width

        # ── Hull hydrodynamics ───────────────────────────────────────
        self.wetted_area  = 0.60    # m²
        self.hull_length  = 1.20    # m
        self.Cf           = 0.004   # ITTC-57 friction coefficient
        self.k_wave       = 0.15    # wave-making coefficient

        # ── Wave-added resistance ────────────────────────────────────
        self.wave_height  = 0.0     # significant wave height H_s (m)
        self.C_aw         = 0.02    # added resistance coefficient

        # ── Limits ───────────────────────────────────────────────────
        self.max_heel_deg       = 25.0
        self.depower_heel_start = 20.0
        self.max_sail_angle     = 88.0
        self.max_rate_dps       = 10.0

        # ── State ────────────────────────────────────────────────────
        self.current_sail_angle = 0.0

    # ── Non-linear stall model ──────────────────────────────────────
    def _aero_coeffs(self, alpha):
        """Lift and drag with post-stall Viterna extrapolation.

        Pre-stall:  CL = 2π·sin(α)·η, CD = CD0 + CL²/(π·AR·e)
        Post-stall: CL = CL_max·sin(2α), CD blends to flat-plate
        Transition: smooth blend over ±stall_width
        """
        abs_a = abs(alpha)

        # Pre-stall coefficients
        CL_pre = 2.0 * np.pi * np.sin(alpha) * self.eta_sail
        CL_pre = np.clip(CL_pre, -self.CL_max, self.CL_max)
        CD_pre = self.CD0_sail + CL_pre**2 / (np.pi * self.aspect_ratio * self.oswald_eff)

        # Post-stall coefficients (Viterna flat-plate model)
        CL_post = self.CL_max * np.sin(2.0 * alpha)
        CD_stall_onset = self.CD0_sail + self.CL_max**2 / (np.pi * self.aspect_ratio * self.oswald_eff)
        CD_post = self.CD_max - (self.CD_max - CD_stall_onset) * np.cos(alpha)**2

        # Smooth blend using sigmoid transition
        blend = 0.5 * (1.0 + np.tanh((abs_a - self.alpha_stall) / (self.stall_width + 1e-9)))

        CL = (1.0 - blend) * CL_pre + blend * CL_post
        CD = (1.0 - blend) * CD_pre + blend * CD_post

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
        """Total hull resistance including wave-added component.

        R_total = R_viscous + R_wavemaking + R_added_waves
        """
        R_visc = 0.5 * RHO_WATER * self.wetted_area * self.Cf * V**2
        R_wave = self.k_wave * V**4 / self.hull_length

        # Wave-added resistance: Gerritsma & Beukelman model
        # R_aw = ½·ρ·S·C_aw·(H_s/L)²·V²
        R_added = 0.0
        if self.wave_height > 0 and self.hull_length > 0:
            R_added = (0.5 * RHO_WATER * self.wetted_area * self.C_aw
                       * (self.wave_height / self.hull_length)**2 * V**2)

        return R_visc + R_wave + R_added

    def set_wave_height(self, H_s):
        """Set significant wave height for added resistance calculation."""
        self.wave_height = max(0.0, H_s)

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

        Proactive heel constraint: rejects trim angles where PREDICTED
        heel exceeds max_heel_deg, even if current heel is within limit.

        Returns (sail_deg, vmg, predicted_speed, diagnostics).
        """
        twa  = np.radians(true_wind_angle_deg)
        tgt  = np.radians(target_bearing_deg)
        best_vmg, best_sail, best_spd, best_heel = -np.inf, 0.0, 0.0, 0.0
        scan = []
        stall_detected = False

        for s_deg in np.arange(0, self.max_sail_angle + 1, 2.0):
            s_rad = np.radians(s_deg)
            V_app, awa = self._apparent_wind(true_wind_speed, twa, boat_speed_hint)
            alpha = awa - s_rad
            V_s = self._solve_speed(self._drive_force(alpha, V_app))
            # Refine at equilibrium speed
            V_app2, awa2 = self._apparent_wind(true_wind_speed, twa, V_s)
            alpha2 = awa2 - s_rad
            V_s = self._solve_speed(self._drive_force(alpha2, V_app2))
            heel = self._estimate_heel(self._heel_force(alpha2, V_app2))
            vmg  = V_s * np.cos(twa - tgt)

            is_stalled = abs(alpha2) > self.alpha_stall
            if is_stalled:
                stall_detected = True

            scan.append({'sail_deg': s_deg, 'speed': V_s,
                         'vmg': vmg, 'heel': heel,
                         'alpha_deg': np.degrees(alpha2),
                         'stalled': is_stalled})

            # PROACTIVE heel constraint: reject if PREDICTED heel violates
            if abs(heel) > self.max_heel_deg:
                continue
            # Also reject if current heel is already near limit and this
            # trim angle would increase it
            if abs(current_heel_deg) > self.depower_heel_start:
                combined = abs(heel) + 0.3 * abs(current_heel_deg)
                if combined > self.max_heel_deg * 1.5:
                    continue

            if vmg > best_vmg:
                best_vmg  = vmg
                best_sail = s_deg
                best_spd  = V_s
                best_heel = heel

        # Fallback: no angle satisfies heel limit → pick lowest heel
        if best_vmg == -np.inf and scan:
            min_heel_pt = min(scan, key=lambda p: abs(p['heel']))
            best_sail = min_heel_pt['sail_deg']
            best_spd  = min_heel_pt['speed']
            best_heel = min_heel_pt['heel']
            best_vmg  = min_heel_pt['vmg']

        # Reactive depowering: if CURRENT heel is already past threshold
        if abs(current_heel_deg) > self.depower_heel_start:
            excess = abs(current_heel_deg) - self.depower_heel_start
            factor = 1.0 + 0.1 * excess
            best_sail = min(best_sail * factor, self.max_sail_angle)
        best_sail = self._rate_limit(best_sail)

        # Best L/D from scan
        best_ld = 0.0
        for pt in scan:
            CL, CD = self._aero_coeffs(np.radians(pt['alpha_deg']))
            if CD > 1e-6:
                best_ld = max(best_ld, abs(CL / CD))

        diag = {
            'vmg': best_vmg, 'predicted_speed': best_spd,
            'predicted_heel': best_heel, 'scan_points': len(scan),
            'best_lift_to_drag': best_ld,
            'stall_detected': stall_detected,
            'wave_height': self.wave_height,
        }
        return best_sail, best_vmg, best_spd, diag

    def to_arduino_angle(self, sail_deg):
        return np.clip(sail_deg, 0.0, self.max_sail_angle)

    def reset(self):
        self.current_sail_angle = 0.0
