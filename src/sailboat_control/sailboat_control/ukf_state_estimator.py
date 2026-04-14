#!/usr/bin/env python3
"""
Square-Root Unscented Kalman Filter (SR-UKF) — Phase-6 Production.

6-DOF sailboat state estimation with high-fidelity dynamics:
  • Non-linear sail aero with post-stall model (Viterna extrapolation)
  • Added-mass effects for surge, sway, and yaw (Fossen formulation)
  • Wave-induced excitation forces (Pierson-Moskowitz spectral model)
  • Rudder stall beyond critical angle of attack (~15°)
  • Square-root form for guaranteed positive-definiteness

State vector (n=10):
  x = [x, y, psi, u, v, r, phi, p, delta_r, delta_s]

Measurement vector (m=6):
  z = [x_gps, y_gps, psi_compass, phi_imu, p_imu, r_imu]
"""
import numpy as np

def _safe_qr(A):
    """QR decomposition that sanitises NaN/Inf before factoring."""
    A = np.array(A, dtype=np.float64)
    A = np.nan_to_num(A, nan=0.0, posinf=0.0, neginf=0.0)
    Q, R = np.linalg.qr(A)
    return Q, R

# ─── Physical constants ────────────────────────────────────────────────
RHO_WATER = 1025.0   # kg/m^3
RHO_AIR   = 1.225    # kg/m^3
G         = 9.81     # m/s^2


class SailboatDynamics:
    """Non-linear 6-DOF sailboat dynamics with production-grade physics.

    Added features over Phase-5:
      • Added mass (m_a_surge, m_a_sway, I_a_yaw) per Fossen
      • Non-linear sail stall model matching VPP
      • Rudder stall beyond critical AoA
      • Wave excitation forces
    """

    def __init__(self):
        # ── Hull parameters ──────────────────────────────────────────
        self.mass       = 25.0     # kg
        self.I_zz       = 8.0      # yaw inertia  (kg·m²)
        self.I_xx       = 4.0      # roll inertia (kg·m²)
        self.hull_len   = 1.2      # m
        self.beam       = 0.40     # m
        self.draft      = 0.30     # m
        self.sail_area  = 0.8      # m²
        self.rudder_area = 0.02    # m²
        self.keel_area  = 0.06     # m²
        self.cog_z      = -0.05    # CoG height (neg = below WL)
        self.metacentric_h = 0.25  # metacentric height (m)

        # ── Hydrodynamic coefficients ────────────────────────────────
        self.Xu   = -4.0
        self.Xuu  = -6.0
        self.Yv   = -20.0
        self.Yvv  = -40.0
        self.Nr   = -3.0
        self.Nrr  = -6.0
        self.Kp   = -5.0
        self.Kpp  = -10.0

        # ── Added mass (Fossen, strip theory for slender hull) ───────
        # m_a ≈ ρ·π·(B/2)²·L for surge approximation (much less than sway)
        self.m_a_surge = 2.0       # kg (small for slender hull)
        self.m_a_sway  = 15.0      # kg (large for flat bottom)
        self.I_a_yaw   = 2.0       # kg·m² (added yaw inertia)
        self.I_a_roll  = 1.0       # kg·m² (added roll inertia)

        # ── Stall model parameters (sail) ────────────────────────────
        self.alpha_stall_sail = np.radians(15.0)
        self.CL_max_sail = 1.2
        self.CD_max_sail = 1.8
        self.stall_width_sail = np.radians(5.0)

        # ── Rudder stall parameters ──────────────────────────────────
        self.alpha_stall_rudder = np.radians(15.0)
        self.CL_max_rudder = 0.9

        # ── Wave state ───────────────────────────────────────────────
        self.wave_height = 0.0     # significant wave height H_s (m)
        self.wave_period = 4.0     # dominant wave period T_p (s)
        self._wave_phase = 0.0     # internal phase accumulator

    def _sail_aero(self, alpha, V_app):
        """Non-linear sail forces with post-stall model."""
        abs_a = abs(alpha)

        # Pre-stall
        CL_pre = 2.0 * np.pi * np.sin(alpha) * 0.75  # η = 0.75
        CL_pre = np.clip(CL_pre, -self.CL_max_sail, self.CL_max_sail)
        CD_pre = 0.05 + CL_pre**2 / (np.pi * 3.0 * 0.85)

        # Post-stall (Viterna)
        CL_post = self.CL_max_sail * np.sin(2.0 * alpha)
        CD_stall = 0.05 + self.CL_max_sail**2 / (np.pi * 3.0 * 0.85)
        CD_post = self.CD_max_sail - (self.CD_max_sail - CD_stall) * np.cos(alpha)**2

        # Smooth blend
        blend = 0.5 * (1.0 + np.tanh((abs_a - self.alpha_stall_sail) / (self.stall_width_sail + 1e-9)))
        CL = (1.0 - blend) * CL_pre + blend * CL_post
        CD = (1.0 - blend) * CD_pre + blend * CD_post

        q = 0.5 * RHO_AIR * V_app**2 * self.sail_area
        Fx = q * (CL * np.sin(abs(alpha)) - CD * np.cos(alpha))
        Fy = q * (CL * np.cos(alpha) + CD * np.sin(abs(alpha)))
        return Fx, Fy

    def _rudder_force(self, rudder_aoa, V_water):
        """Rudder force with stall beyond critical angle."""
        abs_aoa = abs(rudder_aoa)
        q = 0.5 * RHO_WATER * V_water**2 * self.rudder_area

        if abs_aoa < self.alpha_stall_rudder:
            CL = 2.0 * np.pi * rudder_aoa
        else:
            # Post-stall: lift drops, drag increases
            sign = np.sign(rudder_aoa)
            CL = sign * self.CL_max_rudder * np.sin(2.0 * rudder_aoa)

        F_rud_y = q * CL
        N_rud = -F_rud_y * 0.5 * self.hull_len
        return F_rud_y, N_rud

    def _wave_forces(self, dt):
        """Wave-induced excitation forces (simplified spectral model).

        Models Pierson-Moskowitz spectrum as sinusoidal forcing with
        amplitude proportional to H_s and frequency from T_p.
        """
        if self.wave_height <= 0:
            return 0.0, 0.0, 0.0, 0.0   # Fx, Fy, N, K (roll)

        omega = 2.0 * np.pi / max(self.wave_period, 1.0)
        self._wave_phase += omega * dt

        amp = self.wave_height  # scaling factor
        phase = self._wave_phase

        # Surge excitation
        Fx_wave = 0.5 * RHO_WATER * G * self.beam * self.draft * amp * np.cos(phase) * 0.1
        # Sway excitation
        Fy_wave = 0.3 * RHO_WATER * G * self.hull_len * self.draft * amp * np.sin(phase) * 0.1
        # Yaw excitation (wave-induced yaw moment)
        N_wave = 0.05 * RHO_WATER * G * self.hull_len**2 * self.draft * amp * np.sin(phase + 0.5) * 0.1
        # Roll excitation
        K_wave = 0.4 * RHO_WATER * G * self.beam * self.draft * amp * np.sin(phase + 1.0) * 0.1

        return Fx_wave, Fy_wave, N_wave, K_wave

    def f(self, x, dt, wind_speed=0.0, wind_angle=0.0):
        """Propagate state one time-step with full 6-DOF dynamics.

        Includes added mass, sail stall, rudder stall, wave excitation.
        """
        x = np.clip(x, -1e4, 1e4)
        px, py, psi, u, v, r, phi, p, dr, ds = x
        u   = np.clip(u, -10, 10)
        v   = np.clip(v, -5, 5)
        r   = np.clip(r, -3, 3)
        p   = np.clip(p, -3, 3)
        phi = np.clip(phi, -0.8, 0.8)

        cpsi, spsi = np.cos(psi), np.sin(psi)
        cphi = np.cos(phi)

        # ── Effective inertia (rigid body + added mass) ──────────────
        m_eff_surge = self.mass + self.m_a_surge
        m_eff_sway  = self.mass + self.m_a_sway
        I_eff_yaw   = self.I_zz + self.I_a_yaw
        I_eff_roll  = self.I_xx + self.I_a_roll

        # ── Apparent wind ────────────────────────────────────────────
        aw_x = wind_speed * np.cos(wind_angle - psi) - u
        aw_y = wind_speed * np.sin(wind_angle - psi) - v
        V_app = np.sqrt(aw_x**2 + aw_y**2) + 1e-6
        alpha = np.arctan2(aw_y, aw_x) - ds

        # ── Sail forces (with stall model) ───────────────────────────
        F_sail_x, F_sail_y = self._sail_aero(alpha, V_app)

        # ── Keel side-force ──────────────────────────────────────────
        leeway_angle = np.arctan2(v, u + 1e-6)
        V_water = np.sqrt(u**2 + v**2) + 1e-6
        CL_keel = 2.0 * np.pi * leeway_angle
        q_water = 0.5 * RHO_WATER * V_water**2 * self.keel_area
        F_keel_y = -q_water * CL_keel

        # ── Rudder force (with stall) ────────────────────────────────
        rudder_aoa = dr - np.arctan2(v + r * 0.5 * self.hull_len, u + 1e-6)
        F_rud_y, N_rud = self._rudder_force(rudder_aoa, V_water)

        # ── Hull drag ────────────────────────────────────────────────
        Fx_hull = self.Xu * u + self.Xuu * u * abs(u)
        Fy_hull = self.Yv * v + self.Yvv * v * abs(v)

        # ── Wave excitation ──────────────────────────────────────────
        Fx_wave, Fy_wave, N_wave, K_wave = self._wave_forces(dt)

        # ── Total forces / moments ───────────────────────────────────
        Fx = F_sail_x + Fx_hull + Fx_wave
        Fy = F_sail_y + F_keel_y + F_rud_y + Fy_hull + Fy_wave
        Nz = N_rud + self.Nr * r + self.Nrr * r * abs(r) + N_wave

        # ── Roll moment ──────────────────────────────────────────────
        K_aero  = F_sail_y * self.cog_z
        K_hydro = -self.mass * G * self.metacentric_h * np.sin(phi)
        K_damp  = self.Kp * p + self.Kpp * p * abs(p)
        Kx = K_aero + K_hydro + K_damp + K_wave

        # ── Accelerations (with added-mass Coriolis coupling) ────────
        # Fossen notation: (m + m_a)·u̇ = Fx + (m + m_a_sway)·v·r
        u_dot = Fx / m_eff_surge + (m_eff_sway / m_eff_surge) * v * r
        v_dot = Fy / m_eff_sway  - (m_eff_surge / m_eff_sway) * u * r
        r_dot = Nz / I_eff_yaw
        p_dot = Kx / I_eff_roll

        # ── Integrate (Euler) ────────────────────────────────────────
        xn = np.empty(10)
        xn[0] = px  + (u * cpsi - v * spsi) * cphi * dt
        xn[1] = py  + (u * spsi + v * cpsi) * cphi * dt
        xn[2] = psi + r * dt
        xn[3] = u   + u_dot * dt
        xn[4] = v   + v_dot * dt
        xn[5] = r   + r_dot * dt
        xn[6] = phi + p * dt
        xn[7] = p   + p_dot * dt
        xn[8] = dr
        xn[9] = ds

        # Clamp to prevent overflow
        xn[3] = np.clip(xn[3], -10.0, 10.0)
        xn[4] = np.clip(xn[4], -5.0, 5.0)
        xn[5] = np.clip(xn[5], -3.0, 3.0)
        xn[7] = np.clip(xn[7], -3.0, 3.0)
        xn[2] = np.arctan2(np.sin(xn[2]), np.cos(xn[2]))
        xn[6] = np.clip(xn[6], np.radians(-45), np.radians(45))

        if not np.all(np.isfinite(xn)):
            xn = np.array([px, py, psi, u, v, r, phi, p, dr, ds])
        return xn


class SquareRootUKF:
    """Square-Root Unscented Kalman Filter for 10-state sailboat model."""

    def __init__(self):
        self.n = 10                     # state dimension
        self.m = 6                      # measurement dimension
        self.dynamics = SailboatDynamics()

        # Merwe scaled sigma-point parameters
        # alpha=0.5 gives moderate spread; kappa=3-n for Gaussian approx
        self.alpha = 0.5
        self.beta  = 2.0
        self.kappa  = max(0.0, 3.0 - self.n)
        self.lam   = self.alpha**2 * (self.n + self.kappa) - self.n

        # Weights
        self._compute_weights()

        # State & square-root covariance factor
        self.x = np.zeros(self.n)
        s0 = np.array([0.5, 0.5, 0.1, 0.2, 0.2, 0.05,
                        0.05, 0.05, 0.01, 0.01])
        self.S = np.diag(s0)

        # Process noise (Cholesky factor)
        q = np.array([0.001, 0.001, 0.001, 0.01, 0.01, 0.005,
                       0.005, 0.005, 0.001, 0.001])
        self.Sq = np.diag(np.sqrt(q))

        # Measurement noise (Cholesky factor)
        r = np.array([0.1, 0.1, 0.01, 0.02, 0.05, 0.02])
        self.Sr = np.diag(np.sqrt(r))

        # Reference for GPS conversion
        self.ref_lat = None
        self.ref_lon = None

        # Wind state (set externally each cycle)
        self.wind_speed = 0.0
        self.wind_angle = 0.0

    def _compute_weights(self):
        n = self.n
        lam = self.lam
        self.Wm = np.full(2 * n + 1, 1.0 / (2.0 * (n + lam)))
        self.Wc = np.full(2 * n + 1, 1.0 / (2.0 * (n + lam)))
        self.Wm[0] = lam / (n + lam)
        self.Wc[0] = lam / (n + lam) + (1 - self.alpha**2 + self.beta)

    def _sigma_points(self, x, S):
        """Generate 2n+1 sigma points from mean x and Cholesky factor S."""
        n = self.n
        sp = np.zeros((2 * n + 1, n))
        sp[0] = x
        gamma = np.sqrt(n + self.lam)
        for i in range(n):
            sp[i + 1]     = x + gamma * S[:, i]
            sp[n + i + 1] = x - gamma * S[:, i]
        return sp

    def _qr_update(self, A, v, sign=1.0):
        """Rank-1 Cholesky update/downdate with full overflow protection."""
        R = np.nan_to_num(A.copy(), nan=0.0, posinf=1e6, neginf=-1e6)
        x = np.nan_to_num(v.copy(), nan=0.0, posinf=1e6, neginf=-1e6)
        R = np.clip(R, -1e6, 1e6)
        x = np.clip(x, -1e6, 1e6)
        n = len(x)
        for k in range(n):
            arg = R[k, k]**2 + sign * x[k]**2
            if not np.isfinite(arg) or arg < 1e-12:
                arg = 1e-12
            rr = np.sqrt(arg)
            c = R[k, k] / rr
            s = x[k] / rr
            if not (np.isfinite(c) and np.isfinite(s)):
                continue
            R[k, k] = rr
            if k < n - 1:
                Rk_old = R[k, k+1:].copy()
                R[k, k+1:] = c * Rk_old + sign * s * x[k+1:]
                x[k+1:]    = c * x[k+1:] - s * Rk_old
                R[k, k+1:] = np.clip(R[k, k+1:], -1e6, 1e6)
                x[k+1:]    = np.clip(x[k+1:], -1e6, 1e6)
        return np.nan_to_num(R, nan=0.0, posinf=1e6, neginf=-1e6)

    # ── GPS helpers ──────────────────────────────────────────────────
    def set_reference(self, lat, lon):
        self.ref_lat = lat
        self.ref_lon = lon

    def latlon_to_xy(self, lat, lon):
        if self.ref_lat is None:
            self.set_reference(lat, lon)
            return 0.0, 0.0
        R = 6371000.0
        x = R * np.radians(lon - self.ref_lon) * np.cos(np.radians(self.ref_lat))
        y = R * np.radians(lat - self.ref_lat)
        return x, y

    def xy_to_latlon(self, x, y):
        if self.ref_lat is None:
            return 0.0, 0.0
        R = 6371000.0
        lat = self.ref_lat + np.degrees(y / R)
        lon = self.ref_lon + np.degrees(x / (R * np.cos(np.radians(self.ref_lat))))
        return lat, lon

    # ── Measurement model ────────────────────────────────────────────
    def h(self, x):
        """Measurement function: z = h(x).
        Returns [x_gps, y_gps, psi, phi, p, r]."""
        return np.array([x[0], x[1], x[2], x[6], x[7], x[5]])

    # ── Predict ──────────────────────────────────────────────────────
    def predict(self, dt):
        """UKF prediction step with SR factorisation."""
        # 1. Generate sigma points
        sp = self._sigma_points(self.x, self.S)

        # 2. Propagate through dynamics
        sp_pred = np.zeros_like(sp)
        for i in range(2 * self.n + 1):
            sp_pred[i] = self.dynamics.f(sp[i], dt,
                                         self.wind_speed, self.wind_angle)

        # 2b. Replace any NaN/Inf sigma points with the current mean
        for i in range(2 * self.n + 1):
            if not np.all(np.isfinite(sp_pred[i])):
                sp_pred[i] = self.x.copy()

        # 3. Predicted mean
        x_pred = np.zeros(self.n)
        for i in range(2 * self.n + 1):
            x_pred += self.Wm[i] * sp_pred[i]
        x_pred = np.nan_to_num(x_pred, nan=0.0, posinf=0.0, neginf=0.0)

        # 4. Square-root covariance via QR decomposition
        #    Build compound matrix [sqrt(Wc_i)*(sp_i - x_pred) | Sq]
        A = np.zeros((2 * self.n + self.n, self.n))
        for i in range(1, 2 * self.n + 1):
            A[i - 1] = np.sqrt(abs(self.Wc[i])) * (sp_pred[i] - x_pred)
        A[2 * self.n:] = self.Sq.T

        _, S_pred = _safe_qr(A)
        S_pred = S_pred[:self.n, :self.n]

        # Rank-1 update/downdate for the 0-th weight
        diff0 = sp_pred[0] - x_pred
        if self.Wc[0] >= 0:
            S_pred = self._qr_update(S_pred, np.sqrt(abs(self.Wc[0])) * diff0, sign=1.0)
        else:
            S_pred = self._qr_update(S_pred, np.sqrt(abs(self.Wc[0])) * diff0, sign=-1.0)

        self.x = x_pred
        self.S = S_pred
        self._sp_pred = sp_pred      # cache for update step

    # ── Update ───────────────────────────────────────────────────────
    def update(self, z):
        """UKF update step with SR factorisation.
        z is measurement vector (6,)."""
        sp_pred = self._sp_pred

        # 1. Transform sigma points through measurement model
        Z = np.zeros((2 * self.n + 1, self.m))
        for i in range(2 * self.n + 1):
            Z[i] = self.h(sp_pred[i])

        # 2. Predicted measurement mean
        z_pred = np.zeros(self.m)
        for i in range(2 * self.n + 1):
            z_pred += self.Wm[i] * Z[i]

        # 3. Innovation covariance square-root
        B = np.zeros((2 * self.n + self.m, self.m))
        for i in range(1, 2 * self.n + 1):
            B[i - 1] = np.sqrt(abs(self.Wc[i])) * (Z[i] - z_pred)
        B[2 * self.n:] = self.Sr.T

        _, Sz = _safe_qr(B)
        Sz = Sz[:self.m, :self.m]

        dz0 = Z[0] - z_pred
        if self.Wc[0] >= 0:
            Sz = self._qr_update(Sz, np.sqrt(abs(self.Wc[0])) * dz0, sign=1.0)
        else:
            Sz = self._qr_update(Sz, np.sqrt(abs(self.Wc[0])) * dz0, sign=-1.0)

        # 4. Cross-covariance (with NaN protection)
        Pxz = np.zeros((self.n, self.m))
        for i in range(2 * self.n + 1):
            dx = np.nan_to_num(sp_pred[i] - self.x, nan=0.0, posinf=0.0, neginf=0.0)
            dz = np.nan_to_num(Z[i] - z_pred, nan=0.0, posinf=0.0, neginf=0.0)
            Pxz += self.Wc[i] * np.outer(dx, dz)
        Pxz = np.nan_to_num(Pxz, nan=0.0, posinf=1e6, neginf=-1e6)

        # 5. Kalman gain with regularised inversion
        Sz = np.nan_to_num(Sz, nan=0.0, posinf=1e6, neginf=-1e6)
        Sz += np.eye(self.m) * 1e-6  # Tikhonov regularisation
        try:
            K = Pxz @ np.linalg.inv(Sz.T @ Sz)
        except np.linalg.LinAlgError:
            K = np.zeros((self.n, self.m))

        K = np.nan_to_num(K, nan=0.0, posinf=1.0, neginf=-1.0)
        K = np.clip(K, -100, 100)

        # 6. State update
        innovation = z - z_pred
        innovation = np.nan_to_num(innovation, nan=0.0, posinf=0.0, neginf=0.0)
        innovation[2] = np.arctan2(np.sin(innovation[2]), np.cos(innovation[2]))
        self.x = self.x + K @ innovation
        self.x = np.nan_to_num(self.x, nan=0.0, posinf=100.0, neginf=-100.0)

        # 7. Covariance update
        U = K @ Sz
        U = np.nan_to_num(U, nan=0.0, posinf=1e6, neginf=-1e6)
        S_new = self.S.copy()
        for j in range(self.m):
            S_new = self._qr_update(S_new, U[:, j], sign=-1.0)
        self.S = S_new

        # Normalise angles and clamp state
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))
        self.x[6] = np.clip(self.x[6], np.radians(-45), np.radians(45))
        self.x[3] = np.clip(self.x[3], -10, 10)
        self.x[4] = np.clip(self.x[4], -5, 5)
        self.x[5] = np.clip(self.x[5], -3, 3)
        self.x[7] = np.clip(self.x[7], -3, 3)

    # ── Convenience update methods ───────────────────────────────────
    def update_gps(self, lat, lon):
        """Partial update from GPS (x, y only)."""
        x_m, y_m = self.latlon_to_xy(lat, lon)
        z = self.h(self.x).copy()
        z[0] = x_m
        z[1] = y_m
        self.update(z)

    def update_imu(self, heading_rad, heel_rad, roll_rate, yaw_rate):
        """Full IMU update."""
        z = self.h(self.x).copy()
        z[2] = heading_rad
        z[3] = heel_rad
        z[4] = roll_rate
        z[5] = yaw_rate
        self.update(z)

    def update_compass(self, heading_rad):
        """Compass-only heading update."""
        z = self.h(self.x).copy()
        z[2] = heading_rad
        self.update(z)

    # ── Accessors ────────────────────────────────────────────────────
    def get_position(self):
        return self.xy_to_latlon(self.x[0], self.x[1])

    def get_heading_deg(self):
        h = np.degrees(self.x[2])
        return h if h >= 0 else h + 360

    def get_speed(self):
        return np.sqrt(self.x[3]**2 + self.x[4]**2)

    def get_heel_deg(self):
        return np.degrees(self.x[6])

    def get_leeway_deg(self):
        return np.degrees(np.arctan2(self.x[4], self.x[3] + 1e-6))

    def get_yaw_rate(self):
        return self.x[5]

    def get_covariance(self):
        return self.S @ self.S.T

    def set_wave_height(self, H_s, T_p=4.0):
        """Set sea state for wave excitation in dynamics model."""
        self.dynamics.wave_height = max(0.0, H_s)
        self.dynamics.wave_period = max(1.0, T_p)
