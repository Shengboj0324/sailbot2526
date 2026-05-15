/*╔══════════════════════════════════════════════════════════════════════════╗
 *║  SAILBOT 2526 — Phase-6 Production Firmware                            ║
 *║  Complete autonomous sailboat controller in a single Arduino file.     ║
 *║                                                                        ║
 *║  Target: Teensy 4.1 (ARM Cortex-M7 @ 600 MHz, 1 MB SRAM)             ║
 *║  Also compiles on Arduino Mega 2560 with reduced UKF (see #ifdef)     ║
 *║                                                                        ║
 *║  Modules included:                                                     ║
 *║    1. Square-Root UKF state estimator (10-state, 6-DOF dynamics)       ║
 *║       — Added mass, sail stall, rudder stall, wave excitation          ║
 *║    2. MPC steering controller (Nomoto yaw model, greedy search)        ║
 *║       — Obstacle avoidance, heel penalty, slew-rate constraints        ║
 *║    3. VPP sail optimizer (aero/hydro equilibrium)                       ║
 *║       — Viterna stall model, wave-added resistance, heel constraint    ║
 *║    4. UKF-aware drift/current estimator with leeway correction         ║
 *║    5. Health monitor with graded severity (HEALTHY→FALLBACK)           ║
 *║    6. Sensor drivers: GPS NMEA, BNO085 IMU, wind vane                  ║
 *║    7. Actuators: rudder servo (±21°), winch stepper (0–88°)            ║
 *║    8. Waypoint navigation with wind-shift replanning                   ║
 *║                                                                        ║
 *║  Serial protocol (matches util/rudder_a.py & winch_a.py):             ║
 *║    Rudder: [0x20, servo_angle]                                         ║
 *║    Winch CW:  [0x12, step3, step2, step1, step0]  (big-endian)        ║
 *║    Winch CCW: [0x13, step3, step2, step1, step0]                      ║
 *║                                                                        ║
 *║  Build: Arduino IDE ≥ 2.0 with Teensyduino, or PlatformIO             ║
 *║  Libs:  Wire.h (I2C), Servo.h, AccelStepper.h                         ║
 *╚══════════════════════════════════════════════════════════════════════════╝*/

#include <math.h>
#include <string.h>
#include <Servo.h>
#include <Wire.h>

// ─── Platform detection ────────────────────────────────────────────────
#if defined(__IMXRT1062__)  // Teensy 4.x
  #define PLATFORM_TEENSY4
  #define UKF_N 10
  #define MPC_HORIZON 10
  #define MPC_CANDIDATES 7
  #define VPP_SCAN_STEP 2
#elif defined(__AVR_ATmega2560__)
  #define PLATFORM_MEGA
  #define UKF_N 6            // Reduced state on Mega (x,y,psi,u,v,r)
  #define MPC_HORIZON 5
  #define MPC_CANDIDATES 5
  #define VPP_SCAN_STEP 4
#else
  #define PLATFORM_TEENSY4   // Default to Teensy
  #define UKF_N 10
  #define MPC_HORIZON 10
  #define MPC_CANDIDATES 7
  #define VPP_SCAN_STEP 2
#endif

// ─── Pin Definitions ───────────────────────────────────────────────────
// Teensy 4.1 pin assignments (adjust for your wiring)
#define RUDDER_SERVO_PIN   2     // PWM-capable pin for rudder servo
#define STEPPER_STEP_PIN   3     // Stepper motor STEP pin
#define STEPPER_DIR_PIN    4     // Stepper motor DIR pin
#define STEPPER_EN_PIN     5     // Stepper motor ENABLE (active LOW)
#define WIND_VANE_PIN      A0    // Analog wind direction sensor
#define GPS_SERIAL         Serial1  // Hardware UART for GPS
#define GPS_BAUD           38400
#define DEBUG_SERIAL       Serial   // USB debug output
#define DEBUG_BAUD         115200
#define CMD_SERIAL         Serial2  // Command serial (from Jetson/RasPi)
#define CMD_BAUD           115200

// ─── Serial Protocol Command Bytes ─────────────────────────────────────
#define CMD_SERVO          0x20
#define CMD_WINCH_CW       0x12
#define CMD_WINCH_CCW      0x13

// ─── Physical Constants ────────────────────────────────────────────────
static const float RHO_AIR   = 1.225f;
static const float RHO_WATER = 1025.0f;
static const float GRAVITY   = 9.81f;
static const float DEG2RAD   = 0.017453293f;
static const float RAD2DEG   = 57.29577951f;
static const float PI_F      = 3.14159265f;
static const float TWO_PI_F  = 6.28318530f;

// ─── Hull Parameters ───────────────────────────────────────────────────
static const float BOAT_MASS     = 25.0f;    // kg
static const float I_ZZ          = 8.0f;     // yaw inertia (kg·m²)
static const float I_XX          = 4.0f;     // roll inertia (kg·m²)
static const float HULL_LEN      = 1.2f;     // m
static const float BEAM          = 0.40f;    // m
static const float DRAFT         = 0.30f;    // m
static const float SAIL_AREA     = 0.80f;    // m²
static const float RUDDER_AREA   = 0.02f;    // m²
static const float KEEL_AREA     = 0.06f;    // m²
static const float COG_Z         = -0.05f;   // CoG height (neg = below WL)
static const float METACENTRIC_H = 0.25f;    // metacentric height (m)

// ─── Hydrodynamic Coefficients ─────────────────────────────────────────
static const float Xu  = -4.0f,  Xuu = -6.0f;
static const float Yv  = -20.0f, Yvv = -40.0f;
static const float Nr  = -3.0f,  Nrr = -6.0f;
static const float Kp  = -5.0f,  Kpp = -10.0f;

// ─── Added Mass (Fossen) ──────────────────────────────────────────────
static const float MA_SURGE = 2.0f;
static const float MA_SWAY  = 15.0f;
static const float IA_YAW   = 2.0f;
static const float IA_ROLL  = 1.0f;

// ─── Stall Model ──────────────────────────────────────────────────────
static const float ALPHA_STALL_SAIL   = 0.2618f;  // 15° in rad
static const float CL_MAX_SAIL       = 1.2f;
static const float CD_MAX_SAIL       = 1.8f;
static const float STALL_WIDTH_SAIL  = 0.0873f;   // 5° in rad
static const float ALPHA_STALL_RUD   = 0.2618f;   // 15° in rad
static const float CL_MAX_RUD        = 0.9f;

// ─── Actuator Limits ──────────────────────────────────────────────────
static const float RUDDER_MIN_DEG    = -21.0f;
static const float RUDDER_MAX_DEG    =  21.0f;
static const float RUDDER_RATE_MAX   =  15.0f;    // °/s
static const int   NEUTRAL_SERVO     = 55;
static const float MAX_SAIL_ANGLE    = 88.0f;
static const float SAIL_RATE_MAX     = 10.0f;     // °/s

// ─── Winch Geometry (from winch_a.py) ─────────────────────────────────
static const float BOOM_LENGTH     = 48.0f;
static const float WINCH_TO_MAST   = 44.0f;
static const float SPOOL_RADIUS    = 1.5f;
static const float GEAR_RATIO      = 5.0f;
static const int   STEPS_PER_REV   = 1600;

// ─── VPP Hull Resistance ──────────────────────────────────────────────
static const float WETTED_AREA     = 0.60f;   // m²
static const float CF_HULL         = 0.004f;  // ITTC-57 friction
static const float K_WAVE          = 0.15f;   // wave-making coeff
static const float C_AW            = 0.02f;   // added wave resistance coeff

// ─── MPC Weights ──────────────────────────────────────────────────────
static const float Q_PSI   = 10.0f;
static const float Q_R     = 1.0f;
static const float R_DR    = 5.0f;
static const float Q_PSI_F = 20.0f;
static const float Q_OBS   = 500.0f;
static const float Q_HEEL  = 50.0f;

// ─── Nomoto Model ─────────────────────────────────────────────────────
static const float NOMOTO_K = 0.30f;
static const float NOMOTO_T = 2.5f;

// ─── Navigation ───────────────────────────────────────────────────────
static const int   MAX_WAYPOINTS    = 20;
static const int   MAX_OBSTACLES    = 10;
static const float WPT_RADIUS       = 5.0f;   // waypoint arrival radius (m)
static const float WIND_SHIFT_THRESH = 30.0f;  // degrees
static const float EARTH_RADIUS     = 6371000.0f;

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 1: Fixed-Size Matrix Utilities (UKF needs 10×10 max)
// ═══════════════════════════════════════════════════════════════════════

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}
static inline float wrap_angle(float a) {
    return atan2f(sinf(a), cosf(a));
}
static inline float absf(float x) { return (x < 0) ? -x : x; }

// Matrix stored as flat array, row-major: A[i*cols + j]
// We only need NxN operations where N ≤ 10
#define MAT(A, r, c, cols) ((A)[(r)*(cols) + (c)])

// Zero a matrix
static void mat_zero(float* A, int rows, int cols) {
    memset(A, 0, rows * cols * sizeof(float));
}

// Identity matrix
static void mat_eye(float* A, int n) {
    mat_zero(A, n, n);
    for (int i = 0; i < n; i++) MAT(A, i, i, n) = 1.0f;
}

// Copy matrix
static void mat_copy(float* dst, const float* src, int rows, int cols) {
    memcpy(dst, src, rows * cols * sizeof(float));
}

// A = A + B (in-place)
static void mat_add(float* A, const float* B, int rows, int cols) {
    for (int i = 0; i < rows * cols; i++) A[i] += B[i];
}

// C = A * B  (A is m×k, B is k×n, C is m×n)
static void mat_mul(float* C, const float* A, const float* B,
                    int m, int k, int n) {
    mat_zero(C, m, n);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            for (int p = 0; p < k; p++)
                MAT(C, i, j, n) += MAT(A, i, p, k) * MAT(B, p, j, n);
}

// Transpose: B = A^T  (A is m×n, B is n×m)
static void mat_transpose(float* B, const float* A, int m, int n) {
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            MAT(B, j, i, m) = MAT(A, i, j, n);
}

// In-place Cholesky decomposition of a symmetric PD matrix A (n×n)
// Returns lower-triangular L such that A = L * L^T
// Returns false if matrix is not PD
static bool cholesky(float* L, const float* A, int n) {
    mat_copy(L, A, n, n);
    for (int j = 0; j < n; j++) {
        float sum = 0;
        for (int k = 0; k < j; k++) sum += MAT(L, j, k, n) * MAT(L, j, k, n);
        float diag = MAT(L, j, j, n) - sum;
        if (diag < 1e-10f) diag = 1e-10f;  // regularise
        MAT(L, j, j, n) = sqrtf(diag);
        for (int i = j + 1; i < n; i++) {
            float s = 0;
            for (int k = 0; k < j; k++) s += MAT(L, i, k, n) * MAT(L, j, k, n);
            MAT(L, i, j, n) = (MAT(L, i, j, n) - s) / MAT(L, j, j, n);
        }
        for (int i = 0; i < j; i++) MAT(L, i, j, n) = 0;
    }
    return true;
}

// Rank-1 Cholesky update: L ← chol(L*L^T + sign * v*v^T)
// sign = +1 for update, -1 for downdate
static void cholesky_rank1(float* L, const float* v, int n, float sign) {
    float x[UKF_N];
    memcpy(x, v, n * sizeof(float));
    for (int k = 0; k < n; k++) {
        float Lkk = MAT(L, k, k, n);
        float arg = Lkk * Lkk + sign * x[k] * x[k];
        if (arg < 1e-12f) arg = 1e-12f;
        float rr = sqrtf(arg);
        float c = Lkk / rr;
        float s = x[k] / rr;
        MAT(L, k, k, n) = rr;
        for (int i = k + 1; i < n; i++) {
            float Lik_old = MAT(L, i, k, n);
            MAT(L, i, k, n) = c * Lik_old + sign * s * x[i];
            x[i] = c * x[i] - s * Lik_old;
        }
    }
}

// Solve L*x = b for lower-triangular L (forward substitution)
static void solve_lower(float* x, const float* L, const float* b, int n) {
    for (int i = 0; i < n; i++) {
        float sum = b[i];
        for (int j = 0; j < i; j++) sum -= MAT(L, i, j, n) * x[j];
        x[i] = sum / (MAT(L, i, i, n) + 1e-12f);
    }
}

// Solve L^T * x = b for lower-triangular L (back substitution)
static void solve_upper(float* x, const float* L, const float* b, int n) {
    for (int i = n - 1; i >= 0; i--) {
        float sum = b[i];
        for (int j = i + 1; j < n; j++) sum -= MAT(L, j, i, n) * x[j];
        x[i] = sum / (MAT(L, i, i, n) + 1e-12f);
    }
}

// Solve (L*L^T) * x = b  using forward/back substitution
static void solve_chol(float* x, const float* L, const float* b, int n) {
    float y[UKF_N];
    solve_lower(y, L, b, n);
    solve_upper(x, L, y, n);
}


// ═══════════════════════════════════════════════════════════════════════
//  SECTION 2: Data Structures
// ═══════════════════════════════════════════════════════════════════════

struct Waypoint {
    float lat, lon;
};

struct Obstacle {
    float x, y;       // position in local frame (m)
    float radius;     // safety buffer (m)
    float vx, vy;     // velocity (m/s) for moving obstacles
};

// ── Sensor readings ────────────────────────────────────────────────────
struct SensorData {
    // GPS
    float gps_lat, gps_lon;
    float gps_speed;            // m/s
    float gps_course;           // degrees
    bool  gps_valid;
    unsigned long gps_last_ms;

    // IMU (BNO085)
    float imu_heading_rad;      // yaw (rad)
    float imu_heel_rad;         // roll (rad)
    float imu_roll_rate;        // p (rad/s)
    float imu_yaw_rate;         // r (rad/s)
    bool  imu_valid;
    unsigned long imu_last_ms;

    // Wind
    float wind_dir_deg;         // true wind direction (°)
    float wind_speed_mps;       // true wind speed (m/s, estimated)
    bool  wind_valid;
    unsigned long wind_last_ms;
};

// ── System state ───────────────────────────────────────────────────────
enum HealthGrade { HEALTHY, DEGRADED, CRITICAL, FALLBACK };

struct SystemState {
    // UKF state vector [x,y,ψ,u,v,r,φ,p,δ_r,δ_s]
    float x[UKF_N];
    float S[UKF_N * UKF_N];    // Cholesky factor of covariance

    // Actuator outputs
    float rudder_deg;           // commanded rudder angle (°)
    float sail_deg;             // commanded sail angle (°)
    float current_sail_deg;     // actual tracked sail position

    // Navigation
    Waypoint waypoints[MAX_WAYPOINTS];
    int      n_waypoints;
    int      current_wpt;
    bool     nav_enabled;

    // Obstacles
    Obstacle obstacles[MAX_OBSTACLES];
    int      n_obstacles;

    // Wind tracking
    float last_plan_wind;

    // MPC
    float last_mpc_rudder_rad;  // previous MPC output

    // Drift estimator
    float drift_vx_buf[20];
    float drift_vy_buf[20];
    int   drift_buf_idx;
    int   drift_buf_count;
    float drift_speed;
    float drift_direction;
    float drift_confidence;

    // Health
    HealthGrade health;
    float ukf_trace;

    // GPS reference for local frame
    float ref_lat, ref_lon;
    bool  ref_set;

    // Timing
    unsigned long last_control_ms;
    unsigned long last_nav_ms;
    unsigned long last_ukf_ms;

    // Wave state
    float wave_height;
    float wave_period;
    float wave_phase;
};

static SensorData sensors;
static SystemState state;
static Servo rudderServo;

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 3: Sensor Drivers
// ═══════════════════════════════════════════════════════════════════════

// ── GPS NMEA Parser ────────────────────────────────────────────────────
static char gps_buf[128];
static int  gps_buf_idx = 0;

static float nmea_to_decimal(const char* raw, char dir) {
    // NMEA: DDDMM.MMMMM → decimal degrees
    float val = atof(raw);
    int   deg = (int)(val / 100);
    float min = val - deg * 100.0f;
    float dec = deg + min / 60.0f;
    if (dir == 'S' || dir == 'W') dec = -dec;
    return dec;
}

static void parse_gga(char* sentence) {
    // $GPGGA,time,lat,N,lon,W,quality,sats,hdop,alt,...
    char* tok[15];
    int n = 0;
    tok[n++] = strtok(sentence, ",");
    while (n < 15 && (tok[n] = strtok(NULL, ",")) != NULL) n++;
    if (n >= 7) {
        int quality = atoi(tok[6]);
        if (quality > 0 && strlen(tok[2]) > 0 && strlen(tok[4]) > 0) {
            sensors.gps_lat = nmea_to_decimal(tok[2], tok[3][0]);
            sensors.gps_lon = nmea_to_decimal(tok[4], tok[5][0]);
            sensors.gps_valid = true;
            sensors.gps_last_ms = millis();
        }
    }
}

static void parse_rmc(char* sentence) {
    // $GPRMC,time,status,lat,N,lon,W,speed_knots,course,...
    char* tok[15];
    int n = 0;
    tok[n++] = strtok(sentence, ",");
    while (n < 15 && (tok[n] = strtok(NULL, ",")) != NULL) n++;
    if (n >= 8 && tok[2][0] == 'A') {
        sensors.gps_lat = nmea_to_decimal(tok[3], tok[4][0]);
        sensors.gps_lon = nmea_to_decimal(tok[5], tok[6][0]);
        sensors.gps_speed = atof(tok[7]) * 0.51444f;  // knots → m/s
        if (n >= 9) sensors.gps_course = atof(tok[8]);
        sensors.gps_valid = true;
        sensors.gps_last_ms = millis();
    }
}

static void gps_read() {
    while (GPS_SERIAL.available()) {
        char c = GPS_SERIAL.read();
        if (c == '$') {
            gps_buf_idx = 0;
        }
        if (gps_buf_idx < (int)sizeof(gps_buf) - 1) {
            gps_buf[gps_buf_idx++] = c;
        }
        if (c == '\n' && gps_buf_idx > 5) {
            gps_buf[gps_buf_idx] = '\0';
            if (strstr(gps_buf, "GGA")) parse_gga(gps_buf);
            else if (strstr(gps_buf, "RMC")) parse_rmc(gps_buf);
            gps_buf_idx = 0;
        }
    }
    // Timeout: mark invalid after 3 seconds
    if (millis() - sensors.gps_last_ms > 3000) sensors.gps_valid = false;
}

// ── BNO085 IMU via I2C ─────────────────────────────────────────────────
// Simplified BNO085 driver — reads rotation vector + gyro
// For production use adafruit_bno08x library; this reads raw I2C reports
#define BNO085_ADDR 0x4A

static void imu_init() {
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz I2C
    // Enable rotation vector report (Report ID 0x05) at 50 Hz
    uint8_t cmd[] = {21, 0, 2, 0, 0xFD, 0x05, 0, 0, 0, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    Wire.beginTransmission(BNO085_ADDR);
    Wire.write(cmd, sizeof(cmd));
    Wire.endTransmission();
    delay(100);
    // Enable gyroscope report (Report ID 0x02) at 50 Hz
    cmd[5] = 0x02;
    Wire.beginTransmission(BNO085_ADDR);
    Wire.write(cmd, sizeof(cmd));
    Wire.endTransmission();
}

static void imu_read() {
    // Read available data from BNO085
    // In production, use the Adafruit BNO08x library for proper SHTP parsing
    // This is a simplified polling approach
    Wire.requestFrom(BNO085_ADDR, 23);
    if (Wire.available() >= 23) {
        uint8_t buf[23];
        for (int i = 0; i < 23; i++) buf[i] = Wire.read();

        // Check for rotation vector report (ID 0x05)
        if (buf[9] == 0x05 && Wire.available() >= 14) {
            // Quaternion: i, j, k, real (Q14 fixed point)
            int16_t qi = (int16_t)(buf[14] | (buf[15] << 8));
            int16_t qj = (int16_t)(buf[16] | (buf[17] << 8));
            int16_t qk = (int16_t)(buf[18] | (buf[19] << 8));
            int16_t qr = (int16_t)(buf[20] | (buf[21] << 8));
            float scale = 1.0f / 16384.0f;  // Q14
            float w = qr * scale, x = qi * scale, y = qj * scale, z = qk * scale;

            // Quaternion → Euler (yaw, pitch, roll)
            sensors.imu_heading_rad = atan2f(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));
            sensors.imu_heel_rad = asinf(clampf(2.0f * (w*y - z*x), -1.0f, 1.0f));
            sensors.imu_valid = true;
            sensors.imu_last_ms = millis();
        }
        // Check for gyroscope report (ID 0x02)
        if (buf[9] == 0x02) {
            int16_t gx = (int16_t)(buf[14] | (buf[15] << 8));
            int16_t gy = (int16_t)(buf[16] | (buf[17] << 8));
            int16_t gz = (int16_t)(buf[18] | (buf[19] << 8));
            float gscale = 1.0f / 16384.0f;
            sensors.imu_roll_rate = gx * gscale;  // rad/s
            sensors.imu_yaw_rate  = gz * gscale;   // rad/s
        }
    }
    if (millis() - sensors.imu_last_ms > 1000) sensors.imu_valid = false;
}

// ── Wind Sensor (analog vane) ──────────────────────────────────────────
static void wind_read() {
    int raw = analogRead(WIND_VANE_PIN);
    // Map ADC (0-1023 on Mega, 0-4095 on Teensy) to 0-360°
#ifdef PLATFORM_TEENSY4
    sensors.wind_dir_deg = (float)raw * 360.0f / 4095.0f;
#else
    sensors.wind_dir_deg = (float)raw * 360.0f / 1023.0f;
#endif
    // Wind speed estimation: if no anemometer, use GPS speed + apparent wind
    // For now, use a reasonable default that can be overridden via serial
    if (sensors.wind_speed_mps < 0.1f) sensors.wind_speed_mps = 5.0f;
    sensors.wind_valid = true;
    sensors.wind_last_ms = millis();
}

// ── GPS coordinate helpers ─────────────────────────────────────────────
static void latlon_to_xy(float lat, float lon, float* x, float* y) {
    if (!state.ref_set) {
        state.ref_lat = lat;
        state.ref_lon = lon;
        state.ref_set = true;
        *x = 0; *y = 0;
        return;
    }
    *x = EARTH_RADIUS * (lon - state.ref_lon) * DEG2RAD * cosf(state.ref_lat * DEG2RAD);
    *y = EARTH_RADIUS * (lat - state.ref_lat) * DEG2RAD;
}

static void xy_to_latlon(float x, float y, float* lat, float* lon) {
    *lat = state.ref_lat + y / EARTH_RADIUS * RAD2DEG;
    *lon = state.ref_lon + x / (EARTH_RADIUS * cosf(state.ref_lat * DEG2RAD)) * RAD2DEG;
}

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 4: UKF State Estimator
// ═══════════════════════════════════════════════════════════════════════

// ── Sail aero with post-stall (Viterna) ────────────────────────────────
static void sail_aero(float alpha, float V_app, float* Fx, float* Fy) {
    float abs_a = fabsf(alpha);
    // Pre-stall
    float CL_pre = 2.0f * PI_F * sinf(alpha) * 0.75f;
    CL_pre = clampf(CL_pre, -CL_MAX_SAIL, CL_MAX_SAIL);
    float CD_pre = 0.05f + CL_pre * CL_pre / (PI_F * 3.0f * 0.85f);
    // Post-stall (Viterna)
    float CL_post = CL_MAX_SAIL * sinf(2.0f * alpha);
    float CD_stall = 0.05f + CL_MAX_SAIL * CL_MAX_SAIL / (PI_F * 3.0f * 0.85f);
    float CD_post = CD_MAX_SAIL - (CD_MAX_SAIL - CD_stall) * cosf(alpha) * cosf(alpha);
    // Smooth blend
    float blend = 0.5f * (1.0f + tanhf((abs_a - ALPHA_STALL_SAIL) / (STALL_WIDTH_SAIL + 1e-9f)));
    float CL = (1.0f - blend) * CL_pre + blend * CL_post;
    float CD = (1.0f - blend) * CD_pre + blend * CD_post;

    float q = 0.5f * RHO_AIR * V_app * V_app * SAIL_AREA;
    *Fx = q * (CL * sinf(fabsf(alpha)) - CD * cosf(alpha));
    *Fy = q * (CL * cosf(alpha) + CD * sinf(fabsf(alpha)));
}

// ── Rudder force with stall ────────────────────────────────────────────
static void rudder_force(float rudder_aoa, float V_water,
                         float* F_rud_y, float* N_rud) {
    float q = 0.5f * RHO_WATER * V_water * V_water * RUDDER_AREA;
    float CL;
    if (fabsf(rudder_aoa) < ALPHA_STALL_RUD) {
        CL = TWO_PI_F * rudder_aoa;
    } else {
        float sign = (rudder_aoa > 0) ? 1.0f : -1.0f;
        CL = sign * CL_MAX_RUD * sinf(2.0f * rudder_aoa);
    }
    *F_rud_y = q * CL;
    *N_rud = -(*F_rud_y) * 0.5f * HULL_LEN;
}

// ── Wave excitation forces ─────────────────────────────────────────────
static void wave_forces(float dt, float* Fxw, float* Fyw, float* Nw, float* Kw) {
    if (state.wave_height <= 0) { *Fxw=0; *Fyw=0; *Nw=0; *Kw=0; return; }
    float omega = TWO_PI_F / fmaxf(state.wave_period, 1.0f);
    state.wave_phase += omega * dt;
    float amp = state.wave_height;
    float ph = state.wave_phase;
    *Fxw = 0.5f * RHO_WATER * GRAVITY * BEAM * DRAFT * amp * cosf(ph) * 0.1f;
    *Fyw = 0.3f * RHO_WATER * GRAVITY * HULL_LEN * DRAFT * amp * sinf(ph) * 0.1f;
    *Nw  = 0.05f * RHO_WATER * GRAVITY * HULL_LEN * HULL_LEN * DRAFT * amp * sinf(ph + 0.5f) * 0.1f;
    *Kw  = 0.4f * RHO_WATER * GRAVITY * BEAM * DRAFT * amp * sinf(ph + 1.0f) * 0.1f;
}

// ── 6-DOF dynamics propagation ─────────────────────────────────────────
// Full 10-state: [x,y,psi,u,v,r,phi,p,dr,ds]
static void dynamics_f(float* xn, const float* xc, float dt,
                       float wind_speed, float wind_angle) {
    float px = xc[0], py = xc[1], psi = xc[2];
    float u = clampf(xc[3], -10, 10);
    float v = clampf(xc[4], -5, 5);
    float r = clampf(xc[5], -3, 3);
#if UKF_N == 10
    float phi = clampf(xc[6], -0.8f, 0.8f);
    float p   = clampf(xc[7], -3, 3);
    float dr  = xc[8], ds = xc[9];
#else
    float phi = 0, p = 0, dr = state.x[8], ds = state.x[9];
#endif

    float cpsi = cosf(psi), spsi = sinf(psi);
    float cphi = cosf(phi);

    // Effective inertia (rigid body + added mass)
    float m_surge = BOAT_MASS + MA_SURGE;
    float m_sway  = BOAT_MASS + MA_SWAY;
    float Iz_eff  = I_ZZ + IA_YAW;
    float Ix_eff  = I_XX + IA_ROLL;

    // Apparent wind
    float aw_x = wind_speed * cosf(wind_angle - psi) - u;
    float aw_y = wind_speed * sinf(wind_angle - psi) - v;
    float V_app = sqrtf(aw_x * aw_x + aw_y * aw_y) + 1e-6f;
    float alpha = atan2f(aw_y, aw_x) - ds;

    // Sail forces (with stall)
    float F_sail_x, F_sail_y;
    sail_aero(alpha, V_app, &F_sail_x, &F_sail_y);

    // Keel side-force
    float V_water = sqrtf(u * u + v * v) + 1e-6f;
    float leeway = atan2f(v, u + 1e-6f);
    float CL_keel = TWO_PI_F * leeway;
    float q_keel = 0.5f * RHO_WATER * V_water * V_water * KEEL_AREA;
    float F_keel_y = -q_keel * CL_keel;

    // Rudder force (with stall)
    float rud_aoa = dr - atan2f(v + r * 0.5f * HULL_LEN, u + 1e-6f);
    float F_rud_y_val, N_rud_val;
    rudder_force(rud_aoa, V_water, &F_rud_y_val, &N_rud_val);

    // Hull drag
    float Fx_hull = Xu * u + Xuu * u * fabsf(u);
    float Fy_hull = Yv * v + Yvv * v * fabsf(v);

    // Wave excitation
    float Fxw, Fyw, Nw, Kw;
    wave_forces(dt, &Fxw, &Fyw, &Nw, &Kw);

    // Total forces
    float Fx = F_sail_x + Fx_hull + Fxw;
    float Fy = F_sail_y + F_keel_y + F_rud_y_val + Fy_hull + Fyw;
    float Nz = N_rud_val + Nr * r + Nrr * r * fabsf(r) + Nw;
    float K_aero = F_sail_y * COG_Z;
    float K_hydro = -BOAT_MASS * GRAVITY * METACENTRIC_H * sinf(phi);
    float K_damp = Kp * p + Kpp * p * fabsf(p);
    float Kx = K_aero + K_hydro + K_damp + Kw;

    // Accelerations (Fossen notation with added-mass Coriolis)
    float u_dot = Fx / m_surge + (m_sway / m_surge) * v * r;
    float v_dot = Fy / m_sway  - (m_surge / m_sway) * u * r;
    float r_dot = Nz / Iz_eff;
    float p_dot = Kx / Ix_eff;

    // Euler integration
    xn[0] = px + (u * cpsi - v * spsi) * cphi * dt;
    xn[1] = py + (u * spsi + v * cpsi) * cphi * dt;
    xn[2] = wrap_angle(psi + r * dt);
    xn[3] = clampf(u + u_dot * dt, -10, 10);
    xn[4] = clampf(v + v_dot * dt, -5, 5);
    xn[5] = clampf(r + r_dot * dt, -3, 3);
#if UKF_N == 10
    xn[6] = clampf(phi + p * dt, -0.785f, 0.785f);
    xn[7] = clampf(p + p_dot * dt, -3, 3);
    xn[8] = dr;
    xn[9] = ds;
#endif
    // NaN guard
    for (int i = 0; i < UKF_N; i++)
        if (!isfinite(xn[i])) xn[i] = xc[i];
}

// ── UKF Sigma-Point Predict & Update ───────────────────────────────────
// Van der Merwe Square-Root UKF
#define UKF_L     UKF_N
#define UKF_NSIG  (2 * UKF_L + 1)
#define UKF_ALPHA 0.01f
#define UKF_BETA  2.0f
#define UKF_KAPPA 0.0f
#define UKF_LAMBDA (UKF_ALPHA * UKF_ALPHA * (UKF_L + UKF_KAPPA) - UKF_L)

#define UKF_M 6  // measurement dimension

// Process noise diagonal
static const float ukf_q_diag[UKF_N] = {
#if UKF_N == 10
    0.5f, 0.5f, 0.02f, 0.3f, 0.2f, 0.05f, 0.02f, 0.05f, 0.001f, 0.001f
#else
    0.5f, 0.5f, 0.02f, 0.3f, 0.2f, 0.05f
#endif
};

// Measurement noise diagonal [x_gps, y_gps, psi, phi, p, r]
static const float ukf_r_diag[UKF_M] = {
    3.0f, 3.0f, 0.05f, 0.02f, 0.01f, 0.01f
};

static float ukf_sigmas[UKF_NSIG][UKF_N];   // sigma points
static float ukf_sigmas_z[UKF_NSIG][UKF_M]; // transformed measurement sigma points
static float ukf_wm[UKF_NSIG];              // mean weights
static float ukf_wc[UKF_NSIG];              // covariance weights

static void ukf_init() {
    memset(state.x, 0, sizeof(state.x));
    mat_zero(state.S, UKF_N, UKF_N);
    // Initial S = diag of initial uncertainty
    float init_std[UKF_N] = {
#if UKF_N == 10
        10, 10, 0.5f, 1, 0.5f, 0.2f, 0.1f, 0.1f, 0.01f, 0.01f
#else
        10, 10, 0.5f, 1, 0.5f, 0.2f
#endif
    };
    for (int i = 0; i < UKF_N; i++) MAT(state.S, i, i, UKF_N) = init_std[i];

    // Compute weights
    float c = UKF_L + UKF_LAMBDA;
    ukf_wm[0] = UKF_LAMBDA / c;
    ukf_wc[0] = UKF_LAMBDA / c + (1.0f - UKF_ALPHA * UKF_ALPHA + UKF_BETA);
    for (int i = 1; i < UKF_NSIG; i++) {
        ukf_wm[i] = 0.5f / c;
        ukf_wc[i] = 0.5f / c;
    }
}

static void ukf_predict(float dt) {
    float gamma = sqrtf((float)UKF_L + UKF_LAMBDA);

    // Generate sigma points: χ₀ = x, χᵢ = x + γ·S[:,i], χ_{n+i} = x - γ·S[:,i]
    for (int j = 0; j < UKF_N; j++) ukf_sigmas[0][j] = state.x[j];
    for (int i = 0; i < UKF_N; i++) {
        for (int j = 0; j < UKF_N; j++) {
            float col = gamma * MAT(state.S, j, i, UKF_N);
            ukf_sigmas[1 + i][j]         = state.x[j] + col;
            ukf_sigmas[1 + UKF_N + i][j] = state.x[j] - col;
        }
    }

    // Propagate each sigma point through dynamics
    float ws = sensors.wind_speed_mps;
    float wa = sensors.wind_dir_deg * DEG2RAD;
    for (int i = 0; i < UKF_NSIG; i++) {
        float tmp[UKF_N];
        dynamics_f(tmp, ukf_sigmas[i], dt, ws, wa);
        memcpy(ukf_sigmas[i], tmp, sizeof(tmp));
    }

    // Predicted mean
    memset(state.x, 0, sizeof(state.x));
    for (int i = 0; i < UKF_NSIG; i++)
        for (int j = 0; j < UKF_N; j++)
            state.x[j] += ukf_wm[i] * ukf_sigmas[i][j];
    state.x[2] = wrap_angle(state.x[2]);

    // Predicted covariance via QR decomposition (simplified rank-1 updates)
    // Start from Q (process noise)
    mat_zero(state.S, UKF_N, UKF_N);
    for (int i = 0; i < UKF_N; i++)
        MAT(state.S, i, i, UKF_N) = sqrtf(fabsf(ukf_q_diag[i]));

    // Rank-1 update for each sigma point
    for (int i = 1; i < UKF_NSIG; i++) {
        float diff[UKF_N];
        for (int j = 0; j < UKF_N; j++) diff[j] = ukf_sigmas[i][j] - state.x[j];
        diff[2] = wrap_angle(diff[2]);
        float w = fabsf(ukf_wc[i]);
        float sqrt_w = sqrtf(w);
        for (int j = 0; j < UKF_N; j++) diff[j] *= sqrt_w;
        cholesky_rank1(state.S, diff, UKF_N, 1.0f);
    }
    // Handle w[0] (can be negative → downdate)
    {
        float diff[UKF_N];
        for (int j = 0; j < UKF_N; j++) diff[j] = ukf_sigmas[0][j] - state.x[j];
        diff[2] = wrap_angle(diff[2]);
        float w0 = ukf_wc[0];
        float sign = (w0 >= 0) ? 1.0f : -1.0f;
        float sqrt_w = sqrtf(fabsf(w0));
        for (int j = 0; j < UKF_N; j++) diff[j] *= sqrt_w;
        cholesky_rank1(state.S, diff, UKF_N, sign);
    }

    // Compute trace for health monitoring
    state.ukf_trace = 0;
    for (int i = 0; i < UKF_N; i++)
        state.ukf_trace += MAT(state.S, i, i, UKF_N) * MAT(state.S, i, i, UKF_N);
}

// ── Measurement model: h(x) → z = [x_gps, y_gps, psi, phi, p, r] ──
static void measurement_h(float* z_out, const float* x_in) {
    z_out[0] = x_in[0];  // x position
    z_out[1] = x_in[1];  // y position
    z_out[2] = x_in[2];  // heading
#if UKF_N == 10
    z_out[3] = x_in[6];  // heel
    z_out[4] = x_in[7];  // roll rate
    z_out[5] = x_in[5];  // yaw rate
#else
    z_out[3] = 0;
    z_out[4] = 0;
    z_out[5] = x_in[5];
#endif
}

static void ukf_update(const float* z_meas) {
    // Transform sigma points through measurement model
    float z_pred[UKF_M];
    memset(z_pred, 0, sizeof(z_pred));

    for (int i = 0; i < UKF_NSIG; i++) {
        measurement_h(ukf_sigmas_z[i], ukf_sigmas[i]);
        for (int j = 0; j < UKF_M; j++)
            z_pred[j] += ukf_wm[i] * ukf_sigmas_z[i][j];
    }
    z_pred[2] = wrap_angle(z_pred[2]);

    // Innovation covariance Sz (square-root)
    float Sz[UKF_M * UKF_M];
    mat_zero(Sz, UKF_M, UKF_M);
    for (int i = 0; i < UKF_M; i++) Sz[i * UKF_M + i] = sqrtf(ukf_r_diag[i]);

    for (int i = 1; i < UKF_NSIG; i++) {
        float dz[UKF_M];
        for (int j = 0; j < UKF_M; j++) dz[j] = ukf_sigmas_z[i][j] - z_pred[j];
        dz[2] = wrap_angle(dz[2]);
        float sqrt_w = sqrtf(fabsf(ukf_wc[i]));
        for (int j = 0; j < UKF_M; j++) dz[j] *= sqrt_w;
        cholesky_rank1(Sz, dz, UKF_M, 1.0f);
    }
    // w[0]
    {
        float dz[UKF_M];
        for (int j = 0; j < UKF_M; j++) dz[j] = ukf_sigmas_z[0][j] - z_pred[j];
        dz[2] = wrap_angle(dz[2]);
        float w0 = ukf_wc[0];
        float sign = (w0 >= 0) ? 1.0f : -1.0f;
        float sqrt_w = sqrtf(fabsf(w0));
        for (int j = 0; j < UKF_M; j++) dz[j] *= sqrt_w;
        cholesky_rank1(Sz, dz, UKF_M, sign);
    }

    // Cross-covariance Pxz
    float Pxz[UKF_N * UKF_M];
    mat_zero(Pxz, UKF_N, UKF_M);
    for (int i = 0; i < UKF_NSIG; i++) {
        float dx[UKF_N], dz[UKF_M];
        for (int j = 0; j < UKF_N; j++) dx[j] = ukf_sigmas[i][j] - state.x[j];
        for (int j = 0; j < UKF_M; j++) dz[j] = ukf_sigmas_z[i][j] - z_pred[j];
        dx[2] = wrap_angle(dx[2]);
        dz[2] = wrap_angle(dz[2]);
        for (int r = 0; r < UKF_N; r++)
            for (int c = 0; c < UKF_M; c++)
                MAT(Pxz, r, c, UKF_M) += ukf_wc[i] * dx[r] * dz[c];
    }

    // Kalman gain K = Pxz * inv(Sz * Sz^T) via forward/back substitution
    // K = Pxz * Sz^{-T} * Sz^{-1}
    float K[UKF_N * UKF_M];
    for (int r = 0; r < UKF_N; r++) {
        float row[UKF_M];
        for (int c = 0; c < UKF_M; c++) row[c] = MAT(Pxz, r, c, UKF_M);
        float y_tmp[UKF_M], k_row[UKF_M];
        solve_lower(y_tmp, Sz, row, UKF_M);
        // Now solve Sz^T * k = y_tmp
        for (int i = UKF_M - 1; i >= 0; i--) {
            float sum = y_tmp[i];
            for (int j = i + 1; j < UKF_M; j++) sum -= Sz[j * UKF_M + i] * k_row[j];
            k_row[i] = sum / (Sz[i * UKF_M + i] + 1e-12f);
        }
        for (int c = 0; c < UKF_M; c++) MAT(K, r, c, UKF_M) = k_row[c];
    }

    // Innovation
    float innov[UKF_M];
    for (int j = 0; j < UKF_M; j++) innov[j] = z_meas[j] - z_pred[j];
    innov[2] = wrap_angle(innov[2]);

    // Update state
    for (int i = 0; i < UKF_N; i++) {
        float correction = 0;
        for (int j = 0; j < UKF_M; j++) correction += MAT(K, i, j, UKF_M) * innov[j];
        state.x[i] += correction;
    }
    state.x[2] = wrap_angle(state.x[2]);

    // Update covariance: downdate S with K * Sz columns
    for (int j = 0; j < UKF_M; j++) {
        float u_vec[UKF_N];
        for (int i = 0; i < UKF_N; i++) {
            u_vec[i] = 0;
            for (int k = 0; k < UKF_M; k++)
                u_vec[i] += MAT(K, i, k, UKF_M) * Sz[k * UKF_M + j];
        }
        cholesky_rank1(state.S, u_vec, UKF_N, -1.0f);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 5: MPC Steering Controller (Greedy search with constraints)
// ═══════════════════════════════════════════════════════════════════════

// Nomoto yaw model step
static void nomoto_step(float* psi_out, float* r_out,
                        float psi, float r, float dr, float dt) {
    float r_dot = (NOMOTO_K * dr - r) / NOMOTO_T;
    *r_out  = r + r_dot * dt;
    *psi_out = wrap_angle(psi + (*r_out) * dt);
}

static float mpc_compute(float heading_rad, float yaw_rate, float target_rad,
                         float boat_x, float boat_y, float boat_spd,
                         float heel_rad) {
    float dt = 0.5f;
    float dr_max = RUDDER_MAX_DEG * DEG2RAD;
    float dr_rate = RUDDER_RATE_MAX * DEG2RAD * dt;
    float last_dr = state.last_mpc_rudder_rad;

    // Generate candidate increments
    float candidates[MPC_CANDIDATES];
    for (int i = 0; i < MPC_CANDIDATES; i++)
        candidates[i] = -dr_rate + (2.0f * dr_rate * i) / (MPC_CANDIDATES - 1.0f);

    float best_cost = 1e10f;
    float best_dr   = last_dr;

    for (int c0 = 0; c0 < MPC_CANDIDATES; c0++) {
        float dr0 = clampf(last_dr + candidates[c0],
                           fmaxf(-dr_max, last_dr - dr_rate),
                           fminf(dr_max, last_dr + dr_rate));
        dr0 = clampf(dr0, -dr_max, dr_max);

        float seq[MPC_HORIZON];
        seq[0] = dr0;
        float psi = heading_rad, r = yaw_rate;
        nomoto_step(&psi, &r, psi, r, dr0, dt);

        // Greedy fill remaining horizon
        for (int k = 1; k < MPC_HORIZON; k++) {
            float best_lc = 1e10f, best_ldr = seq[k-1];
            for (int c = 0; c < MPC_CANDIDATES; c++) {
                float dr_try = clampf(seq[k-1] + candidates[c],
                                      fmaxf(-dr_max, seq[k-1] - dr_rate),
                                      fminf(dr_max, seq[k-1] + dr_rate));
                dr_try = clampf(dr_try, -dr_max, dr_max);
                float pt, rt;
                nomoto_step(&pt, &rt, psi, r, dr_try, dt);
                float e = wrap_angle(pt - target_rad);
                float lc = Q_PSI * e * e + Q_R * rt * rt;
                if (lc < best_lc) { best_lc = lc; best_ldr = dr_try; }
            }
            seq[k] = best_ldr;
            nomoto_step(&psi, &r, psi, r, best_ldr, dt);
        }

        // Evaluate full sequence cost
        psi = heading_rad; r = yaw_rate;
        float prev = last_dr;
        float cost = 0;
        float bx = boat_x, by = boat_y;

        for (int k = 0; k < MPC_HORIZON; k++) {
            nomoto_step(&psi, &r, psi, r, seq[k], dt);
            float e = wrap_angle(psi - target_rad);
            cost += Q_PSI * e * e + Q_R * r * r + R_DR * (seq[k]-prev)*(seq[k]-prev);
            prev = seq[k];

            // Obstacle avoidance penalty
            bx += boat_spd * cosf(psi) * dt;
            by += boat_spd * sinf(psi) * dt;
            float t_fut = (k + 1) * dt;
            for (int o = 0; o < state.n_obstacles; o++) {
                float ox = state.obstacles[o].x + state.obstacles[o].vx * t_fut;
                float oy = state.obstacles[o].y + state.obstacles[o].vy * t_fut;
                float d = sqrtf((bx-ox)*(bx-ox) + (by-oy)*(by-oy));
                float viol = fmaxf(0, state.obstacles[o].radius - d);
                cost += Q_OBS * viol * viol;
            }

            // Predictive heel penalty
            float pred_heel = fabsf(heel_rad) + 0.3f * fabsf(r);
            float hviol = fmaxf(0, pred_heel - 25.0f * DEG2RAD);
            cost += Q_HEEL * hviol * hviol;
        }
        // Terminal cost
        float ef = wrap_angle(psi - target_rad);
        cost += Q_PSI_F * ef * ef;

        if (cost < best_cost) { best_cost = cost; best_dr = seq[0]; }
    }

    state.last_mpc_rudder_rad = best_dr;
    return best_dr * RAD2DEG;
}

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 6: VPP Sail Optimizer
// ═══════════════════════════════════════════════════════════════════════

// Aero coefficients with stall (same model as UKF dynamics)
static void vpp_aero_coeffs(float alpha, float* CL, float* CD) {
    float abs_a = fabsf(alpha);
    float CL_pre = 2.0f * PI_F * sinf(alpha) * 0.75f;
    CL_pre = clampf(CL_pre, -CL_MAX_SAIL, CL_MAX_SAIL);
    float CD_pre = 0.05f + CL_pre * CL_pre / (PI_F * 3.0f * 0.85f);
    float CL_post = CL_MAX_SAIL * sinf(2.0f * alpha);
    float CD_stall = 0.05f + CL_MAX_SAIL * CL_MAX_SAIL / (PI_F * 3.0f * 0.85f);
    float CD_post = CD_MAX_SAIL - (CD_MAX_SAIL - CD_stall) * cosf(alpha) * cosf(alpha);
    float blend = 0.5f * (1.0f + tanhf((abs_a - ALPHA_STALL_SAIL) / (STALL_WIDTH_SAIL + 1e-9f)));
    *CL = (1.0f - blend) * CL_pre + blend * CL_post;
    *CD = (1.0f - blend) * CD_pre + blend * CD_post;
}

static float hull_resistance(float V) {
    float R_visc = 0.5f * RHO_WATER * WETTED_AREA * CF_HULL * V * V;
    float R_wave = K_WAVE * V * V * V * V / HULL_LEN;
    float R_added = 0;
    if (state.wave_height > 0 && HULL_LEN > 0) {
        float ratio = state.wave_height / HULL_LEN;
        R_added = 0.5f * RHO_WATER * WETTED_AREA * C_AW * ratio * ratio * V * V;
    }
    return R_visc + R_wave + R_added;
}

static float solve_speed(float F_drive) {
    if (F_drive < 0.01f) return 0.0f;
    float lo = 0, hi = 8.0f;
    for (int i = 0; i < 20; i++) {
        float mid = 0.5f * (lo + hi);
        float R = hull_resistance(mid);
        if (R < F_drive) lo = mid; else hi = mid;
    }
    return 0.5f * (lo + hi);
}

static float estimate_heel(float F_side) {
    float moment = fabsf(F_side) * fabsf(COG_Z);
    float restoring = BOAT_MASS * GRAVITY * METACENTRIC_H;
    if (restoring < 0.01f) return 0;
    float phi = asinf(clampf(moment / restoring, -1.0f, 1.0f));
    return phi * RAD2DEG;
}

// VPP optimize: returns optimal sail angle in degrees [0, 88]
static float vpp_optimize(float wind_speed, float wind_angle_deg,
                          float target_bearing_deg, float boat_speed_hint,
                          float current_heel_deg) {
    float twa = wind_angle_deg * DEG2RAD;
    float tgt = target_bearing_deg * DEG2RAD;
    float best_vmg = -1e10f;
    float best_sail = 0;

    for (int s_deg = 0; s_deg <= (int)MAX_SAIL_ANGLE; s_deg += VPP_SCAN_STEP) {
        float s_rad = s_deg * DEG2RAD;
        // Apparent wind
        float aw_x = wind_speed * cosf(twa) + boat_speed_hint;
        float aw_y = wind_speed * sinf(twa);
        float V_app = sqrtf(aw_x * aw_x + aw_y * aw_y) + 1e-6f;
        float awa = atan2f(aw_y, aw_x);
        float alpha = awa - s_rad;

        float CL, CD;
        vpp_aero_coeffs(alpha, &CL, &CD);
        float q = 0.5f * RHO_AIR * V_app * V_app * SAIL_AREA;
        float F_drive = q * (CL * sinf(fabsf(alpha)) - CD * cosf(alpha));
        float F_side  = q * (CL * cosf(alpha) + CD * sinf(fabsf(alpha)));

        float V_s = solve_speed(fmaxf(F_drive, 0));

        // Refine at equilibrium
        aw_x = wind_speed * cosf(twa) + V_s;
        V_app = sqrtf(aw_x * aw_x + aw_y * aw_y) + 1e-6f;
        awa = atan2f(aw_y, aw_x);
        alpha = awa - s_rad;
        vpp_aero_coeffs(alpha, &CL, &CD);
        q = 0.5f * RHO_AIR * V_app * V_app * SAIL_AREA;
        F_drive = q * (CL * sinf(fabsf(alpha)) - CD * cosf(alpha));
        F_side  = q * (CL * cosf(alpha) + CD * sinf(fabsf(alpha)));
        V_s = solve_speed(fmaxf(F_drive, 0));

        float heel = estimate_heel(F_side);
        float vmg = V_s * cosf(twa - tgt);

        // Proactive heel constraint
        if (fabsf(heel) > 25.0f) continue;
        if (fabsf(current_heel_deg) > 20.0f) {
            float combined = fabsf(heel) + 0.3f * fabsf(current_heel_deg);
            if (combined > 37.5f) continue;  // 25 * 1.5
        }

        if (vmg > best_vmg) {
            best_vmg = vmg;
            best_sail = (float)s_deg;
        }
    }

    // Fallback: if nothing passes heel constraint, find minimum heel
    if (best_vmg < -1e9f) {
        float min_heel = 1e10f;
        for (int s_deg = 0; s_deg <= (int)MAX_SAIL_ANGLE; s_deg += VPP_SCAN_STEP) {
            float s_rad = s_deg * DEG2RAD;
            float aw_x = wind_speed * cosf(twa) + boat_speed_hint;
            float aw_y = wind_speed * sinf(twa);
            float V_app = sqrtf(aw_x * aw_x + aw_y * aw_y) + 1e-6f;
            float awa = atan2f(aw_y, aw_x);
            float alpha = awa - s_rad;
            float CL, CD;
            vpp_aero_coeffs(alpha, &CL, &CD);
            float q = 0.5f * RHO_AIR * V_app * V_app * SAIL_AREA;
            float F_side = q * (CL * cosf(alpha) + CD * sinf(fabsf(alpha)));
            float heel = fabsf(estimate_heel(F_side));
            if (heel < min_heel) { min_heel = heel; best_sail = (float)s_deg; }
        }
    }

    // Reactive depowering
    if (fabsf(current_heel_deg) > 20.0f) {
        float excess = fabsf(current_heel_deg) - 20.0f;
        float factor = 1.0f + 0.1f * excess;
        best_sail = fminf(best_sail * factor, MAX_SAIL_ANGLE);
    }

    // Rate limit
    float max_change = SAIL_RATE_MAX * 0.5f;  // assuming 0.5s control period
    float delta = best_sail - state.current_sail_deg;
    delta = clampf(delta, -max_change, max_change);
    best_sail = state.current_sail_deg + delta;
    best_sail = clampf(best_sail, 0, MAX_SAIL_ANGLE);
    state.current_sail_deg = best_sail;

    return best_sail;
}

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 7: UKF-Aware Drift/Current Estimator
// ═══════════════════════════════════════════════════════════════════════

static void drift_update_from_ukf(float gps_vx, float gps_vy,
                                   float heading_rad, float surge, float sway) {
    // Body-to-earth rotation of UKF velocities
    float ch = cosf(heading_rad), sh = sinf(heading_rad);
    float boat_vx = surge * ch - sway * sh;
    float boat_vy = surge * sh + sway * ch;

    // Current = GPS_velocity - boat_velocity
    float cx = gps_vx - boat_vx;
    float cy = gps_vy - boat_vy;

    int idx = state.drift_buf_idx;
    state.drift_vx_buf[idx] = cx;
    state.drift_vy_buf[idx] = cy;
    state.drift_buf_idx = (idx + 1) % 20;
    if (state.drift_buf_count < 20) state.drift_buf_count++;

    // Compute running average
    float sum_x = 0, sum_y = 0;
    for (int i = 0; i < state.drift_buf_count; i++) {
        sum_x += state.drift_vx_buf[i];
        sum_y += state.drift_vy_buf[i];
    }
    sum_x /= state.drift_buf_count;
    sum_y /= state.drift_buf_count;

    state.drift_speed = sqrtf(sum_x * sum_x + sum_y * sum_y);
    state.drift_direction = atan2f(sum_y, sum_x) * RAD2DEG;
    state.drift_confidence = fminf(1.0f, state.drift_buf_count / 10.0f);
}

// Get leeway-corrected heading for MPC target
static float drift_compensate_heading(float target_deg, float leeway_deg,
                                       float boat_speed) {
    float target = target_deg;
    // Subtract leeway (steer upwind to compensate)
    target -= leeway_deg;

    // Add drift compensation (steer into current)
    if (state.drift_confidence > 0.3f && boat_speed > 0.3f) {
        float ratio = state.drift_speed / boat_speed;
        float comp = asinf(clampf(ratio, -1.0f, 1.0f)) * RAD2DEG;
        comp = clampf(comp, -30.0f, 30.0f);

        // Determine if current pushes us CW or CCW of course
        float drift_rel = state.drift_direction - target;
        if (drift_rel > 180) drift_rel -= 360;
        if (drift_rel < -180) drift_rel += 360;
        if (drift_rel > 0) target -= comp; else target += comp;
    }

    // Normalise to [0, 360)
    while (target < 0) target += 360;
    while (target >= 360) target -= 360;
    return target;
}

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 8: Actuator Drivers
// ═══════════════════════════════════════════════════════════════════════

// ── Rudder Servo ───────────────────────────────────────────────────────
// Matches rudder_a.py: angle [-21, 21]° → servo [34, 76]
//   servo_angle = NEUTRAL_SERVO + rudder_angle  (NEUTRAL=55)
//   clamped to [34, 76] for hardware safety
static void set_rudder(float rudder_deg) {
    rudder_deg = clampf(rudder_deg, RUDDER_MIN_DEG, RUDDER_MAX_DEG);
    int servo_angle = NEUTRAL_SERVO + (int)roundf(rudder_deg);
    servo_angle = (servo_angle < 34) ? 34 : (servo_angle > 76) ? 76 : servo_angle;
    rudderServo.write(servo_angle);
    state.rudder_deg = rudder_deg;
#if UKF_N == 10
    state.x[8] = rudder_deg * DEG2RAD;
#endif
}

// ── Winch Stepper ──────────────────────────────────────────────────────
// Matches winch_a.py: law-of-cosines geometry for boom angle → steps
static int angle_to_steps(float angle_deg) {
    if (angle_deg < 0) angle_deg = -angle_deg;  // symmetric
    if (angle_deg > MAX_SAIL_ANGLE) angle_deg = MAX_SAIL_ANGLE;

    float angle_rad = angle_deg * DEG2RAD;
    // Retracted line length at 0° (boom fully sheeted in)
    float c_min_sq = BOOM_LENGTH * BOOM_LENGTH + WINCH_TO_MAST * WINCH_TO_MAST
                     - 2.0f * BOOM_LENGTH * WINCH_TO_MAST * cosf(0);
    float c_min = sqrtf(c_min_sq);
    // Line length at desired angle
    float c_sq = BOOM_LENGTH * BOOM_LENGTH + WINCH_TO_MAST * WINCH_TO_MAST
                 - 2.0f * BOOM_LENGTH * WINCH_TO_MAST * cosf(angle_rad);
    float c = sqrtf(c_sq);
    // Line to pay out
    float delta_line = c - c_min;
    if (delta_line < 0) delta_line = 0;
    // Spool rotations → motor steps
    float spool_rev = delta_line / (TWO_PI_F * SPOOL_RADIUS);
    float motor_rev = spool_rev * GEAR_RATIO;
    return (int)roundf(motor_rev * STEPS_PER_REV);
}

// Track current stepper position (absolute step count)
static int stepper_current_steps = 0;

static void set_sail(float sail_deg) {
    sail_deg = clampf(sail_deg, 0, MAX_SAIL_ANGLE);
    int target_steps = angle_to_steps(sail_deg);
    int delta = target_steps - stepper_current_steps;
    if (delta == 0) return;

    // Direction
    bool cw = (delta > 0);
    int abs_delta = (delta > 0) ? delta : -delta;

    // Drive stepper
    digitalWrite(STEPPER_DIR_PIN, cw ? HIGH : LOW);
    digitalWrite(STEPPER_EN_PIN, LOW);  // enable

    for (int i = 0; i < abs_delta; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(500);
    }

    stepper_current_steps = target_steps;
    state.sail_deg = sail_deg;
#if UKF_N == 10
    state.x[9] = sail_deg * DEG2RAD;
#endif
}

// ── Serial command output (to secondary Arduino if applicable) ─────────
// Matches rudder_a.py / winch_a.py serial protocol
static void send_rudder_cmd(float rudder_deg) {
    uint8_t servo_byte = (uint8_t)(NEUTRAL_SERVO + (int)roundf(
        clampf(rudder_deg, RUDDER_MIN_DEG, RUDDER_MAX_DEG)));
    uint8_t pkt[2] = { CMD_SERVO, servo_byte };
    CMD_SERIAL.write(pkt, 2);
}

static void send_winch_cmd(int steps, bool cw) {
    uint8_t cmd = cw ? CMD_WINCH_CW : CMD_WINCH_CCW;
    uint8_t pkt[5];
    pkt[0] = cmd;
    pkt[1] = (steps >> 24) & 0xFF;
    pkt[2] = (steps >> 16) & 0xFF;
    pkt[3] = (steps >> 8) & 0xFF;
    pkt[4] = steps & 0xFF;
    CMD_SERIAL.write(pkt, 5);
}

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 9: Health Monitor (Graded Severity)
// ═══════════════════════════════════════════════════════════════════════

static void health_update() {
    HealthGrade grade = HEALTHY;
    unsigned long now = millis();

    // Sensor timeouts
    bool gps_timeout = (now - sensors.gps_last_ms > 3000);
    bool imu_timeout = (now - sensors.imu_last_ms > 1000);

    if (gps_timeout || imu_timeout) grade = CRITICAL;

    // UKF covariance trace
    if (state.ukf_trace > 50.0f && grade < DEGRADED) grade = DEGRADED;
    if (state.ukf_trace > 200.0f) grade = FALLBACK;

    // Heel angle
    float heel_deg = 0;
#if UKF_N == 10
    heel_deg = fabsf(state.x[6]) * RAD2DEG;
#endif
    if (heel_deg > 35.0f && grade < DEGRADED) grade = DEGRADED;

    state.health = grade;

    // Debug output
    static unsigned long last_health_print = 0;
    if (now - last_health_print > 5000) {
        last_health_print = now;
        const char* labels[] = {"HEALTHY", "DEGRADED", "CRITICAL", "FALLBACK"};
        DEBUG_SERIAL.print("[HEALTH] ");
        DEBUG_SERIAL.print(labels[grade]);
        DEBUG_SERIAL.print(" trace=");
        DEBUG_SERIAL.print(state.ukf_trace, 1);
        DEBUG_SERIAL.print(" heel=");
        DEBUG_SERIAL.print(heel_deg, 1);
        DEBUG_SERIAL.print(" GPS=");
        DEBUG_SERIAL.print(gps_timeout ? "TIMEOUT" : "OK");
        DEBUG_SERIAL.print(" IMU=");
        DEBUG_SERIAL.println(imu_timeout ? "TIMEOUT" : "OK");
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 10: Navigation Logic
// ═══════════════════════════════════════════════════════════════════════

static float bearing_to_wpt(float boat_x, float boat_y, int wpt_idx) {
    if (wpt_idx >= state.n_waypoints) return 0;
    float wx, wy;
    latlon_to_xy(state.waypoints[wpt_idx].lat, state.waypoints[wpt_idx].lon,
                 &wx, &wy);
    float dx = wx - boat_x;
    float dy = wy - boat_y;
    return atan2f(dx, dy) * RAD2DEG;  // bearing in degrees from N
}

static float distance_to_wpt(float boat_x, float boat_y, int wpt_idx) {
    if (wpt_idx >= state.n_waypoints) return 1e6f;
    float wx, wy;
    latlon_to_xy(state.waypoints[wpt_idx].lat, state.waypoints[wpt_idx].lon,
                 &wx, &wy);
    float dx = wx - boat_x;
    float dy = wy - boat_y;
    return sqrtf(dx * dx + dy * dy);
}

static void navigation_tick() {
    if (!state.nav_enabled || state.n_waypoints == 0) return;
    if (state.current_wpt >= state.n_waypoints) {
        state.nav_enabled = false;  // mission complete
        set_rudder(0);
        return;
    }

    float boat_x = state.x[0], boat_y = state.x[1];
    float heading_rad = state.x[2];
    float heading_deg = heading_rad * RAD2DEG;
    float surge = state.x[3], sway = state.x[4];
    float yaw_rate = state.x[5];
    float boat_speed = sqrtf(surge * surge + sway * sway);
#if UKF_N == 10
    float heel_rad = state.x[6];
    float heel_deg = heel_rad * RAD2DEG;
    float leeway_deg = atan2f(sway, surge + 1e-6f) * RAD2DEG;
#else
    float heel_rad = 0, heel_deg = 0, leeway_deg = 0;
#endif

    // Check waypoint arrival
    float dist = distance_to_wpt(boat_x, boat_y, state.current_wpt);
    if (dist < WPT_RADIUS) {
        state.current_wpt++;
        DEBUG_SERIAL.print("[NAV] Waypoint reached! Next: ");
        DEBUG_SERIAL.println(state.current_wpt);
        if (state.current_wpt >= state.n_waypoints) {
            state.nav_enabled = false;
            return;
        }
    }

    // Target bearing
    float target_bearing = bearing_to_wpt(boat_x, boat_y, state.current_wpt);

    // Wind-shift detection → replan
    float wind_delta = fabsf(sensors.wind_dir_deg - state.last_plan_wind);
    if (wind_delta > 180) wind_delta = 360 - wind_delta;
    if (wind_delta > WIND_SHIFT_THRESH) {
        state.last_plan_wind = sensors.wind_dir_deg;
        state.last_mpc_rudder_rad = 0;  // MPC reset
        DEBUG_SERIAL.println("[NAV] Wind shift detected — replanning");
    }

    // Apply drift + leeway compensation
    float comp_target = drift_compensate_heading(target_bearing, leeway_deg, boat_speed);

    // ── MPC steering ────────────────────────────────────────────────
    float rudder = mpc_compute(heading_rad, yaw_rate, comp_target * DEG2RAD,
                               boat_x, boat_y, boat_speed, heel_rad);
    set_rudder(rudder);

    // ── VPP sail trim ───────────────────────────────────────────────
    float sail = vpp_optimize(sensors.wind_speed_mps, sensors.wind_dir_deg,
                              target_bearing, boat_speed, heel_deg);
    set_sail(sail);

    // Also send commands via serial if secondary controller is connected
    send_rudder_cmd(rudder);
    int target_steps = angle_to_steps(sail);
    int delta_steps = target_steps - stepper_current_steps;
    if (delta_steps != 0) {
        send_winch_cmd(abs(delta_steps), delta_steps > 0);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 11: Serial Command Interface (receive waypoints, etc.)
// ═══════════════════════════════════════════════════════════════════════
// Protocol from Jetson/RasPi over CMD_SERIAL:
//   'W' lat lon        — add waypoint
//   'O' x y r vx vy    — add obstacle
//   'G'                — start navigation (GO)
//   'S'                — stop navigation (STOP)
//   'H' wave_height    — set wave height
//   'V' wind_speed     — set wind speed override
//   'C'                — clear waypoints & obstacles

static char cmd_buf[128];
static int  cmd_idx = 0;

static void process_command(char* line) {
    if (line[0] == 'W' && state.n_waypoints < MAX_WAYPOINTS) {
        float lat, lon;
        if (sscanf(line + 1, "%f %f", &lat, &lon) == 2) {
            state.waypoints[state.n_waypoints].lat = lat;
            state.waypoints[state.n_waypoints].lon = lon;
            state.n_waypoints++;
            DEBUG_SERIAL.print("[CMD] Waypoint added: ");
            DEBUG_SERIAL.print(lat, 6); DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.println(lon, 6);
        }
    } else if (line[0] == 'O' && state.n_obstacles < MAX_OBSTACLES) {
        float ox, oy, r, ovx = 0, ovy = 0;
        int n = sscanf(line + 1, "%f %f %f %f %f", &ox, &oy, &r, &ovx, &ovy);
        if (n >= 3) {
            Obstacle& o = state.obstacles[state.n_obstacles];
            o.x = ox; o.y = oy; o.radius = r; o.vx = ovx; o.vy = ovy;
            state.n_obstacles++;
            DEBUG_SERIAL.println("[CMD] Obstacle added");
        }
    } else if (line[0] == 'G') {
        state.nav_enabled = true;
        state.current_wpt = 0;
        state.last_plan_wind = sensors.wind_dir_deg;
        DEBUG_SERIAL.println("[CMD] Navigation STARTED");
    } else if (line[0] == 'S') {
        state.nav_enabled = false;
        set_rudder(0);
        DEBUG_SERIAL.println("[CMD] Navigation STOPPED");
    } else if (line[0] == 'H') {
        float h;
        if (sscanf(line + 1, "%f", &h) == 1) {
            state.wave_height = fmaxf(0, h);
            state.wave_period = 4.0f;
            DEBUG_SERIAL.print("[CMD] Wave height set: ");
            DEBUG_SERIAL.println(h);
        }
    } else if (line[0] == 'V') {
        float v;
        if (sscanf(line + 1, "%f", &v) == 1) {
            sensors.wind_speed_mps = v;
            DEBUG_SERIAL.print("[CMD] Wind speed override: ");
            DEBUG_SERIAL.println(v);
        }
    } else if (line[0] == 'C') {
        state.n_waypoints = 0;
        state.n_obstacles = 0;
        state.current_wpt = 0;
        state.nav_enabled = false;
        DEBUG_SERIAL.println("[CMD] Cleared all waypoints & obstacles");
    }
}

static void cmd_read() {
    while (CMD_SERIAL.available()) {
        char c = CMD_SERIAL.read();
        if (c == '\n' || c == '\r') {
            if (cmd_idx > 0) {
                cmd_buf[cmd_idx] = '\0';
                process_command(cmd_buf);
                cmd_idx = 0;
            }
        } else if (cmd_idx < (int)sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_idx++] = c;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  SECTION 12: setup() and loop()
// ═══════════════════════════════════════════════════════════════════════

void setup() {
    // Serial ports
    DEBUG_SERIAL.begin(DEBUG_BAUD);
    GPS_SERIAL.begin(GPS_BAUD);
    CMD_SERIAL.begin(CMD_BAUD);

    // Actuator pins
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    digitalWrite(STEPPER_EN_PIN, HIGH);  // disabled until needed
    rudderServo.attach(RUDDER_SERVO_PIN);
    rudderServo.write(NEUTRAL_SERVO);

    // IMU
    imu_init();

    // Clear state
    memset(&sensors, 0, sizeof(sensors));
    memset(&state, 0, sizeof(state));
    state.wave_period = 4.0f;

    // Initialise UKF
    ukf_init();

    delay(500);
    DEBUG_SERIAL.println("╔═══════════════════════════════════════════╗");
    DEBUG_SERIAL.println("║  SAILBOT 2526 — Phase-6 Production FW    ║");
    DEBUG_SERIAL.println("║  UKF + MPC + VPP + Drift + Health        ║");
#ifdef PLATFORM_TEENSY4
    DEBUG_SERIAL.println("║  Platform: Teensy 4.x (10-state UKF)     ║");
#else
    DEBUG_SERIAL.println("║  Platform: Arduino Mega (6-state UKF)    ║");
#endif
    DEBUG_SERIAL.println("╚═══════════════════════════════════════════╝");
    DEBUG_SERIAL.println("Commands: W lat lon | O x y r | G | S | H h | V v | C");
}

// ── Timing intervals ──────────────────────────────────────────────────
#define UKF_INTERVAL_MS    100   // 10 Hz state estimation
#define CTRL_INTERVAL_MS   500   // 2 Hz control loop (MPC + VPP)
#define NAV_INTERVAL_MS    5000  // 0.2 Hz navigation planning
#define DRIFT_INTERVAL_MS  5000  // 0.2 Hz drift estimator update
#define HEALTH_INTERVAL_MS 2000  // 0.5 Hz health check

void loop() {
    unsigned long now = millis();

    // ── Always: read sensors ────────────────────────────────────────
    gps_read();
    imu_read();
    wind_read();
    cmd_read();

    // ── 10 Hz: UKF predict + update ─────────────────────────────────
    if (now - state.last_ukf_ms >= UKF_INTERVAL_MS) {
        float dt = (now - state.last_ukf_ms) / 1000.0f;
        if (dt > 1.0f) dt = 0.1f;  // clamp after long pauses
        state.last_ukf_ms = now;

        ukf_predict(dt);

        // Build measurement vector from sensors
        if (sensors.gps_valid && sensors.imu_valid) {
            float boat_x, boat_y;
            latlon_to_xy(sensors.gps_lat, sensors.gps_lon, &boat_x, &boat_y);

            float z_meas[UKF_M] = {
                boat_x,
                boat_y,
                sensors.imu_heading_rad,
                sensors.imu_heel_rad,
                sensors.imu_roll_rate,
                sensors.imu_yaw_rate
            };
            ukf_update(z_meas);
        }
    }

    // ── 2 Hz: Control loop (MPC + VPP) ──────────────────────────────
    if (now - state.last_control_ms >= CTRL_INTERVAL_MS) {
        state.last_control_ms = now;

        if (state.nav_enabled) {
            navigation_tick();
        }

        // Debug telemetry
        static unsigned long last_telem = 0;
        if (now - last_telem > 2000) {
            last_telem = now;
            DEBUG_SERIAL.print("[TEL] hdg=");
            DEBUG_SERIAL.print(state.x[2] * RAD2DEG, 1);
            DEBUG_SERIAL.print(" spd=");
            DEBUG_SERIAL.print(sqrtf(state.x[3]*state.x[3] + state.x[4]*state.x[4]), 2);
            DEBUG_SERIAL.print(" rud=");
            DEBUG_SERIAL.print(state.rudder_deg, 1);
            DEBUG_SERIAL.print(" sail=");
            DEBUG_SERIAL.print(state.sail_deg, 1);
#if UKF_N == 10
            DEBUG_SERIAL.print(" heel=");
            DEBUG_SERIAL.print(state.x[6] * RAD2DEG, 1);
#endif
            DEBUG_SERIAL.print(" drift=");
            DEBUG_SERIAL.print(state.drift_speed, 2);
            DEBUG_SERIAL.print("m/s@");
            DEBUG_SERIAL.print(state.drift_direction, 0);
            DEBUG_SERIAL.println("°");
        }
    }

    // ── 0.2 Hz: Drift estimator ─────────────────────────────────────
    static unsigned long last_drift_ms = 0;
    if (now - last_drift_ms >= DRIFT_INTERVAL_MS && sensors.gps_valid) {
        last_drift_ms = now;
        float gps_vx = sensors.gps_speed * sinf(sensors.gps_course * DEG2RAD);
        float gps_vy = sensors.gps_speed * cosf(sensors.gps_course * DEG2RAD);
        drift_update_from_ukf(gps_vx, gps_vy,
                               state.x[2], state.x[3], state.x[4]);
    }

    // ── 0.5 Hz: Health monitor ──────────────────────────────────────
    static unsigned long last_health_ms = 0;
    if (now - last_health_ms >= HEALTH_INTERVAL_MS) {
        last_health_ms = now;
        health_update();
    }
}