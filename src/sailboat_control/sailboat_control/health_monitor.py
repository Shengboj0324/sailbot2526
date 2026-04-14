#!/usr/bin/env python3
"""
System Health Monitor — Phase-5 Enhanced.

Monitors sensor timeouts, actuator errors, system health, and Phase-5
controller diagnostics (UKF covariance, MPC cost, VPP L/D ratio).
Publishes severity-graded health status for state_management_node to
trigger automatic fallback from Phase-5 → Phase-4 controllers.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Bool, String
from sensor_msgs.msg import NavSatFix, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time
import json
import numpy as np


class HealthMonitor(Node):
    """Monitor system health and publish diagnostics."""

    # Severity levels
    HEALTHY   = 'HEALTHY'
    DEGRADED  = 'DEGRADED'
    CRITICAL  = 'CRITICAL'
    FALLBACK  = 'FALLBACK'   # Phase-5 failed, use Phase-4

    def __init__(self):
        super().__init__('health_monitor')

        # ── Timeout thresholds (seconds) ─────────────────────────────
        self.gps_timeout     = 2.0
        self.imu_timeout     = 1.0
        self.wind_timeout    = 5.0
        self.compass_timeout = 2.0

        # ── Last message timestamps ──────────────────────────────────
        self.last_gps_time     = None
        self.last_imu_time     = None
        self.last_wind_time    = None
        self.last_compass_time = None

        # ── Sensor status ────────────────────────────────────────────
        self.gps_ok     = False
        self.imu_ok     = False
        self.wind_ok    = False
        self.compass_ok = False

        # ── Actuator status ──────────────────────────────────────────
        self.rudder_ok = True
        self.sail_ok   = True

        # ── System metrics ───────────────────────────────────────────
        self.boat_speed     = 0.0
        self.heading        = 0.0
        self.wind_direction = 0.0

        # ── Phase-5 diagnostics ──────────────────────────────────────
        self.ukf_trace        = 0.0     # trace(P) — uncertainty indicator
        self.ukf_max_eigenval = 0.0     # max eigenvalue of P
        self.mpc_cost         = 0.0     # last MPC optimisation cost
        self.vpp_lift_to_drag = 0.0     # best L/D from last VPP cycle
        self.heel_angle       = 0.0
        self.leeway_deg       = 0.0
        self.phase5_active    = False

        # Thresholds for Phase-5 health
        self.ukf_trace_warn     = 50.0    # trace(P) above this → DEGRADED
        self.ukf_trace_critical = 200.0   # trace(P) above this → FALLBACK
        self.mpc_cost_warn      = 100.0
        self.heel_critical      = 35.0    # degrees

        # ── Subscribers ──────────────────────────────────────────────
        self.gps_sub     = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.imu_sub     = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.wind_sub    = self.create_subscription(Float32, 'wind/direction', self.wind_callback, 10)
        self.compass_sub = self.create_subscription(Float32, 'compass/heading', self.compass_callback, 10)
        self.speed_sub   = self.create_subscription(Float64, 'gps/speed', self.speed_callback, 10)
        self.heading_sub = self.create_subscription(Float32, 'state/heading', self.heading_callback, 10)

        # Phase-5 diagnostics subscriber (from navigation_node)
        self.phase5_diag_sub = self.create_subscription(
            String, 'navigation/phase5_diagnostics', self.phase5_diag_callback, 10
        )

        # ── Publishers ───────────────────────────────────────────────
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.health_pub      = self.create_publisher(String, 'system/health', 10)

        # Timer for health checks
        self.timer = self.create_timer(1.0, self.check_health)

        self.get_logger().info('Health Monitor initialized (Phase-5 enhanced)')
    
    def gps_callback(self, msg):
        self.last_gps_time = time.time()
        self.gps_ok = msg.status.status >= 0

    def imu_callback(self, msg):
        self.last_imu_time = time.time()
        self.imu_ok = True

    def wind_callback(self, msg):
        self.last_wind_time = time.time()
        self.wind_ok = True
        self.wind_direction = msg.data

    def compass_callback(self, msg):
        self.last_compass_time = time.time()
        self.compass_ok = True

    def speed_callback(self, msg):
        self.boat_speed = msg.data

    def heading_callback(self, msg):
        self.heading = msg.data

    def phase5_diag_callback(self, msg):
        """Parse Phase-5 controller diagnostics from navigation_node."""
        try:
            data = json.loads(msg.data)
            self.phase5_active = True

            if data.get('controller') == 'MPC':
                self.mpc_cost = data.get('cost', 0.0)
            elif data.get('controller') == 'VPP':
                self.vpp_lift_to_drag = data.get('best_lift_to_drag', 0.0)
                self.heel_angle = data.get('predicted_heel', 0.0)

            # UKF trace comes via nav_status (separate topic); can also
            # arrive here if navigation_node includes it
            if 'ukf_trace' in data:
                self.ukf_trace = data['ukf_trace']
            if 'leeway_deg' in data:
                self.leeway_deg = data['leeway_deg']
        except Exception:
            pass

    # ── Health grade computation ─────────────────────────────────────
    def _compute_health_grade(self, critical_ok):
        """Return (grade, detail_string) for current system state."""
        issues = []

        # Sensor-level grading
        if not critical_ok:
            grade = self.CRITICAL
            if not self.gps_ok:
                issues.append('GPS_TIMEOUT')
            if not self.imu_ok:
                issues.append('IMU_TIMEOUT')
        else:
            grade = self.HEALTHY

        # Phase-5 controller grading
        if self.phase5_active:
            if self.ukf_trace > self.ukf_trace_critical:
                grade = self.FALLBACK
                issues.append(f'UKF_DIVERGE(trace={self.ukf_trace:.1f})')
            elif self.ukf_trace > self.ukf_trace_warn:
                if grade == self.HEALTHY:
                    grade = self.DEGRADED
                issues.append(f'UKF_WARN(trace={self.ukf_trace:.1f})')

            if self.mpc_cost > self.mpc_cost_warn:
                if grade == self.HEALTHY:
                    grade = self.DEGRADED
                issues.append(f'MPC_HIGH_COST({self.mpc_cost:.1f})')

            if abs(self.heel_angle) > self.heel_critical:
                if grade == self.HEALTHY:
                    grade = self.DEGRADED
                issues.append(f'HEEL({self.heel_angle:.1f}°)')

        # Wind sensor is non-critical but degrades performance
        if not self.wind_ok and grade == self.HEALTHY:
            grade = self.DEGRADED
            issues.append('WIND_TIMEOUT')

        detail = '; '.join(issues) if issues else 'all systems nominal'
        return grade, detail

    def check_health(self):
        """Check system health and publish diagnostics."""
        current_time = time.time()

        # Sensor timeout checks
        if self.last_gps_time is not None:
            if current_time - self.last_gps_time > self.gps_timeout:
                self.gps_ok = False
        if self.last_imu_time is not None:
            if current_time - self.last_imu_time > self.imu_timeout:
                self.imu_ok = False
        if self.last_wind_time is not None:
            if current_time - self.last_wind_time > self.wind_timeout:
                self.wind_ok = False
        if self.last_compass_time is not None:
            if current_time - self.last_compass_time > self.compass_timeout:
                self.compass_ok = False

        critical_ok = self.gps_ok and self.imu_ok
        grade, detail = self._compute_health_grade(critical_ok)

        # ── Build diagnostic array ───────────────────────────────────
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Sensor statuses
        for name, ok in [('GPS', self.gps_ok), ('IMU', self.imu_ok),
                         ('Wind Sensor', self.wind_ok), ('Compass', self.compass_ok)]:
            s = DiagnosticStatus()
            s.name = name
            s.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
            s.message = 'OK' if ok else 'TIMEOUT'
            diag_array.status.append(s)

        # Phase-5 controller status
        if self.phase5_active:
            p5 = DiagnosticStatus()
            p5.name = 'Phase-5 Controllers'
            if grade in (self.HEALTHY, self.DEGRADED):
                p5.level = DiagnosticStatus.OK if grade == self.HEALTHY else DiagnosticStatus.WARN
            else:
                p5.level = DiagnosticStatus.ERROR
            p5.message = f'{grade}: {detail}'
            p5.values.append(KeyValue(key='ukf_trace', value=f'{self.ukf_trace:.2f}'))
            p5.values.append(KeyValue(key='mpc_cost', value=f'{self.mpc_cost:.2f}'))
            p5.values.append(KeyValue(key='vpp_lift_to_drag', value=f'{self.vpp_lift_to_drag:.2f}'))
            p5.values.append(KeyValue(key='heel_angle', value=f'{self.heel_angle:.1f}°'))
            p5.values.append(KeyValue(key='leeway', value=f'{self.leeway_deg:.1f}°'))
            diag_array.status.append(p5)

        # Overall system status
        system_status = DiagnosticStatus()
        system_status.name = 'System'
        system_status.level = (DiagnosticStatus.OK if grade == self.HEALTHY
                               else DiagnosticStatus.WARN if grade == self.DEGRADED
                               else DiagnosticStatus.ERROR)
        system_status.message = f'{grade}: {detail}'
        system_status.values.append(KeyValue(key='boat_speed', value=f'{self.boat_speed:.2f} m/s'))
        system_status.values.append(KeyValue(key='heading', value=f'{self.heading:.1f}°'))
        system_status.values.append(KeyValue(key='wind_direction', value=f'{self.wind_direction:.1f}°'))
        diag_array.status.append(system_status)

        self.diagnostics_pub.publish(diag_array)

        # Publish graded health as JSON
        health_msg = String()
        health_msg.data = json.dumps({
            'grade': grade,
            'detail': detail,
            'sensors': {'gps': self.gps_ok, 'imu': self.imu_ok,
                        'wind': self.wind_ok, 'compass': self.compass_ok},
            'phase5': {'active': self.phase5_active,
                       'ukf_trace': self.ukf_trace,
                       'mpc_cost': self.mpc_cost,
                       'vpp_ld': self.vpp_lift_to_drag,
                       'heel': self.heel_angle},
        })
        self.health_pub.publish(health_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

