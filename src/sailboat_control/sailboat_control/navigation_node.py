import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import math
import numpy as np
import json
from path_planning.path_planning.leg import Leg
from path_planning.path_planning.waypoint import Waypoint
from typing import List, Tuple, Optional

# ── Phase-5 modules (UKF / MPC / VPP) with graceful fallback ────────
try:
    from sailboat_control.ukf_state_estimator import SquareRootUKF
    UKF_AVAILABLE = True
except ImportError:
    UKF_AVAILABLE = False

try:
    from sailboat_control.mpc_controller import MPCSteering
    MPC_AVAILABLE = True
except ImportError:
    MPC_AVAILABLE = False

try:
    from sailboat_control.vpp_sail_optimizer import VPPSailOptimizer
    VPP_AVAILABLE = True
except ImportError:
    VPP_AVAILABLE = False

# ── Phase-4 fallback modules ────────────────────────────────────────
try:
    from sailboat_control.adaptive_pid import AdaptivePID, calculate_heading_error
    ADAPTIVE_PID_AVAILABLE = True
except ImportError:
    ADAPTIVE_PID_AVAILABLE = False

try:
    from sailboat_control.drift_estimator import DriftEstimator
    DRIFT_AVAILABLE = True
except ImportError:
    DRIFT_AVAILABLE = False


class PIDController:
    """Simple PID controller for rudder control (fallback)"""
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.last_error = None

    def update(self, error, dt):
        """Update PID controller with new error value"""
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        D = 0
        if self.last_error is not None:
            D = self.Kd * (error - self.last_error) / dt
        self.last_error = error
        return P + I + D

    def reset(self):
        """Reset the PID controller state"""
        self.integral = 0
        self.last_error = None


class NavigationNode(Node):
    """
    Dedicated ROS2 node for autonomous navigation of the sailboat.

    This node:
    1. Handles path planning with tacking/jibing calculations
    2. Controls the rudder to follow planned paths
    3. Monitors progress and transitions between waypoints
    4. Publishes navigation status for monitoring
    5. Calculates optimal sail angle based on apparent wind
    """

    def __init__(self):
        super().__init__('navigation_node')

        # Navigation parameters
        self.declare_parameter('rudder_update_rate', 2.0)  # Hz
        self.declare_parameter('sail_update_rate', 1.0)    # Hz

        # PID parameters (Phase-4 fallback)
        self.declare_parameter('rudder_kp', 70.0)
        self.declare_parameter('rudder_ki', 0.5)
        self.declare_parameter('rudder_kd', 35.0)
        self.declare_parameter('use_adaptive_pid', True)
        self.declare_parameter('use_gain_scheduling', True)
        self.declare_parameter('use_feedforward', True)

        # Phase-5 control architecture selector
        self.declare_parameter('use_ukf', True)
        self.declare_parameter('use_mpc', True)
        self.declare_parameter('use_vpp', True)

        self.declare_parameter('waypoint_threshold', 5.0)   # meters

        # Get parameter values
        self.rudder_update_rate = self.get_parameter('rudder_update_rate').value
        self.sail_update_rate = self.get_parameter('sail_update_rate').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value

        # Phase-5 architecture flags
        self.use_ukf = self.get_parameter('use_ukf').value and UKF_AVAILABLE
        self.use_mpc = self.get_parameter('use_mpc').value and MPC_AVAILABLE
        self.use_vpp = self.get_parameter('use_vpp').value and VPP_AVAILABLE

        # Get PID parameters
        kp = self.get_parameter('rudder_kp').value
        ki = self.get_parameter('rudder_ki').value
        kd = self.get_parameter('rudder_kd').value
        self.use_adaptive_pid = self.get_parameter('use_adaptive_pid').value
        self.use_gain_scheduling = self.get_parameter('use_gain_scheduling').value
        self.use_feedforward = self.get_parameter('use_feedforward').value

        # ── Phase-5: UKF state estimator ─────────────────────────────
        if self.use_ukf:
            self.ukf = SquareRootUKF()
            self.get_logger().info('Phase-5: Square-Root UKF active (10-DOF, 6-DOF dynamics)')
        else:
            self.ukf = None

        # ── Phase-5: MPC steering controller ─────────────────────────
        if self.use_mpc:
            self.mpc = MPCSteering(horizon=10, dt=1.0 / self.rudder_update_rate)
            self.get_logger().info('Phase-5: MPC steering active (Nomoto, N=10)')
        else:
            self.mpc = None

        # ── Phase-5: VPP sail optimizer ──────────────────────────────
        if self.use_vpp:
            self.vpp = VPPSailOptimizer()
            self.get_logger().info('Phase-5: VPP sail optimizer active (aero/hydro equilibrium)')
        else:
            self.vpp = None

        # ── Phase-4 fallback: Adaptive PID ───────────────────────────
        if ADAPTIVE_PID_AVAILABLE and self.use_adaptive_pid:
            self.rudder_pid = AdaptivePID(Kp_base=kp, Ki_base=ki, Kd_base=kd)
            if not self.use_mpc:
                self.get_logger().info('Using Adaptive PID controller (Phase-4)')
        else:
            self.rudder_pid = PIDController(Kp=kp, Ki=ki, Kd=kd)
            if not self.use_mpc:
                self.get_logger().info('Using standard PID controller')

        # ── Phase-3: Drift estimator ─────────────────────────────────
        self.drift_estimator = DriftEstimator() if DRIFT_AVAILABLE else None

        # Additional IMU state
        self.imu_heading = 0.0    # degrees
        self.heel_angle = 0.0     # degrees
        self.roll_rate = 0.0      # rad/s
        self.yaw_rate = 0.0       # rad/s (Phase-5: used by MPC)
        self.wind_speed = 0.0     # m/s (Phase-5: used by VPP)

        # Navigation state
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0      # in degrees, from GPS track or compass
        self.previous_lat = 0.0
        self.previous_lon = 0.0
        self.previous_heading = 0.0
        self.wind_direction = 0.0       # in degrees
        self.current_speed = 0.0        # in m/s
        self.rotational_velocity = 0.0  # in rad/s
        self.gps_timestamp = 0          # for calculating heading

        # Path planning
        self.leg_calculator = Leg()     # From path_planning
        self.active_course = []         # List of [lat, lon] waypoints
        self.current_target_idx = 0     # Index of current waypoint
        self.navigation_enabled = False # Whether we're in autonomous mode
        self.last_path_time = 0.0       # Time when path was last calculated

        # Path visualization data
        self.full_path = []             # Complete path including intermediates

        # Initialize path planning timer (slow rate - 1Hz)
        self.path_timer = self.create_timer(1.0, self.path_planning_callback)

        # Rudder control timer - 1Hz
        self.rudder_timer = self.create_timer(
            1.0/self.rudder_update_rate,  # 1.0 seconds
            self.rudder_control_callback
        )

        # Sail control timer - 1Hz
        self.sail_timer = self.create_timer(
            1.0/self.sail_update_rate,  # 1.0 seconds
            self.sail_control_callback
        )

        # Publishers
        self.rudder_publisher = self.create_publisher(
            Float32,
            'rudder/command',
            10
        )

        self.sail_angle_publisher = self.create_publisher(
            Float32,
            'sail/angle',  # New topic for sail angle
            10
        )

        self.nav_status_publisher = self.create_publisher(
            String,
            'navigation/status',
            10
        )

        # Subscribers
        # GPS position
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10
        )

        # GPS speed
        self.speed_subscription = self.create_subscription(
            Float64,
            'gps/speed',
            self.speed_callback,
            10
        )

        # Wind direction
        self.wind_subscription = self.create_subscription(
            Float32,
            'wind/direction',
            self.wind_callback,
            10
        )

        # Navigation commands - enable/disable
        self.command_subscription = self.create_subscription(
            Int32,
            'navigation/command',
            self.command_callback,
            10
        )

        # Target waypoint - receive from event controller
        self.waypoint_subscription = self.create_subscription(
            String,
            'navigation/target',
            self.waypoint_callback,
            10
        )

        # Boat status subscription to monitor mode changes
        self.boat_status_subscription = self.create_subscription(
            String,
            'boat_status',
            self.boat_status_callback,
            10
        )

        # Phase-5: IMU subscriber for UKF fusion
        self.imu_subscription = self.create_subscription(
            String, 'imu/data', self.imu_callback, 10
        )

        # Phase-5: Wind speed subscriber for VPP
        self.wind_speed_subscription = self.create_subscription(
            Float32, 'wind/speed', self.wind_speed_callback, 10
        )

        # Phase-5: Heel angle subscriber
        self.heel_subscription = self.create_subscription(
            Float32, 'imu/heel', self.heel_callback, 10
        )

        # Phase-5 diagnostics publisher
        self.diag_publisher = self.create_publisher(
            String, 'navigation/phase5_diagnostics', 10
        )

        arch = []
        if self.use_ukf:  arch.append('UKF')
        if self.use_mpc:  arch.append('MPC')
        if self.use_vpp:  arch.append('VPP')
        self.get_logger().info(
            f"Navigation node initialised — Phase-5 architecture: "
            f"{'+'.join(arch) if arch else 'Phase-4 fallback'}"
        )

    def gps_callback(self, msg: NavSatFix):
        """Handle GPS position updates and calculate heading directly from position changes"""
        timestamp = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds

        # Store previous position for heading calculation
        self.previous_lat = self.current_lat
        self.previous_lon = self.current_lon

        # Update current position
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        # Calculate heading from GPS track if we have previous position
        if self.previous_lat != 0.0 and self.previous_lon != 0.0:
            # Time elapsed since last GPS update
            dt = timestamp - self.gps_timestamp

            if dt > 0.1:  # Only update if enough time passed (100ms)
                # Calculate heading from position change
                dx = self.current_lon - self.previous_lon
                dy = self.current_lat - self.previous_lat

                if abs(dx) > 1e-7 or abs(dy) > 1e-7:  # Avoid tiny movements
                    # Convert to heading (0-360°, 0 = North, clockwise)
                    self.current_heading = math.degrees(math.atan2(dx, dy)) % 360

                    # Calculate rotational velocity (in rad/s)
                    # This will (might) be replaced with IMU data when available
                    heading_change = math.radians((self.current_heading - self.previous_heading + 180) % 360 - 180)
                    self.rotational_velocity = heading_change / dt
                    self.previous_heading = self.current_heading

        self.gps_timestamp = timestamp

        # ── Phase-5: Feed GPS into UKF ───────────────────────────────
        if self.ukf is not None:
            self.ukf.update_gps(msg.latitude, msg.longitude)
            # Extract fused state from UKF
            ukf_lat, ukf_lon = self.ukf.get_position()
            if ukf_lat != 0.0 or ukf_lon != 0.0:
                self.current_lat = ukf_lat
                self.current_lon = ukf_lon
            self.current_heading = self.ukf.get_heading_deg()
            self.current_speed = self.ukf.get_speed()
            self.heel_angle = self.ukf.get_heel_deg()
            self.yaw_rate = self.ukf.get_yaw_rate()

        # If in autonomous mode, check if we've reached the target
        if self.navigation_enabled and self.active_course:
            self.check_waypoint_reached()

    def speed_callback(self, msg: Float64):
        """Handle GPS speed updates"""
        self.current_speed = msg.data

    def wind_callback(self, msg: Float32):
        """Handle wind direction updates"""
        self.wind_direction = msg.data

    def wind_speed_callback(self, msg: Float32):
        """Handle wind speed updates (Phase-5: feeds VPP)"""
        self.wind_speed = msg.data
        # Feed wind into UKF dynamics model
        if self.ukf is not None:
            self.ukf.wind_speed = msg.data
            self.ukf.wind_angle = math.radians(self.wind_direction)

    def heel_callback(self, msg: Float32):
        """Handle heel angle updates from IMU"""
        self.heel_angle = msg.data

    def imu_callback(self, msg: String):
        """Handle IMU data (JSON) — feeds UKF heading, heel, gyro."""
        try:
            data = json.loads(msg.data)
            heading = data.get('heading', self.current_heading)
            heel = data.get('heel', 0.0)
            roll_rate = data.get('roll_rate', 0.0)
            yaw_rate = data.get('yaw_rate', 0.0)

            self.imu_heading = heading
            self.roll_rate = roll_rate
            self.yaw_rate = yaw_rate

            if self.ukf is not None:
                self.ukf.update_imu(
                    math.radians(heading),
                    math.radians(heel),
                    roll_rate,
                    yaw_rate
                )
                # Run prediction step at IMU rate
                self.ukf.predict(0.02)  # ~50 Hz IMU assumption
        except Exception as e:
            self.get_logger().debug(f"IMU parse error: {e}")

    def command_callback(self, msg: Int32):
        """Handle navigation commands"""
        command = msg.data

        if command == 1:  # Enable autonomous navigation
            self.navigation_enabled = True
            self.rudder_pid.reset()
            if self.mpc is not None:
                self.mpc.reset()
            if self.vpp is not None:
                self.vpp.reset()
            self.get_logger().info("Autonomous navigation enabled (Phase-5 controllers reset)")
        elif command == 0:  # Disable autonomous navigation
            self.navigation_enabled = False
            self.get_logger().info("Autonomous navigation disabled")

    def boat_status_callback(self, msg: String):
        """Handle boat status updates to monitor mode changes"""
        try:
            status = json.loads(msg.data)
            control_mode = status.get('control_mode', '')
            
            # If mode is RC, disable autonomous navigation
            if control_mode == 'rc':
                if self.navigation_enabled:
                    self.navigation_enabled = False
                    self.get_logger().info("Navigation disabled - boat in RC mode")
            # If mode is autonomous, enable navigation 
            elif control_mode == 'autonomous':
                if not self.navigation_enabled:
                    self.navigation_enabled = True
                    self.rudder_pid.reset()
                    self.get_logger().info("Navigation enabled - boat in autonomous mode")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing boat status: {e}")

    def waypoint_callback(self, msg: String):
        """Handle new target waypoint from event controller"""
        try:
            # Parse waypoint from JSON
            waypoint_data = json.loads(msg.data)

            # Extract lat/lon from JSON
            lat = waypoint_data.get('lat', 0.0)
            lon = waypoint_data.get('lon', 0.0)

            self.get_logger().info(f"Received new target waypoint: lat={lat}, lon={lon}")

            # Calculate path to this waypoint
            self.calculate_path((self.current_lat, self.current_lon), (lat, lon))

            # Reset PID controller for new path
            self.rudder_pid.reset()

        except Exception as e:
            self.get_logger().error(f"Error processing waypoint: {e}")

    def calculate_path(self, start: Tuple[float, float], end: Tuple[float, float]):
        """Calculate path from start to end, considering wind direction"""
        self.get_logger().info(f"Calculating path from {start} to {end}")

        # Use Leg calculator to determine optimal path
        waypoints = self.leg_calculator.calculate_path(
            start,
            end,
            self.wind_direction,
            self.current_heading
        )

        # Update navigation state
        self.active_course = [start] + waypoints
        self.current_target_idx = 1  # First target is first waypoint after start
        self.last_path_time = self.get_clock().now().nanoseconds / 1e9

        # Log path details
        if len(waypoints) > 1:
            self.get_logger().info(
                f"Path requires tacking/jibing - {len(waypoints)} points"
            )
        else:
            self.get_logger().info("Direct path possible")

        # Store full path for visualization
        self.full_path = self.active_course

        # Publish navigation status update
        self.publish_status()

        return self.active_course

    def path_planning_callback(self):
        """
        Path planning timer callback - runs at slower rate
        Handles checking if we need to recalculate path
        """
        if not self.navigation_enabled:
            return

        # Check for significant wind direction changes
        # This would be a good spot to recalculate path if wind shifts

        # For now, just publish status updates
        self.publish_status()

    def calculate_apparent_wind_angle(self) -> float:
        """
        Calculate wind angle relative to boat
        
        Since we're using fixed wind speed, we just return the true wind angle
        relative to the boat heading.

        Returns:
            Wind angle in degrees relative to boat heading
        """
        # Calculate true wind angle relative to boat
        # wind_direction is where wind is coming FROM
        wind_angle_relative = (self.wind_direction - self.current_heading) % 360

        # Normalize to [-180, 180]
        if wind_angle_relative > 180:
            wind_angle_relative -= 360

        return wind_angle_relative

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-180, 180] range"""
        angle = angle % 360
        if angle > 180:
            angle = -180 + (angle - 180)
        return angle

    def calculate_sail_angle(self) -> float:
        """
        Calculate optimal sail angle based on apparent wind
        Uses the angle of attack formula from the physics simulator

        Returns:
            Optimal sail angle in degrees
        """
        # Get apparent wind angle relative to boat
        apparent_wind = self.calculate_apparent_wind_angle()

        # Apply the angle of attack formula from the simulator
        # aoa(x) = (44/90) * x where x is normalized to [-180, 180]
        normalized_wind = self.normalize_angle(apparent_wind)
        sail_angle = (44.0 / 90.0) * normalized_wind

        # Clamp to reasonable sail angle range
        sail_angle = max(-90.0, min(90.0, sail_angle))

        return sail_angle

    def sail_control_callback(self):
        """Sail control callback — uses VPP (Phase-5) or AOA formula (fallback)."""
        if not self.navigation_enabled:
            return

        if self.vpp is not None:
            # ── Phase-5: VPP dynamic sail optimisation ───────────────
            target = self.get_current_target()
            target_bearing = 0.0
            if target:
                target_bearing = self.calculate_heading_to_target(target)

            sail_deg, vmg, pred_speed, diag = self.vpp.optimize(
                true_wind_speed=max(self.wind_speed, 0.5),
                true_wind_angle_deg=self.wind_direction,
                target_bearing_deg=target_bearing,
                boat_speed_hint=max(self.current_speed, 0.5),
                current_heel_deg=self.heel_angle
            )
            # Ensure Arduino-safe output [0, 88]
            sail_deg = self.vpp.to_arduino_angle(sail_deg)

            msg = Float32()
            msg.data = float(sail_deg)
            self.sail_angle_publisher.publish(msg)

            self.get_logger().info(
                f"VPP sail: {sail_deg:.1f}° | VMG={vmg:.2f} m/s | "
                f"L/D={diag.get('best_lift_to_drag', 0):.1f} | "
                f"heel={self.heel_angle:.1f}°"
            )

            # Publish Phase-5 diagnostics
            diag_msg = String()
            diag_msg.data = json.dumps({
                'controller': 'VPP', 'sail_deg': sail_deg,
                'vmg': vmg, 'predicted_speed': pred_speed, **diag
            })
            self.diag_publisher.publish(diag_msg)
        else:
            # ── Fallback: Phase-4 AOA formula ────────────────────────
            optimal_sail_angle = self.calculate_sail_angle()
            msg = Float32()
            msg.data = float(optimal_sail_angle)
            self.sail_angle_publisher.publish(msg)

    def rudder_control_callback(self):
        """Rudder control callback — uses MPC (Phase-5) or PID (fallback)."""
        if not self.navigation_enabled:
            return
        if not self.active_course:
            return

        target = self.get_current_target()
        if not target:
            return

        target_heading = self.calculate_heading_to_target(target)

        if self.mpc is not None:
            # ── Phase-5: Model Predictive Control ────────────────────
            yaw_rate_dps = math.degrees(self.yaw_rate)
            rudder_deg, diag = self.mpc.compute(
                heading_deg=self.current_heading,
                yaw_rate_dps=yaw_rate_dps,
                target_heading_deg=target_heading,
                boat_speed=max(self.current_speed, 0.5),
                wind_speed=self.wind_speed,
                wind_dir_deg=self.wind_direction
            )
            self.set_rudder_angle(rudder_deg)

            self.get_logger().info(
                f"MPC rudder: {rudder_deg:.1f}° | target={target_heading:.1f}° | "
                f"heading={self.current_heading:.1f}° | cost={diag['cost']:.2f}"
            )

            diag_msg = String()
            diag_msg.data = json.dumps({
                'controller': 'MPC', 'rudder_deg': rudder_deg,
                'heading': self.current_heading,
                'target': target_heading, **diag
            })
            self.diag_publisher.publish(diag_msg)
        else:
            # ── Fallback: PID ────────────────────────────────────────
            rudder_angle = self.calculate_rudder_angle_pid(target_heading)
            self.set_rudder_angle(rudder_angle)

    def calculate_heading_to_target(self, target: Tuple[float, float]) -> float:
        """
        Calculate desired heading to target waypoint

        Args:
            target: Tuple of (lat, lon) for target position

        Returns:
            Desired heading in degrees (0-360)
        """
        # Calculate direction vector to target
        dx = target[1] - self.current_lon  # lon difference
        dy = target[0] - self.current_lat  # lat difference

        # Convert to heading (0-360°, 0 = North, clockwise)
        heading = math.degrees(math.atan2(dx, dy)) % 360

        return heading

    def print_angle(self, angle: float) -> float:
        """Normalize angle to be between -180 and 180 degrees"""
        angle = angle % 360
        if angle > 180:
            angle = -180 + (angle - 180)
        return angle

    def calculate_rudder_angle_pid(self, target_heading: float) -> float:
        """
        Calculate rudder angle using PID controller

        Args:
            target_heading: Desired heading in degrees

        Returns:
            Rudder angle command in degrees
        """
        # Calculate heading error
        heading_error = self.print_angle(target_heading - self.current_heading)

        # Get time step
        dt = 1.0 / self.rudder_update_rate

        # Update PID controller
        if ADAPTIVE_PID_AVAILABLE and self.use_adaptive_pid:
            # Use adaptive PID with gain scheduling and feedforward
            rudder_correction, terms = self.rudder_pid.update(
                heading_error, dt, self.current_speed,
                self.use_gain_scheduling, self.use_feedforward
            )

            # Detailed logging with PID terms
            self.get_logger().info(
                f"Adaptive PID: target={target_heading:.1f}°, current={self.current_heading:.1f}°, "
                f"error={heading_error:.1f}°, speed={self.current_speed:.2f} m/s, "
                f"P={terms['P']:.2f}, I={terms['I']:.2f}, D={terms['D']:.2f}, FF={terms['FF']:.2f}, "
                f"Kp={terms['Kp']:.1f}, rudder={rudder_correction:.1f}°"
            )
        else:
            # Use standard PID
            heading_error_scaled = heading_error / 180.0
            rudder_correction = self.rudder_pid.update(heading_error_scaled, dt)
            rudder_correction = max(min(rudder_correction, 20), -20) * -1

            # Standard logging
            self.get_logger().info(
                f"PID Rudder: target={target_heading:.1f}°, "
                f"current={self.current_heading:.1f}°, "
                f"error={heading_error:.1f}°, "
                f"scaled_error={heading_error_scaled:.3f}, "
                f"rudder={rudder_correction:.1f}°"
            )

        return rudder_correction

    def set_rudder_angle(self, angle: float):
        """Send rudder angle command"""
        # Clamp angle to reasonable range
        angle = max(-45.0, min(45.0, angle))

        # Create and publish message
        msg = Float32()
        msg.data = float(angle)
        self.rudder_publisher.publish(msg)

    def get_current_target(self) -> Optional[Tuple[float, float]]:
        """Get current target waypoint from active course"""
        if (not self.active_course or
            self.current_target_idx >= len(self.active_course)):
            return None

        return self.active_course[self.current_target_idx]

    def check_waypoint_reached(self):
        """Check if current target waypoint has been reached"""
        target = self.get_current_target()
        if not target:
            return False

        # Calculate distance to target using haversine formula
        distance = self.calculate_distance(
            self.current_lat, self.current_lon,
            target[0], target[1]
        )

        # Check if target reached
        if distance < self.waypoint_threshold:
            self.get_logger().info(
                f"Waypoint {self.current_target_idx} reached: {target} "
                f"(distance: {distance:.1f}m)"
            )

            # Move to next waypoint
            self.current_target_idx += 1

            # Check if we've completed the course
            if self.current_target_idx >= len(self.active_course):
                self.get_logger().info("Navigation course completed")
            else:
                next_target = self.get_current_target()
                self.get_logger().info(f"New target: {next_target}")

            # Publish updated status
            self.publish_status()

            return True

        return False

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two points in meters"""
        # Haversine formula
        R = 6371000  # Earth radius in meters

        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Differences
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        # Haversine formula
        a = (math.sin(dlat/2)**2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    def publish_status(self):
        """Publish navigation status with Phase-5 diagnostics"""
        status = {
            "enabled": self.navigation_enabled,
            "current_position": [self.current_lat, self.current_lon],
            "heading": self.current_heading,
            "speed": self.current_speed,
            "wind_direction": self.wind_direction,
            "wind_speed": self.wind_speed,
            "apparent_wind_angle": self.calculate_apparent_wind_angle(),
            "optimal_sail_angle": self.calculate_sail_angle(),
            "active_course": self.active_course,
            "current_target_idx": self.current_target_idx,
            "full_path": self.full_path,
            "phase5_architecture": {
                "ukf": self.use_ukf,
                "mpc": self.use_mpc,
                "vpp": self.use_vpp,
            },
            "heel_angle": self.heel_angle,
            "yaw_rate_dps": math.degrees(self.yaw_rate),
        }

        # UKF covariance trace (uncertainty indicator)
        if self.ukf is not None:
            try:
                P = self.ukf.get_covariance()
                status["ukf_trace"] = float(np.trace(P))
                status["leeway_deg"] = self.ukf.get_leeway_deg()
            except Exception:
                pass

        # PID gains (fallback or diagnostic)
        try:
            status["pid_gains"] = {
                "kp": self.rudder_pid.Kp,
                "ki": self.rudder_pid.Ki,
                "kd": self.rudder_pid.Kd
            }
        except AttributeError:
            pass

        # Target info
        target = self.get_current_target()
        if target:
            status["current_target"] = target
            distance = self.calculate_distance(
                self.current_lat, self.current_lon,
                target[0], target[1]
            )
            status["distance_to_target"] = distance

        msg = String()
        msg.data = json.dumps(status, default=str)
        self.nav_status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
