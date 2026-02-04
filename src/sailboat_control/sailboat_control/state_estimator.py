#!/usr/bin/env python3
"""
Extended Kalman Filter for boat state estimation
Fuses GPS, IMU, and Compass data for accurate position and heading
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for boat state estimation
    
    State vector (9 dimensions):
    [x, y, heading, vx, vy, heading_rate, ax, ay, heading_accel]
    """
    
    def __init__(self):
        # State vector
        self.state = np.zeros(9)
        
        # State covariance matrix
        self.P = np.eye(9) * 1.0
        
        # Process noise covariance
        self.Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01, 0.5, 0.5, 0.05])
        
        # Measurement noise covariances
        self.R_gps_pos = np.diag([0.01, 0.01])  # RTK GPS: 10cm std
        self.R_gps_vel = np.diag([0.05, 0.05])  # GPS velocity: 5cm/s std
        self.R_imu_heading = 0.0005  # IMU heading: ~1° std
        self.R_imu_rate = 0.0001  # IMU gyro: very accurate
        self.R_imu_accel = np.diag([0.1, 0.1])  # IMU accel: 0.1 m/s² std
        
        # Reference position for local frame
        self.ref_lat = None
        self.ref_lon = None
        self.last_time = None
    
    def set_reference_position(self, lat, lon):
        """Set reference position for local coordinate frame"""
        self.ref_lat = lat
        self.ref_lon = lon
    
    def latlon_to_xy(self, lat, lon):
        """Convert lat/lon to local x/y coordinates"""
        if self.ref_lat is None:
            self.set_reference_position(lat, lon)
            return 0.0, 0.0
        
        R_earth = 6371000  # Earth radius in meters
        x = R_earth * np.radians(lon - self.ref_lon) * np.cos(np.radians(self.ref_lat))
        y = R_earth * np.radians(lat - self.ref_lat)
        return x, y
    
    def xy_to_latlon(self, x, y):
        """Convert local x/y to lat/lon"""
        if self.ref_lat is None:
            return 0.0, 0.0
        
        R_earth = 6371000
        lat = self.ref_lat + np.degrees(y / R_earth)
        lon = self.ref_lon + np.degrees(x / (R_earth * np.cos(np.radians(self.ref_lat))))
        return lat, lon
    
    def predict(self, dt):
        """Prediction step using constant acceleration motion model"""
        # State transition matrix
        F = np.eye(9)
        F[0, 3] = dt  # x += vx * dt
        F[0, 6] = 0.5 * dt**2  # x += 0.5 * ax * dt²
        F[1, 4] = dt  # y += vy * dt
        F[1, 7] = 0.5 * dt**2  # y += 0.5 * ay * dt²
        F[2, 5] = dt  # heading += heading_rate * dt
        F[2, 8] = 0.5 * dt**2  # heading += 0.5 * heading_accel * dt²
        F[3, 6] = dt  # vx += ax * dt
        F[4, 7] = dt  # vy += ay * dt
        F[5, 8] = dt  # heading_rate += heading_accel * dt
        
        # Predict state
        self.state = F @ self.state
        
        # Normalize heading to [-π, π]
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update_gps_position(self, lat, lon):
        """Update with GPS position measurement"""
        x_meas, y_meas = self.latlon_to_xy(lat, lon)
        z = np.array([x_meas, y_meas])
        
        H = np.zeros((2, 9))
        H[0, 0] = 1  # Measure x
        H[1, 1] = 1  # Measure y
        
        y_innov = z - H @ self.state
        S = H @ self.P @ H.T + self.R_gps_pos
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y_innov
        self.P = (np.eye(9) - K @ H) @ self.P
    
    def update_gps_velocity(self, vx, vy):
        """Update with GPS velocity measurement"""
        z = np.array([vx, vy])
        
        H = np.zeros((2, 9))
        H[0, 3] = 1  # Measure vx
        H[1, 4] = 1  # Measure vy
        
        y_innov = z - H @ self.state
        S = H @ self.P @ H.T + self.R_gps_vel
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y_innov
        self.P = (np.eye(9) - K @ H) @ self.P
    
    def update_imu_heading(self, heading_rad):
        """Update with IMU heading measurement"""
        H = np.zeros((1, 9))
        H[0, 2] = 1  # Measure heading
        
        predicted_heading = self.state[2]
        innovation = heading_rad - predicted_heading
        innovation = np.arctan2(np.sin(innovation), np.cos(innovation))
        
        S = H @ self.P @ H.T + self.R_imu_heading
        K = self.P @ H.T / S
        
        self.state = self.state + K.flatten() * innovation
        self.P = (np.eye(9) - np.outer(K, H)) @ self.P
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
    
    def update_imu_gyro(self, heading_rate):
        """Update with IMU gyroscope (heading rate)"""
        H = np.zeros((1, 9))
        H[0, 5] = 1  # Measure heading_rate
        
        y_innov = heading_rate - H @ self.state
        S = H @ self.P @ H.T + self.R_imu_rate
        K = self.P @ H.T / S
        
        self.state = self.state + K.flatten() * y_innov
        self.P = (np.eye(9) - np.outer(K, H)) @ self.P
    
    def update_imu_accel(self, ax, ay):
        """Update with IMU accelerometer"""
        z = np.array([ax, ay])
        
        H = np.zeros((2, 9))
        H[0, 6] = 1  # Measure ax
        H[1, 7] = 1  # Measure ay
        
        y_innov = z - H @ self.state
        S = H @ self.P @ H.T + self.R_imu_accel
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y_innov
        self.P = (np.eye(9) - K @ H) @ self.P
    
    def get_state(self):
        """Return current state estimate"""
        lat, lon = self.xy_to_latlon(self.state[0], self.state[1])
        return {
            'position': (lat, lon),
            'position_xy': (self.state[0], self.state[1]),
            'heading': np.degrees(self.state[2]),
            'heading_rad': self.state[2],
            'velocity': (self.state[3], self.state[4]),
            'speed': np.sqrt(self.state[3]**2 + self.state[4]**2),
            'heading_rate': np.degrees(self.state[5]),
            'heading_rate_rad': self.state[5],
            'acceleration': (self.state[6], self.state[7]),
            'covariance': self.P
        }


class StateEstimatorNode(Node):
    """
    ROS2 Node that fuses GPS, IMU, and Compass data using EKF

    Subscribes to:
    - 'gps/fix': GPS position
    - 'gps/speed': GPS speed
    - 'imu/data': Full IMU data
    - 'imu/heading': IMU heading

    Publishes:
    - 'state/pose': Estimated pose with covariance
    - 'state/twist': Estimated velocity with covariance
    - 'state/heading': Estimated heading (Float32)
    - 'state/position': Estimated GPS position (NavSatFix)
    """

    def __init__(self):
        super().__init__('state_estimator_node')

        # Create EKF
        self.ekf = ExtendedKalmanFilter()

        # Parameters
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.publish_rate = self.get_parameter('publish_rate').value

        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.gps_speed_sub = self.create_subscription(Float64, 'gps/speed', self.gps_speed_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.imu_heading_sub = self.create_subscription(Float32, 'imu/heading', self.imu_heading_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'state/pose', 10)
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, 'state/twist', 10)
        self.heading_pub = self.create_publisher(Float32, 'state/heading', 10)
        self.position_pub = self.create_publisher(NavSatFix, 'state/position', 10)

        # Timer for prediction and publishing
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)

        # Last GPS velocity
        self.last_gps_speed = 0.0
        self.last_gps_heading = 0.0

        self.get_logger().info('State Estimator Node initialized')

    def gps_callback(self, msg: NavSatFix):
        """Handle GPS position update"""
        if msg.status.status >= 0:  # Valid fix
            self.ekf.update_gps_position(msg.latitude, msg.longitude)

    def gps_speed_callback(self, msg: Float64):
        """Handle GPS speed update"""
        self.last_gps_speed = msg.data

    def imu_callback(self, msg: Imu):
        """Handle full IMU data"""
        # Update with gyroscope (heading rate)
        heading_rate = msg.angular_velocity.z
        self.ekf.update_imu_gyro(heading_rate)

        # Update with accelerometer
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        self.ekf.update_imu_accel(ax, ay)

    def imu_heading_callback(self, msg: Float32):
        """Handle IMU heading"""
        heading_rad = np.radians(msg.data)
        self.ekf.update_imu_heading(heading_rad)

    def timer_callback(self):
        """Prediction and publishing at fixed rate"""
        current_time = self.get_clock().now()

        # Calculate dt
        if self.ekf.last_time is not None:
            dt = (current_time.nanoseconds - self.ekf.last_time) / 1e9
        else:
            dt = 1.0 / self.publish_rate

        self.ekf.last_time = current_time.nanoseconds

        # Prediction step
        self.ekf.predict(dt)

        # Publish state estimate
        self.publish_state()

    def publish_state(self):
        """Publish current state estimate"""
        state = self.ekf.get_state()

        # Publish pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = state['position_xy'][0]
        pose_msg.pose.pose.position.y = state['position_xy'][1]
        pose_msg.pose.pose.position.z = 0.0

        heading_rad = state['heading_rad']
        pose_msg.pose.pose.orientation.z = np.sin(heading_rad / 2)
        pose_msg.pose.pose.orientation.w = np.cos(heading_rad / 2)

        cov = np.zeros(36)
        cov[0] = state['covariance'][0, 0]
        cov[7] = state['covariance'][1, 1]
        cov[35] = state['covariance'][2, 2]
        pose_msg.pose.covariance = cov.tolist()
        self.pose_pub.publish(pose_msg)

        # Publish twist
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header = pose_msg.header
        twist_msg.twist.twist.linear.x = state['velocity'][0]
        twist_msg.twist.twist.linear.y = state['velocity'][1]
        twist_msg.twist.twist.angular.z = state['heading_rate_rad']
        self.twist_pub.publish(twist_msg)

        # Publish heading
        heading_msg = Float32()
        heading_msg.data = float(state['heading'])
        self.heading_pub.publish(heading_msg)

        # Publish position
        pos_msg = NavSatFix()
        pos_msg.header = pose_msg.header
        pos_msg.latitude = state['position'][0]
        pos_msg.longitude = state['position'][1]
        pos_msg.status.status = 1  # GPS fix
        self.position_pub.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

