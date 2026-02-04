#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from collections import deque
import numpy as np
import math


class AdaptiveWindKalmanFilter:
    """
    Adaptive Kalman filter for wind direction with gust detection

    State: [wind_direction, wind_rate]
    Measurement: wind_direction (from sensor)
    """

    def __init__(self):
        # State: [wind_direction (rad), wind_rate (rad/s)]
        self.state = np.array([0.0, 0.0])

        # State covariance
        self.P = np.eye(2) * 10.0

        # Base process noise (steady conditions)
        self.Q_base = np.diag([0.01, 0.001])

        # Gust process noise (gusty conditions)
        self.Q_gust = np.diag([1.0, 0.1])

        # Current process noise (adaptive)
        self.Q = self.Q_base.copy()

        # Measurement noise
        self.R = 0.1  # ~6° sensor noise

        # Gust detection
        self.innovation_threshold = 0.26  # 15° in radians
        self.gust_detected = False
        self.gust_counter = 0
        self.gust_threshold_count = 3

        # Steady wind estimate
        self.steady_wind = 0.0
        self.steady_wind_alpha = 0.01

        # Innovation history
        self.innovation_history = deque(maxlen=10)

        # Last update time
        self.last_time = None

    def predict(self, dt):
        """Predict step with constant angular velocity model"""
        F = np.array([[1, dt], [0, 1]])

        # Predict state
        self.state = F @ self.state

        # Normalize angle to [-π, π]
        self.state[0] = np.arctan2(np.sin(self.state[0]), np.cos(self.state[0]))

        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement_deg, current_time):
        """Update with wind direction measurement"""
        # Convert to radians and normalize
        measurement_rad = np.radians(measurement_deg)
        measurement_rad = np.arctan2(np.sin(measurement_rad), np.cos(measurement_rad))

        # Calculate dt
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
        else:
            dt = 0.1
        self.last_time = current_time

        # Prediction step
        self.predict(dt)

        # Measurement matrix
        H = np.array([[1, 0]])

        # Innovation (handle circular difference)
        predicted_angle = self.state[0]
        innovation = measurement_rad - predicted_angle
        innovation = np.arctan2(np.sin(innovation), np.cos(innovation))

        # Store innovation
        self.innovation_history.append(abs(innovation))

        # Gust detection
        if abs(innovation) > self.innovation_threshold:
            self.gust_counter += 1
        else:
            self.gust_counter = max(0, self.gust_counter - 1)

        # Update gust status
        if self.gust_counter >= self.gust_threshold_count:
            if not self.gust_detected:
                self.gust_detected = True
                self.Q = self.Q_gust.copy()
        else:
            if self.gust_detected:
                self.gust_detected = False
                self.Q = self.Q_base.copy()

        # Adaptive process noise
        if len(self.innovation_history) >= 5:
            innovation_std = np.std(self.innovation_history)
            scale_factor = 1.0 + innovation_std / 0.1
            self.Q = self.Q_base * scale_factor

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T / S

        # Update state
        self.state = self.state + K.flatten() * innovation

        # Normalize angle
        self.state[0] = np.arctan2(np.sin(self.state[0]), np.cos(self.state[0]))

        # Update covariance
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P

        # Update steady wind estimate (only when not in gust)
        if not self.gust_detected:
            diff = self.state[0] - self.steady_wind
            diff = np.arctan2(np.sin(diff), np.cos(diff))
            self.steady_wind += self.steady_wind_alpha * diff
            self.steady_wind = np.arctan2(np.sin(self.steady_wind), np.cos(self.steady_wind))

    def get_wind(self):
        """Get current wind estimate"""
        current_deg = np.degrees(self.state[0])
        if current_deg < 0:
            current_deg += 360

        steady_deg = np.degrees(self.steady_wind)
        if steady_deg < 0:
            steady_deg += 360

        rate_deg = np.degrees(self.state[1])
        uncertainty_deg = np.degrees(np.sqrt(self.P[0, 0]))

        return {
            'current': current_deg,
            'steady': steady_deg,
            'rate': rate_deg,
            'gust_detected': self.gust_detected,
            'uncertainty': uncertainty_deg,
            'innovation_std': np.degrees(np.std(self.innovation_history)) if len(self.innovation_history) > 0 else 0.0
        }

class WindSmootherNode(Node):
    """
    ROS2 Node for advanced wind filtering using adaptive Kalman filter

    Subscribes to:
    - 'wind/raw_direction': Raw wind direction in degrees

    Publishes to:
    - 'wind/direction': Smoothed wind direction in degrees
    - 'wind/steady_direction': Steady wind estimate in degrees
    - 'wind/gust_detected': Boolean flag for gust detection
    """

    def __init__(self):
        super().__init__('wind_smoother_node')

        # Declare parameters
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('use_kalman', True)  # Use Kalman filter vs median
        self.declare_parameter('window_size', 20)   # For median filter fallback

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.use_kalman = self.get_parameter('use_kalman').value
        self.window_size = self.get_parameter('window_size').value

        # Initialize filters
        if self.use_kalman:
            self.wind_filter = AdaptiveWindKalmanFilter()
            self.get_logger().info('Using Adaptive Kalman Filter for wind smoothing')
        else:
            self.wind_buffer = deque(maxlen=self.window_size)
            self.get_logger().info(f'Using median filter with window size: {self.window_size}')

        self.smoothed_wind_direction = 0.0

        # Publishers
        self.smoothed_publisher = self.create_publisher(Float32, 'wind/direction', 10)
        self.steady_publisher = self.create_publisher(Float32, 'wind/steady_direction', 10)
        self.gust_publisher = self.create_publisher(Bool, 'wind/gust_detected', 10)

        # Subscriber
        self.raw_subscription = self.create_subscription(
            Float32, 'wind/raw_direction', self.raw_wind_callback, 10
        )

        # Create timer for periodic updates (only for median filter)
        if not self.use_kalman:
            self.timer = self.create_timer(1.0/self.update_rate, self.publish_smoothed_data)

        self.get_logger().info('Wind smoother node initialized')
    
    def raw_wind_callback(self, msg: Float32):
        """Process raw wind measurement"""
        raw_direction = msg.data
        current_time = self.get_clock().now()

        if self.use_kalman:
            # Update Kalman filter
            self.wind_filter.update(raw_direction, current_time)

            # Get filtered wind
            wind_data = self.wind_filter.get_wind()

            # Publish smoothed wind
            smoothed_msg = Float32()
            smoothed_msg.data = float(wind_data['current'])
            self.smoothed_publisher.publish(smoothed_msg)

            # Publish steady wind
            steady_msg = Float32()
            steady_msg.data = float(wind_data['steady'])
            self.steady_publisher.publish(steady_msg)

            # Publish gust detection
            gust_msg = Bool()
            gust_msg.data = wind_data['gust_detected']
            self.gust_publisher.publish(gust_msg)

            # Log periodically
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0

            if self._log_counter % 50 == 0:
                self.get_logger().info(
                    f"Wind: {wind_data['current']:.1f}° "
                    f"(steady: {wind_data['steady']:.1f}°, "
                    f"gust: {wind_data['gust_detected']}, "
                    f"uncertainty: ±{wind_data['uncertainty']:.1f}°)"
                )
        else:
            # Use median filter (original implementation)
            self.wind_buffer.append(raw_direction)
            self.smoothed_wind_direction = self.calculate_circular_median()

            self.get_logger().debug(
                f"Raw: {raw_direction:.1f}°, Buffer size: {len(self.wind_buffer)}, "
                f"Smoothed: {self.smoothed_wind_direction:.1f}°"
            )
    
    def calculate_circular_median(self):
        """
        Calculate the median of circular data (wind directions).
        
        This handles the wrap-around at 0/360 degrees by converting to unit vectors,
        averaging them, and converting back to an angle.
        
        Returns:
            Median wind direction in degrees (0-360)
        """
        if not self.wind_buffer:
            return 0.0
        
        # For a single value, just return it
        if len(self.wind_buffer) == 1:
            return self.wind_buffer[0]
        
        # Convert degrees to radians for calculations
        angles_rad = [math.radians(angle) for angle in self.wind_buffer]
        
        # Method 1: Vector averaging for circular median
        # Convert to unit vectors
        x_components = [math.cos(angle) for angle in angles_rad]
        y_components = [math.sin(angle) for angle in angles_rad]
        
        # Calculate mean vector
        mean_x = np.mean(x_components)
        mean_y = np.mean(y_components)
        
        # Convert back to angle
        mean_angle_rad = math.atan2(mean_y, mean_x)
        mean_angle_deg = math.degrees(mean_angle_rad)
        
        # Normalize to 0-360 range
        if mean_angle_deg < 0:
            mean_angle_deg += 360
        
        # Alternative Method: Find the angle that minimizes circular distance
        # This is more computationally intensive but can be more robust
        if len(self.wind_buffer) >= 5:  # Only use for sufficient data
            median_angle = self.find_circular_median_minimizing_distance()
            return median_angle
        
        return mean_angle_deg
    
    def find_circular_median_minimizing_distance(self):
        """
        Find the angle that minimizes the sum of circular distances to all other angles.
        This is a more robust circular median calculation.
        
        Returns:
            Circular median in degrees (0-360)
        """
        min_total_distance = float('inf')
        median_angle = 0.0
        
        # Check each angle in the buffer as a potential median
        for candidate in self.wind_buffer:
            total_distance = 0.0
            
            # Calculate sum of circular distances from this candidate to all others
            for angle in self.wind_buffer:
                # Circular distance between two angles
                diff = abs(candidate - angle)
                if diff > 180:
                    diff = 360 - diff
                total_distance += diff
            
            # Update if this candidate has lower total distance
            if total_distance < min_total_distance:
                min_total_distance = total_distance
                median_angle = candidate
        
        return median_angle
    
    def circular_distance(self, angle1, angle2):
        """
        Calculate the shortest angular distance between two angles.
        
        Args:
            angle1, angle2: Angles in degrees
            
        Returns:
            Shortest angular distance in degrees (0-180)
        """
        diff = abs(angle1 - angle2)
        if diff > 180:
            diff = 360 - diff
        return diff
    
    def publish_smoothed_data(self):
        """
        Publish the smoothed wind direction.
        Only publishes if we have data in the buffer.
        """
        if self.wind_buffer:
            # Create and publish message
            msg = Float32()
            msg.data = float(self.smoothed_wind_direction)
            self.smoothed_publisher.publish(msg)
            
            # Log statistics periodically (every 10th update)
            if hasattr(self, '_update_count'):
                self._update_count += 1
            else:
                self._update_count = 0
                
            if self._update_count % 10 == 0:
                # Calculate some statistics for logging
                if len(self.wind_buffer) > 1:
                    # Calculate circular standard deviation
                    angles_rad = [math.radians(angle) for angle in self.wind_buffer]
                    x_components = [math.cos(angle) for angle in angles_rad]
                    y_components = [math.sin(angle) for angle in angles_rad]
                    
                    r = math.sqrt(np.mean(x_components)**2 + np.mean(y_components)**2)
                    circular_std = math.degrees(math.sqrt(-2 * math.log(r))) if r > 0 else 0
                    
                    self.get_logger().info(
                        f"Wind smoother stats - Buffer: {len(self.wind_buffer)}/{self.window_size}, "
                        f"Smoothed: {self.smoothed_wind_direction:.1f}°, "
                        f"Circular StdDev: {circular_std:.1f}°"
                    )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WindSmootherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()