#!/usr/bin/env python3
"""
Optimal Sail Controller using polar diagrams
Implements apparent wind calculation, polar-based trim, and depowering logic
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Bool
import numpy as np
import yaml
import os

class OptimalSailController:
    """Optimal sail control using polar diagrams"""
    
    def __init__(self, polar_data=None):
        # Default polar data if none provided
        if polar_data is None:
            self.polar_data = self.get_default_polar_data()
        else:
            self.polar_data = polar_data
        
        self.current_sail_angle = 0.0
    
    def get_default_polar_data(self):
        """Default polar diagram data"""
        return {
            'twa': [0, 30, 45, 60, 90, 120, 135, 150, 180],
            'tws': [5, 8, 10, 12, 15, 20],
            'sail_angle': [
                [0, 0, 0, 0, 0, 0],      # 0° TWA
                [10, 12, 15, 18, 20, 22],  # 30° TWA
                [15, 18, 22, 25, 28, 30],  # 45° TWA
                [25, 30, 35, 38, 40, 42],  # 60° TWA
                [40, 45, 50, 52, 55, 58],  # 90° TWA
                [50, 55, 60, 62, 65, 68],  # 120° TWA
                [60, 65, 70, 72, 75, 78],  # 135° TWA
                [70, 75, 80, 82, 85, 88],  # 150° TWA
                [80, 85, 88, 88, 88, 88]   # 180° TWA
            ]
        }
    
    def calculate_apparent_wind(self, true_wind_angle, true_wind_speed, boat_speed, boat_heading):
        """Calculate apparent wind from true wind and boat motion"""
        # Convert to vectors
        true_wind_x = true_wind_speed * np.cos(np.radians(true_wind_angle))
        true_wind_y = true_wind_speed * np.sin(np.radians(true_wind_angle))
        
        # Boat velocity (opposite direction)
        boat_x = -boat_speed * np.cos(np.radians(boat_heading))
        boat_y = -boat_speed * np.sin(np.radians(boat_heading))
        
        # Apparent wind = true wind - boat velocity
        apparent_x = true_wind_x - boat_x
        apparent_y = true_wind_y - boat_y
        
        # Convert back to angle and speed
        apparent_speed = np.sqrt(apparent_x**2 + apparent_y**2)
        apparent_angle = np.degrees(np.arctan2(apparent_y, apparent_x))
        
        return apparent_angle, apparent_speed
    
    def get_optimal_sail_angle(self, apparent_wind_angle, apparent_wind_speed):
        """Look up optimal sail angle from polar diagram"""
        # Normalize angle to 0-180 (symmetric for port/starboard)
        awa = abs(apparent_wind_angle)
        if awa > 180:
            awa = 360 - awa
        
        # Bilinear interpolation
        twa_idx = np.searchsorted(self.polar_data['twa'], awa)
        tws_idx = np.searchsorted(self.polar_data['tws'], apparent_wind_speed)
        
        # Clamp to valid range
        twa_idx = np.clip(twa_idx, 1, len(self.polar_data['twa']) - 1)
        tws_idx = np.clip(tws_idx, 1, len(self.polar_data['tws']) - 1)
        
        # Get surrounding points
        twa_low, twa_high = self.polar_data['twa'][twa_idx-1], self.polar_data['twa'][twa_idx]
        tws_low, tws_high = self.polar_data['tws'][tws_idx-1], self.polar_data['tws'][tws_idx]
        
        # Interpolation weights
        twa_weight = (awa - twa_low) / (twa_high - twa_low) if twa_high != twa_low else 0
        tws_weight = (apparent_wind_speed - tws_low) / (tws_high - tws_low) if tws_high != tws_low else 0
        
        # Bilinear interpolation
        sail_00 = self.polar_data['sail_angle'][twa_idx-1][tws_idx-1]
        sail_01 = self.polar_data['sail_angle'][twa_idx-1][tws_idx]
        sail_10 = self.polar_data['sail_angle'][twa_idx][tws_idx-1]
        sail_11 = self.polar_data['sail_angle'][twa_idx][tws_idx]
        
        sail_0 = sail_00 * (1 - tws_weight) + sail_01 * tws_weight
        sail_1 = sail_10 * (1 - tws_weight) + sail_11 * tws_weight
        
        optimal_sail = sail_0 * (1 - twa_weight) + sail_1 * twa_weight
        
        return optimal_sail
    
    def apply_depowering(self, sail_angle, heel_angle, gust_detected, max_heel=25.0):
        """Depower sail in high wind or excessive heel"""
        depower_factor = 1.0
        
        # Depower based on heel angle
        if abs(heel_angle) > max_heel:
            depower_factor = 1.0 + 0.5 * (abs(heel_angle) - max_heel) / 10.0
            depower_factor = min(depower_factor, 1.5)
        
        # Additional depowering in gusts
        if gust_detected:
            depower_factor *= 1.2
        
        # Apply depowering (ease sail out)
        depowered_sail = sail_angle * depower_factor
        
        # Clamp to physical limits
        depowered_sail = np.clip(depowered_sail, 0, 88)
        
        return depowered_sail
    
    def rate_limit_sail(self, target_sail, max_rate=10.0, dt=1.0):
        """Limit rate of sail change for smooth control"""
        max_change = max_rate * dt
        
        change = target_sail - self.current_sail_angle
        
        if abs(change) > max_change:
            change = np.sign(change) * max_change
        
        self.current_sail_angle += change
        return self.current_sail_angle
    
    def calculate_sail_angle(self, true_wind_angle, true_wind_speed, boat_speed, 
                            boat_heading, heel_angle, gust_detected):
        """Calculate optimal sail angle with all considerations"""
        # Calculate apparent wind
        awa, aws = self.calculate_apparent_wind(
            true_wind_angle, true_wind_speed, boat_speed, boat_heading
        )
        
        # Get optimal sail angle from polars
        optimal_sail = self.get_optimal_sail_angle(awa, aws)
        
        # Apply depowering
        depowered_sail = self.apply_depowering(optimal_sail, heel_angle, gust_detected)
        
        # Rate limiting
        final_sail = self.rate_limit_sail(depowered_sail, max_rate=10.0, dt=1.0)
        
        return final_sail, awa, aws


class OptimalSailControllerNode(Node):
    """ROS2 Node for optimal sail control"""

    def __init__(self):
        super().__init__('optimal_sail_controller')

        # Create controller
        self.controller = OptimalSailController()

        # State
        self.true_wind_angle = 0.0
        self.true_wind_speed = 8.0  # Default
        self.boat_speed = 0.0
        self.boat_heading = 0.0
        self.heel_angle = 0.0
        self.gust_detected = False

        # Parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.update_rate = self.get_parameter('update_rate').value

        # Subscribers
        self.wind_sub = self.create_subscription(Float32, 'wind/direction', self.wind_callback, 10)
        self.speed_sub = self.create_subscription(Float64, 'gps/speed', self.speed_callback, 10)
        self.heading_sub = self.create_subscription(Float32, 'state/heading', self.heading_callback, 10)
        self.gust_sub = self.create_subscription(Bool, 'wind/gust_detected', self.gust_callback, 10)
        self.heel_sub = self.create_subscription(Float32, 'imu/roll', self.heel_callback, 10)

        # Publisher
        self.sail_pub = self.create_publisher(Float32, 'sail/target_angle', 10)

        # Timer
        self.timer = self.create_timer(1.0/self.update_rate, self.control_loop)

        self.get_logger().info('Optimal Sail Controller initialized')

    def wind_callback(self, msg):
        self.true_wind_angle = msg.data

    def speed_callback(self, msg):
        self.boat_speed = msg.data

    def heading_callback(self, msg):
        self.boat_heading = msg.data

    def gust_callback(self, msg):
        self.gust_detected = msg.data

    def heel_callback(self, msg):
        self.heel_angle = msg.data

    def control_loop(self):
        """Calculate and publish optimal sail angle"""
        sail_angle, awa, aws = self.controller.calculate_sail_angle(
            self.true_wind_angle, self.true_wind_speed,
            self.boat_speed, self.boat_heading,
            self.heel_angle, self.gust_detected
        )

        # Publish
        msg = Float32()
        msg.data = float(sail_angle)
        self.sail_pub.publish(msg)

        # Log periodically
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0

        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f"Sail: {sail_angle:.1f}°, AWA: {awa:.1f}°, AWS: {aws:.1f} m/s, "
                f"Heel: {self.heel_angle:.1f}°, Gust: {self.gust_detected}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OptimalSailControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

