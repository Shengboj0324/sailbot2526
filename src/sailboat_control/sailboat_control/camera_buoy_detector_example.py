#!/usr/bin/env python3
"""
Example camera buoy detector node that publishes angle offset and distance.
This is a template for your actual camera vision processing node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class CameraBuoyDetectorNode(Node):
    def __init__(self):
        super().__init__('camera_buoy_detector')
        
        # Publishers for buoy tracking data
        self.angle_publisher = self.create_publisher(
            Float32,
            'camera/buoy_angle_offset',
            10
        )
        
        self.distance_publisher = self.create_publisher(
            Float32,
            'camera/buoy_distance',
            10
        )
        
        # Timer for publishing updates (10Hz)
        self.timer = self.create_timer(0.1, self.publish_buoy_data)
        
        self.get_logger().info("Camera buoy detector node initialized")
    
    def publish_buoy_data(self):
        """
        Process camera image and publish buoy angle offset and distance.
        
        This is where you would:
        1. Capture camera frame
        2. Process image to detect orange buoy
        3. Calculate angle offset from boat heading
        4. Estimate distance based on buoy size in image
        5. Publish the data
        """
        
        # Example: Replace with actual computer vision processing
        buoy_detected = True  # From your vision algorithm
        
        if buoy_detected:
            # Calculate angle offset (degrees)
            # Positive = buoy is to starboard, Negative = buoy is to port
            angle_offset = 0.0  # Replace with actual calculation
            
            # Calculate distance (meters)
            # Based on buoy size in image or other method
            distance = 10.0  # Replace with actual calculation
            
            # Publish angle offset
            angle_msg = Float32()
            angle_msg.data = angle_offset
            self.angle_publisher.publish(angle_msg)
            
            # Publish distance
            distance_msg = Float32()
            distance_msg.data = distance
            self.distance_publisher.publish(distance_msg)
            
            self.get_logger().debug(
                f"Buoy detected - angle: {angle_offset:.1f}°, distance: {distance:.1f}m"
            )
        else:
            # If no buoy detected, publish large angle to indicate not found
            angle_msg = Float32()
            angle_msg.data = 999.0  # Special value indicating no detection
            self.angle_publisher.publish(angle_msg)
            
            distance_msg = Float32()
            distance_msg.data = float('inf')
            self.distance_publisher.publish(distance_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraBuoyDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()