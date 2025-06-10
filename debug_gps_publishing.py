#!/usr/bin/env python3
"""
Debug script to monitor GPS data flow and verify publishing to web server
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Int32, String
import json
import time

class GPSDebugNode(Node):
    def __init__(self):
        super().__init__('gps_debug_node')
        
        # Subscribe to GPS topics
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10
        )
        
        self.speed_sub = self.create_subscription(
            Float64,
            'gps/speed',
            self.speed_callback,
            10
        )
        
        self.fix_sub = self.create_subscription(
            Int32,
            'gps/fix_quality',
            self.fix_quality_callback,
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            'boat_status',
            self.status_callback,
            10
        )
        
        # Store latest data
        self.latest_gps = {
            "latitude": 0.0,
            "longitude": 0.0,
            "speed": 0.0,
            "fix_quality": 0,
            "timestamp": 0
        }
        
        self.get_logger().info("GPS Debug Node started - monitoring GPS data flow")
        
        # Create timer to display data periodically
        self.create_timer(2.0, self.display_status)
        
    def gps_callback(self, msg: NavSatFix):
        self.latest_gps["latitude"] = msg.latitude
        self.latest_gps["longitude"] = msg.longitude
        self.latest_gps["timestamp"] = time.time()
        self.get_logger().info(f"GPS Fix received: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}")
        
    def speed_callback(self, msg: Float64):
        self.latest_gps["speed"] = msg.data
        self.get_logger().info(f"GPS Speed received: {msg.data:.2f} m/s ({msg.data * 3.6:.2f} km/h)")
        
    def fix_quality_callback(self, msg: Int32):
        self.latest_gps["fix_quality"] = msg.data
        fix_map = {0: "Invalid", 1: "GPS", 2: "DGPS", 4: "RTK Fixed", 5: "RTK Float"}
        fix_status = fix_map.get(msg.data, "Unknown")
        self.get_logger().info(f"GPS Fix Quality received: {msg.data} ({fix_status})")
        
    def status_callback(self, msg: String):
        try:
            status = json.loads(msg.data)
            self.get_logger().info(f"Boat Status: Mode={status.get('control_mode', 'unknown')}, Event={status.get('event_type', 'unknown')}")
        except:
            pass
            
    def display_status(self):
        """Display current GPS data status"""
        print("\n" + "="*60)
        print("GPS DATA STATUS")
        print("="*60)
        print(f"Latitude:     {self.latest_gps['latitude']:.6f}")
        print(f"Longitude:    {self.latest_gps['longitude']:.6f}")
        print(f"Speed:        {self.latest_gps['speed']:.2f} m/s ({self.latest_gps['speed'] * 3.6:.2f} km/h)")
        print(f"Fix Quality:  {self.latest_gps['fix_quality']}")
        
        # Check data freshness
        age = time.time() - self.latest_gps['timestamp']
        if age < 5.0:
            print(f"Data Age:     {age:.1f}s (FRESH)")
        else:
            print(f"Data Age:     {age:.1f}s (STALE - check GPS connection)")
            
        # Show what would be sent to web server
        print("\nData that should be sent to web server:")
        print(f"GPS,{self.latest_gps['latitude']:.6f},{self.latest_gps['longitude']:.6f},{self.latest_gps['speed']:.2f},{self.latest_gps['fix_quality']}")
        print("="*60)

def main(args=None):
    rclpy.init(args=args)
    node = GPSDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()