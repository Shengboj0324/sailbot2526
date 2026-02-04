#!/usr/bin/env python3
"""
System Health Monitor
Monitors sensor timeouts, actuator errors, and system health
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Bool, String
from sensor_msgs.msg import NavSatFix, Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time

class HealthMonitor(Node):
    """
    Monitor system health and publish diagnostics
    """
    
    def __init__(self):
        super().__init__('health_monitor')
        
        # Timeout thresholds (seconds)
        self.gps_timeout = 2.0
        self.imu_timeout = 1.0
        self.wind_timeout = 5.0
        self.compass_timeout = 2.0
        
        # Last message times
        self.last_gps_time = None
        self.last_imu_time = None
        self.last_wind_time = None
        self.last_compass_time = None
        
        # Sensor status
        self.gps_ok = False
        self.imu_ok = False
        self.wind_ok = False
        self.compass_ok = False
        
        # Actuator status
        self.rudder_ok = True
        self.sail_ok = True
        
        # System metrics
        self.boat_speed = 0.0
        self.heading = 0.0
        self.wind_direction = 0.0
        
        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.wind_sub = self.create_subscription(Float32, 'wind/direction', self.wind_callback, 10)
        self.compass_sub = self.create_subscription(Float32, 'compass/heading', self.compass_callback, 10)
        self.speed_sub = self.create_subscription(Float64, 'gps/speed', self.speed_callback, 10)
        self.heading_sub = self.create_subscription(Float32, 'state/heading', self.heading_callback, 10)
        
        # Publisher
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.health_pub = self.create_publisher(String, 'system/health', 10)
        
        # Timer for health checks
        self.timer = self.create_timer(1.0, self.check_health)
        
        self.get_logger().info('Health Monitor initialized')
    
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
    
    def check_health(self):
        """Check system health and publish diagnostics"""
        current_time = time.time()
        
        # Check sensor timeouts
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
        
        # Create diagnostic message
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # GPS status
        gps_status = DiagnosticStatus()
        gps_status.name = 'GPS'
        gps_status.level = DiagnosticStatus.OK if self.gps_ok else DiagnosticStatus.ERROR
        gps_status.message = 'OK' if self.gps_ok else 'TIMEOUT'
        diag_array.status.append(gps_status)
        
        # IMU status
        imu_status = DiagnosticStatus()
        imu_status.name = 'IMU'
        imu_status.level = DiagnosticStatus.OK if self.imu_ok else DiagnosticStatus.ERROR
        imu_status.message = 'OK' if self.imu_ok else 'TIMEOUT'
        diag_array.status.append(imu_status)
        
        # Wind sensor status
        wind_status = DiagnosticStatus()
        wind_status.name = 'Wind Sensor'
        wind_status.level = DiagnosticStatus.OK if self.wind_ok else DiagnosticStatus.WARN
        wind_status.message = 'OK' if self.wind_ok else 'TIMEOUT'
        diag_array.status.append(wind_status)
        
        # Overall system status
        system_status = DiagnosticStatus()
        system_status.name = 'System'
        
        critical_ok = self.gps_ok and self.imu_ok
        if critical_ok:
            system_status.level = DiagnosticStatus.OK
            system_status.message = 'All critical systems operational'
        else:
            system_status.level = DiagnosticStatus.ERROR
            system_status.message = 'Critical system failure'
        
        system_status.values.append(KeyValue(key='boat_speed', value=f'{self.boat_speed:.2f} m/s'))
        system_status.values.append(KeyValue(key='heading', value=f'{self.heading:.1f}°'))
        system_status.values.append(KeyValue(key='wind_direction', value=f'{self.wind_direction:.1f}°'))
        
        diag_array.status.append(system_status)
        
        # Publish diagnostics
        self.diagnostics_pub.publish(diag_array)
        
        # Publish simple health message
        health_msg = String()
        if critical_ok:
            health_msg.data = 'HEALTHY'
        else:
            health_msg.data = 'DEGRADED'
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

