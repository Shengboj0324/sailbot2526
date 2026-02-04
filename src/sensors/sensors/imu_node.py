#!/usr/bin/env python3
"""
IMU Node for BNO085 9-DOF sensor
Provides orientation, angular velocity, and linear acceleration data
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np
import time

class IMUNode(Node):
    """
    ROS2 Node for IMU sensor (simulated for testing without hardware)
    
    Publishes:
    - 'imu/data': Full IMU data (orientation, angular velocity, linear acceleration)
    - 'imu/heading': Heading in degrees (0-360)
    - 'imu/roll': Roll angle in degrees
    - 'imu/pitch': Pitch angle in degrees
    """
    
    def __init__(self):
        super().__init__('imu_node')
        
        # Parameters
        self.declare_parameter('update_rate', 50.0)  # Hz
        self.declare_parameter('simulated', True)  # Use simulated data for testing
        
        self.update_rate = self.get_parameter('update_rate').value
        self.simulated = self.get_parameter('simulated').value
        
        # Initialize hardware or simulation
        if not self.simulated:
            try:
                import board
                import busio
                import adafruit_bno08x
                from adafruit_bno08x.i2c import BNO08X_I2C
                
                i2c = busio.I2C(board.SCL, board.SDA)
                self.bno = BNO08X_I2C(i2c)
                self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
                self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
                self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
                self.get_logger().info('BNO085 IMU initialized')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize IMU hardware: {e}. Using simulation.')
                self.simulated = True
        
        # Simulation state
        self.sim_heading = 0.0
        self.sim_roll = 0.0
        self.sim_pitch = 0.0
        
        # Publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.heading_publisher = self.create_publisher(Float32, 'imu/heading', 10)
        self.roll_publisher = self.create_publisher(Float32, 'imu/roll', 10)
        self.pitch_publisher = self.create_publisher(Float32, 'imu/pitch', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0/self.update_rate, self.publish_imu_data)
        
        self.get_logger().info(f'IMU node initialized (simulated={self.simulated})')
    
    def quaternion_to_euler(self, quat_i, quat_j, quat_k, quat_real):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (quat_real * quat_i + quat_j * quat_k)
        cosr_cosp = 1 - 2 * (quat_i * quat_i + quat_j * quat_j)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (quat_real * quat_j - quat_k * quat_i)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (quat_real * quat_k + quat_i * quat_j)
        cosy_cosp = 1 - 2 * (quat_j * quat_j + quat_k * quat_k)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees and normalize yaw to 0-360
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)
        if yaw_deg < 0:
            yaw_deg += 360
        
        return roll_deg, pitch_deg, yaw_deg
    
    def get_simulated_data(self):
        """Generate simulated IMU data for testing"""
        # Slowly varying heading
        self.sim_heading = (self.sim_heading + 0.1) % 360
        self.sim_roll = 5.0 * np.sin(time.time() * 0.5)  # Gentle rocking
        self.sim_pitch = 3.0 * np.sin(time.time() * 0.3)
        
        # Convert to quaternion
        yaw_rad = np.radians(self.sim_heading)
        roll_rad = np.radians(self.sim_roll)
        pitch_rad = np.radians(self.sim_pitch)
        
        cy = np.cos(yaw_rad * 0.5)
        sy = np.sin(yaw_rad * 0.5)
        cp = np.cos(pitch_rad * 0.5)
        sp = np.sin(pitch_rad * 0.5)
        cr = np.cos(roll_rad * 0.5)
        sr = np.sin(roll_rad * 0.5)
        
        quat_real = cr * cp * cy + sr * sp * sy
        quat_i = sr * cp * cy - cr * sp * sy
        quat_j = cr * sp * cy + sr * cp * sy
        quat_k = cr * cp * sy - sr * sp * cy
        
        # Simulated gyro and accel
        gyro_x, gyro_y, gyro_z = 0.01, 0.01, 0.001
        accel_x, accel_y, accel_z = 0.0, 0.0, 9.81
        
        return (quat_i, quat_j, quat_k, quat_real), (gyro_x, gyro_y, gyro_z), (accel_x, accel_y, accel_z)
    
    def publish_imu_data(self):
        """Read and publish IMU data"""
        try:
            if self.simulated:
                quat, gyro, accel = self.get_simulated_data()
                quat_i, quat_j, quat_k, quat_real = quat
                gyro_x, gyro_y, gyro_z = gyro
                accel_x, accel_y, accel_z = accel
            else:
                quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
                gyro_x, gyro_y, gyro_z = self.bno.gyro
                accel_x, accel_y, accel_z = self.bno.acceleration
            
            # Convert to Euler
            roll, pitch, yaw = self.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
            
            # Publish full IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.orientation.x = quat_i
            imu_msg.orientation.y = quat_j
            imu_msg.orientation.z = quat_k
            imu_msg.orientation.w = quat_real
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            self.imu_publisher.publish(imu_msg)
            
            # Publish individual angles
            self.heading_publisher.publish(Float32(data=float(yaw)))
            self.roll_publisher.publish(Float32(data=float(roll)))
            self.pitch_publisher.publish(Float32(data=float(pitch)))
            
        except Exception as e:
            self.get_logger().error(f'Error reading IMU: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

