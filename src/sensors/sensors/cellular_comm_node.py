#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32, Float64
from sensor_msgs.msg import NavSatFix
import websocket
import struct
import threading
import time
import json

class CellularCommNode(Node):
    """
    ROS Node for cellular/internet communication between the sailbot and a remote control station.
    Receives commands from the remote station and publishes them to the 'radio_commands' topic.
    Sends status updates back to the remote station when requested, using simple text format.

    Now also handles RC control commands for rudder and sail angles.
    """
    def __init__(self):
        super().__init__('cellular_comm_node')

        # Parameters for WebSocket connection
        self.declare_parameter('relay_url', 'wss://sailbot-relay.onrender.com')
        self.declare_parameter('auth_token', 'antonius')
        self.declare_parameter('connection_type', 'boat')
        self.declare_parameter('status_interval', 5.0)  # Interval in seconds for auto status updates

        self.relay_url = self.get_parameter('relay_url').value
        self.auth_token = self.get_parameter('auth_token').value
        self.connection_type = self.get_parameter('connection_type').value
        self.status_interval = self.get_parameter('status_interval').value

        # Build WebSocket URL with query parameters
        self.ws_url = f"{self.relay_url}?type={self.connection_type}&auth={self.auth_token}"

        # Command publisher
        self.command_publisher = self.create_publisher(
            Int32,
            'radio_commands',
            10
        )

        # RC Control publishers
        self.rudder_command_publisher = self.create_publisher(
            Float32,
            'rudder/command',
            10
        )

        self.sail_angle_publisher = self.create_publisher(
            Float32,
            'sail/angle',
            10
        )

        # Subscribe to status info
        self.status_subscription = self.create_subscription(
            String,
            'boat_status',
            self.status_callback,
            10
        )

        # Subscribe to GPS data
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10
        )

        # Subscribe to GPS speed
        self.speed_subscription = self.create_subscription(
            Float64,
            'gps/speed',
            self.speed_callback,
            10
        )

        # Subscribe to GPS fix quality
        self.fix_quality_subscription = self.create_subscription(
            Int32,
            'gps/fix_quality',
            self.fix_quality_callback,
            10
        )

        # Subscribe to wind direction
        self.wind_subscription = self.create_subscription(
            Float32,
            'wind/direction',
            self.wind_callback,
            10
        )

        # Initialize sensor data storage
        self.latest_gps = {
            "latitude": 0.0,
            "longitude": 0.0,
            "fix_quality": 0,
            "speed": 0.0,
            "timestamp": 0
        }
        self.latest_wind_direction = 0.0

        # Status information - stored in simple format
        self.control_mode = "rc"
        self.event_type = "developer_mode"
        self.current_waypoint = None
        self.total_waypoints = 0
        self.last_completed_waypoint = None

        # WebSocket connection will be None initially
        self.ws = None
        self.ws_lock = threading.Lock()

        # Create thread for WebSocket connection with auto-reconnect
        self.running = True
        self.ws_thread = threading.Thread(target=self.websocket_handler)
        self.ws_thread.daemon = True
        self.ws_thread.start()

        # Timer for periodic status updates
        self.create_timer(self.status_interval, self.send_periodic_status)

        self.get_logger().info('Cellular Communication Node initialized')

    def on_ws_open(self, ws):
        """Called when WebSocket connection is established"""
        self.get_logger().info('Connected to relay server')
        # Send initial connection message
        self.send_initial_message()

    def on_ws_message(self, ws, message):
        """Called when a WebSocket message is received"""
        try:
            # Handle binary messages (commands)
            if isinstance(message, bytes):
                self.process_binary_message(message)
            else:
                pass  # Ignore text messages
        except Exception as e:
            self.get_logger().error(f'Error processing WebSocket message: {e}')

    def on_ws_error(self, ws, error):
        """Called when WebSocket encounters an error"""
        self.get_logger().error(f'WebSocket error: {error}')

    def on_ws_close(self, ws, close_status_code, close_msg):
        """Called when WebSocket connection is closed"""
        self.get_logger().debug(f'WebSocket connection closed: {close_status_code}')

    def websocket_handler(self):
        """Background thread to handle WebSocket connection with auto-reconnect"""
        while self.running:
            try:
                # Create WebSocket connection
                self.get_logger().debug(f'Connecting to WebSocket: {self.ws_url}')
                
                websocket.enableTrace(False)  # Set to True for debugging
                self.ws = websocket.WebSocketApp(
                    self.ws_url,
                    on_open=self.on_ws_open,
                    on_message=self.on_ws_message,
                    on_error=self.on_ws_error,
                    on_close=self.on_ws_close
                )
                
                # Run WebSocket (this blocks until connection is closed)
                self.ws.run_forever()
                
                # Connection closed, wait before reconnecting
                if self.running:
                    self.get_logger().debug('Reconnecting...')
                    time.sleep(2)
                    
            except Exception as e:
                self.get_logger().error(f'WebSocket handler error: {e}')
                if self.running:
                    time.sleep(2)

    def process_binary_message(self, message):
        """Process binary message received from WebSocket"""
        buffer = bytearray(message)
        
        # Check for command headers (0xC0 or 0xAA)
        if len(buffer) < 2:
            return
            
        # Handle new format (0xAA header) for RC commands
        if buffer[0] == 0xAA:
            command = buffer[1]
            
            # New RC format needs 5 bytes total
            if command in [10, 11] and len(buffer) >= 5:
                # Extract 2-byte value (little-endian) and checksum
                value_low = buffer[2]
                value_high = buffer[3]
                checksum = buffer[4]
                
                # Reconstruct the value
                value = value_low | (value_high << 8)
                
                # Validate checksum
                expected_checksum = (command ^ value) & 0xFF
                if checksum == expected_checksum:
                    # Send acknowledgment
                    ack_msg = bytearray([0xA0, command, command ^ 0xFF])
                    self.safe_ws_send(ack_msg)
                    
                    # Process RC command with direct angle value
                    if command == 10:  # Rudder
                        # Value is already the angle in degrees
                        self.process_rc_rudder_angle(float(value))
                    elif command == 11:  # Sail
                        # Value is already the angle in degrees
                        self.process_rc_sail_angle(float(value))
                else:
                    self.get_logger().warning(f'Invalid checksum for RC command {command}')
                    
        # Handle old format (0xC0 header)
        elif buffer[0] == 0xC0:
            command = buffer[1]
            
            # Old RC commands need 4 bytes total
            if command in [10, 11] and len(buffer) >= 4:
                # Extract data and checksum
                data_byte = buffer[2]
                checksum = buffer[3]
                
                # Validate checksum
                expected_checksum = (command ^ data_byte) ^ 0xFF
                if checksum == expected_checksum:
                    # Send acknowledgment
                    ack_msg = bytearray([0xA0, command, command ^ 0xFF])
                    self.safe_ws_send(ack_msg)
                    
                    # Process RC command (old byte format)
                    if command == 10:  # Rudder
                        self.process_rc_rudder_command(data_byte)
                    elif command == 11:  # Sail
                        self.process_rc_sail_command(data_byte)
                else:
                    self.get_logger().warning(f'Invalid checksum for RC command {command}')
                    
            # Standard commands need 3 bytes total
            elif len(buffer) >= 3:
                # Validate checksum (simple XOR)
                if buffer[1] ^ 0xFF == buffer[2]:
                    # Extract command
                    command = buffer[1]
                    
                    # Send immediate acknowledgment
                    ack_msg = bytearray([0xA0, command, command ^ 0xFF])
                    self.safe_ws_send(ack_msg)
                    
                    # Special handling for status request command (9)
                    if command == 9:
                        pass  # Status update requested
                        self.send_status_update()
                    else:
                        # Process other commands
                        msg = Int32()
                        msg.data = command
                        self.command_publisher.publish(msg)
                        
                        # Log received command
                        self.get_logger().debug(f'Command: {command}')
                else:
                    self.get_logger().warning('Invalid checksum in received command')

    def send_initial_message(self):
        """Send initial message to establish connection with the remote terminal"""
        try:
            # Simple initial message: 0xB0 (header) + 0x55 + 0xAA (magic bytes)
            init_msg = bytearray([0xB0, 0x55, 0xAA])
            self.safe_ws_send(init_msg)
            pass  # Initial message sent
        except Exception as e:
            self.get_logger().error(f'Error sending initial message: {e}')

    def safe_ws_send(self, data):
        """Safely send data through WebSocket with error handling"""
        with self.ws_lock:
            if self.ws and self.ws.sock and self.ws.sock.connected:
                try:
                    if isinstance(data, (bytes, bytearray)):
                        self.ws.send(data, opcode=websocket.ABNF.OPCODE_BINARY)
                    else:
                        self.ws.send(data)
                    return True
                except Exception as e:
                    self.get_logger().error(f'WebSocket send error: {e}')
                    return False
            else:
                pass  # Not connected
                return False

    def gps_callback(self, msg: NavSatFix):
        """Store latest GPS position data"""
        self.latest_gps["latitude"] = msg.latitude
        self.latest_gps["longitude"] = msg.longitude
        self.latest_gps["timestamp"] = self.get_clock().now().to_msg().sec

    def speed_callback(self, msg: Float64):
        """Store latest GPS speed data"""
        self.latest_gps["speed"] = msg.data

    def fix_quality_callback(self, msg: Int32):
        """Store latest GPS fix quality data"""
        self.latest_gps["fix_quality"] = msg.data

    def wind_callback(self, msg: Float32):
        """Store latest wind direction data"""
        self.latest_wind_direction = msg.data
        pass  # Wind direction updated

    def process_rc_rudder_command(self, data_byte):
        """Process RC rudder command (old format with byte value)"""
        # Convert byte (0-255) back to angle (-45 to 45 degrees)
        rudder_angle = (data_byte * 90.0 / 255.0) - 45.0
        rudder_angle = round(rudder_angle, 0)
        # Publish rudder command
        msg = Float32()
        msg.data = rudder_angle
        self.rudder_command_publisher.publish(msg)

        self.get_logger().debug(f'RC Rudder: {rudder_angle:.1f}°')

    def process_rc_sail_command(self, data_byte):
        """Process RC sail command (old format with byte value)"""
        # Convert byte (0-255) back to angle (0 to 90 degrees)
        sail_angle = data_byte * 90.0 / 255.0

        # Publish sail angle command
        msg = Float32()
        msg.data = sail_angle
        self.sail_angle_publisher.publish(msg)

        self.get_logger().debug(f'RC Sail: {sail_angle:.1f}°')

    def process_rc_rudder_angle(self, rudder_angle):
        """Process RC rudder command with direct angle value"""
        # Clamp to valid range
        rudder_angle = max(-45.0, min(45.0, rudder_angle))
        
        # Publish rudder command
        msg = Float32()
        msg.data = rudder_angle
        self.rudder_command_publisher.publish(msg)

        self.get_logger().debug(f'RC Rudder: {rudder_angle:.1f}°')

    def process_rc_sail_angle(self, sail_angle):
        """Process RC sail command with direct angle value"""
        # Clamp to valid range
        sail_angle = max(0.0, min(90.0, sail_angle))
        
        # Publish sail angle command
        msg = Float32()
        msg.data = sail_angle
        self.sail_angle_publisher.publish(msg)

        self.get_logger().debug(f'RC Sail: {sail_angle:.1f}°')

    def status_callback(self, msg):
        """Process status update from the boat system"""
        try:
            # Parse the JSON status
            status_data = json.loads(msg.data)

            # Extract the values we need in simple format
            self.control_mode = status_data.get("control_mode", "rc")
            self.event_type = status_data.get("event_type", "developer_mode")

            # Handle waypoint information
            if "current_waypoint" in status_data and status_data["current_waypoint"]:
                self.current_waypoint = status_data["current_waypoint"]

            if "last_completed_waypoint" in status_data and status_data["last_completed_waypoint"]:
                self.last_completed_waypoint = status_data["last_completed_waypoint"]

            self.total_waypoints = status_data.get("total_waypoints", 0)

            # Log that we received a status update
            pass  # Status received

            # Send a status update when mode changes
            self.send_status_update()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse status JSON: {e}")

    def send_periodic_status(self):
        """Send periodic status updates to remote terminal"""
        self.send_status_update()

    def send_status_update(self):
        """Send status update using plain text format"""
        try:
            # Send GPS data in simple text format
            # Format: GPS,lat,lon,speed,fix_quality
            gps_message = f"GPS,{self.latest_gps['latitude']:.6f},{self.latest_gps['longitude']:.6f},{self.latest_gps['speed']:.2f},{self.latest_gps['fix_quality']}\n"
            if self.safe_ws_send(gps_message.encode('utf-8')):
                pass  # GPS sent

            # Send wind direction data
            # Format: WIND,direction
            wind_message = f"WIND,{self.latest_wind_direction:.1f}\n"
            if self.safe_ws_send(wind_message.encode('utf-8')):
                pass  # Wind sent

            # Send boat status with actual control mode and event type
            # Format: STATUS,control_mode,event_type
            status_message = f"STATUS,{self.control_mode},{self.event_type}\n"
            if self.safe_ws_send(status_message.encode('utf-8')):
                pass  # Status sent

            # Send waypoint info if available
            if self.current_waypoint:
                try:
                    current_wp_info = None
                    if isinstance(self.current_waypoint, dict):
                        # If current_waypoint is already a dict
                        current_wp_info = str(self.current_waypoint.get("lat", 0)) + "," + str(self.current_waypoint.get("long", 0))
                    else:
                        # If it's something else (assuming it's an object with lat/long attributes)
                        try:
                            current_wp_info = str(getattr(self.current_waypoint, "lat", 0)) + "," + str(getattr(self.current_waypoint, "long", 0))
                        except:
                            current_wp_info = "unknown"

                    # Format: WAYPOINT,info,total
                    waypoint_message = f"WAYPOINT,{current_wp_info},{self.total_waypoints}\n"
                    if self.safe_ws_send(waypoint_message.encode('utf-8')):
                        pass  # Waypoint sent
                except Exception as e:
                    self.get_logger().warning(f"Error formatting waypoint data: {e}")

        except Exception as e:
            self.get_logger().error(f'Error sending status update: {e}')

    def destroy_node(self):
        """Clean up resources when node is shutdown"""
        self.running = False
        if hasattr(self, 'ws_thread') and self.ws_thread.is_alive():
            # Close WebSocket connection
            if self.ws:
                self.ws.close()
            self.ws_thread.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CellularCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()