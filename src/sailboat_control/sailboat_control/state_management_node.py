#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32
from .boat import Boat, ControlMode
import json

class StateManagementNode(Node):
    def __init__(self):
        """initialize the state management ROS node"""
        super().__init__('state_management_node')

        # Radio command subscription
        self.radio_subscriber = self.create_subscription(
            Int32,
            'radio_commands',
            self.radio_callback,
            10
        )
        
        # Global wind angle subscription for dynamic updates
        self.global_wind_subscriber = self.create_subscription(
            Float32,
            'global_wind_angle',
            self.global_wind_callback,
            10
        )

        # Add status publisher for boat status
        self.status_publisher = self.create_publisher(
            String,
            'boat_status',
            10
        )

        # Publishers for rudder and sail control
        self.rudder_publisher = self.create_publisher(
            Float32,
            'rudder/command',
            10
        )
        
        self.sail_publisher = self.create_publisher(
            Float32,
            'sail/angle',
            10
        )

        # Get event type from parameter or default to developer_mode
        self.declare_parameter('event_type', 'developer_mode')
        event_type = self.get_parameter('event_type').value
        
        # Get global wind parameters
        self.declare_parameter('global_wind_angle', 0.0)
        self.declare_parameter('use_global_wind', False)
        self.declare_parameter('wind_sensor_timeout', 5.0)
        
        global_wind_angle = self.get_parameter('global_wind_angle').value
        use_global_wind = self.get_parameter('use_global_wind').value
        wind_sensor_timeout = self.get_parameter('wind_sensor_timeout').value
        
        # Create the boat instance and pass the node for sensor access
        self.boat = Boat(event_type, self)
        self.boat.start_event()
        
        # Configure global wind settings if event control exists
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            self.boat.event_control.set_global_wind_angle(global_wind_angle)
            self.boat.event_control.enable_global_wind(use_global_wind)
            self.boat.event_control.set_wind_sensor_timeout(wind_sensor_timeout)
        
        self.get_logger().info(f"Boat initialized with event type: {event_type}")
        if use_global_wind:
            self.get_logger().info(f"Using global wind angle: {global_wind_angle}°")

        # Command handler mapping
        self.command_handlers = {
            0: self._handle_rc_control,
            1: self._handle_start_autonomous,
            2: self._handle_rc_interrupt,
            3: self._handle_resume_autonomous,
            4: self._handle_emergency_stop
        }

        # Publish initial status
        self.publish_status()

        # Create a timer to periodically publish status (every 5 seconds)
        self.create_timer(5.0, self.publish_status)
        
        # Create a control loop timer for continuous autonomous control (10Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("State management node initialized")

    def radio_callback(self, msg: Int32):
        """handle incoming radio commands"""
        command = msg.data
        handler = self.command_handlers.get(command)

        if handler:
            handler()
            self._log_state_change()
            # Publish status update immediately after state change
            self.publish_status()
        else:
            self.get_logger().warn(f"Unknown command received: {command}")
    
    def global_wind_callback(self, msg: Float32):
        """handle global wind angle updates"""
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            self.boat.event_control.set_global_wind_angle(msg.data)
            self.get_logger().info(f"Updated global wind angle to {msg.data}°")

    def _handle_rc_control(self):
        """handle RC control command"""
        # If switching from autonomous to RC, mark it as permanent
        if self.boat.state.control_mode == ControlMode.AUTONOMOUS:
            self.boat.state.rc_enabled_after_autonomous = True
        
        self.boat.state.control_mode = ControlMode.RC
        self.get_logger().info("Switched to RC control")
        # Call RC control method
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            self.boat.event_control.handle_rc()

    def _handle_start_autonomous(self):
        """handle start autonomous command"""
        self.boat.start_autonomous()
        # If event supports autonomous, call its autonomous control
        if hasattr(self.boat, 'event_control') and self.boat.event_control and \
           hasattr(self.boat.event_control, 'handle_autonomous'):
            self.boat.event_control.handle_autonomous()

    def _handle_rc_interrupt(self):
        """handle RC interrupt command"""
        last_waypoint = self.boat.handle_rc_interrupt()
        if last_waypoint:
            self.get_logger().info(f"RC interrupt - last waypoint: {last_waypoint}")
        
        # Set sail and rudder to neutral positions
        self._set_rudder_angle(0.0)
        self._set_sail_angle(0.0)
        
        # Switch to RC control permanently
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            self.boat.event_control.handle_rc()
            self.boat.event_control.disable_autonomous_navigation() # Disable navigation control

    def _handle_resume_autonomous(self):
        """handle resume autonomous command"""
        self.boat.resume_autonomous()
        # Resume autonomous control if event supports it
        if hasattr(self.boat, 'event_control') and self.boat.event_control and \
           hasattr(self.boat.event_control, 'handle_autonomous'):
            self.boat.event_control.handle_autonomous()

    def _handle_emergency_stop(self):
        """handle emergency stop command"""
        self.boat.state.control_mode = ControlMode.RC
        self.boat.state.is_event_active = False
        self.get_logger().warn("Emergency stop activated")
        # Force RC control
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            self.boat.event_control.handle_rc()

    def _log_state_change(self):
        """Log current state after changes"""
        status = self.boat.get_system_status()
        self.get_logger().info(
            f"State update - mode: {status['control_mode']}, "
            f"event: {status['event_type']}, "
            f"autonomous enabled: {status['autonomous_enabled']}"
        )

    def publish_status(self):
        """Publish boat status for radio communication"""
        status = self.boat.get_system_status()

        # Add GPS and wind data if available
        if hasattr(self.boat, 'event_control') and self.boat.event_control:
            try:
                current_pos = self.boat.event_control.get_current_position()
                status['position'] = current_pos
                status['speed'] = self.boat.event_control.get_current_speed()
                status['wind_direction'] = self.boat.event_control.get_wind_direction()
                
                # Add search-specific status if this is a search event
                if self.boat.event_type.lower() == "search" and hasattr(self.boat.event_control, 'get_search_status'):
                    status['search_status'] = self.boat.event_control.get_search_status()
                
                # Add wind status information
                if hasattr(self.boat.event_control, 'get_wind_status'):
                    status['wind_status'] = self.boat.event_control.get_wind_status()
            except AttributeError:
                # Handle case where event_control doesn't have all methods
                self.get_logger().debug("Some event_control methods not available")

        # Convert to JSON string
        status_json = json.dumps(status)

        # Create and publish message
        msg = String()
        msg.data = status_json
        self.status_publisher.publish(msg)

        # Log status update
        self.get_logger().debug(f"Published status update: {status_json}")

    def _set_rudder_angle(self, angle: float):
        """Set rudder angle"""
        msg = Float32()
        msg.data = angle
        self.rudder_publisher.publish(msg)
        self.get_logger().info(f"Set rudder angle to {angle}°")
    
    def _set_sail_angle(self, angle: float):
        """Set sail angle"""
        msg = Float32()
        msg.data = angle
        self.sail_publisher.publish(msg)
        self.get_logger().info(f"Set sail angle to {angle}°")
    
    def control_loop(self):
        """Main control loop that runs continuously"""
        # Only run autonomous control if in autonomous mode
        if self.boat.state.control_mode == ControlMode.AUTONOMOUS:
            if hasattr(self.boat, 'event_control') and self.boat.event_control and \
               hasattr(self.boat.event_control, 'handle_autonomous'):
                # Call the event's autonomous control method
                self.boat.event_control.handle_autonomous()
        elif self.boat.state.control_mode == ControlMode.RC:
            if hasattr(self.boat, 'event_control') and self.boat.event_control:
                self.boat.event_control.handle_rc()

def main(args=None):
    rclpy.init(args=args)
    node = StateManagementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
