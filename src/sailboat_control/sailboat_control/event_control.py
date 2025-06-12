from abc import ABC, abstractmethod
from .common import ControlMode
from path_planning.path_planning.waypoint import Waypoint
from path_planning.path_planning.leg import Leg
from typing import List, Optional, Tuple
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Float64, String
from .buoy_detection import BuoyDetector
from std_msgs.msg import String, Int32
import json
from math import cos, radians
from collections import deque
import time

class EventControl(ABC):
    """base class for event control"""
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str = "unknown"):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.node = node  # ros2 node for subscriptions
        self.event_type = event_type # Store event type for specific logic
        self.navigation_enabled_by_event_control = False # Track if this instance enabled nav

        # latest sensor data storage
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_speed = 0.0
        self.wind_direction = 0.0
        self.gps_time = ""
        self.satellites = 0
        
        # Global wind angle backup
        self.use_global_wind = False
        self.global_wind_angle = 0.0
        self.wind_sensor_timeout = 5.0  # seconds without wind data before using global
        self.last_wind_update_time = None
        
        # GPS position buffering for delayed position calculations
        self.position_buffer = deque(maxlen=100)  # Store last 100 positions (10 seconds at 10Hz)
        self.position_buffer_delay = 3.0  # seconds to look back for position
        
        # Control update timing
        self.last_rudder_update_time = 0.0
        self.last_sail_update_time = 0.0
        self.rudder_update_interval = 3.0  # seconds between rudder updates
        self.sail_update_interval = 10.0  # seconds between sail updates
        self.last_wind_for_sail = 0.0  # Track wind angle for sail changes
        self.significant_wind_change = 15.0  # degrees of wind change to trigger sail update

        # set up buoy detector with default 5 meter threshold
        self.buoy_detector = BuoyDetector(threshold_distance=5.0)

        # set up subscriptions
        self._setup_subscriptions()

        # publishers for navigation system
        self.waypoint_publisher = node.create_publisher(
            String,
            'navigation/target',
            10
        )

        self.nav_command_publisher = node.create_publisher(
            Int32,
            'navigation/command',
            10
        )

    def _setup_subscriptions(self):
        """setup gps and wind sensor subscriptions"""
        self.gps_subscription = self.node.create_subscription(
            NavSatFix,
            'gps/fix',
            self._gps_callback,
            10
        )

        # New subscription for GPS speed
        self.speed_subscription = self.node.create_subscription(
            Float64,
            'gps/speed',
            self._speed_callback,
            10
        )

        # New subscription for GPS time
        self.time_subscription = self.node.create_subscription(
            String,
            'gps/time',
            self._time_callback,
            10
        )

        self.wind_subscription = self.node.create_subscription(
            Float32,
            'wind/direction',
            self._wind_callback,
            10
        )

    def _gps_callback(self, msg: NavSatFix):
        """store latest gps position data and check for proximity to buoys"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # Add position to buffer with timestamp
        current_time = time.time()
        self.position_buffer.append({
            'time': current_time,
            'lat': msg.latitude,
            'lon': msg.longitude
        })

        # Check if we're near any buoys
        newly_reached_buoys = self.buoy_detector.check_buoy_proximity(
            (self.current_lat, self.current_lon),
            self.waypoints
        )

        # Log any newly reached buoys
        for buoy in newly_reached_buoys:
            self.node.get_logger().info(
                f"Reached buoy at lat={buoy.lat}, lon={buoy.long} "
                f"(pass #{buoy.n_passed})"
            )

            # Update the current waypoint index based on the most recently passed buoy
            # This logic runs continuously, even if in RC mode, so BuoyDetector state is up-to-date.
            current_buoy_object = self.buoy_detector.get_current_buoy() # Renamed for clarity
            if current_buoy_object is not None:
                try:
                    current_reached_idx = self.waypoints.index(current_buoy_object)

                    # Determine next logical waypoint index, handling looping for endurance
                    new_potential_idx = current_reached_idx + 1
                    is_looping_event = "endurance" in self.event_type.lower() # Example check

                    if is_looping_event and self.waypoints:
                        self.current_waypoint_index = new_potential_idx % len(self.waypoints)
                    elif new_potential_idx < len(self.waypoints):
                        self.current_waypoint_index = new_potential_idx
                    else: # End of a non-looping event sequence
                        self.current_waypoint_index = new_potential_idx # Will be out of bounds

                    next_target_waypoint = self.get_next_waypoint() # Uses the updated index

                    if next_target_waypoint:
                        self.node.get_logger().info(
                            f"Next target (major event waypoint): {next_target_waypoint.lat}, {next_target_waypoint.long}"
                        )
                        # If navigation was previously enabled by this EventControl instance,
                        # and we assume StateManagementNode handles global mode,
                        # then we can send the next waypoint.
                        # A more robust system would check global boat_state.control_mode here.
                        if self.navigation_enabled_by_event_control:
                             self.send_waypoint(next_target_waypoint)
                    else:
                        self.node.get_logger().info("All major event waypoints reached or event sequence complete.")
                        if self.navigation_enabled_by_event_control:
                            # self.disable_autonomous_navigation() # Consider if this should be here or handled by StateManagementNode
                            self.navigation_enabled_by_event_control = False # Mark as no longer controlling nav

                except ValueError:
                    self.node.get_logger().error("Current buoy from detector not found in master waypoints list!")

        # Log position updates for debugging (can be made less frequent if needed)
        # self.node.get_logger().debug(f"Position update: lat={self.current_lat:.6f}, lon={self.current_lon:.6f}")

    def _speed_callback(self, msg: Float64):
        """store latest speed data"""
        self.current_speed = msg.data
        self.node.get_logger().debug(f"Speed update: {self.current_speed:.2f} m/s")

    def _time_callback(self, msg: String):
        """store latest gps time data"""
        self.gps_time = msg.data
        self.node.get_logger().debug(f"GPS time update: {self.gps_time}")

    def _wind_callback(self, msg: Float32):
        """store latest wind direction"""
        self.wind_direction = msg.data
        self.last_wind_update_time = self.node.get_clock().now()
        self.node.get_logger().debug(f"Wind direction update: {self.wind_direction:.1f}°")

    def get_current_position(self) -> Tuple[float, float]:
        """get latest position"""
        return (self.current_lat, self.current_lon)

    def get_current_speed(self) -> float:
        """get latest speed in m/s"""
        return self.current_speed

    def get_gps_time(self) -> str:
        """get latest gps time"""
        return self.gps_time

    def get_wind_direction(self) -> float:
        """get latest wind direction with fallback to global wind"""
        # Check if we should use global wind
        if self.use_global_wind:
            return self.global_wind_angle
            
        # Check if wind sensor data is stale
        if self.last_wind_update_time is not None:
            time_since_update = self.node.get_clock().now() - self.last_wind_update_time
            if time_since_update.nanoseconds / 1e9 > self.wind_sensor_timeout:
                self.node.get_logger().warning(
                    f"Wind sensor timeout ({time_since_update.nanoseconds / 1e9:.1f}s), using global wind: {self.global_wind_angle}°"
                )
                return self.global_wind_angle
        
        return self.wind_direction

    def get_current_buoy(self) -> Optional[Waypoint]:
        """get the most recently passed buoy"""
        return self.buoy_detector.get_current_buoy()

    def get_next_waypoint(self) -> Optional[Waypoint]:
        """get the next target waypoint based on most recently passed buoy"""
        if not self.waypoints:
            return None

        if 0 <= self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]

        # If index is out of bounds (e.g., end of a non-looping event)
        return None

    def set_buoy_threshold(self, threshold_meters: float):
        """set the threshold distance for buoy detection"""
        self.buoy_detector.threshold_distance = threshold_meters
        self.node.get_logger().info(f"Buoy detection threshold set to {threshold_meters} meters")

    def reset_buoy_detection(self):
        """reset buoy detection state, marking all buoys as not reached"""
        self.buoy_detector.reset()
        for buoy in self.waypoints:
            buoy.marked = False
            buoy.n_passed = 0
        self.current_waypoint_index = 0
        self.node.get_logger().info("Buoy detection state reset")

    def get_reached_buoys_count(self) -> int:
        """get the number of buoys that have been reached at least once"""
        return self.buoy_detector.get_reached_buoys_count()

    def handle_rc(self):
        """standard rc control implementation for all events"""
        current_pos = self.get_current_position()
        wind_dir = self.get_wind_direction()
        speed = self.get_current_speed()
        # implement standard rc control logic here
        pass
    
    def set_global_wind_angle(self, angle: float):
        """Set the global wind angle for backup use"""
        self.global_wind_angle = angle % 360  # Normalize to 0-360 range
        self.node.get_logger().info(f"Global wind angle set to {self.global_wind_angle}°")
    
    def enable_global_wind(self, enable: bool = True):
        """Enable or disable use of global wind angle"""
        self.use_global_wind = enable
        if enable:
            self.node.get_logger().info(f"Using global wind angle: {self.global_wind_angle}°")
        else:
            self.node.get_logger().info("Using wind sensor data")
    
    def set_wind_sensor_timeout(self, timeout_seconds: float):
        """Set the timeout for wind sensor data before falling back to global"""
        self.wind_sensor_timeout = timeout_seconds
        self.node.get_logger().info(f"Wind sensor timeout set to {timeout_seconds} seconds")
    
    def get_wind_status(self) -> dict:
        """Get current wind data status"""
        status = {
            "current_wind_direction": self.get_wind_direction(),
            "sensor_wind_direction": self.wind_direction,
            "global_wind_angle": self.global_wind_angle,
            "use_global_wind": self.use_global_wind,
            "wind_source": "global" if self.use_global_wind else "sensor"
        }
        
        if self.last_wind_update_time is not None:
            time_since_update = self.node.get_clock().now() - self.last_wind_update_time
            status["seconds_since_update"] = time_since_update.nanoseconds / 1e9
            status["sensor_timed_out"] = status["seconds_since_update"] > self.wind_sensor_timeout
        else:
            status["seconds_since_update"] = None
            status["sensor_timed_out"] = True
            
        return status
    
    def get_delayed_position(self, delay_seconds: float = None) -> Optional[Tuple[float, float]]:
        """Get GPS position from specified seconds ago"""
        if delay_seconds is None:
            delay_seconds = self.position_buffer_delay
            
        if not self.position_buffer:
            return None
            
        current_time = time.time()
        target_time = current_time - delay_seconds
        
        # Find the position closest to target time
        best_position = None
        min_time_diff = float('inf')
        
        for pos in self.position_buffer:
            time_diff = abs(pos['time'] - target_time)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                best_position = pos
                
        if best_position and min_time_diff < 1.0:  # Within 1 second of target
            return (best_position['lat'], best_position['lon'])
        return None
    
    def should_update_rudder(self) -> bool:
        """Check if it's time to update rudder"""
        current_time = time.time()
        if current_time - self.last_rudder_update_time >= self.rudder_update_interval:
            self.last_rudder_update_time = current_time
            return True
        return False
    
    def should_update_sail(self) -> bool:
        """Check if it's time to update sail (time-based or significant wind change)"""
        current_time = time.time()
        current_wind = self.get_wind_direction()
        
        # Check time interval
        time_elapsed = current_time - self.last_sail_update_time >= self.sail_update_interval
        
        # Check significant wind change
        wind_change = abs(current_wind - self.last_wind_for_sail)
        # Handle wrap-around at 360 degrees
        if wind_change > 180:
            wind_change = 360 - wind_change
            
        significant_change = wind_change >= self.significant_wind_change
        
        if time_elapsed or significant_change:
            self.last_sail_update_time = current_time
            self.last_wind_for_sail = current_wind
            if significant_change:
                self.node.get_logger().info(f"Significant wind change detected: {wind_change:.1f}°")
            return True
        return False

    def enable_autonomous_navigation(self):
        """Enable autonomous navigation via NavigationNode"""
        msg = Int32()
        msg.data = 1  # 1 = enable
        self.nav_command_publisher.publish(msg)
        self.navigation_enabled_by_event_control = True # Track that this instance enabled it
        self.node.get_logger().info("EventControl: Enabled autonomous navigation (sent command to NavigationNode)")

    def disable_autonomous_navigation(self):
        """Disable autonomous navigation via NavigationNode"""
        msg = Int32()
        msg.data = 0  # 0 = disable
        self.nav_command_publisher.publish(msg)
        self.navigation_enabled_by_event_control = False # Track that this instance disabled it
        self.node.get_logger().info("EventControl: Disabled autonomous navigation (sent command to NavigationNode)")

    def send_waypoint(self, waypoint: Waypoint):
        """Send waypoint to navigation system"""
        # Create JSON waypoint data
        waypoint_data = {
            'lat': waypoint.lat,
            'lon': waypoint.long
        }

        # Convert to JSON string
        waypoint_json = json.dumps(waypoint_data)

        # Create and publish message
        msg = String()
        msg.data = waypoint_json
        self.waypoint_publisher.publish(msg)

        self.node.get_logger().info(
            f"Sent waypoint to navigation: lat={waypoint.lat}, lon={waypoint.long}"
        )


class FleetRaceControl(EventControl):
    """pure rc control - no autonomous capability"""
    pass  # only uses the base rc control

class PrecisionNavigationControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str):
        super().__init__(waypoints, node, event_type)
        self.node.get_logger().info(f"PrecisionNavigationControl initialized for event: {event_type}")

    def handle_autonomous(self):
        """precision navigation autonomous control"""
        self.node.get_logger().info("PrecisionNavigationControl: Handling autonomous sequence...")

        # The _gps_callback should have already updated current_waypoint_index
        # based on any buoys passed, even during RC.
        # So, get_next_waypoint() should give us the correct current target.
        next_wp = self.get_next_waypoint()

        if next_wp:
            self.node.get_logger().info(f"PrecisionNavigationControl: Targeting waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: ({next_wp.lat}, {next_wp.long})")
            self.enable_autonomous_navigation()
            self.send_waypoint(next_wp)
        else:
            self.node.get_logger().info("PrecisionNavigationControl: All waypoints reached or no waypoints defined for this event.")
            # Optionally disable navigation if the event is truly over and doesn't loop
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False

class StationKeepingControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str):
        super().__init__(waypoints, node, event_type)
        self.node.get_logger().info(f"StationKeepingControl initialized for event: {event_type}")
        # For station keeping, waypoints[0] is typically the center.
        # A radius parameter would also be needed for real station keeping.
        self.station_keeping_radius = 20.0 # Example radius in meters, should be configurable

    def handle_autonomous(self):
        """station keeping autonomous control"""
        self.node.get_logger().info("StationKeepingControl: Handling autonomous...")

        # The first waypoint is the center of the station-keeping zone.
        station_center_wp = self._get_waypoint_at_index(0) # Always target the first defined waypoint as center

        if station_center_wp:
            self.node.get_logger().info(f"StationKeepingControl: Target center at ({station_center_wp.lat}, {station_center_wp.long})")

            # For now, basic strategy: continuously tell NavigationNode to go to/hold at the center.
            # NavigationNode would need to be smart about "holding" vs just "reaching".
            # A more advanced StationKeepingControl would calculate micro-waypoints or adjust
            # parameters for NavigationNode.
            self.enable_autonomous_navigation()
            self.send_waypoint(station_center_wp)

            # Note: True station keeping (maintaining position within a radius) is complex.
            # This implementation just sends the center point. The NavigationNode
            # would need a "loiter" or "station keep" capability, or this node
            # would need to generate a series of small waypoints to simulate it.
            # For example, if NavigationNode only goes to a point and stops,
            # this class would need to detect drifting and resend the center point.
        else:
            self.node.get_logger().warning("StationKeepingControl: No center waypoint defined for station keeping.")
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False

class EnduranceControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str):
        super().__init__(waypoints, node, event_type)
        self.node.get_logger().info(f"EnduranceControl initialized for event: {event_type}")

    def handle_autonomous(self):
        """endurance autonomous control - waypoints should loop"""
        self.node.get_logger().info("EnduranceControl: Handling autonomous sequence...")

        # _gps_callback handles advancing current_waypoint_index, including looping
        # for "endurance" type events.
        next_wp = self.get_next_waypoint() # Will get waypoints[0] after last if looping is correct

        if next_wp:
            self.node.get_logger().info(f"EnduranceControl: Targeting waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: ({next_wp.lat}, {next_wp.long})")
            self.enable_autonomous_navigation()
            self.send_waypoint(next_wp)
        else:
            # This case should ideally not be reached if waypoints are defined and looping works.
            self.node.get_logger().error("EnduranceControl: No next waypoint available, though endurance should loop. Check waypoint list and index logic.")
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False

class PayloadControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str):
        super().__init__(waypoints, node, event_type)
        self.node.get_logger().info(f"PayloadControl initialized for event: {event_type}")

    def handle_autonomous(self):
        """payload delivery autonomous control"""
        self.node.get_logger().info("PayloadControl: Handling autonomous sequence...")
        next_wp = self.get_next_waypoint()

        if next_wp:
            self.node.get_logger().info(f"PayloadControl: Targeting waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: ({next_wp.lat}, {next_wp.long})")
            self.enable_autonomous_navigation()
            self.send_waypoint(next_wp)

            # Additional logic for payload deployment could be triggered here
            # by checking if the *current* waypoint (which is next_wp if it's the first in sequence,
            # or previous one if sequence is ongoing) has been reached and is the payload drop zone.
            # E.g., if self.buoy_detector.is_buoy_reached(specific_payload_waypoint):
            #    self.node.get_logger().info("Payload waypoint reached! Deploying payload (simulated).")
            #    # ... actual payload deployment command ...
        else:
            self.node.get_logger().info("PayloadControl: All waypoints reached or no waypoints defined.")
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False

class DeveloperControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str): # Added event_type
        super().__init__(waypoints, node, event_type) # Pass event_type
        self.node.get_logger().info(f"DeveloperControl initialized for event: {event_type}")
        
        # RC/Autonomous mode switching state
        self.previous_mode = None
        self.autonomous_restart_timer = None
        self.course_reset_pending = False
        self.autonomous_enable_time = None
        
        # Publishers for direct control
        self.rudder_publisher = node.create_publisher(
            Float32,
            'rudder/command',
            10
        )
        self.sail_publisher = node.create_publisher(
            Float32,
            'sail/angle',
            10
        )

    def handle_rc(self):
        """Handle RC mode - zero controls and reset course"""
        self.node.get_logger().info("DeveloperControl: Entering RC mode")
        
        # Zero rudder and sail
        self._zero_controls()
        
        # Reset course
        self._reset_course()
        
        # Disable autonomous navigation
        self.disable_autonomous_navigation()
        
        # Cancel any pending autonomous restart
        if self.autonomous_restart_timer:
            self.autonomous_restart_timer.cancel()
            self.autonomous_restart_timer = None
        
        self.previous_mode = ControlMode.RC

    def handle_autonomous(self):
        """developer testing autonomous control"""
        # Check if we're transitioning from RC to autonomous
        if self.previous_mode == ControlMode.RC:
            self.node.get_logger().info("DeveloperControl: Transitioning from RC to Autonomous")
            
            # Zero controls immediately
            self._zero_controls()
            
            # Set up 10-second delay before restarting course
            self.course_reset_pending = True
            self.autonomous_enable_time = self.node.get_clock().now()
            
            # Store the previous mode
            self.previous_mode = ControlMode.AUTONOMOUS
            
            # Wait before enabling navigation
            return
        
        # Check if we're waiting for the delay period
        if self.course_reset_pending and self.autonomous_enable_time:
            elapsed = (self.node.get_clock().now() - self.autonomous_enable_time).nanoseconds / 1e9
            if elapsed < 10.0:
                remaining = 10.0 - elapsed
                self.node.get_logger().info(f"DeveloperControl: Waiting {remaining:.1f}s before starting autonomous...")
                return
            else:
                # Delay period has passed, reset course and start
                self.node.get_logger().info("DeveloperControl: Starting autonomous navigation after delay")
                self._reset_course()
                self.course_reset_pending = False
                self.autonomous_enable_time = None
        
        # Normal autonomous operation
        self.node.get_logger().info("DeveloperControl: Handling autonomous...")
        # Get next waypoint. _gps_callback should keep current_waypoint_index updated.
        next_wp = self.get_next_waypoint()

        if next_wp:
            self.node.get_logger().info(f"DeveloperControl: Targeting waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: ({next_wp.lat}, {next_wp.long})")
            # Enable autonomous navigation
            self.enable_autonomous_navigation()

            # Send waypoint to navigation system
            self.send_waypoint(next_wp)
        else:
            self.node.get_logger().warning("DeveloperControl: No waypoint available for navigation or sequence complete.")
            # self.disable_autonomous_navigation()
            self.navigation_enabled_by_event_control = False
            
        self.previous_mode = ControlMode.AUTONOMOUS

    def _zero_controls(self):
        """Zero rudder and sail controls"""
        self.node.get_logger().info("DeveloperControl: Zeroing rudder and sail")
        
        # Zero rudder
        rudder_msg = Float32()
        rudder_msg.data = 0.0
        self.rudder_publisher.publish(rudder_msg)
        
        # Zero sail (0 degrees = fully in)
        sail_msg = Float32()
        sail_msg.data = 0.0
        self.sail_publisher.publish(sail_msg)

    def _reset_course(self):
        """Reset the course to start from the beginning"""
        self.node.get_logger().info("DeveloperControl: Resetting course")
        
        # Reset waypoint index to start
        self.current_waypoint_index = 0
        
        # Reset all waypoint passed counts
        for waypoint in self.waypoints:
            waypoint.n_passed = 0
            
        # Clear any navigation state
        self.navigation_enabled_by_event_control = False

class SearchControl(EventControl):
    def __init__(self, waypoints: List[Waypoint], node: Node, event_type: str):
        super().__init__(waypoints, node, event_type)
        self.node.get_logger().info(f"SearchControl initialized for event: {event_type}")

        # Search parameters
        self.search_active = False
        self.buoy_found = False
        self.buoy_hit = False
        self.search_start_time = None
        self.search_timeout = 600  # 10 minutes in seconds

        # Visual tracking parameters
        self.buoy_angle_offset = 0.0  # Angle difference from boat heading to buoy
        self.buoy_distance = float('inf')  # Distance to buoy in meters
        self.hit_distance_threshold = 2.0  # Distance threshold to consider buoy "hit"

        # Control parameters
        self.angle_tolerance = 5.0  # degrees - acceptable angle error
        self.rudder_gain = 2.0  # P-controller gain for rudder control
        self.max_rudder_angle = 20  # Maximum rudder angle in degrees

        # Publishers for control
        self.rudder_publisher = node.create_publisher(
            Float32,
            'control/rudder_angle',
            10
        )

        self.sail_publisher = node.create_publisher(
            Float32,
            'control/sail_angle',
            10
        )

        # Publisher for web server notifications
        self.web_notification_publisher = node.create_publisher(
            String,
            'web/notification',
            10
        )

        # Additional subscriptions for camera and distance
        self._setup_search_subscriptions()

    def _setup_search_subscriptions(self):
        """Setup subscriptions specific to search event"""
        # Subscribe to camera angle offset
        self.camera_angle_subscription = self.node.create_subscription(
            Float32,
            'camera/buoy_angle_offset',
            self._camera_angle_callback,
            10
        )

        # Subscribe to buoy distance
        self.buoy_distance_subscription = self.node.create_subscription(
            Float32,
            'camera/buoy_distance',
            self._buoy_distance_callback,
            10
        )

    def _camera_angle_callback(self, msg: Float32):
        """Handle camera angle offset updates"""
        self.buoy_angle_offset = msg.data
        if not self.buoy_found and abs(msg.data) < 90:  # Buoy is visible
            self.buoy_found = True
            self.node.get_logger().info("Buoy detected by camera!")
            self._send_web_notification("buoy_detected", {"angle_offset": msg.data})

    def _buoy_distance_callback(self, msg: Float32):
        """Handle buoy distance updates"""
        self.buoy_distance = msg.data

        # Check if buoy has been hit
        if not self.buoy_hit and self.buoy_distance < self.hit_distance_threshold:
            self.buoy_hit = True
            self.node.get_logger().info(f"Buoy hit! Distance: {self.buoy_distance}m")
            self._handle_buoy_hit()

    def handle_autonomous(self):
        """Visual servoing control for search event"""
        # Note: Search event doesn't use traditional waypoint navigation
        # It uses visual servoing and direct control

        if not self.search_active:
            self._start_search()

        # Check for timeout
        if self.search_start_time:
            elapsed = self.node.get_clock().now().seconds_nanoseconds()[0] - self.search_start_time
            if elapsed > self.search_timeout:
                self.node.get_logger().error("Search timeout reached!")
                self._send_web_notification("search_timeout", {"elapsed_time": elapsed})
                # Don't disable navigation - just report timeout
                return

        if self.buoy_hit:
            # Already hit the buoy, maintain upwind pointing
            self._point_upwind()
        elif self.buoy_found:
            # Visual servoing to approach buoy
            self._visual_servo_control()
        else:
            # Continue straight while looking for buoy
            self._maintain_downwind_course()

    def _start_search(self):
        """Initialize search parameters"""
        self.search_active = True
        self.search_start_time = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.node.get_logger().info("Search started - looking for orange buoy")
        self._send_web_notification("search_started", {"timestamp": self.search_start_time})

        # Start with downwind sailing
        self._maintain_downwind_course()

    def _visual_servo_control(self):
        """Control rudder to minimize angle offset to buoy"""
        # Only update rudder at specified interval
        if not self.should_update_rudder():
            return
            
        # Get delayed position for more stable heading calculation
        delayed_pos = self.get_delayed_position()
        current_pos = self.get_current_position()
        
        if delayed_pos and current_pos:
            # Log position data being used
            self.node.get_logger().debug(
                f"Using positions - Current: ({current_pos[0]:.6f}, {current_pos[1]:.6f}), "
                f"3s ago: ({delayed_pos[0]:.6f}, {delayed_pos[1]:.6f})"
            )
        
        # P-controller for rudder based on angle offset
        rudder_angle = -self.rudder_gain * self.buoy_angle_offset
        
        # Clamp rudder angle
        rudder_angle = max(-self.max_rudder_angle, min(self.max_rudder_angle, rudder_angle))
        
        # Publish rudder command
        rudder_msg = Float32()
        rudder_msg.data = rudder_angle
        self.rudder_publisher.publish(rudder_msg)
        
        self.node.get_logger().info(
            f"Rudder update: angle_offset={self.buoy_angle_offset:.1f}°, "
            f"rudder={rudder_angle:.1f}°, distance={self.buoy_distance:.1f}m"
        )

    def _maintain_downwind_course(self):
        """Maintain downwind course while searching"""
        # Update sail if needed (time-based or wind change)
        if self.should_update_sail():
            sail_msg = Float32()
            sail_msg.data = 90.0  # Full out for downwind
            self.sail_publisher.publish(sail_msg)
            self.node.get_logger().info("Sail updated for downwind course")

        # Only update rudder at specified interval
        if self.should_update_rudder():
            # Get delayed position for navigation calculations
            delayed_pos = self.get_delayed_position()
            current_pos = self.get_current_position()
            
            # Keep rudder centered while searching
            rudder_msg = Float32()
            rudder_msg.data = 0.0
            self.rudder_publisher.publish(rudder_msg)
            self.node.get_logger().info("Rudder centered for downwind search")

    def _point_upwind(self):
        """Point the boat upwind after hitting buoy"""
        # Get current wind direction
        wind_dir = self.get_wind_direction()

        # Update sail if needed
        if self.should_update_sail():
            # Set sail for close-hauled upwind sailing
            sail_msg = Float32()
            sail_msg.data = 15.0  # Close-hauled sail angle
            self.sail_publisher.publish(sail_msg)
            self.node.get_logger().info(f"Sail set for upwind (wind at {wind_dir}°)")

        # Update rudder at specified interval
        if self.should_update_rudder():
            # Get delayed position for heading calculations
            delayed_pos = self.get_delayed_position()
            current_pos = self.get_current_position()
            
            # Use a moderate rudder angle to maintain upwind heading
            # In a full implementation, you'd calculate based on current heading vs wind
            rudder_msg = Float32()
            rudder_msg.data = 30.0  # Initial turn to port (adjust based on wind side)
            self.rudder_publisher.publish(rudder_msg)
            self.node.get_logger().info("Rudder set for upwind pointing")

    def _handle_buoy_hit(self):
        """Handle actions when buoy is hit"""
        # Send notification to web server
        hit_data = {
            "timestamp": self.node.get_clock().now().seconds_nanoseconds()[0],
            "distance": self.buoy_distance,
            "search_duration": self.node.get_clock().now().seconds_nanoseconds()[0] - self.search_start_time
        }
        self._send_web_notification("buoy_hit", hit_data)

        # Log success
        self.node.get_logger().info(f"Search successful! Duration: {hit_data['search_duration']:.1f}s")

        # Start pointing upwind
        self._point_upwind()

    def _send_web_notification(self, event_type: str, data: dict):
        """Send notification to web server"""
        notification = {
            "event": event_type,
            "data": data,
            "timestamp": self.node.get_clock().now().seconds_nanoseconds()[0]
        }

        msg = String()
        msg.data = json.dumps(notification)
        self.web_notification_publisher.publish(msg)

        self.node.get_logger().info(f"Web notification sent: {event_type}")

    def reset_search(self):
        """Reset search state for a new run"""
        self.search_active = False
        self.buoy_found = False
        self.buoy_hit = False
        self.search_start_time = None
        self.buoy_angle_offset = 0.0
        self.buoy_distance = float('inf')
        self.node.get_logger().info("Search state reset")

    def configure_search(self, hit_threshold: float = None, rudder_gain: float = None, angle_tolerance: float = None,
                        rudder_interval: float = None, sail_interval: float = None):
        """Configure search parameters"""
        if hit_threshold is not None:
            self.hit_distance_threshold = hit_threshold
            self.node.get_logger().info(f"Hit distance threshold set to {hit_threshold}m")

        if rudder_gain is not None:
            self.rudder_gain = rudder_gain
            self.node.get_logger().info(f"Rudder gain set to {rudder_gain}")

        if angle_tolerance is not None:
            self.angle_tolerance = angle_tolerance
            self.node.get_logger().info(f"Angle tolerance set to {angle_tolerance}°")
            
        if rudder_interval is not None:
            self.rudder_update_interval = rudder_interval
            self.node.get_logger().info(f"Rudder update interval set to {rudder_interval}s")
            
        if sail_interval is not None:
            self.sail_update_interval = sail_interval
            self.node.get_logger().info(f"Sail update interval set to {sail_interval}s")

    def get_search_status(self) -> dict:
        """Get current search status"""
        status = {
            "search_active": self.search_active,
            "buoy_found": self.buoy_found,
            "buoy_hit": self.buoy_hit,
            "angle_offset": self.buoy_angle_offset,
            "distance": self.buoy_distance,
            "elapsed_time": 0
        }

        if self.search_start_time:
            status["elapsed_time"] = self.node.get_clock().now().seconds_nanoseconds()[0] - self.search_start_time

        return status

def create_event_control(event_type: str, waypoints: List[Waypoint], node: Node) -> Optional[EventControl]:
    """factory function to create appropriate event control"""
    control_classes = {
        "fleet_race": FleetRaceControl,
        "precision_navigation": PrecisionNavigationControl,
        "station_keeping": StationKeepingControl,
        "endurance": EnduranceControl,
        "payload": PayloadControl,
        "developer_mode": DeveloperControl,
        "search": SearchControl
    }

    control_class = control_classes.get(event_type.lower())
    if control_class:
        # Pass event_type to the constructor of the specific EventControl class
        return control_class(waypoints, node, event_type)
    else:
        node.get_logger().error(f"Unknown event type: {event_type}. Cannot create EventControl.")
        return None # Return None instead of raising ValueError to prevent node crash
