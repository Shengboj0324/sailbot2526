# Search Event Implementation

## Overview
The search event implements visual-based autonomous control for locating and touching an orange buoy within a 100m search area.

## Key Features
- Visual servoing using camera angle feedback
- Distance-based buoy hit detection
- Automatic upwind pointing after buoy hit
- Web server notifications for key events
- 10-minute timeout protection

## How It Works

### 1. Initialization
- Boat starts 50m from search area
- Positioned for downwind approach
- Search reference position defined in waypoints

### 2. Search Phase
- Boat sails downwind while camera searches for orange buoy
- When buoy detected, visual servoing begins

### 3. Visual Servoing
- Camera publishes angle offset between boat heading and buoy
- P-controller adjusts rudder to minimize angle offset
- Boat approaches buoy while maintaining course

### 4. Hit Detection
- Distance threshold (default 2m) determines buoy "hit"
- Upon hit:
  - Web notification sent
  - Boat points upwind
  - Search complete

## Required Topics

### Subscriptions
- `camera/buoy_angle_offset` (Float32): Angle difference to buoy in degrees
- `camera/buoy_distance` (Float32): Distance to buoy in meters
- Standard GPS and wind topics

### Publications
- `control/rudder_angle` (Float32): Rudder control commands
- `control/sail_angle` (Float32): Sail control commands
- `web/notification` (String): JSON notifications for web server

## Usage

### Launch with search event:
```bash
ros2 run sailboat_control state_management_node --ros-args -p event_type:=search
```

### Configure search parameters:
```python
# In your code
search_control.configure_search(
    hit_threshold=3.0,      # Distance in meters
    rudder_gain=1.5,        # P-controller gain
    angle_tolerance=10.0    # Acceptable angle error
)
```

## Web Notifications

The system sends JSON notifications for:
- `search_started`: Search begins
- `buoy_detected`: Buoy first seen by camera
- `buoy_hit`: Buoy successfully touched
- `search_timeout`: 10-minute timeout reached

## Camera Integration

See `camera_buoy_detector_example.py` for a template camera node that:
1. Processes camera images
2. Detects orange buoy
3. Calculates angle offset and distance
4. Publishes required data

## Testing

1. Start the state management node with search event
2. Start your camera buoy detector node
3. Send autonomous start command (1) via radio
4. Monitor web notifications and boat behavior