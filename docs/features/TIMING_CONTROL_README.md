# Autonomous Control Timing System

## Overview
The autonomous control system now implements precise timing controls for rudder and sail updates, with GPS position buffering for improved navigation stability.

## Key Features

### 1. **Timed Control Updates**
- **Rudder**: Updates every 3 seconds (configurable)
- **Sail**: Updates every 10 seconds OR when wind changes significantly (15°+)
- Prevents excessive actuator commands and reduces wear

### 2. **GPS Position Buffering**
- Maintains history of last 100 GPS positions (10 seconds at 10Hz)
- Uses positions from 3 seconds ago for navigation calculations
- Provides more stable heading/velocity calculations
- Reduces GPS noise effects

### 3. **Smart Wind-Based Sail Control**
- Monitors wind direction changes
- Updates sail immediately if wind shifts more than 15°
- Otherwise follows 10-second update interval
- Optimizes sail efficiency while minimizing adjustments

## Usage

### Launch with Timing Parameters
```bash
ros2 run sailboat_control state_management_node --ros-args \
    -p event_type:=search \
    -p rudder_update_interval:=3.0 \
    -p sail_update_interval:=10.0 \
    -p gps_buffer_delay:=3.0 \
    -p global_wind_angle:=270.0
```

### Parameters
- `rudder_update_interval`: Seconds between rudder updates (default: 3.0)
- `sail_update_interval`: Seconds between sail updates (default: 10.0)
- `gps_buffer_delay`: Seconds to look back for GPS position (default: 3.0)

### Programmatic Configuration
```python
# Configure timing for search event
search_control.configure_search(
    rudder_interval=2.0,     # 2 second rudder updates
    sail_interval=15.0       # 15 second sail updates
)

# Or set directly on event control
event_control.rudder_update_interval = 4.0
event_control.sail_update_interval = 8.0
event_control.position_buffer_delay = 2.0
```

## Implementation Details

### GPS Position Buffer
```python
# Position stored with timestamp
{
    'time': 1234567890.123,
    'lat': 42.12345,
    'lon': -71.54321
}
```

### Update Decision Logic
```python
# Rudder updates when:
current_time - last_rudder_update >= rudder_interval

# Sail updates when:
(current_time - last_sail_update >= sail_interval) OR
(abs(wind_change) >= 15.0°)
```

### Visual Servo Control (Search Event)
1. Camera provides angle offset to buoy
2. Every 3 seconds:
   - Get current position
   - Get position from 3 seconds ago
   - Calculate rudder adjustment
   - Send rudder command

## Benefits

1. **Stability**: Reduces oscillations from rapid control changes
2. **Efficiency**: Minimizes actuator wear and power consumption
3. **Noise Reduction**: GPS buffering smooths position data
4. **Wind Adaptation**: Responds quickly to significant wind shifts
5. **Predictability**: Consistent timing improves debugging

## Monitoring

The system logs timing information:
```
[INFO] Control timing - Rudder: 3.0s, Sail: 10.0s, GPS delay: 3.0s
[INFO] Rudder update: angle_offset=5.2°, rudder=10.4°, distance=15.3m
[INFO] Sail updated for downwind course
[INFO] Significant wind change detected: 18.5°
```

## Integration with Global Wind

The timing system works seamlessly with the global wind backup:
- Wind changes (sensor or global) trigger sail updates
- Timing controls apply regardless of wind source
- GPS buffering independent of wind system