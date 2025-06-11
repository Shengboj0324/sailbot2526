# Global Wind Angle Feature

## Overview
The global wind angle feature provides a backup wind direction for when the boat's wind sensor fails or is unreliable. The system automatically falls back to the global wind angle if the sensor data becomes stale.

## Features
- Manual global wind angle setting via ROS parameters
- Dynamic updates via ROS topic
- Automatic fallback when sensor times out
- Wind status reporting

## Usage

### Launch with Global Wind Parameters
```bash
ros2 run sailboat_control state_management_node --ros-args \
    -p event_type:=search \
    -p global_wind_angle:=45.0 \
    -p use_global_wind:=true \
    -p wind_sensor_timeout:=5.0
```

Parameters:
- `global_wind_angle`: Backup wind direction in degrees (0-360)
- `use_global_wind`: Force use of global wind (true/false)
- `wind_sensor_timeout`: Seconds without sensor data before using global wind

### Dynamic Wind Angle Updates
Publish to update the global wind angle during operation:
```bash
ros2 topic pub /global_wind_angle std_msgs/msg/Float32 "data: 90.0"
```

### Programmatic Control
```python
# In your code
event_control.set_global_wind_angle(45.0)  # Set backup angle
event_control.enable_global_wind(True)     # Force global wind use
event_control.set_wind_sensor_timeout(10.0)  # 10 second timeout
```

## Wind Status
The system publishes wind status in the boat status message:
```json
{
  "wind_status": {
    "current_wind_direction": 45.0,
    "sensor_wind_direction": 0.0,
    "global_wind_angle": 45.0,
    "use_global_wind": false,
    "wind_source": "sensor",
    "seconds_since_update": 2.3,
    "sensor_timed_out": false
  }
}
```

## Automatic Fallback
The system automatically uses global wind when:
1. Wind sensor hasn't updated for `wind_sensor_timeout` seconds
2. `use_global_wind` is set to true
3. No wind sensor data has been received yet

## Integration with Search Event
The search event uses `get_wind_direction()` which automatically returns:
- Sensor data if available and fresh
- Global wind angle if sensor is stale or forced

This ensures the boat can complete the search event even with sensor failures.