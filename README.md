# Sailbot 2526 - Autonomous Sailing Robot Control System

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![Python](https://img.shields.io/badge/Python-3.12-green)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://ubuntu.com/desktop)
[![License](https://img.shields.io/badge/License-TODO-lightgrey)](#)

## Overview

Sailbot 2526 is a comprehensive autonomous sailing robot control system built on ROS2 (Robot Operating System 2). The system is designed for autonomous sailing robotics competitions, providing advanced path planning, sensor integration, and autonomous decision-making capabilities for various sailing events.

## Key Features

- **Multi-Event Support**: Fleet race, precision navigation, station keeping, endurance, payload, and search events
- **Advanced Path Planning**: Fortran-optimized algorithms for high-performance navigation calculations
- **Visual Servoing**: Camera-based buoy detection and approach for search events
- **Sensor Integration**: GPS, wind sensors, rudder/winch control, and cellular communication
- **Autonomous/RC Mode Switching**: Seamless transition between manual and autonomous control
- **Global Wind Backup**: Reliable wind direction backup system for sensor failures
- **Timing Control**: Precise actuator control timing for optimal performance
- **Web Integration**: Real-time status reporting and notifications

## System Architecture

```
sailbot2526/
├── src/
│   ├── sailboat_control/     # Core autonomous control logic
│   ├── sensors/             # Sensor nodes and hardware interfaces  
│   └── path_planning/       # Navigation algorithms and waypoint management
├── docs/                    # Documentation
├── util/                    # Utility scripts
└── start_sailbot.sh        # System launcher script
```

### Core Components

- **[Sailboat Control](src/sailboat_control/)**: Event management, boat state control, autonomous decision making
- **[Sensors](src/sensors/)**: GPS, wind sensors, rudder/winch control, cellular communication
- **[Path Planning](src/path_planning/)**: High-performance navigation algorithms with Fortran optimization

## Quick Start

### Prerequisites

**System Requirements:**
- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Python 3.12+
- GNU Fortran compiler

**Install Dependencies:**
```bash
# System dependencies
sudo apt update && sudo apt install -y \
    python3-serial python3-smbus i2c-tools \
    gfortran meson ros-jazzy-desktop

# Python dependencies  
pip3 install websocket-client numpy
```

### Build and Launch

1. **Clone and Build:**
```bash
git clone https://github.com/your-username/sailbot2526.git
cd sailbot2526
```

2. **Launch the System:**
```bash
./start_sailbot.sh
```

This script will:
- Build the Fortran modules for optimized path planning
- Compile all ROS2 packages
- Launch all sensor nodes and control systems

### Quick Test

```bash
# In a new terminal, test GPS node
ros2 topic echo /gps/fix

# Test wind sensor  
ros2 topic echo /wind/direction

# Check system status
ros2 topic echo /boat/status
```

## Supported Events

| Event Type | Description | Autonomous Features |
|------------|-------------|-------------------|
| **Fleet Race** (`F`) | Traditional sailing race | RC mode only |
| **Precision Navigation** (`Pr`) | Navigate precisely through waypoints | Full autonomous waypoint following |
| **Station Keeping** (`S`) | Maintain position in designated area | Position holding algorithms |
| **Endurance** (`E`) | Long-duration autonomous sailing | Extended waypoint sequences |
| **Payload** (`P`) | Carry payload while navigating | Specialized control for added weight |
| **Search** (`Se`) | Visual-based buoy detection and approach | Camera-based visual servoing |
| **Developer Mode** (`D`) | Testing and development | Local test waypoints |

### Event Selection

```bash
# Launch with specific event type
ros2 run sailboat_control state_management_node --ros-args -p event_type:=search

# Or specify in launch parameters
./start_sailbot.sh --event-type precision_navigation
```

## Navigation Features

### Advanced Path Planning
- **Fortran-optimized algorithms** for 10-50x performance improvements
- **Polar performance curves** for optimal sail trim calculations  
- **Weather routing** with wind pattern integration
- **Obstacle avoidance** algorithms

### Visual Servoing (Search Event)
- Real-time orange buoy detection using camera input
- P-controller based visual servoing for precise approach
- Distance-based hit detection with configurable thresholds
- Automatic upwind recovery after buoy contact

### Global Wind Backup System
Provides reliable wind direction when primary sensor fails:
- Manual wind angle configuration via ROS parameters
- Dynamic updates through ROS topics
- Automatic fallback with configurable timeout
- Status reporting for wind source monitoring

## Configuration

### Launch Parameters

```bash
ros2 run sailboat_control state_management_node --ros-args \
    -p event_type:=search \
    -p global_wind_angle:=270.0 \
    -p use_global_wind:=false \
    -p wind_sensor_timeout:=5.0 \
    -p rudder_update_interval:=3.0 \
    -p sail_update_interval:=10.0
```

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `event_type` | `developer_mode` | Competition event type |
| `global_wind_angle` | `0.0` | Backup wind direction (degrees) |
| `wind_sensor_timeout` | `5.0` | Timeout before using global wind (seconds) |
| `rudder_update_interval` | `3.0` | Rudder control update frequency |
| `sail_update_interval` | `10.0` | Sail control update frequency |

## ROS2 Topics

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/gps/fix` | `NavSatFix` | GPS position data |
| `/wind/direction` | `Float32` | Wind direction (degrees) |
| `/control/rudder_angle` | `Float32` | Rudder position command |
| `/control/sail_angle` | `Float32` | Sail position command |
| `/camera/buoy_angle_offset` | `Float32` | Visual servo angle error |
| `/boat/status` | `String` | JSON system status |
| `/web/notification` | `String` | Web server notifications |

### Monitor System Status

```bash
# Real-time system monitoring
ros2 topic echo /boat/status

# Visual servoing feedback
ros2 topic echo /camera/buoy_angle_offset

# Wind system status  
ros2 topic echo /wind/status
```

## Documentation

### Feature Documentation
- **[Global Wind System](docs/features/GLOBAL_WIND_README.md)** - Backup wind direction system
- **[Search Event](docs/features/SEARCH_EVENT_README.md)** - Visual buoy detection and approach
- **[Timing Control](docs/features/TIMING_CONTROL_README.md)** - Precise actuator timing control

### Development Documentation  
- **[Fortran Integration](docs/development/FORTRAN_README.md)** - High-performance path planning
- **[Fortran Improvements](docs/development/FORTRAN_IMPROVEMENTS.md)** - Performance optimization details
- **[Fortran Integration Guide](docs/development/FORTRAN_INTEGRATION.md)** - Integration methodology

### Architecture Documentation
- **[System Architecture](docs/architecture/)** - Overall system design
- **[Sensor Integration](docs/architecture/)** - Hardware interface specifications
- **[Event Management](docs/architecture/)** - Event system design

## Development

### Building Fortran Modules

```bash
cd src/path_planning
./build_all_fortran.sh
```

### Running Tests

```bash
# Path planning tests
cd src/path_planning  
python3 test_fortran_leg.py

# ROS2 package tests
colcon test --packages-select sailboat_control sensors path_planning
```

### Development Mode

```bash
# Launch in developer mode with local test waypoints
ros2 run sailboat_control state_management_node --ros-args -p event_type:=developer_mode
```

## Troubleshooting

### Common Issues

**Fortran Module Build Failures:**
```bash
# Check dependencies
which gfortran
python3 -c "import numpy.f2py"

# Manual rebuild
cd src/path_planning
./build_fortran.sh
```

**GPS Connection Issues:**
```bash
# Check serial port permissions
sudo usermod -a -G dialout $USER
# Then logout and login again

# Test GPS hardware
sudo cat /dev/ttyAMA0
```

**Wind Sensor Timeout:**
```bash  
# Enable global wind backup
ros2 param set /sailboat_control use_global_wind true
ros2 param set /sailboat_control global_wind_angle 270.0
```

### Debug Mode

```bash
# Enable verbose logging
export ROS_LOG_LEVEL=DEBUG
ros2 run sailboat_control state_management_node --ros-args -p event_type:=search
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style
- Follow PEP 8 for Python code
- Use meaningful variable names and comments
- Test all changes with the provided test scripts
- Update documentation for new features

## License

This project is licensed under the TODO License - see the [LICENSE](LICENSE) file for details.

## Competition Integration

This system is designed for autonomous sailing robotics competitions including:
- **SailBot** collegiate autonomous sailing competitions
- **World Robotic Sailing Championship (WRSC)**
- **International Robotic Sailing Conference (IRSC)** events

### Competition Rules Compliance
- Adheres to standard autonomous sailing competition protocols
- Implements required safety features and manual override capabilities
- Supports competition-standard waypoint navigation and event types

## Support

- **Issues**: [GitHub Issues](https://github.com/your-username/sailbot2526/issues)
- **Documentation**: [Project Wiki](https://github.com/your-username/sailbot2526/wiki)
- **Discussions**: [GitHub Discussions](https://github.com/your-username/sailbot2526/discussions)

---

**Built with care for autonomous sailing robotics**