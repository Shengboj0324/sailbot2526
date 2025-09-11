# Sailbot 2526 Documentation

Welcome to the comprehensive documentation for the Sailbot 2526 autonomous sailing robot control system.

## Documentation Structure

### Features
Documentation for specific system features and capabilities:

- **[Global Wind System](features/GLOBAL_WIND_README.md)** - Backup wind direction system for sensor failures
- **[Search Event](features/SEARCH_EVENT_README.md)** - Visual-based buoy detection and autonomous approach
- **[Timing Control](features/TIMING_CONTROL_README.md)** - Precise actuator timing and GPS buffering system

### Architecture
System design and architectural documentation:

- **System Overview** - High-level system architecture (Coming Soon)
- **Sensor Integration** - Hardware interface specifications (Coming Soon)
- **Event Management** - Event system design patterns (Coming Soon)

### Development
Development guides and technical implementation details:

- **[Fortran Integration](development/FORTRAN_README.md)** - High-performance path planning with Fortran
- **[Fortran Improvements](development/FORTRAN_IMPROVEMENTS.md)** - Performance optimization and benchmarking
- **[Fortran Integration Guide](development/FORTRAN_INTEGRATION.md)** - Implementation methodology and best practices

## Quick Navigation

### Getting Started
- [Main README](../README.md) - Project overview and quick start guide
- [Installation Guide](../README.md#quick-start) - Dependencies and setup
- [Launch Instructions](../README.md#build-and-launch) - How to start the system

### Feature Guides
- **Wind System**: Learn about the [global wind backup](features/GLOBAL_WIND_README.md) for reliable operation
- **Visual Navigation**: Understand the [search event](features/SEARCH_EVENT_README.md) visual servoing system
- **Control Timing**: Configure [timing controls](features/TIMING_CONTROL_README.md) for optimal performance

### Development Resources
- **Performance**: Implement [Fortran optimization](development/FORTRAN_README.md) for path planning
- **Testing**: Run tests with the provided scripts in each module
- **Debugging**: Use debug modes and logging features described in feature docs

## Event Documentation

### Supported Competition Events

| Event Type | Description | Documentation Status |
|------------|-------------|---------------------|
| **Fleet Race** | Traditional RC sailing | Basic implementation |
| **Precision Navigation** | Autonomous waypoint following | Core features complete |
| **Station Keeping** | Position holding | Core features complete |
| **Endurance** | Long-duration sailing | Core features complete |
| **Payload** | Sailing with payload | Core features complete |
| **Search** | Visual buoy detection | [Fully documented](features/SEARCH_EVENT_README.md) |
| **Developer Mode** | Testing and development | Core features complete |

### ROS2 Integration

All events are integrated with ROS2 Jazzy and support:
- Parameter-based configuration
- Real-time topic monitoring
- Status reporting and logging
- Web server notifications

## Configuration Reference

### Key Parameters

See individual feature documentation for detailed parameter lists:

- **[Global Wind Parameters](features/GLOBAL_WIND_README.md#usage)** - Wind system configuration
- **[Search Parameters](features/SEARCH_EVENT_README.md#usage)** - Visual servoing tuning
- **[Timing Parameters](features/TIMING_CONTROL_README.md#usage)** - Control loop timing

### ROS2 Topics

Comprehensive topic list available in the [main README](../README.md#ros2-topics).

## Testing and Validation

### Test Scripts
- **Fortran Testing**: `src/path_planning/test_fortran_leg.py`
- **Build Verification**: `src/path_planning/build_and_test.sh`
- **Performance Benchmarks**: `src/path_planning/quick_benchmark.py`

### Validation Procedures
1. **Hardware Tests**: Sensor connectivity and calibration
2. **Software Tests**: ROS2 node communication and timing
3. **Integration Tests**: Full system autonomous operation
4. **Performance Tests**: Path planning and control loop timing

## Support and Contributing

### Getting Help
- **Issues**: Report bugs and request features via GitHub Issues
- **Questions**: Use GitHub Discussions for technical questions
- **Documentation**: Suggest improvements to docs via Pull Requests

### Contributing
1. Follow the contribution guidelines in the [main README](../README.md#contributing)
2. Update relevant documentation when adding features
3. Ensure all tests pass before submitting PRs
4. Add new documentation for significant features

---

**Return to [Main README](../README.md)**