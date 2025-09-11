# Fortran Path Planning Module

## Overview

The leg calculation module has been ported to Fortran for improved performance. The implementation maintains the same Python API while using Fortran for computationally intensive path calculations.

## Architecture

1. **leg_fortran.f90** - Pure Fortran implementation of path planning algorithms
2. **leg.py** - Python wrapper that maintains the original API
3. **build_fortran.sh** - Compilation script using f2py
4. **test_fortran_leg.py** - Test suite for verification

## Key Changes

- Fixed wind speed at 8 mph (3.58 m/s) - no longer a parameter
- Extensive debug printing in both Python and Fortran for verification
- Fallback to Python implementation if Fortran module unavailable

## Building

### Prerequisites

For Ubuntu 24.04 (Python 3.12+):
```bash
sudo apt-get install gfortran meson
pip install numpy
```

For older systems:
```bash
sudo apt-get install gfortran
pip install numpy
```

### Manual Build
```bash
cd path_planning
./build_fortran.sh
```

### Automatic Build
The Fortran module is automatically built when running:
```bash
./start_sailbot.sh
```

## Testing

Run the test suite:
```bash
./build_and_test.sh
```

Or just tests:
```bash
python3 test_fortran_leg.py
```

## Debug Output

Both Python and Fortran components include extensive debug printing:
- **[PYTHON]** - Python wrapper operations
- **[FORTRAN]** - Fortran calculations
- **[BUILD]** - Build process
- **[TEST]** - Test execution

## Performance

Expected performance improvements:
- 10-50x speedup for path calculations
- Most beneficial when calculating multiple paths per second
- Minimal overhead from Python-Fortran interface

## Integration Notes

1. The module gracefully falls back to Python if Fortran compilation fails
2. All ROS2 nodes automatically use the Fortran implementation when available
3. No changes required to navigation_node.py or event_control.py
4. The API remains exactly the same - only internal implementation changed

## Troubleshooting

If Fortran module fails to load:
1. Check gfortran installation: `which gfortran`
2. Check numpy installation: `python3 -c "import numpy.f2py"`
3. Run manual build: `./build_fortran.sh`
4. Check for .so file: `ls path_planning/leg_fortran_module*.so`

The system will continue to work with Python fallback if Fortran fails.