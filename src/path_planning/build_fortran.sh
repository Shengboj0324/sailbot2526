#!/bin/bash

# Build script for compiling Fortran leg module

echo "[BUILD] Starting Fortran compilation for leg module..."

# Navigate to the package directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if gfortran is installed
if ! command -v gfortran &> /dev/null; then
    echo "[BUILD] ERROR: gfortran not found. Please install it:"
    echo "        sudo apt-get install gfortran"
    exit 1
fi

# Check if f2py is available
if ! python3 -c "import numpy.f2py" &> /dev/null; then
    echo "[BUILD] ERROR: numpy.f2py not found. Please install numpy:"
    echo "        pip install numpy"
    exit 1
fi

# Check Python version and meson availability
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "[BUILD] Python version: $PYTHON_VERSION"

# Create build directory
mkdir -p build
cd path_planning

# Check if we need meson (Python >= 3.12)
if [ $(echo "$PYTHON_VERSION >= 3.12" | bc) -eq 1 ]; then
    echo "[BUILD] Python 3.12+ detected, checking for meson..."
    if ! command -v meson &> /dev/null; then
        echo "[BUILD] WARNING: meson not found, trying legacy build method..."
        cd ..
        ./build_fortran_legacy.sh
        exit $?
    fi
fi

# Compile the Fortran module using f2py
echo "[BUILD] Compiling leg_fortran.f90..."

# Use f2py to compile the Fortran module
python3 -m numpy.f2py -c leg_fortran.f90 -m leg_fortran_module --opt='-O3' --verbose

if [ $? -eq 0 ]; then
    echo "[BUILD] Fortran compilation successful!"
    echo "[BUILD] Module created: leg_fortran_module"
    
    # List the generated files
    echo "[BUILD] Generated files:"
    ls -la leg_fortran_module*
    
    # Copy the module to the install location if it exists
    if [ -d "../../install/path_planning/lib/python3.12/site-packages/path_planning/path_planning" ]; then
        echo "[BUILD] Copying module to install directory..."
        cp leg_fortran_module*.so ../../install/path_planning/lib/python3.12/site-packages/path_planning/path_planning/
    fi
else
    echo "[BUILD] ERROR: Fortran compilation failed!"
    exit 1
fi

cd ..

echo "[BUILD] Build complete!"