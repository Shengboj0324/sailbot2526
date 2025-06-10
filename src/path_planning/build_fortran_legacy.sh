#!/bin/bash

# Legacy build script for Python < 3.12 or when meson is not available
# Uses the older distutils-based f2py approach

echo "[BUILD] Starting Fortran compilation for leg module (legacy mode)..."

# Navigate to the package directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if gfortran is installed
if ! command -v gfortran &> /dev/null; then
    echo "[BUILD] ERROR: gfortran not found. Please install it:"
    echo "        sudo apt-get install gfortran"
    exit 1
fi

# Create build directory
mkdir -p build
cd path_planning

# Try to use older f2py syntax that works with distutils
echo "[BUILD] Compiling leg_fortran.f90 using legacy f2py..."

# Method 1: Direct compilation with gfortran and f2py
f2py -c -m leg_fortran_module leg_fortran.f90 --fcompiler=gnu95 --opt='-O3'

if [ $? -ne 0 ]; then
    echo "[BUILD] Legacy method failed, trying alternative..."
    
    # Method 2: Two-step process
    echo "[BUILD] Step 1: Creating signature file..."
    f2py -m leg_fortran_module -h leg_fortran.pyf leg_fortran.f90 --overwrite-signature
    
    if [ $? -eq 0 ]; then
        echo "[BUILD] Step 2: Compiling from signature..."
        f2py -c leg_fortran.pyf leg_fortran.f90 --fcompiler=gnu95 --opt='-O3'
    fi
fi

if [ $? -eq 0 ]; then
    echo "[BUILD] Fortran compilation successful!"
    echo "[BUILD] Module created: leg_fortran_module"
    
    # List the generated files
    echo "[BUILD] Generated files:"
    ls -la leg_fortran_module*
else
    echo "[BUILD] ERROR: All compilation methods failed!"
    echo "[BUILD] The system will fall back to Python-only mode."
    exit 1
fi

cd ..

echo "[BUILD] Build complete!"