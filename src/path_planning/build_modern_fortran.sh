#!/bin/bash

# Build script for modern high-performance Fortran module
# Uses gfortran with aggressive optimizations and modern standards

echo "[BUILD] Starting modern Fortran compilation..."

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR/path_planning"

# Compiler flags for maximum performance
FFLAGS="-std=f2018 -O3 -march=native -mtune=native -ffast-math -funroll-loops"
FFLAGS="$FFLAGS -fno-protect-parens -fstack-arrays -fcoarray=single"
FFLAGS="$FFLAGS -Wall -Wextra -Wno-unused-dummy-argument"

# Additional optimization flags
FFLAGS="$FFLAGS -flto -finline-functions"
FFLAGS="$FFLAGS -fprefetch-loop-arrays -fipa-pta"

# Build the modern library
echo "[BUILD] Compiling modern version with flags: $FFLAGS"
gfortran $FFLAGS -fPIC -shared -o leg_modern.so leg_modern.f90

if [ $? -ne 0 ]; then
    echo "[BUILD] ERROR: Modern compilation failed!"
    exit 1
fi

# Build the parallel library with OpenMP
echo "[BUILD] Compiling parallel version with OpenMP..."
PFLAGS="$FFLAGS -fopenmp -DUSE_OPENMP"
gfortran $PFLAGS -fPIC -shared -o leg_parallel.so leg_parallel.f90 -lgomp

if [ $? -eq 0 ]; then
    echo "[BUILD] Modern Fortran module compiled successfully!"
    echo "[BUILD] Library created: leg_modern.so"
    ls -la leg_modern.so
    
    # Copy to install directory if it exists
    if [ -d "../../install/path_planning/lib/python3.12/site-packages/path_planning/path_planning" ]; then
        echo "[BUILD] Copying to install directory..."
        cp leg_modern.so ../../install/path_planning/lib/python3.12/site-packages/path_planning/path_planning/
    fi
else
    echo "[BUILD] ERROR: Compilation failed!"
    exit 1
fi

echo "[BUILD] Build complete!"