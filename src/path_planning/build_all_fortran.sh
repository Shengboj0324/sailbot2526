#!/bin/bash

# Master build script for all Fortran implementations
# Compiles original, modern, and parallel versions

echo "========================================"
echo "Building All Fortran Implementations"
echo "========================================"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check prerequisites
echo "Checking prerequisites..."
if ! command -v gfortran &> /dev/null; then
    echo "ERROR: gfortran not found!"
    exit 1
fi

if ! command -v meson &> /dev/null; then
    echo "WARNING: meson not found - original f2py may fail on Python 3.12+"
fi

echo "Using gfortran version:"
gfortran --version | head -1

# Build original f2py version
echo ""
echo "1. Building original f2py version..."
echo "------------------------------------"
./build_fortran.sh
ORIGINAL_STATUS=$?

# Build modern Fortran versions
echo ""
echo "2. Building modern Fortran versions..."
echo "-------------------------------------"
./build_modern_fortran.sh
MODERN_STATUS=$?

# Summary
echo ""
echo "========================================"
echo "Build Summary"
echo "========================================"
echo -n "Original f2py version: "
if [ $ORIGINAL_STATUS -eq 0 ]; then
    echo "✓ SUCCESS"
else
    echo "✗ FAILED"
fi

echo -n "Modern versions: "
if [ $MODERN_STATUS -eq 0 ]; then
    echo "✓ SUCCESS"
else
    echo "✗ FAILED"
fi

# Copy all modules to install directory
if [ -d "$SCRIPT_DIR/install/path_planning/lib/python3.12/site-packages/path_planning/path_planning" ]; then
    echo ""
    echo "Copying modules to install directory..."
    cp path_planning/*.so "$SCRIPT_DIR/install/path_planning/lib/python3.12/site-packages/path_planning/path_planning/" 2>/dev/null
fi

echo ""
echo "Build complete!"
echo ""
echo "Available modules:"
ls -la path_planning/*.so 2>/dev/null || echo "No modules found"