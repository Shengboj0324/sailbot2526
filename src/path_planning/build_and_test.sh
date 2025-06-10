#!/bin/bash

# Build and test the Fortran leg module

echo "============================================"
echo "Building and Testing Fortran Leg Module"
echo "============================================"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Build Fortran module
echo ""
echo "Step 1: Building Fortran module..."
echo "-----------------------------------"
./build_fortran.sh

if [ $? -ne 0 ]; then
    echo "ERROR: Fortran build failed!"
    exit 1
fi

# Run tests
echo ""
echo "Step 2: Running tests..."
echo "------------------------"
python3 test_fortran_leg.py

if [ $? -eq 0 ]; then
    echo ""
    echo "============================================"
    echo "SUCCESS: All tests passed!"
    echo "============================================"
else
    echo ""
    echo "============================================"
    echo "FAILURE: Some tests failed!"
    echo "============================================"
    exit 1
fi