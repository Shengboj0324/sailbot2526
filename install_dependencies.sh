#!/bin/bash

# Install all dependencies for Sailbot including Fortran support

echo "Installing Sailbot dependencies..."

# Update package list
sudo apt update

# Install system packages
echo "Installing system packages..."
sudo apt install -y \
    python3-serial \
    python3-smbus \
    i2c-tools \
    gfortran \
    meson \
    python3-pip \
    bc

# Install Python packages
echo "Installing Python packages..."
pip3 install websocket-client numpy

echo "Dependencies installed successfully!"
echo ""
echo "You can now run ./start_sailbot.sh"