#!/bin/bash

# ROS2 Build Script for shared_core package

echo "Building shared_core package for ROS2..."

# Check if we're in a ROS2 workspace
if [ ! -f "package.xml" ]; then
    echo "Error: package.xml not found. Please run this script from the shared_core package directory."
    exit 1
fi

# Check if we're in a ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 environment not sourced. Please source your ROS2 installation first."
    echo "Example: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install openpyxl simpleaudio pynput numpy

# Install ROS2 dependencies
echo "Installing ROS2 dependencies..."
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-tf-transformations

# Build the package
echo "Building package..."
colcon build --packages-select shared_core

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "To use the package, source the workspace:"
    echo "source install/setup.bash"
    echo ""
    echo "Then run the DWA node:"
    echo "ros2 run shared_core dwa_node --param 1"
else
    echo "Build failed!"
    exit 1
fi 