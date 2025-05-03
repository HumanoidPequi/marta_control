#!/bin/bash

# Setup script for MARTA control system

# Create build directory if it doesn't exist
mkdir -p build devel

# Build the packages
catkin_make

# Source the workspace
source devel/setup.bash

echo "==================================="
echo "MARTA Control System setup complete"
echo "==================================="
echo ""
echo "You can now launch the following modules:"
echo "1. Walking module:           roslaunch walking walk.launch"
echo "2. Inverse kinematics:       roslaunch kinematics inverse_kinematics.launch"
echo "3. Kick actions:             roslaunch actions kick.launch"
echo "4. Get-up actions:           roslaunch actions getup.launch"
echo ""
echo "To use these commands in new terminal sessions, run:"
echo "source $(pwd)/devel/setup.bash"