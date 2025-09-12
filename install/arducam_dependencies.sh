#!/bin/bash
set -e

echo "Installing ArduCam dependencies..."
mkdir -p ~/Downloads && cd ~/Downloads

if [ ! -d libcamera ]; then
    echo "Cloning arducam (Arducam_tof_camera)..."
    git clone https://github.com/ArduCAM/Arducam_tof_camera.git arducam
fi
cd arducam

echo "Installing dependencies..."
./Install_dependencies.sh

echo "Installation complete!"


