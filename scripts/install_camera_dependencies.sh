#!/bin/bash
set -e

echo "Installing dependencies..."
sudo apt update
sudo apt install -y build-essential git meson ninja-build pkg-config python3-ply \
    libdrm-dev libudev-dev libjpeg-dev libtiff5-dev libpng-dev libboost-all-dev  \
    libavcodec-dev libavformat-dev libavdevice-dev libavfilter-dev libavutil-dev \
    libegl1-mesa-dev libgles2-mesa-dev libv4l-dev v4l-utils libcamera-dev \
    libexif-dev libcamera-tools ros-jazzy-cv-bridge ros-jazzy-camera-info-manager-py 

CONFIG_FILE="/boot/firmware/config.txt"
OVERLAY="dtoverlay=imx219,cam0"
if ! grep -Fxq "$OVERLAY" "$CONFIG_FILE"; then
    echo "$OVERLAY" | sudo tee -a "$CONFIG_FILE"
    echo "Added $OVERLAY to $CONFIG_FILE"
else
    echo "Camera overlay already present in $CONFIG_FILE"
fi

mkdir -p ~/Downloads && cd ~/Downloads

if [ ! -d libcamera ]; then
    echo "Cloning libcamera..."
    git clone --recursive https://github.com/raspberrypi/libcamera.git
fi
cd libcamera
meson setup build --wipe
ninja -C build
sudo ninja -C build install

cd ~/Downloads
if [ ! -d rpicam-apps ]; then
    echo "Cloning rpicam-apps..."
    git clone https://github.com/raspberrypi/rpicam-apps.git
fi
cd rpicam-apps
meson setup build --wipe
ninja -C build
sudo ninja -C build install

echo "Installation complete! Please reboot your Raspberry Pi now to enable the camera."
