#!/bin/bash
set -e

echo "Installing ArduCam dependencies..."
#sudo systemctl --no-block restart netplan-wpa-wlan0.service ssh.service
sudo apt update
sudo apt install -y curl libopencv-dev python3-pip python3-opencv python3-numpy \
     libcamera-tools ros-jazzy-cv-bridge ros-jazzy-camera-info-manager-py \
     ros-jazzy-image-geometry

echo "Adding overlay to config.txt.. "
FIND_FILE=""
HAS_DTOVERLAY="true"
if [ -f "/boot/firmware/config.txt" ]; then # Bookworm
    FIND_FILE="/boot/firmware/config.txt"
    if [ $(grep -c -e '^dtoverlay=arducam-pivariety$' $FIND_FILE) -lt '1' ]; then
        HAS_DTOVERLAY="false"
    fi
elif [ -f "/boot/config.txt" ]; then # Bullseye and earlier
    FIND_FILE="/boot/config.txt"
    if [ $(grep -c -e '^dtoverlay=arducam-pivariety$' $FIND_FILE) -lt '1' ]; then
        HAS_DTOVERLAY="false"
    fi
fi

if [ "$FIND_FILE" = "" ]; then
    echo -e "\033[31m[ERR] No config.txt file found."
    exit 1
fi

if [ "$HAS_DTOVERLAY" = "false" ]; then
    echo -e "\033[31m[WARN]\033[0m dtoverlay=arducam-pivariety not found in $FIND_FILE."
    # remove all line which has dtoverlay=arducam-pivariety
    sudo sed -i '/dtoverlay=arducam-pivariety/d' $FIND_FILE
    echo "dtoverlay=arducam-pivariety" | sudo tee -a $FIND_FILE
    # if "Raspberry Pi 5" in `cat /sys/firmware/devicetree/base/model`
    if [ -f /sys/firmware/devicetree/base/model ]; then
        if grep -q "Raspberry Pi 5" /sys/firmware/devicetree/base/model; then
            echo "dtoverlay=arducam-pivariety,cam0" | sudo tee -a $FIND_FILE
        fi
    fi
    echo -e "\033[32m[INFO]\033[0m dtoverlay=arducam-pivariety added to $FIND_FILE."
fi

if [ $(grep -c "camera_auto_detect=1" $FIND_FILE) -ne '0' ]; then
    sudo sed -i "s/\(^camera_auto_detect=1\)/camera_auto_detect=0/" $FIND_FILE
fi
if [ $(grep -c "camera_auto_detect=0" $FIND_FILE) -lt '1' ]; then
    sudo bash -c "echo camera_auto_detect=0 >> $FIND_FILE"
fi

echo "Installing ArduCam SDK... "
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://arducam.github.io/arducam_ppa/KEY.gpg | sudo gpg --dearmor -o /etc/apt/keyrings/arducam.gpg
echo "deb [signed-by=/etc/apt/keyrings/arducam.gpg] https://arducam.github.io/arducam_ppa/ linux main" | sudo tee /etc/apt/sources.list.d/arducam.list > /dev/null

sudo apt update
sudo apt-get install -y arducam-config-parser-dev arducam-evk-sdk-dev arducam-tof-sdk-dev

echo "Installing ArduCam Python library... "
sudo python3 -m pip install ArducamDepthCamera --break-system-packages

echo "Installation complete!"
echo "However, the script settings will only take effect after restarting."
echo "Reboot now? (y/n):"
read -r USER_INPUT
case $USER_INPUT in
'y' | 'Y')
    echo "reboot"
    sudo reboot
    ;;
*)
    echo "Please restart yourself later."
    exit 1
    ;;
esac


