#!/bin/bash
# Fix I2C device permissions and ensure access for non-root users

# Exit immediately if a command fails
set -e

# Detect current user
USER_NAME=${SUDO_USER:-$USER}

echo "Fixing I2C permissions for user: $USER_NAME"

# 1. Ensure i2c group exists
if ! getent group i2c >/dev/null; then
    echo "Creating group 'i2c'..."
    sudo groupadd i2c
fi

# 2. Add current user to i2c group
echo "Adding $USER_NAME to group 'i2c'..."
sudo usermod -aG i2c "$USER_NAME"

# 3. Create udev rule for all /dev/i2c-* devices
echo "Creating udev rule..."
RULE_FILE="/etc/udev/rules.d/90-i2c.rules"
sudo bash -c "cat > $RULE_FILE" <<'EOF'
KERNEL=="i2c-[0-9]*", MODE="0660", GROUP="i2c"
EOF

# 4. Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 5. Apply to current session
echo "Udev rule installed at $RULE_FILE"
echo "You must log out or reboot for group changes to take effect."

# Optional: ask user to reboot now
read -p "Would you like to reboot now? [y/N]: " REBOOT
if [[ "$REBOOT" =~ ^[Yy]$ ]]; then
    echo "Rebooting..."
    sudo reboot
else
    echo "Run 'newgrp i2c' to apply group changes immediately."
fi
