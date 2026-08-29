#!/bin/bash
# ============================================
# How to use (Only first time)):
#   chmod +x setup_opi.sh
#   ./setup_opi.sh
# ============================================

set -e
echo "======================================"
echo " SynMech Robot — Setup Orange Pi PC+"
echo "======================================"

# Install udev rules
echo ""
echo "[1/3] Installing udev rules..."
if [ -f "$(dirname "$0")/../synmech_bringup/config/99-synmech-usb.rules" ]; then
    sudo cp "$(dirname "$0")/../synmech_bringup/config/99-synmech-usb.rules" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "Success: Udev rules installed. Lidar appear at /dev/rplidar"
else
    echo "Error: Cannot find file udev rules!"
fi

# Install Lidar Locker Service
echo ""
echo "[1.5/3] Installing Lidar Locker service..."
if [ -f "$(dirname "$0")/../synmech_bringup/config/lidar_locker.service" ]; then
    REPO_DIR=$(realpath "$(dirname "$0")/..")
    cat "$REPO_DIR/synmech_bringup/config/lidar_locker.service" | sed "s|REPO_DIR_PLACEHOLDER|$REPO_DIR|g" > /tmp/lidar_locker.service
    sudo cp /tmp/lidar_locker.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable --now lidar_locker.service
    echo "Success: Lidar Locker installed and started!"
else
    echo "Error: Cannot find lidar_locker.service template!"
fi

# Add user to dialout group
echo ""
echo "[2/3] Add user to group dialout..."
sudo usermod -aG dialout $USER
echo "Success: Added $USER to group dialout"
echo "Warning: Need logout/login again"

# Checking Docker
echo ""
echo "[3/3] Checking Docker..."
if command -v docker &> /dev/null; then
    echo "Success - Docker installed: $(docker --version)"
    if command -v docker-compose &> /dev/null; then
        echo "Success - Docker Compose installed: $(docker-compose --version)"
    else
        echo "Failed - Docker Compose not installed. Run: sudo apt install docker-compose"
    fi
else
    echo "Failed - Docker not installed. Run:"
    echo "         curl -fsSL https://get.docker.com | sh"
    echo "         sudo usermod -aG docker $USER"
fi

echo ""
echo ""
echo ""
echo " Done! Next steps:"
echo ""
echo " 1. Logout/login again"
echo " 2. Plug in Lidar USB & Check by running: ls -la /dev/rplidar"
echo " 3. Launching Docker:"
echo "    cd synmech_mobot/docker"
echo "    docker-compose up"
echo ""
echo " 4. Configured IPs correctly in ~/catkin_ws/src/synmech_mobot/network.env"
echo " 5. On Laptop run commands:"
echo "    source ~/catkin_ws/src/synmech_mobot/network.env"
echo "    roslaunch synmech_bringup remote_view.launch"
