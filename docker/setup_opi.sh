#!/bin/bash
# ============================================
# Script cài đặt nhanh trên Orange Pi PC+
# Chạy 1 lần duy nhất khi setup lần đầu
#
# Cách dùng:
#   chmod +x setup_opi.sh
#   ./setup_opi.sh
# ============================================

set -e
echo "======================================"
echo " SynMech Robot — Setup Orange Pi PC+"
echo "======================================"

# 1. Cài udev rules (gán tên USB cố định)
echo ""
echo "[1/3] Installing udev rules..."
if [ -f "$(dirname "$0")/../synmech_bringup/config/99-synmech-usb.rules" ]; then
    sudo cp "$(dirname "$0")/../synmech_bringup/config/99-synmech-usb.rules" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "  ✓ Udev rules installed. Lidar appear at /dev/rplidar"
else
    echo "  ✗ Cannot find file udev rules!"
fi

# 2. Thêm user vào group dialout (truy cập USB không cần sudo)
echo ""
echo "[2/3] Add user to group dialout..."
sudo usermod -aG dialout $USER
echo "  ✓ Added $USER to group dialout"
echo "  ⚠ Need logout/login again"

# 3. Kiểm tra Docker
echo ""
echo "[3/3] Checking Docker..."
if command -v docker &> /dev/null; then
    echo "  ✓ Docker đã cài: $(docker --version)"
    if command -v docker-compose &> /dev/null; then
        echo "  ✓ Docker Compose installed: $(docker-compose --version)"
    else
        echo "  ✗ Docker Compose not installed. Run: sudo apt install docker-compose"
    fi
else
    echo "  ✗ Docker not installed. Run:"
    echo "    curl -fsSL https://get.docker.com | sh"
    echo "    sudo usermod -aG docker $USER"
fi

echo ""
echo "======================================"
echo " Done! Next steps:"
echo ""
echo " 1. Logout/login again"
echo " 2. Plug in Lidar USB → Check by running: ls -la /dev/rplidar"
echo " 3. Launching Docker:"
echo "    cd synmech_mobot/docker"
echo "    docker-compose up"
echo ""
echo " 4. On Laptop run commands:"
echo "    export ROS_MASTER_URI=http://192.168.0.99:2710"
echo "    export ROS_IP=192.168.0.105"
echo "    roslaunch synmech_bringup remote_view.launch"
echo "======================================"
