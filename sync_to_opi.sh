#!/bin/bash
# Script đồng bộ code từ Laptop sang Orange Pi

cd "$(dirname "$0")" # Trỏ về thư mục gốc synmech_mobot

# Đọc cấu hình mạng
if [ -f "network.env" ]; then
    export $(grep -v '^#' "network.env" | xargs)
else
    echo "Lỗi: Không tìm thấy file network.env"
    exit 1
fi

echo "[INFO] Synchronizing workspace to target: ${OPI_USER}@${OPI_IP}..."

# Rsync đồng bộ file (Bỏ qua các thư mục không cần thiết để tăng tốc)
rsync -avz --progress \
    --exclude='.git' \
    --exclude='.pio' \
    --exclude='synmech_stm32/.vscode' \
    ./ ${OPI_USER}@${OPI_IP}:~/catkin_ws/src/synmech_mobot/

if [ $? -eq 0 ]; then
    echo "[INFO] Synchronization completed successfully."
else
    echo "[ERROR] Synchronization failed. Verify network connectivity and credentials."
fi
