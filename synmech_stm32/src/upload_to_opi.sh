#!/bin/bash
cd "$(dirname "$0")" # Go to scripts' directory

# Đọc cấu hình mạng
ENV_FILE="../network.env"
if [ -f "$ENV_FILE" ]; then
    export $(grep -v '^#' "$ENV_FILE" | xargs)
else
    echo "Error: cannot find file config $ENV_FILE"
    exit 1
fi

echo "Building firmware with PlatformIO..."
pio run

if [ $? -eq 0 ]; then
    echo "Success! Sending file to Orange Pi (${OPI_IP})..."
    scp .pio/build/genericSTM32F103C8/firmware.bin ${OPI_USER}@${OPI_IP}:/tmp/
    
    if [ $? -eq 0 ]; then
        echo "Sent file. Ready to press RESET button on STM32..."
        sleep 2
        echo "Running flash command on Orange Pi (Polling for 60s)..."
        ssh -t ${OPI_USER}@${OPI_IP} '
            echo "Press RESET button on STM32 now..."
            for i in {1..60}; do
                output=$(hid-flash /tmp/firmware.bin dummy 2>&1)
                if [[ "$output" == *"Finish"* || "$output" == *"Success"* || "$output" == *"bytes written"* ]]; then
                    echo "$output"
                    echo "SUCCESS!"
                    exit 0
                fi
                sleep 1
            done
            echo "Timeout (60s). Cannot find out BOOTLOADER."
            exit 1
        '
    else
        echo "Error: Cannnot send file to Orange Pi. Please check network/password."
    fi
else
    echo "Error: Failed. Check code before rebuild again."
fi
