#!/bin/bash
# Script to send speed setup commands to CAN IDs 21-26
# Example setup command (replace with actual command if needed)
START_CMD="8800000000000000"
CLEAR_CMD="9B00000000000000"
SPEED_CMD="A200000000000000"
for ID in 21 22 23 24 25 26; do
    HEX_ID=$(printf "%02X" $ID)
    echo "Sending to CAN ID $HEX_ID: cansend can0 ${HEX_ID}#${START_CMD}"
    sudo cansend can0 ${HEX_ID}#${START_CMD}
    echo "Sending to CAN ID $HEX_ID: cansend can0 ${HEX_ID}#${CLEAR_CMD}"
    sudo cansend can0 ${HEX_ID}#${CLEAR_CMD}
    echo "Sending to CAN ID $HEX_ID: cansend can0 ${HEX_ID}#${SPEED_CMD}"
    sudo cansend can0 ${HEX_ID}#${SPEED_CMD}
done
echo "Speed setup commands sent to CAN IDs 21-26."
