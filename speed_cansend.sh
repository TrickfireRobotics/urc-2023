#!/bin/bash
# This script should run when you launch the robot. It loads the CAN driver dependencies, sets up the CAN interface, and sends the necessary commands to the motors to enable speed mode.
START_CMD="8800000000000000"
CLEAR_CMD="9B00000000000000"
SPEED_CMD="A200000000000000"
#Ensure driver dependencies are laoded
modprobe can
modprobe can_raw
modprobe mttcan
#Set up the CAN interface
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
echo "CAN interface can0 set up successfully."
#Enable the motors in speed mode (not torque mode)
for ID in 21 22 23 24 25 26; do
    CAN_ID=$((ID + 140))
    echo "Sending to CAN ID $CAN_ID: cansend can0 ${CAN_ID}#${START_CMD}"
    sudo cansend can0 ${CAN_ID}#${START_CMD}
    echo "Sending to CAN ID $CAN_ID: cansend can0 ${CAN_ID}#${CLEAR_CMD}"
    sudo cansend can0 ${CAN_ID}#${CLEAR_CMD}
    echo "Sending to CAN ID $CAN_ID: cansend can0 ${CAN_ID}#${SPEED_CMD}"
    sudo cansend can0 ${CAN_ID}#${SPEED_CMD}
done
echo "Speed setup commands sent to CAN IDs 21-26."
