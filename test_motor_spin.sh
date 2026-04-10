#!/usr/bin/env bash

CMD="cansend can0 156#A200E80300000000"
DURATION=4        # seconds
INTERVAL=0.05     # 20 Hz (adjust if you want)

END_TIME=$(($(date +%s%N) + DURATION*1000000000))

while [ $(date +%s%N) -lt $END_TIME ]; do
    $CMD
    sleep $INTERVAL
done