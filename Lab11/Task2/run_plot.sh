#!/bin/bash

echo "Waiting for serial device..."
while true; do
    PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1)
    if [ -n "$PORT" ]; then
        echo "Found: $PORT"
        break
    fi
    echo "No port found, retrying in 2 seconds..."
    sleep 2
done

sudo fuser -k $PORT 2>/dev/null
sleep 1

sed -i "s|PORT *= *'/dev/tty[^']*'|PORT = '$PORT'|" Lab9_sensorplot.py
echo "Starting plot on $PORT..."
python3 Lab9_sensorplot.py
