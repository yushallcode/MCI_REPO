#!/usr/bin/env python3

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PORT = "/dev/ttyUSB0"
BAUD = 115200
WINDOW = 200 # number of samples to display

ser = serial.Serial(PORT, BAUD, timeout=0.1)

acc_data = deque(maxlen=WINDOW)
gyro_data = deque(maxlen=WINDOW)
filt_data = deque(maxlen=WINDOW)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))


def update(_frame):
    while ser.in_waiting:
        try:
            line = ser.readline().decode("utf-8").strip()
            parts = line.split(",")
            if len(parts) == 3:
                acc_data.append(float(parts[0]))
                gyro_data.append(float(parts[1]))
                filt_data.append(float(parts[2]))
        except (ValueError, UnicodeDecodeError):
            pass

    ax1.clear()
    ax1.plot(list(acc_data), label="Accel Angle", alpha=0.7)
    ax1.plot(list(filt_data), label="Filtered Angle", linewidth=2)
    ax1.set_ylabel("Angle (deg)")
    ax1.legend(loc="upper right")
    ax1.set_ylim(-120, 120)
    ax1.grid(True, alpha=0.3)

    ax2.clear()
    ax2.plot(list(gyro_data), label="Gyro X (dps)", color="green")
    ax2.set_ylabel("Angular Rate (dps)")
    ax2.set_xlabel("Sample")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)


ani = animation.FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()
ser.close()