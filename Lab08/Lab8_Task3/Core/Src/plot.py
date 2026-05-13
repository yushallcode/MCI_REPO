import serial
import matplotlib.pyplot as plt
from drawnow import *
import re  # Required for text parsing

# === Setup Serial ===
# IMPORTANT: If you are using an external USB-to-TTL adapter for USART1,
# the port is usually /dev/ttyUSB0. If you are using the board's USB, keep it /dev/ttyACM0
SERIAL_PORT = '/dev/ttyACM0' # Change to '/dev/ttyACM0' if needed
BAUD_RATE = 115200

try:
    sinWaveData = serial.Serial(SERIAL_PORT, BAUD_RATE)
except Exception as e:
    print(f"Failed to open port {SERIAL_PORT}. Did you check the permissions?")
    exit()

plt.ion()

tempValues = []
xValues = []
yValues = []
zValues = []
time_ms = []
cnt = 0

# === Plotting Function ===
def makeFig():
    plt.clf()
    plt.subplots_adjust(hspace=0.4)
    
    # Top Plot: Temperature
    plt.subplot(2,1,1)
    plt.title('Live Temperature')
    plt.grid(True)
    plt.ylabel('Temp (C)')
    plt.plot(time_ms, tempValues, 'r.-', label='Temperature')
    plt.legend(loc='upper left')
    
    # Bottom Plot: Gyroscope X, Y, Z
    plt.subplot(2,1,2)
    plt.title('Gyro X Y Z (dps)')
    plt.grid(True)
    plt.xlabel('Samples')
    plt.ylabel('Degrees per Sec')
    plt.plot(time_ms, xValues, 'g.-', label='X')
    plt.plot(time_ms, yValues, 'b.-', label='Y')
    plt.plot(time_ms, zValues, 'm.-', label='Z')
    plt.legend(loc='upper left')

# === Main Loop ===
print(f"Listening for data on {SERIAL_PORT}...")

while True:
    while sinWaveData.inWaiting() == 0:
        pass

    try:
        # Read line and ignore decoding errors from garbage startup bytes
        line = sinWaveData.readline().decode('utf-8', errors='ignore').strip()
        
        # Skip empty lines or the "System Initialized" startup message
        if line == "" or "System" in line:
            print(line)
            continue

        # Extract numbers using Regex
        # Matches format: "Temp: 25 C | X: -1, Y: 5, Z: 0 (dps)"
        match = re.search(r'Temp:\s*(-?\d+).*?X:\s*(-?\d+).*?Y:\s*(-?\d+).*?Z:\s*(-?\d+)', line)

        if match:
            # group(1) is Temp, group(2) is X, etc.
            temp = float(match.group(1))
            x = float(match.group(2))
            y = float(match.group(3))
            z = float(match.group(4))

            tempValues.append(temp)
            xValues.append(x)
            yValues.append(y)
            zValues.append(z)

            time_ms.append(cnt)
            cnt += 1

            drawnow(makeFig)
            plt.pause(0.0001)

            # Keep last 100 samples
            # Note: 500 makes `drawnow` very slow and laggy. 100 is smoother for live plotting.
            if len(tempValues) > 100:
                tempValues.pop(0)
                xValues.pop(0)
                yValues.pop(0)
                zValues.pop(0)
                time_ms.pop(0)
        else:
            # If the regex doesn't match, print it so you can see what went wrong
            print("Ignored unformatted line:", line)

    except Exception as e:
        print("Error processing line:", e)