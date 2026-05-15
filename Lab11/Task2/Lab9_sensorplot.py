import matplotlib
matplotlib.use('TkAgg')
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ============================================================
# CONFIG — adjust port for your OS:
#   Linux/Mac : '/dev/ttyACM0' or '/dev/ttyUSB0'
#   Windows   : 'COM3', 'COM4', etc.
# ============================================================
PORT = '/dev/ttyUSB0'
BAUD        = 115200
MAX_SAMPLES = 500       # rolling window size

# ---- Serial setup ----
ser = serial.Serial(PORT, BAUD, timeout=1)

# ---- Data buffers ----
# UART format (5 fields): angle_x10, gyroX_x100, accX, pid, cutoff
angle_vals  = deque(maxlen=MAX_SAMPLES)   # tilt angle (degrees)
gyro_vals   = deque(maxlen=MAX_SAMPLES)   # gyro X     (dps)
acc_vals    = deque(maxlen=MAX_SAMPLES)   # accel X    (mg)
pid_vals    = deque(maxlen=MAX_SAMPLES)   # PID output (PWM)
cutoff_vals = deque(maxlen=MAX_SAMPLES)   # motor cutoff flag
time_ms     = deque(maxlen=MAX_SAMPLES)   # timestamp (ms)
cnt = 0   # running timestamp

# ---- Figure: 3 subplots ----
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig.suptitle('STM32 Self-Balancing Robot - Live Sensor Data', fontsize=13)


def is_data_line(raw):
    """
    Returns True if the line looks like a CSV data line (starts with
    an optional minus sign followed by a digit). Correctly handles
    negative values — the old raw[0].lstrip('-') approach broke on them.
    """
    stripped = raw.lstrip('-')
    return len(stripped) > 0 and stripped[0].isdigit()


def update(_frame):
    """Called by FuncAnimation at each interval; reads all waiting serial bytes."""
    global cnt

    while ser.in_waiting:
        raw = ''
        try:
            raw = ser.readline().decode(errors='ignore').strip()

            # Skip empty lines and human-readable startup messages
            if not raw or not is_data_line(raw):
                continue

            values = raw.split(',')

            # Expect exactly 5 fields: angle_x10, gyroX_x100, accX, pid, cutoff
            if len(values) != 5:
                continue

            # angle sent as integer x10 to preserve sign on values like -0.3
            angle   = int(values[0]) / 10.0    # e.g. -35  -> -3.5 degrees
            gyroX   = int(values[1]) / 100.0   # e.g.  47  ->  0.47 dps
            accX    = float(values[2])          # mg (integer, no scaling needed)
            pid_out = float(values[3])          # PID output (-999 to +999)
            cutoff  = int(values[4])            # motor cutoff flag (0 or 1)

            angle_vals.append(angle)
            gyro_vals.append(gyroX)
            acc_vals.append(accX)
            pid_vals.append(pid_out)
            cutoff_vals.append(cutoff)
            time_ms.append(cnt)

            # STM32 sends every 20 ISR ticks x 5 ms/tick = 100 ms
            cnt += 100

        except Exception as e:
            print(f"Parse error ({e!r}) on line: {raw!r}")

    t = list(time_ms)

    # ---- Subplot 1: Tilt Angle ----
    ax1.cla()
    ax1.set_title('Tilt Angle (Complementary Filter)')
    ax1.set_ylabel('Degrees')
    ax1.grid(True, alpha=0.4)
    ax1.axhline(0,   color='black', linewidth=0.8, linestyle='--', label='Setpoint ~ 0 deg')
    ax1.axhline(30,  color='red',   linewidth=0.8, linestyle=':',  label='Cutoff +30 deg')
    ax1.axhline(-30, color='red',   linewidth=0.8, linestyle=':',  label='Cutoff -30 deg')
    ax1.plot(t, list(angle_vals), 'b.-', markersize=3, linewidth=1, label='Angle')
    cutoff_t = [t[i] for i, c in enumerate(cutoff_vals) if c == 1]
    cutoff_a = [list(angle_vals)[i] for i, c in enumerate(cutoff_vals) if c == 1]
    if cutoff_t:
        ax1.scatter(cutoff_t, cutoff_a, color='red', zorder=5, s=20, label='Motor cutoff')
    ax1.legend(loc='upper left', fontsize=8)

    # ---- Subplot 2: Gyroscope X ----
    ax2.cla()
    ax2.set_title('Gyroscope X (L3GD20)')
    ax2.set_ylabel('Angular rate (dps)')
    ax2.grid(True, alpha=0.4)
    ax2.axhline(0, color='black', linewidth=0.8, linestyle='--')
    ax2.plot(t, list(gyro_vals), 'g.-', markersize=3, linewidth=1, label='Gyro X')
    ax2.legend(loc='upper left', fontsize=8)

    # ---- Subplot 3: Accelerometer X ----
    ax3.cla()
    ax3.set_title('Accelerometer X (LSM303DLHC)')
    ax3.set_ylabel('Acceleration (mg)')
    ax3.set_xlabel('Time (ms)')
    ax3.grid(True, alpha=0.4)
    ax3.axhline(0, color='black', linewidth=0.8, linestyle='--')
    ax3.plot(t, list(acc_vals), 'm.-', markersize=3, linewidth=1, label='Accel X')
    ax3.legend(loc='upper left', fontsize=8)

    plt.tight_layout()


# ---- Animate: update every 100 ms ----
ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)

try:
    plt.show()
finally:
    ser.close()
    print("Serial port closed.")
