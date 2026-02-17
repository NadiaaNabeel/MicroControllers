import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyACM0', 115200)
plt.ion()

# === Plot Styles to Rotate Through ===
plot_styles = ['r.-', 'b.-', 'g.-', 'm.-', 'c.-', 'y.-', 'k.-']

# === Data Storage ===
NUM_SAMPLES = 20
time_ms = []
cnt = 0
all_signals = []  # List of lists. all_signals[0] = i values, all_signals[1] = j values, etc.
signals = ["Temperature"]
all_signals = [[]]   # one signal only

# === Plotting Function ===
def makeFig(title, xLabel, yLabel, yLimit, *args):
    plt.clf()
    plt.title(title)
    plt.grid(True)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    plt.ylim(yLimit[0], yLimit[1])  

    for arg in args:
        plt.plot(time_ms, arg[0], arg[1], label=arg[2])

    plt.legend(loc='upper left')

# === Main Loop ===
while True:
    try:
        line = sinWaveData.readline().decode('utf-8', errors='ignore').strip()

        if not line:
            continue

        try:
            temp = int(line)
        except:
            continue

        if not (0 <= temp <= 50):
            continue

        all_signals[0].append(temp)
        time_ms.append(cnt)
        cnt += 1

        if len(time_ms) > NUM_SAMPLES:
            time_ms.pop(0)
            all_signals[0].pop(0)

        args = [(all_signals[0], 'r.-', "Temperature")]

        drawnow(lambda: makeFig("Temperature Plot", "Time (ms)", "Value", [10, 30], *args))
        plt.pause(0.001)

    except Exception as e:
        print("Error:", e)

