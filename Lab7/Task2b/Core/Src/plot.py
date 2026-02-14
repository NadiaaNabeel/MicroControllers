import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyUSB0', 115200)
plt.ion()  # Enable interactive modecl

adcValues = []
filteredValues = []
time_ms = []
cnt = 0  # Time index (1 ms steps)

# === Plotting Function ===
def makeFig():
    plt.clf()
    plt.title('Live ADC & Filtered Data')
    plt.grid(True)
    plt.xlabel('Time (ms)')
    plt.ylabel('ADC Value')
    plt.ylim(0, 4095)  # Adjust range as needed
    plt.plot(time_ms, adcValues, 'r.-', label='Raw ADC')
    plt.plot(time_ms, filteredValues, 'b.-', label='Filtered Mean')
    plt.legend(loc='upper left')

# === Main Loop ===
while True:
    while sinWaveData.inWaiting() == 0:
        pass  # Wait for data
    try:
        line = sinWaveData.readline().decode().strip()

        # Split into parts
        values = line.split(',')

        # Must have exactly 2 numbers, both non-empty
        if len(values) != 2 or values[0] == '' or values[1] == '':
            continue

        try:
            raw = int(values[0])
            filtered = int(float(values[1]))  # STM32 sends filtered as float
        except ValueError:
            continue  # skip lines that still fail

        # Store values
        adcValues.append(raw)
        filteredValues.append(filtered)
        print(raw,filtered)
        time_ms.append(cnt)
        cnt += 25

        # Keep only last 500 samples
        if len(adcValues) > 500:
            adcValues.pop(0)
            filteredValues.pop(0)
            time_ms.pop(0)

        drawnow(makeFig)
        plt.pause(0.0001)

    except Exception as e:
        print("Error:", e)
