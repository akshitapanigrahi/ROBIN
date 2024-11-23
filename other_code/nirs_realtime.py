import serial
import numpy as np
import pyqtgraph as pg
from scipy.signal import butter, filtfilt, find_peaks
from pyqtgraph.Qt import QtGui, QtCore
import time

# === Initialization ===
Fs = 200  # Sampling rate in Hz
numReadings = 150  # Window size for plotting
voltageData = np.zeros((2, numReadings))  # [red, IR] voltage
PI_combined_data = np.zeros(numReadings)
timeVector = np.arange(numReadings) / Fs

# Reference voltages and constants
refVoltageRed = 1.99
refVoltageIR = 1.97
epsilonO2Hb = 0.25
epsilonHHb = 0.39
pathLength = 0.15

# Bandpass filter setup
FcLow = 0.5  # Hz
FcHigh = 2.5  # Hz
b, a = butter(2, [FcLow / (Fs / 2), FcHigh / (Fs / 2)], btype='band')
b_low, a_low = butter(2, FcLow / (Fs / 2), btype='low')

# === Serial Communication Setup ===
arduino = serial.Serial("COM11", 9600)
time.sleep(2)

# === PyQtGraph Setup ===
app = QtGui.QGuiApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Real-Time NIRS Data")
win.resize(1000, 600)
win.setWindowTitle('Real-Time NIRS Monitoring')

# Create four plots
plotHbT = win.addPlot(title="NIRS HbT Readings")
curveHbT = plotHbT.plot(pen='y')

win.nextRow()

plotSpO2 = win.addPlot(title="NIRS SpO2 Readings")
curveSpO2 = plotSpO2.plot(pen='g')
plotSpO2.setYRange(50, 120)

win.nextRow()

plotPI = win.addPlot(title="Perfusion Index (PI)")
curvePI = plotPI.plot(pen='r')

win.nextRow()

plotPulse = win.addPlot(title="Heart Rate")
curvePulse = plotPulse.plot(pen='b')

# Data buffers
HbT = np.zeros(numReadings)
SpO2 = np.zeros(numReadings)
pulse = np.zeros(numReadings)

# Update function for real-time plotting
def update():
    global voltageData, HbT, SpO2, PI_combined_data, pulse

    try:
        # Read data from Arduino
        data = arduino.readline().decode().strip()
        voltage = [float(x) for x in data.split(',')]

        # Update the circular buffer
        voltageData[0, :-1] = voltageData[0, 1:]
        voltageData[1, :-1] = voltageData[1, 1:]
        voltageData[0, -1] = voltage[0]  # Red voltage
        voltageData[1, -1] = voltage[1]  # IR voltage

        # Extract red and IR signals
        red = voltageData[0, :]
        IR = voltageData[1, :]
        red[red <= 0] = 1e-6
        IR[IR <= 0] = 1e-6

        # Absorbance calculations
        redAbsorbance = np.log10(refVoltageRed / red)
        irAbsorbance = np.log10(refVoltageIR / IR)

        # O2Hb and HHb calculations
        O2Hb = np.abs(irAbsorbance / (epsilonO2Hb * pathLength))
        HHb = np.abs(redAbsorbance / (epsilonHHb * pathLength))

        # HbT and SpO2 calculations
        HbT = O2Hb + HHb
        SpO2 = (O2Hb / HbT) * 100

        # Filtering
        redDC = filtfilt(b_low, a_low, red)
        IRDC = filtfilt(b_low, a_low, IR)
        redAC = filtfilt(b, a, red)
        IRAC = filtfilt(b, a, IR)

        # Perfusion Index (PI) calculation
        PI_red = (np.std(redAC) / np.mean(redDC)) * 100
        PI_IR = (np.std(IRAC) / np.mean(IRDC)) * 100
        PI_combined = (PI_red + PI_IR) / 2

        # Update the PI buffer
        PI_combined_data[:-1] = PI_combined_data[1:]
        PI_combined_data[-1] = PI_combined

        # Heart Rate (Pulse) calculation
        peaks, _ = find_peaks(IRAC, distance=round(Fs * 0.5))
        timeDiff = np.diff(peaks) / Fs
        if len(timeDiff) > 0:
            pulse = np.pad(60 / timeDiff, (0, numReadings - len(timeDiff)), constant_values=0)
        else:
            pulse = np.zeros(numReadings)

        # Update plots
        curveHbT.setData(timeVector, HbT * 6.45)
        curveSpO2.setData(timeVector, SpO2)
        curvePI.setData(timeVector, PI_combined_data)
        curvePulse.setData(timeVector, pulse)

    except Exception as e:
        print(f"Error: {e}")

# Set up a timer for periodic updates
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # Update every 50 ms

# Start the Qt event loop
QtGui.QApplication.instance().exec_()

# Cleanup
arduino.close()