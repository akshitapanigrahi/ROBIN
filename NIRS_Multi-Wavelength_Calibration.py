import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QLabel, QWidget
import serial
import time
from scipy.signal import butter, filtfilt, find_peaks

# Initialize variables
Fs = 200  # Sampling rate in Hz
numReadings = 150  # Window size for plotting
numLEDs = 4  # Number of LEDs (1 Red, 3 IR)
voltageData = np.zeros((numLEDs, numReadings))
PI_combined_data = np.zeros(numReadings)
timeVector = np.arange(numReadings) / Fs

# Initialize calibration variables
skintoneFactor = 1
initialRedAbsorbance = []
initialIRAbsorbance1 = []

# Bandpass filter setup
FcLow = 0.5  # Hz
FcHigh = 2.5  # Hz
b, a = butter(2, [FcLow, FcHigh], btype='bandpass', fs=Fs)
b_low, a_low = butter(2, FcLow, btype='low', fs=Fs)

# Setup PyQt Application
app = QApplication(sys.argv)
win = QWidget()
layout = QVBoxLayout()
win.setLayout(layout)

# Create labels for SpO2, HbT, PI, and Pulse
labels = {
    "SpO2": QLabel("SpO2: "),
    "HbT": QLabel("HbT: "),
    "PI": QLabel("PI: "),
    "Pulse": QLabel("HR: "),
}

for key in labels:
    layout.addWidget(labels[key])

# Create plots
plots = {
    "HbT": pg.PlotWidget(title="NIRS HbT Readings"),
    "SpO2": pg.PlotWidget(title="NIRS SpO2 Readings"),
    "PI": pg.PlotWidget(title="Perfusion Index (PI)"),
    "Pulse": pg.PlotWidget(title="Heart Rate"),
}

for key in plots:
    layout.addWidget(plots[key])

curves = {
    key: plots[key].plot(pen='y') for key in plots
}

win.setWindowTitle("Real-Time NIRS Readings")
win.resize(800, 600)
win.show()

# Connect to Arduino
arduino = serial.Serial('COM8', 9600)
time.sleep(2)  # Allow time for connection

# Real-time data reading and plotting
def update():
    global skintoneFactor, initialRedAbsorbance, initialIRAbsorbance1, voltageData, PI_combined_data

    data = arduino.readline().decode().strip()
    voltage = np.array([float(x) for x in data.split(",")])
    voltageData[:, :-1] = voltageData[:, 1:]  # Shift data
    voltageData[:, -1] = voltage  # Append new data

    red, IR1, IR2, IR3 = voltageData

    # Avoid divide-by-zero errors
    red[red <= 0] = 1e-6
    IR1[IR1 <= 0] = 1e-6
    IR2[IR2 <= 0] = 1e-6
    IR3[IR3 <= 0] = 1e-6

    # Absorbance calculations
    refVoltageRed, refVoltageIR = 1.99, 1.97
    redAbsorbance = np.log10(refVoltageRed / red)
    irAbsorbance1 = np.log10(refVoltageIR / IR1)

    # Calibration
    if len(initialRedAbsorbance) < 10:
        initialRedAbsorbance.append(np.mean(redAbsorbance))
        initialIRAbsorbance1.append(np.mean(irAbsorbance1))
        if len(initialRedAbsorbance) == 10:
            skintoneRatio = np.mean(initialRedAbsorbance) / np.mean(initialIRAbsorbance1)
            skintoneFactor = 1.1 if skintoneRatio < 1.5 else 1
    else:
        redAbsorbance *= skintoneFactor

    # HbO2 and HHb calculations
    epsilonO2Hb, epsilonHHb, pathLength = 0.25, 0.39, 0.15
    O2Hb = abs(irAbsorbance1) / (epsilonO2Hb * pathLength)
    HHb = abs(redAbsorbance) / (epsilonHHb * pathLength)
    HbT = O2Hb + HHb
    SpO2 = (O2Hb / HbT) * 100

    # Perfusion Index
    redDC = filtfilt(b_low, a_low, red)
    redAC = filtfilt(b, a, red)
    PI_red = (np.std(redAC) / np.mean(redDC)) * 100

    PI_combined = PI_red
    PI_combined_data[:-1] = PI_combined_data[1:]  # Shift data
    PI_combined_data[-1] = PI_combined

    # Pulse calculation
    IRAC = filtfilt(b, a, IR1)
    peaks, _ = find_peaks(IRAC, distance=int(Fs * 0.5))
    timeDiff = np.diff(peaks) / Fs
    pulse = [0] + list(60 / timeDiff)

    # Update plots
    curves["HbT"].setData(timeVector, HbT)
    curves["SpO2"].setData(timeVector, SpO2)
    curves["PI"].setData(timeVector, PI_combined_data)
    curves["Pulse"].setData(timeVector, pulse[:numReadings])

    # Update labels
    labels["SpO2"].setText(f"SpO2: {round(SpO2[-1], 2)} %")
    labels["HbT"].setText(f"HbT: {round(HbT[-1], 2)} g/dL")
    labels["PI"].setText(f"PI: {round(PI_combined, 2)} %")
    labels["Pulse"].setText(f"HR: {round(pulse[-1], 2)} bpm")


# Set timer for periodic updates
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

# Start PyQt application
sys.exit(app.exec_())
