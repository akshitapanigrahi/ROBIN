import sys
import serial
import numpy as np
import pyqtgraph as pg
from scipy.signal import butter, filtfilt, find_peaks
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import pyqtSignal, QTimer, QDateTime
from qasync import asyncSlot
import qasync
import asyncio
import time
import os

# === Initialization ===
Fs = 200  # Sampling rate in Hz
numReadings = 150  # Window size for plotting
voltageData = np.zeros((2, numReadings))  # [red, IR] voltage
PI_combined_data = np.zeros(numReadings)
timeVector = np.arange(numReadings) / Fs

# Reference voltages and constants
refVoltageRed = 1.95
refVoltageIR = 1.96
refVoltageIR2 = 2.0
epsilonO2Hb = 0.25
epsilonO2Hb2 = 0.28
epsilonHHb = 0.39
pathLength = 0.15

# Bandpass filter setup
FcLow = 0.5  # Hz
FcHigh = 2.5  # Hz
b, a = butter(2, [FcLow / (Fs / 2), FcHigh / (Fs / 2)], btype='band')
b_low, a_low = butter(2, FcLow / (Fs / 2), btype='low')

# Serial Communication Setup
arduino = serial.Serial("COM11", 9600)
time.sleep(2)

# GUI File
qtDesignerFile = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robin_simple_gui.ui")

class MyApp(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(qtDesignerFile, self)

        # Plot setup
        # Plot setup (Stacked Vertically)
        self.plot_layout = self.graphicsView.addLayout()

        # Create separate rows for each plot
        self._curveHbT = self.plot_layout.addPlot(title="NIRS HbT Readings").plot(pen='y')
        self.plot_layout.nextRow()  # Move to the next row for vertical stacking

        self._curveSpO2 = self.plot_layout.addPlot(title="NIRS SpO2 Readings").plot(pen='g')
        self.plot_layout.nextRow()

        self._curvePI = self.plot_layout.addPlot(title="Perfusion Index (PI)").plot(pen='r')
        self.plot_layout.nextRow()

        self._curvePulse = self.plot_layout.addPlot(title="Heart Rate").plot(pen='b')


        # Button connections using asyncSlot
        self.streamButton.clicked.connect(self.handle_start)
        self.stopButton.clicked.connect(self.handle_stop)
        self.resetButton.clicked.connect(self.handle_reset)

        # Data buffers
        self.HbT = np.zeros(numReadings)
        self.SpO2 = np.zeros(numReadings)
        self.pulse = np.zeros(numReadings)

        # Timer for real-time plotting
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)

        # Control flags
        self.is_streaming = False

    @asyncSlot()
    async def handle_start(self):
        self.log.setText("Starting data stream...")
        self.is_streaming = True
        self.timer.start(50)

    @asyncSlot()
    async def handle_stop(self):
        self.log.setText("Stopping data stream...")
        self.is_streaming = False
        self.timer.stop()

    @asyncSlot()
    async def handle_reset(self):
        self.log.setText("Resetting data...")
        self.is_streaming = False
        self.timer.stop()

        # Clear the data buffers completely
        self.HbT = np.zeros(numReadings)
        self.SpO2 = np.zeros(numReadings)
        self.pulse = np.zeros(numReadings)
        PI_combined_data[:] = 0
        voltageData[:, :] = 0

        # Clear the plots
        self._curveHbT.setData([], [])
        self._curveSpO2.setData([], [])
        self._curvePI.setData([], [])
        self._curvePulse.setData([], [])

        self.log.setText("Data reset complete.")

    def calculate_hr_fft(self, signal, fs):
        if len(signal) < 2 * fs:
            return 0  # Not enough data for FFT
        signal = signal - np.mean(signal)  # Remove DC component
        freqs = np.fft.rfftfreq(len(signal), 1 / fs)
        fft_magnitude = np.abs(np.fft.rfft(signal))
        valid_idx = (freqs >= 0.5) & (freqs <= 3.5)  # HR range: 30–210 BPM
        if not np.any(valid_idx):
            return 0  # No valid frequencies
        dominant_freq = freqs[valid_idx][np.argmax(fft_magnitude[valid_idx])]
        hr = dominant_freq * 60
        return hr

    def update_plot(self):
        if not self.is_streaming:
            return

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
            self.HbT = O2Hb + HHb
            self.SpO2 = (O2Hb / self.HbT) * 100

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

            # peaks, _ = find_peaks(IRAC, distance=round(Fs * 0.5))
            # timeDiff = np.diff(peaks) / Fs
            # if len(timeDiff) > 0:
            #     self.pulse = np.pad(60 / timeDiff, (0, numReadings - len(timeDiff)), constant_values=0)
            # else:
            #     self.pulse = np.zeros(numReadings)

            # Heart Rate (Pulse) calculation
            # try:
            #     hr = self.calculate_hr_fft(IRAC, Fs)
            #     self.pulse[:-1] = self.pulse[1:]  # Shift left
            #     self.pulse[-1] = hr  # Insert the latest HR value
            # except Exception as e:
            #     print(f"Heart rate calculation error: {e}")
            #     self.pulse[-1] = 0  # Set HR to 0 in case of error

            # Update plots
            try:
                self._curveHbT.setData(timeVector, self.HbT[:len(timeVector)] * 6.45)
                self._curveSpO2.setData(timeVector, self.SpO2[:len(timeVector)])
                self._curvePI.setData(timeVector, PI_combined_data[:len(timeVector)])
                self._curvePulse.setData(timeVector, self.pulse[:len(timeVector)])
            except Exception as e:
                print(f"Plotting error: {e}")


        except Exception as e:
            print(f"Error: {e}")

    def closeEvent(self, event):
        arduino.close()
        super().closeEvent(event)
        asyncio.get_event_loop().stop()


def main():
    app = QtWidgets.QApplication(sys.argv)
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    mainWindow = MyApp()
    mainWindow.show()

    with loop:
        loop.run_forever()


if __name__ == "__main__":
    main()