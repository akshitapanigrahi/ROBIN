import sys
import serial
import numpy as np
import pyqtgraph as pg
from scipy.signal import butter, filtfilt, find_peaks
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import pyqtSignal, QTimer
from qasync import asyncSlot
import qasync
import asyncio
import time
import os

# === Initialization ===
Fs = 200  # Sampling rate in Hz
numReadings = 200  # Window size for plotting
voltageData = np.zeros((3, numReadings))  # [red, IR, IR2] voltage
PI_combined_data = np.zeros(numReadings)
timeVector = np.arange(numReadings) / Fs

# Reference voltages and constants
refVoltageRed = 1.91
refVoltageIR = 1.95  # First IR wavelength (e.g., 880 nm)
refVoltageIR2 = 1.94  # Second IR wavelength (e.g., 940 nm)
epsilonO2Hb = 0.27  # Extinction coefficient for 880 nm
epsilonO2Hb2 = 0.28  # Extinction coefficient for 940 nm
epsilonHHb = 0.39  # Extinction coefficient for Red
pathLength = 0.09

# Bandpass and Lowpass filter setup
FcLow = 0.5  # Hz
FcHigh = 3.0  # Hz
b, a = butter(2, [FcLow / (Fs / 2), FcHigh / (Fs / 2)], btype='band')  # Bandpass
b_low, a_low = butter(2, FcLow / (Fs / 2), btype='low')  # Lowpass

# Serial Communication Setup
arduino = serial.Serial("/dev/cu.usbmodem11201", 9600)
time.sleep(2)

# GUI File
qtDesignerFile = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robin_simple_gui.ui")

class MyApp(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(qtDesignerFile, self)

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
        self.timer.start(100) #was 50

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

        # Reset labels
        self.labelSpO2.setText("SpO₂: --%")
        self.labelHbT.setText("HbT: --")
        self.labelPI.setText("PI: --%")
        self.labelHR.setText("HR: -- BPM")

        self.log.setText("Data reset complete.")

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
        voltageData[2, :-1] = voltageData[2, 1:]
        voltageData[0, -1] = voltage[0]  # Red voltage
        voltageData[1, -1] = voltage[1]  # IR voltage
        voltageData[2, -1] = voltage[2]  # IR2 voltage

        # Extract signals
        red = voltageData[0, :]
        IR = voltageData[1, :]
        IR2 = voltageData[2, :]

        # Apply filtering to separate AC and DC components
        redDC = filtfilt(b_low, a_low, red)
        IRDC = filtfilt(b_low, a_low, IR)
        IR2DC = filtfilt(b_low, a_low, IR2)

        redAC = filtfilt(b, a, red)
        IRAC = filtfilt(b, a, IR)
        IR2AC = filtfilt(b, a, IR2)

        # Calculate absorbances using DC components
        redAbsorbance = np.log10(refVoltageRed / redDC)
        irAbsorbance = np.log10(refVoltageIR / IRDC)
        ir2Absorbance = np.log10(refVoltageIR2 / IR2DC)

        # Calculate hemoglobin components
        O2Hb = np.abs(irAbsorbance / (epsilonO2Hb * pathLength))
        O2Hb2 = np.abs(ir2Absorbance / (epsilonO2Hb2 * pathLength))
        HHb = np.abs(redAbsorbance / (epsilonHHb * pathLength))

        # Average O2Hb from both IR wavelengths
        avgO2Hb = (O2Hb + O2Hb2) / 2

        # Total hemoglobin (HbT)
        self.HbT = avgO2Hb + HHb

        # SpO2 calculation
        self.SpO2 = (avgO2Hb / self.HbT) * 100
        self.SpO2 = np.clip(self.SpO2, 0, 100)

        # Perfusion Index (PI)
        PI_red = (np.std(redAC) / np.mean(redDC)) * 100
        PI_IR1 = (np.std(IRAC) / np.mean(IRDC)) * 100
        PI_IR2 = (np.std(IR2AC) / np.mean(IR2DC)) * 100
        PI_IR = (PI_IR1 + PI_IR2) / 2
        PI_combined = (PI_red + PI_IR) / 2
        PI_combined_data[:-1] = PI_combined_data[1:]
        PI_combined_data[-1] = PI_combined

        # Heart Rate Calculation
        # Use only the last 100 samples of IRAC for peak detection
        recent_IRAC = IRAC[-100:]  # Focus on the last 100 samples (0.5 seconds)
        peaks, _ = find_peaks(recent_IRAC, distance=round(Fs * 0.5))
        #peaks, _ = find_peaks(IRAC, distance=round(Fs * 0.5))  # Detect peaks in IRAC
        # if len(peaks) > 1:
        #     timeDiff = np.diff(peaks) / Fs
        #     self.pulse = 60 / np.mean(timeDiff)  # HR in BPM
        # else:
        #     self.pulse = 0  # Default to 0 if no peaks

        if len(peaks) > 1:
            timeDiff = np.diff(peaks) / Fs
            self.pulse = 60 / np.mean(timeDiff)  # HR in BPM
        else:
            # Fallback to frequency-domain HR calculation
            freqs = np.fft.rfftfreq(len(IRAC), d=1/Fs)
            fft_spectrum = np.abs(np.fft.rfft(IRAC))
            dominant_freq = freqs[np.argmax(fft_spectrum[1:]) + 1]  # Skip DC component
            self.pulse = dominant_freq * 60

        # Print metrics to the console
        print(f"HbT: {self.HbT:.2f}, SpO₂: {self.SpO2:.2f}%, PI: {PI_combined:.2f}%, HR: {self.pulse:.2f} BPM")

        # Update labels with current values
        self.labelSpO2.setText(f"SpO₂: {self.SpO2:.2f}%")
        self.labelHbT.setText(f"HbT: {self.HbT:.2f}")
        self.labelPI.setText(f"PI: {PI_combined:.2f}%")
        self.labelHR.setText(f"HR: {self.pulse:.2f} BPM")

        # Update plots
        self._curveHbT.setData(timeVector, self.HbT[:len(timeVector)])
        self._curveSpO2.setData(timeVector, self.SpO2[:len(timeVector)])
        self._curvePI.setData(timeVector, PI_combined_data[:len(timeVector)])
        self._curvePulse.setData(timeVector, np.full(len(timeVector), self.pulse))

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
