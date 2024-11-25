import sys
import serial
import numpy as np
import pyqtgraph as pg
pg.setConfigOption('background', 'w')  # Set global background to white
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
numReadings = 200  # Window size for plotting
voltageData = np.zeros((3, numReadings))  # [red, IR] voltage
PI_combined_data = np.zeros(numReadings)
timeVector = np.arange(numReadings) / Fs

# Reference voltages and constants
refVoltageRed = 1.91
refVoltageIR = 1.95
refVoltageIR2 = 1.94  # For the additional IR wavelength
epsilonO2Hb = 0.27 # 880 nm
epsilonO2Hb2 = 0.28  # For the additional IR wavelength, 940 nm
epsilonHHb = 0.39 #red
#epsilonHHb2 = 0.42  #dont think we need this one since we only have 1 red LED?
pathLength = 0.07

# Bandpass filter setup
FcLow = 0.5  # Hz
FcHigh = 2.5  # Hz
b, a = butter(2, [FcLow / (Fs / 2), FcHigh / (Fs / 2)], btype='band')
b_low, a_low = butter(2, FcLow / (Fs / 2), btype='low')

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

        self.update_counter = 0

        # Create HbT plot
        self.plotHbT = self.plot_layout.addPlot(title="NIRS HbT Readings")
        self._curveHbT = self.plotHbT.plot(pen='k', width=10)
        self.plotHbT.setYRange(0, 30)  # Set Y-axis range for HbT
        self.plotHbT.setLabel('left', 'HbT (µM)', units='µM')  # Add Y-axis label
        self.plot_layout.nextRow()

        # Create SpO2 plot
        self.plotSpO2 = self.plot_layout.addPlot(title="NIRS SpO2 Readings")
        self._curveSpO2 = self.plotSpO2.plot(pen='k', width=10)
        self.plotSpO2.setYRange(50, 100)  # Set Y-axis range for SpO2
        self.plotSpO2.setLabel('left', 'SpO₂ (%)', units='%')  # Add Y-axis label
        self.plot_layout.nextRow()

        # Create PI plot
        self.plotPI = self.plot_layout.addPlot(title="Perfusion Index (PI)")
        self._curvePI = self.plotPI.plot(pen='k', width=10)
        self.plotPI.setYRange(0, 20)  # Set Y-axis range for PI
        self.plotPI.setLabel('left', 'PI (%)', units='%')  # Add Y-axis label
        self.plot_layout.nextRow()

        # Create Heart Rate plot
        self.plotPulse = self.plot_layout.addPlot(title="Heart Rate")
        self._curvePulse = self.plotPulse.plot(pen='k', width=10)
        self.plotPulse.setYRange(40, 120)  # Set Y-axis range for Heart Rate
        self.plotPulse.setLabel('left', 'HR (BPM)', units='BPM')  # Add Y-axis label

        # Button connections using asyncSlot
        self.streamButton.clicked.connect(self.handle_start)
        self.stopButton.clicked.connect(self.handle_stop)
        self.resetButton.clicked.connect(self.handle_reset)

        # Timer for real-time plotting
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)

        # Control flags
        self.is_streaming = False

        # Data buffers
        self.HbT = np.zeros(numReadings)
        self.SpO2 = np.zeros(numReadings)
        self.pulse = np.zeros(numReadings)

    @asyncSlot()
    async def handle_start(self):
        self.log.setText("Starting data stream...")
        self.is_streaming = True
        self.timer.start(5)

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

    # def calculate_hr_fft(self, signal, fs):
    #     if len(signal) < 2 * fs:
    #         return 0  # Not enough data for FFT
    #     signal = signal - np.mean(signal)  # Remove DC component
    #     freqs = np.fft.rfftfreq(len(signal), 1 / fs)
    #     fft_magnitude = np.abs(np.fft.rfft(signal))
    #     valid_idx = (freqs >= 0.5) & (freqs <= 3.5)  # HR range: 30–210 BPM
    #     if not np.any(valid_idx):
    #         return 0  # No valid frequencies
    #     dominant_freq = freqs[valid_idx][np.argmax(fft_magnitude[valid_idx])]
    #     hr = dominant_freq * 60
    #     return hr

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

            # Extract red and IR signals
            red = voltageData[0, :]
            IR = voltageData[1, :]
            IR2 = voltageData[2, :]
            red[red <= 0] = 1e-6
            IR[IR <= 0] = 1e-6
            IR2[IR2 <= 0] = 1e-6

            print("Red Voltage:", voltage[0], "IR Voltage:", voltage[1])

            # Absorbance calculations
            # redAbsorbance = np.log10(refVoltageRed / red)
            # irAbsorbance = np.log10(refVoltageIR / IR)
            # ir2Absorbance = np.log10(refVoltageIR2 / IR2)

            # O2Hb and HHb calculations
            # O2Hb = np.abs(irAbsorbance / (epsilonO2Hb * pathLength))
            # O2Hb2 = np.abs(ir2Absorbance / (epsilonO2Hb2 * pathLength))
            # HHb = np.abs(redAbsorbance / (epsilonHHb * pathLength))
            # HHb2 = np.abs(redAbsorbance / (epsilonHHb * pathLength))

            # avgO2Hb = (O2Hb + O2Hb2) / 2

            # HbT and SpO2 calculations
            # self.HbT = O2Hb + HHb
            # self.SpO2 = (O2Hb / self.HbT) * 100

            # Filtering
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

            O2Hb = np.abs(irAbsorbance / (epsilonO2Hb * pathLength))
            O2Hb2 = np.abs(ir2Absorbance / (epsilonO2Hb2 * pathLength))
            HHb = np.abs(redAbsorbance / (epsilonHHb * pathLength))
            HHb2 = np.abs(redAbsorbance / (epsilonHHb * pathLength))

            avgO2Hb = (O2Hb + O2Hb2) / 2

            self.HbT = (avgO2Hb + HHb)
            self.SpO2 = (avgO2Hb / self.HbT) * 100

            # Perfusion Index (PI) calculation
            PI_red = (np.std(redAC) / np.mean(redDC)) * 100
            PI_IR = (np.std(IRAC) / np.mean(IRDC)) * 100
            PI_IR2 = (np.std(IR2AC) / np.mean(IR2DC)) * 100

            PI_combined = (PI_red + PI_IR + PI_IR2) / 3

            # # Update the PI buffer
            # PI_combined_data[:-1] = PI_combined_data[1:]
            # PI_combined_data[-1] = PI_combined

            #         # Heart Rate Calculation
            # # Use only the last 100 samples of IRAC for peak detection
            # recent_IRAC = IRAC[-100:]  # Focus on the last 100 samples (0.5 seconds)
            # peaks, _ = find_peaks(recent_IRAC, distance=round(Fs * 0.5))
            # #peaks, _ = find_peaks(IRAC, distance=round(Fs * 0.5))  # Detect peaks in IRAC
            # # if len(peaks) > 1:
            # #     timeDiff = np.diff(peaks) / Fs
            # #     self.pulse = 60 / np.mean(timeDiff)  # HR in BPM
            # # else:
            # #     self.pulse = 0  # Default to 0 if no peaks

            # # Ensure pulse is a buffer (array) that stores historical HR values
            # if len(peaks) > 1:
            #     timeDiff = np.diff(peaks) / Fs
            #     hr = 60 / np.mean(timeDiff)  # HR in BPM
            # else:
            #     # Fallback to frequency-domain HR calculation
            #     freqs = np.fft.rfftfreq(len(IRAC), d=1/Fs)
            #     fft_spectrum = np.abs(np.fft.rfft(IRAC))
            #     dominant_freq = freqs[np.argmax(fft_spectrum[1:]) + 1]  # Skip DC component
            #     hr = dominant_freq * 60

            # Perfusion Index (PI) calculation
            try:
                PI_red = (np.std(redAC) / max(np.mean(redDC), 1e-6)) * 100
                PI_IR = (np.std(IRAC) / max(np.mean(IRDC), 1e-6)) * 100
                PI_combined = (PI_red + PI_IR) / 2
                PI_combined_data[:-1] = PI_combined_data[1:]
                PI_combined_data[-1] = PI_combined
            except ZeroDivisionError:
                PI_combined = 0

            # Heart Rate (Pulse) calculation
            try:
                recent_IRAC = IR2AC[-Fs * 5:]  # Last 5 seconds
                peaks, _ = find_peaks(recent_IRAC, height=0.02, distance=round(Fs * 0.6))
                if len(peaks) > 1:
                    timeDiff = np.diff(peaks) / Fs
                    hr = 60 / np.mean(timeDiff)
                else:
                    # freqs = np.fft.rfftfreq(len(recent_IRAC), d=1/Fs)
                    # fft_spectrum = np.abs(np.fft.rfft(recent_IRAC))
                    # dominant_freq = freqs[np.argmax(fft_spectrum[1:]) + 1]  # Skip DC component
                    # print(dominant_freq)
                    # hr = dominant_freq * 60
                    freqs = np.fft.rfftfreq(len(IRAC), 1 / Fs)
                    fft_spectrum = np.abs(np.fft.rfft(IRAC))
                    valid_idx = (freqs >= 0.5) & (freqs <= 3.5)  # Restrict to HR-relevant frequencies
                    if np.any(valid_idx):
                        dominant_freq = freqs[valid_idx][np.argmax(fft_spectrum[valid_idx])]
                    else:
                        dominant_freq = 0  # No valid frequency found
                    hr = dominant_freq * 60  # Convert frequency (Hz) to BPM

                self.pulse[:-1] = self.pulse[1:]
                self.pulse[-1] = hr
            except Exception as e:
                print(f"HR calculation error: {e}")
                hr = 0

            # Update pulse buffer
            self.pulse[:-1] = self.pulse[1:]  # Shift left
            self.pulse[-1] = hr  # Add the latest HR value


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
                self.update_counter += 1
                if self.update_counter % 2 == 0:  # Update only every 5th tick
                    # Update labels with current values
                    # self.labelSpO2.setText(f"SpO₂: {int(self.SpO2[-1])}%")
                    # self.labelHbT.setText(f"HbT: {int(self.HbT[-1]*6.45)}")
                    # self.labelPI.setText(f"PI: {int(PI_combined)}%")
                    # self.labelHR.setText(f"HR: {int(self.pulse[-1])} BPM")
                    # self._curveHbT.setData(timeVector, self.HbT[:len(timeVector)] * 6.45)
                    # self._curveSpO2.setData(timeVector, self.SpO2[:len(timeVector)])
                    # self._curvePI.setData(timeVector, PI_combined_data[:len(timeVector)])
                    # self._curvePulse.setData(timeVector, self.pulse[:len(timeVector)])
                    self._curveHbT.setData(timeVector, 6.45 * self.HbT[:len(timeVector)])
                    self._curveSpO2.setData(timeVector, self.SpO2[:len(timeVector)])
                    self._curvePI.setData(timeVector, PI_combined_data[:len(timeVector)])
                    self._curvePulse.setData(timeVector, self.pulse[:len(timeVector)])
                    self.update_counter = 0
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