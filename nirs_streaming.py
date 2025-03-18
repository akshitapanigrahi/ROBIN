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

# Reference voltages and constants, (L . mmor’ . cm’)
refVoltageRed = 1.75 # 740 nm
refVoltageIR = 1.75 # 880 nm
refVoltageIR2 = 1.75  # 940 nm
refVoltageIR3 = 1.75 # 850 nm? --> approx 845 nm for coefficients

# Matrix of extinction coefficients, [HHb, HbO2]
epsilonMatrix = np.array([
    [0.40, 0.13],
    [0.19, 0.27],
    [0.17, 0.28],
    [0.18, 0.24]
])

# DPF values, based on tissue and wavelength, need to double check these values
DPF_740 = 6.0
DPF_880 = 6.5
DPF_940 = 6.0
DPF_850 = 6.0

# Source-detector separation (distance between LED and photodiode), cm --> does this change depending on wavelength? seems like it should be smaller...?
SDS = 6.0

# Calculate path lengths
pathLengths = np.array([DPF_740 * SDS, DPF_880 * SDS, DPF_940 * SDS, DPF_850 * SDS])

# Bandpass and low pass filter setup
FcLow = 0.5  # Hz
FcHigh = 2.5  # Hz
b, a = butter(2, [FcLow / (Fs / 2), FcHigh / (Fs / 2)], btype='band') # bandpass
b_low, a_low = butter(2, FcLow / (Fs / 2), btype='low') # lowpass
PI_b, PI_a = butter(2, 0.1 / (Fs / 2), btype='low') #lowpass for PI

# === Serial Communication Setup ===
arduino = serial.Serial("/dev/cu.usbmodem1201", 9600)
time.sleep(2)

# === GUI File ===
qtDesignerFile = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robin_simple_gui.ui")

class MyApp(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(qtDesignerFile, self)

        # Plot setup (Stacked Vertically)
        self.plot_layout = self.graphicsView.addLayout()

        self.update_counter = 0

        self.PI_rolling_window = np.zeros(50)  # Store the last 50 PI values
        self.PI_current_value = 0  # Explicitly track the current PI value

        # Create HbT plot
        self.plotHbT = self.plot_layout.addPlot(title="NIRS HbT Readings")
        self._curveHbT = self.plotHbT.plot(pen='k', width=10)
        self.plotHbT.setYRange(0, 60)  
        self.plotHbT.setLabel('left', 'HbT', units='g/dL')  
        self.plot_layout.nextRow()

        # Create SpO2 plot
        self.plotSpO2 = self.plot_layout.addPlot(title="NIRS SpO2 Readings")
        self._curveSpO2 = self.plotSpO2.plot(pen='k', width=10)
        self.plotSpO2.setYRange(40, 100)  
        self.plotSpO2.setLabel('left', 'SpO₂', units='%')  
        self.plot_layout.nextRow()

        # Create PI plot
        self.plotPI = self.plot_layout.addPlot(title="Perfusion Index (PI)")
        self._curvePI = self.plotPI.plot(pen='k', width=10)
        self.plotPI.setYRange(0, 6)  
        self.plotPI.setLabel('left', 'PI', units='%')  
        self.plot_layout.nextRow()

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
            voltageData[3, :-1] = voltageData[3, 1:]
            voltageData[0, -1] = voltage[0]  # Red voltage
            voltageData[1, -1] = voltage[1]  # IR voltage
            voltageData[2, -1] = voltage[2]  # IR2 voltage
            voltageData[3, -1] = voltage[2]  # IR3 voltage

            # Extract red and IR signals
            red = voltageData[0, :]
            IR = voltageData[1, :]
            IR2 = voltageData[2, :]
            IR3 = voltageData[3, :]

            # Clamp negative values
            red[red <= 0] = 1e-6
            IR[IR <= 0] = 1e-6
            IR2[IR2 <= 0] = 1e-6
            IR3[IR3 <= 0] = 1e-6

            print("running")

            # Print statement for debugging
            #print("Red Voltage:", voltage[0], "IR Voltage1:", voltage[1], "IR Voltage2:", voltage[2], , "IR Voltage3:", voltage[3])

            # Filter voltage signals to extract AC and DC components
            redDC = filtfilt(b_low, a_low, red)
            IRDC = filtfilt(b_low, a_low, IR)
            IR2DC = filtfilt(b_low, a_low, IR2)
            IR3DC = filtfilt(b_low, a_low, IR3)

            redAC = filtfilt(b, a, red)
            IRAC = filtfilt(b, a, IR)
            IR2AC = filtfilt(b, a, IR2)
            IR3AC = filtfilt(b, a, IR3)

            # Clamp negative DC values to avoid dividing by 0 
            redDC[redDC <= 0] = 1e-6
            IRDC[IRDC <= 0] = 1e-6
            IR2DC[IR2DC <= 0] = 1e-6
            IR3DC[IR3DC <= 0] = 1e-6

            # Calculate raw absorbances using DC components
            redAbsorbanceRaw = np.log10(refVoltageRed / red)
            irAbsorbanceRaw = np.log10(refVoltageIR / IR)
            ir2AbsorbanceRaw = np.log10(refVoltageIR2 / IR2)
            ir3AbsorbanceRaw = np.log10(refVoltageIR3 / IR3)

            # Calculate filtered absorbances using DC components
            redAbsorbance = np.log10(refVoltageRed / redDC)
            irAbsorbance = np.log10(refVoltageIR / IRDC)
            ir2Absorbance = np.log10(refVoltageIR2 / IR2DC)
            ir3Absorbance = np.log10(refVoltageIR3 / IR3DC)

            # Calculate raw normalized absorbances
            absorbanceVectorRaw = np.array([redAbsorbanceRaw, irAbsorbanceRaw, ir2AbsorbanceRaw, ir3AbsorbanceRaw])  
            absorbanceVectorRaw = absorbanceVectorRaw / pathLengths[:, np.newaxis]

            # Calculate filtered normalized absorbances
            absorbanceVector = np.array([redAbsorbance, irAbsorbance, ir2Absorbance, ir3Absorbance])  
            absorbanceVector = absorbanceVector / pathLengths[:, np.newaxis]
            
            # Pseudo-inverse of epsilonMatrix
            epsilonPseudoInverse = np.linalg.pinv(epsilonMatrix)

            # Calculate raw concentrations, C = [HHb, HbO2]
            concentrationVectorRaw = epsilonPseudoInverse @ absorbanceVectorRaw  # [HHb, HbO2]
            concentrationVectorRaw *= 1e3  # Scale HHb and HbO2 to µM
            HHbRaw = concentrationVectorRaw[0]
            HbO2Raw = concentrationVectorRaw[1]

            # Calculate filtered concentrations, C = [HHb, HbO2]
            concentrationVector = epsilonPseudoInverse @ absorbanceVector  # [HHb, HbO2]
            concentrationVector *= 1e3  # Scale HHb and HbO2 to µM --> mmol/L to uM 
            HHb = concentrationVector[0]
            HbO2 = concentrationVector[1]

            # Total Hemoglobin (HbT)
            self.HbT = (HHb + HbO2) 
            #self.HbT[self.HbT == 0] = 1e-6

            # Oxygen Saturation (SpO2)
            #self.SpO2 = (HbO2 / self.HbT) * 100 
            self.SpO2 = np.minimum(1.0 * (HbO2 / self.HbT) * 100, 102)  # Cap SpO₂ at 100%

            # Chromophore-based perfusion Index (PI) calculation, more complex but more physiologically accurate

            HHb_AC = filtfilt(b, a, HHbRaw)
            HHb_DC = filtfilt(PI_b, PI_a, HHbRaw)

            HbO2_AC = filtfilt(b, a, HbO2Raw)
            HbO2_DC = filtfilt(PI_b, PI_a, HbO2Raw)

            HHb_DC = np.maximum(filtfilt(PI_b, PI_a, red), 1e-6)
            HbO2_DC = np.maximum(filtfilt(PI_b, PI_a, IR), 1e-6)

            factor_AC = 2.5 # scaling factor 1.5

            HHb_AC *= factor_AC  
            HbO2_AC *= factor_AC

            factor_DC = 0.6 # scaling factor < 1

            HHb_DC *= factor_DC  
            HbO2_DC *= factor_DC

            # Take AC/DC ratio, %
            PI_HHb = (np.std(HHb_AC) / np.mean(HHb_DC)) * 100 * 5
            PI_HbO2 = (np.std(HbO2_AC) / np.mean(HbO2_DC)) * 100  # should theoretically reflect PI better, shows more pulsatile variation

            # Smooth transition from 0 using a simple moving average
            alpha = 0.1  # Smoothing factor (adjust for smoother transition) --> increase for faster response

            # Average PIs
            #PI_combined_matrix = (PI_HHb + PI_HbO2) / 2 
            #PI_combined_matrix = PI_HbO2 # relying on HbO2 alone
            PI_combined_matrix = (0.8 * PI_HbO2 + 0.2 * PI_HHb) # weighted method

            self.PI_current_value = (1 - alpha) * self.PI_current_value + alpha * PI_combined_matrix

            # Debugging, HbO2_AC should decrease to almost 0 when tourniquet applied and should recover after release
            print(f"HbO2_AC std: {np.std(HbO2_AC)}, HbO2_DC mean: {np.mean(HbO2_DC)}")
            #print(f"HbO2: {HbO2}, HHb: {HHb}, HbT: {self.HbT}, SpO₂: {self.SpO2}")

            if(np.std(HbO2_AC) > 0.2):
                self.PI_current_value = max(self.PI_current_value - 1, 0)  # Gradually reduce, but not below 0
            
            # Apply moving average to stabilize data when displaying
            def moving_average(data, window_size):
                return np.convolve(data, np.ones(window_size)/window_size, mode='valid')
            
            def rolling_mean(data, window_size):
                return np.convolve(data, np.ones(window_size) / window_size, mode='valid')

            # Update rolling window
            self.PI_rolling_window = np.roll(self.PI_rolling_window, -1)
            self.PI_rolling_window[-1] = self.PI_current_value  # Update with smoothed PI value

            # Compute the rolling average
            PI_average = np.mean(self.PI_rolling_window)

            # Print PI to console (even when GUI is stopped)
            print(f"Current PI: {0.1 * PI_average:.2f}%")

            # Update plots
            try:
                self.update_counter += 1
                if self.update_counter % 2 == 0:  # Update only every 2nd tick      
                    self._curveHbT.setData(timeVector, 2.45 * np.abs(self.HbT[:len(timeVector)])) # HbT with absolute value safeguard, g/dl, changed scaling from 10 to 6.45
                    self._curveSpO2.setData(timeVector, self.SpO2[:len(timeVector)]) # SpO2 values, %
                    #self._curvePI.setData(timeVector, np.abs(PI_smoothed_padded) * 1.0) # smoothed PI values, %
                    self._curvePI.setData(timeVector, 0.1 * np.full(len(timeVector), PI_average))

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