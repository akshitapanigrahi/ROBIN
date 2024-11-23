# README: Real-Time NIRS GUI Application

This project provides a Python-based GUI application to read, process, and visualize Near-Infrared Spectroscopy (NIRS) data from an Arduino Nano BLE 33 microcontroller. It measures red and infrared light intensities using photodiodes and calculates physiological parameters such as total hemoglobin concentration (HbT), oxygen saturation (SpO2), and perfusion index (PI).

---

## Features
- **Real-time Data Plotting**: Visualize HbT, SpO2, PI, and heart rate in stacked plots.
- **Customizable Sampling**: Set the sampling frequency and plot update intervals.
- **Filter Implementation**: Real-time signal filtering using a bandpass filter.
- **Data Reset**: Reset the GUI and data buffers with a single button click.

---

## Setup Instructions

### Step 1: Set Up the Environment (`ROBIN`)
1. **Clone the Repository**:
   ```bash
   git clone <repository_url>
   cd <repository_directory>
   ```

2. **Create the Python Environment**:
   ```bash
   python -m venv ROBIN
   source ROBIN/bin/activate  # On Windows: ROBIN\Scripts\activate
   ```

3. **Install Dependencies**:
   Install all required libraries from the `requirements.txt` file:
   ```bash
   pip install -r requirements.txt
   ```

   If `requirements.txt` is not available, install these libraries manually:
   ```bash
   pip install pyqt5 pyqtgraph qasync numpy scipy
   ```

4. **Install Arduino IDE**:
   Download and install the [Arduino IDE](https://www.arduino.cc/en/software).

---

### Step 2: Flash the Arduino Nano BLE 33

1. **Connect the Nano BLE 33**:
   Plug the Arduino Nano BLE 33 into your computer via USB.

2. **Open Arduino IDE**:
   Launch the Arduino IDE and install any necessary board support files (Arduino Nano 33 BLE).

3. **Load the Code**:
   Copy the following Arduino code into the IDE:
   ```cpp
   const int redPhotodiodePin = A0;
   const int irPhotodiodePin1 = A3;

   int rawRed = 0;
   int rawIR1 = 0;

   float redVoltage = 0;
   float irVoltage1 = 0;

   void setup() {
       Serial.begin(9600);
   }

   void loop() {
       rawRed = analogRead(redPhotodiodePin);
       rawIR1 = analogRead(irPhotodiodePin1);

       redVoltage = (rawRed / 1023.0) * 3.3;
       irVoltage1 = (rawIR1 / 1023.0) * 3.3;

       Serial.print(redVoltage);
       Serial.print(",");
       Serial.println(irVoltage1);

       delay(10); // 200 Hz sampling rate
   }
   ```

4. **Select the Board and Port**:
   - Go to `Tools > Board` and select **Arduino Nano 33 BLE**.
   - Go to `Tools > Port` and select the appropriate COM port.

5. **Upload the Code**:
   Click the **Upload** button to flash the code onto the Arduino.

---

### Step 3: Run the Python Application

1. **Connect the Arduino**:
   Ensure the Arduino is still connected and running the uploaded code.

2. **Check COM Port**:
   Confirm the COM port used by the Arduino in your system (e.g., `COM11` on Windows, `/dev/ttyUSB0` on Linux). Update the `serial.Serial` line in the Python script if necessary:
   ```python
   arduino = serial.Serial("COM11", 9600)
   ```

3. **Launch the Application**:
   Run the Python script:
   ```bash
   python nirs_streaming.py
   ```

4. **Interact with the GUI**:
   - Click **Start** to begin data streaming.
   - Click **Stop** to pause the data stream.
   - Use **Reset** to clear plots and buffers.

---

### Troubleshooting
- **Serial Port Issues**: If the COM port is not recognized, check the Arduino connection and ensure the correct drivers are installed.
- **Dependency Errors**: Reinstall missing Python libraries using `pip`.
- **Plotting Issues**: Ensure the Arduino is sending data properly by monitoring it in the Arduino Serial Monitor.

---

### Dependencies
- Python 3.8+
- PyQt5
- PyQtGraph
- qasync
- NumPy
- SciPy

---

This application facilitates seamless interaction between hardware and software, offering real-time physiological data visualization for NIRS studies.
