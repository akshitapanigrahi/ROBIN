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

---
#include <ArduinoBLE.h>

// Pins for your photodiodes
const int redPhotodiodePin = A0;
const int irPhotodiodePin1 = A1;
const int irPhotodiodePin2 = A2;
const int irPhotodiodePin3 = A3;

// We'll create a custom service and characteristic UUID.
// You can generate your own UUIDs, or reuse these examples.
BLEService sensorService("12345678-1234-5678-1234-56789ABCDEF0");
BLECharacteristic dataCharacteristic(
    "87654321-4321-6789-4321-0FEDCBA98765", 
    BLERead | BLENotify, 
    32  // enough size to hold our comma-separated values as text
);

void setup() {
  Serial.begin(115200); // Optional for debugging over Serial Monitor.

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE module!");
    while (1);
  }
  Serial.println("BLE initialized.");

  // Set device name and advertise the custom service
  BLE.setLocalName("NIRS_BLE");
  BLE.setAdvertisedService(sensorService);

  // Add the characteristic to the service
  sensorService.addCharacteristic(dataCharacteristic);

  // Add the service to the BLE stack
  BLE.addService(sensorService);

  // Begin advertising
  BLE.advertise();
  Serial.println("BLE advertising started...");
}

void loop() {
  // Keep the BLE stack updated
  BLE.poll();

  // Read your 3 photodiodes
  int rawRed = analogRead(redPhotodiodePin);
  int rawIR1 = analogRead(irPhotodiodePin1);
  int rawIR2 = analogRead(irPhotodiodePin2);
  int rawIR3 = analogRead(irPhotodiodePin3);


  // Convert from 0-1023 ADC to 0-3.3V (Nano 33 BLE default is 0-3.3V)
  float redVoltage = (rawRed / 1023.0) * 3.3;
  float irVoltage1 = (rawIR1 / 1023.0) * 3.3;
  float irVoltage2 = (rawIR2 / 1023.0) * 3.3;
  float irVoltage3 = (rawIR3 / 1023.0) * 3.3;

  // Create a comma-separated string: "redVoltage,irVoltage1,irVoltage2"
  // Example: "1.23,2.34,3.45"
  char buf[32];
  snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f", redVoltage, irVoltage1, irVoltage2, irVoltage3);

  // Write the string to the characteristic, which will notify connected clients
  dataCharacteristic.writeValue((const unsigned char*)buf, strlen(buf));

  Serial.print(redVoltage);
  Serial.print(",");
  Serial.print(irVoltage1);
  Serial.print(",");
  Serial.print(irVoltage2);
  Serial.print(",");
  Serial.println(irVoltage3);

  // ~5 ms delay -> ~200 Hz
  delay(5);
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
