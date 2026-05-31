# PennAiR Custom Thrust Stand with DAQ

A complete thrust testing system with real-time GUI visualization for measuring load, motor RPM, voltage, current, and temperature during motor testing.

## Features

- **Live Data Visualization**: Real-time graphs of thrust, RPM, temperature, voltage, current, and power
- **Flexible Display**: Toggle individual graphs on/off with checkboxes
- **Hover Readout**: Crosshair + marker with x,y values that snap to nearest data point
- **History Tab**: Browse past CSVs, view plots and raw data table, refresh to rescan files
- **Metadata in CSV**: Enter Motor and Propeller; values are included at top of CSV export
- **Data Export**: Save test results to CSV for analysis
- **Professional GUI**: Interface built with PyQt5
- **Comprehensive Sensing**: Thrust, RPM, temperature, voltage, current (with computed power)

## Hardware
* HX711 Load Cell Amplifier + Load Cell
* DHT11 Temperature Sensor
* ACS712 30A Current Sensor (We will proably switch this out for a sensor that can take higher current)
* Hall Effect Sensor
* 0–25V Voltage Sensor
* Arduino ESP32

## Wiring Guide

### HX711 Load Cell Amplifier
* VCC -> VCC
* GND -> GND
* DT -> D0
* SCK -> D1

Load Cell Wires:
* Red -> E+
* Black -> E-
* White -> A-
* Green -> A+

### DHT11 Temperature Sensor
* VCC -> 5V
* DATA -> D4
* GND -> GND

### Hall Effect RPM Sensor
* VCC -> 5V
* GND -> GND
* Signal -> D5

### ACS712 Current Sensor
* VCC -> 5V
* GND -> GND
* OUT -> A0

Current Measurement Wiring:
* Power Supply (+) -> Motor (+)
* Power Supply (–) -> ACS712 -> Motor (–)

### Voltage Sensor (0–25V DC)
* VCC -> 5V
* GND -> GND
* OUT -> A2

Voltage Measurement Wiring :
* VIN+ -> Voltage Source Positive
* VIN- -> Voltage Source Ground

---

## Software Setup

### Step 1: Arduino Setup

1. **Install Arduino IDE** (if not already installed)
2. **Install Required Libraries**:
   - HX711_ADC
   - DHT sensor library
   
   Go to: `Sketch` → `Include Library` → `Manage Libraries` and search for each

3. **Upload Arduino Code**:
   - Open `ThrustTestingAllSensors` in Arduino IDE
   - Select Board: `ESP32 Dev Module`
   - Select the correct COM port
   - Click Upload

4. **Calibrate Sensors** (via Arduino Serial Monitor at 9600 baud):
   - Enter the number of magnets in your motor
   - Press `t` to tare the load cell (with no weight)
   - Press `z` to zero the current sensor (with no current flowing)
   - **Important**: Close Serial Monitor when done!

### Step 2: Python GUI Setup

1. **Navigate to Project Directory**:
   ```powershell
   cd \copy\to\your\path
   ```

2. **Install Python Dependencies**:
   ```powershell
   pip install -r requirements.txt
   ```
   
   This installs:
   - PyQt5 (GUI framework)
   - pyqtgraph (real-time plotting)
   - pyserial (serial communication)
   - numpy (data processing)

3. **Run the GUI**:
   ```powershell
   python thrust_gui.py
   ```

### Step 3: Running a Test (Live Tab)

1. **Connect to Arduino**:
   - Select your COM port from the dropdown (e.g., COM10)
   - Click "Refresh Ports" if you don't see your device
   - Click "Connect"
   - Status should show "Connected" in green

2. **Start Data Collection**:
   - Click "Start Test"
   - All graphs will begin updating in real-time
  - Use checkboxes to show/hide specific measurements:
    - ☑ Thrust (g)
    - ☑ RPM
    - ☑ Temperature (°C)
    - ☑ Voltage (V)
    - ☑ Current (A)
    - ☑ Power (W)
  - Hover over a graph to see a crosshair and dot marker; the label shows the nearest data point’s x and y

3. **Stop and Export**:
   - Click "Stop Test" when finished
  - Optional: Click "Auto-Scale All" (larger "A" button) to fit axes to the full run
  - Click "Export to CSV" to save your data
   - Choose a filename and location
   - Data is saved with timestamp and ready for analysis

### Step 4: Viewing Past Tests (History Tab)

1. Open the "History" tab
2. Click "Refresh" to rescan for `thrust_test_*.csv` files in the project directory
3. Select a CSV to view
   - Plots for thrust, RPM, temperature, voltage, current, and computed power
   - Raw data table with all rows
4. Hover over plots to see snapped x,y readouts with markers

## Troubleshooting

### "Access is denied" / "PermissionError"
- **Cause**: Another program is using the serial port
- **Solution**: 
  - Close Arduino IDE Serial Monitor
  - Close any other serial terminal programs
  - Unplug and replug the Arduino USB cable
  - Click "Refresh Ports" in the GUI

### No Data Appearing
- Verify Arduino is running (LED should blink)
- Check sensor connections
- Test with Arduino Serial Monitor first to verify data output
- Ensure baud rate is 9600

### Graphs Not Updating
- Make sure you clicked "Start Test"
- Check that checkboxes are enabled for desired graphs
- Verify Arduino is sending data continuously

## Project Files

### Arduino Code
- `ThrustTestingAllSensors` - Main sensor reading code (most current)
- `ThrustSensorsCurrentVoltage` - Alternative version with button control

### Python GUI
- `thrust_gui.py` - Main GUI application
- `serial_reader.py` - Serial communication handler
- `requirements.txt` - Python dependencies
- `README_GUI.md` - Detailed GUI documentation

## CSV Export Format

Each CSV includes optional metadata rows, a blank spacer row, then the data header and rows:
```csv
Motor,TMotor 2820
Propeller,APC 10x4.7
Exported,2025-11-01 17:45:12

Time (s),Thrust (g),RPM,Temperature (°C),Voltage (V),Current (A),Power (W)
0.000,0.000,0.0,25.00,12.000,0.000,0.000
0.100,12.345,5500.0,25.10,12.050,2.456,29.622
...
```

Notes:
- Power (W) is included as a CSV column (Voltage × Current per sample).
- Hover readouts in the app snap to the nearest plotted data point for precise values.

## Serial Commands (Arduino)

When connected via Serial Monitor:
- `t` - Tare load cell
- `r` - Recalibrate load cell
- `c` - Change calibration factor
- `z` - Zero current sensor
