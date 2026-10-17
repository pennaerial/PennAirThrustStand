# Thrust Stand GUI

A real-time GUI application for monitoring and visualizing thrust test data from your Arduino-based thrust stand.

## Features

- **Live plotting** of 5 measurements over time:
  - Thrust (grams)
  - RPM
  - Temperature (°C)
  - Voltage (V)
  - Current (A)

- **Checkbox controls** to show/hide individual graphs
- **Serial port selection** with auto-detection
- **Real-time updates** (100ms refresh rate)
- **Data buffering** (keeps last 500 data points)
- Clean, professional interface using PyQt5

## Installation

### 1. Install Python Requirements

```powershell
pip install -r requirements.txt
```

This will install:
- PyQt5 (GUI framework)
- pyqtgraph (high-performance plotting)
- pyserial (serial communication)
- numpy (numerical operations)

### 2. Upload Arduino Code

Make sure you have uploaded the `ThrustTestingAllSensors` or `ThrustSensorsCurrentVoltage` sketch to your Arduino ESP32.

## Usage

### 1. Run the Application

```powershell
python thrust_gui.py
```

### 2. Connect to Arduino

1. **Select Serial Port**: Choose your Arduino's COM port from the dropdown
   - If you don't see your port, click "Refresh Ports"
   
2. **Click "Connect"**: Establishes connection with the Arduino
   - Status will show "Connected" in green

3. **Click "Start Test"**: Begins data acquisition and live plotting
   - All graphs will start updating in real-time

### 3. Control Graphs

Use the checkboxes on the control panel to show/hide specific graphs:
- ☑ Thrust (g)
- ☑ RPM
- ☑ Temperature (°C)
- ☑ Voltage (V)
- ☑ Current (A)

Uncheck any box to hide that graph and save screen space.

### 4. Stop Test

Click "Stop Test" to pause data acquisition while staying connected.

### 5. Disconnect

Click "Disconnect" to close the serial connection.

## GUI Layout

```
┌─────────────────────────────────────────────────────────┐
│  Controls                                               │
│  [Port Selection] [Connect] [Start] [Stop] [Checkboxes]│
├─────────────────────────────────────────────────────────┤
│  Thrust (grams) Graph                                   │
├─────────────────────────────────────────────────────────┤
│  RPM Graph                                              │
├─────────────────────────────────────────────────────────┤
│  Temperature (°C) Graph                                 │
├─────────────────────────────────────────────────────────┤
│  Voltage (V) Graph                                      │
├─────────────────────────────────────────────────────────┤
│  Current (A) Graph                                      │
└─────────────────────────────────────────────────────────┘
```

## Troubleshooting

### No Serial Ports Detected
- Make sure your Arduino is connected via USB
- Install the CH340/CP2102 driver for ESP32 (if needed)
- Click "Refresh Ports" button

### Connection Error
- Verify the correct COM port is selected
- Ensure no other application is using the serial port (e.g., Arduino IDE Serial Monitor)
- Check the baud rate matches Arduino code (default: 9600)

### No Data Appearing
- Verify Arduino is sending data (check with Arduino IDE Serial Monitor first)
- Ensure all sensors are properly connected
- Check that Arduino code is printing data in the expected format

### Graphs Not Updating
- Verify "Start Test" button was clicked
- Check that Arduino is continuously sending data
- Look for error messages in the terminal/console

## Data Format

The GUI expects Arduino serial output in this format:
```
Load cell: 123.45
Temperature: 25.6
RPM: 5500
Voltage: 12.3 V
Current: 2.456 A
```

The parser is flexible and will extract numeric values after the colons.

## Customization

### Adjust Data Buffer Size

Edit `thrust_gui.py` line 22:
```python
self.max_points = 500  # Change to desired number
```

### Change Update Rate

Edit `thrust_gui.py` line 211:
```python
self.timer.start(100)  # milliseconds (100 = 10 Hz)
```

### Modify Graph Colors

Edit `thrust_gui.py` lines 53-57 to change RGB colors:
```python
self.thrust_plot = self.create_plot("Thrust (grams)", "g", (255, 0, 0))  # Red
self.rpm_plot = self.create_plot("RPM", "RPM", (0, 255, 0))  # Green
# etc...
```

## Files

- `thrust_gui.py` - Main GUI application
- `serial_reader.py` - Serial communication module
- `requirements.txt` - Python dependencies
- `README_GUI.md` - This file

## Tips

- **Calibration**: Use Arduino serial commands (t, r, c, z) before starting GUI
- **Multiple Tests**: Click "Stop Test" and "Start Test" to clear data and begin a new test
- **Screen Layout**: Uncheck graphs you don't need to focus on important measurements
- **Performance**: Lower the update rate if experiencing lag on slower computers

## Future Enhancements

Possible improvements:
- Export data to CSV
- Save/load test configurations
- Overlaying multiple test runs
- Statistical analysis of test data
- Auto-scaling for Y-axes
- Zoom and pan controls
