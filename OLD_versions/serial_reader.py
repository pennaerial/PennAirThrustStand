import serial
import serial.tools.list_ports
from collections import deque
import time
from datetime import datetime

class SerialReader:
    def __init__(self, baudrate=9600):
        self.ser = None
        self.baudrate = baudrate
        self.is_connected = False
        self.buffer = deque(maxlen=100)  # store last 100 lines
        self.header_printed = False

    def connect(self, port):
        self.ser = serial.Serial(port, self.baudrate, timeout=0.1)
        time.sleep(2)  # wait for Arduino to reset
        self.is_connected = True
        self.ser.reset_input_buffer()
        self.header_printed = False  # Reset header flag on new connection

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.is_connected = False
        self.header_printed = False

    def send_command(self, cmd):
        """Send a command to the Arduino."""
        if self.is_connected and self.ser:
            self.ser.write((cmd + "\n").encode())
            self.ser.flush()

    def read_data(self):
        """
        Reads available serial lines and returns the latest valid CSV line as a dictionary.
        Handles multiple formats:
        - Old format: time,thrust,rpm,temperature,voltage,current,power (7 columns)
        - New format: time,thrust,rpm,temperature,voltage,current,power,throttle (8 columns)
        - Header line: time,thrust,rpm,temperature,voltage,current,power,throttle (skipped)
        """
        if not self.is_connected:
            return None

        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line == "":
                continue
            
            # Skip header lines
            if line.lower().startswith('time') or line.lower().startswith('timestamp'):
                self.header_printed = True
                continue
            
            parts = line.split(",")
            
            # Handle 7-column format (old, no throttle)
            if len(parts) == 7:
                try:
                    data = {
                        'time': float(parts[0]),
                        'thrust': float(parts[1]),
                        'rpm': float(parts[2]),
                        'temperature': float(parts[3]),
                        'voltage': float(parts[4]),
                        'current': float(parts[5]),
                        'power': float(parts[6]),
                        'throttle': 0.0  # Default throttle
                    }
                    # Generate timestamp if not present
                    data['timestamp'] = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    self.buffer.append(data)
                except ValueError:
                    continue
            
            # Handle 8-column format (new, with throttle)
            elif len(parts) == 8:
                try:
                    data = {
                        'time': float(parts[0]),
                        'thrust': float(parts[1]),
                        'rpm': float(parts[2]),
                        'temperature': float(parts[3]),
                        'voltage': float(parts[4]),
                        'current': float(parts[5]),
                        'power': float(parts[6]),
                        'throttle': float(parts[7])
                    }
                    # Generate timestamp if not present
                    data['timestamp'] = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    self.buffer.append(data)
                except ValueError:
                    continue

        if self.buffer:
            return self.buffer[-1]  # return most recent valid line
        return None

    @staticmethod
    def list_ports():
        """List available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

