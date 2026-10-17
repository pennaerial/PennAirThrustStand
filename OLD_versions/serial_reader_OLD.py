import serial
import serial.tools.list_ports
from collections import deque
import time

class SerialReader:
    def __init__(self, baudrate=9600):
        self.ser = None
        self.baudrate = baudrate
        self.is_connected = False
        self.buffer = deque(maxlen=100)  # store last 100 lines

    def connect(self, port):
        self.ser = serial.Serial(port, self.baudrate, timeout=0.1)
        time.sleep(2)  # wait for Arduino to reset
        self.is_connected = True
        self.ser.reset_input_buffer()

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.is_connected = False

    def send_command(self, cmd):
        if self.is_connected:
            self.ser.write((cmd + "\n").encode())

    def read_data(self):
        """
        Reads available serial lines and returns the latest valid CSV line as a dictionary.
        Expects Arduino to print:
        time,thrust,rpm,temperature,voltage,current,power,throttle
        """
        if not self.is_connected:
            return None

        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line == "":
                continue
            # Only accept lines that have 8 comma-separated values
            parts = line.split(",")
            if len(parts) != 8:
                continue
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
