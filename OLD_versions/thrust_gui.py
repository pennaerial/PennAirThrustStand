# thrust_gui.py
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QCheckBox, QPushButton, QComboBox, 
                             QLabel, QGroupBox, QMessageBox, QFileDialog, QLineEdit, 
                             QTabWidget, QListWidget, QTableWidget, QTableWidgetItem)
from PyQt5.QtGui import QPalette, QColor
import os
import glob
from PyQt5.QtCore import QTimer, QPointF
import pyqtgraph as pg
from collections import deque
import numpy as np
import csv
from datetime import datetime
from serial_reader import SerialReader
try:
    import qdarktheme  # optional modern theming
except Exception:
    qdarktheme = None


class ThrustStandGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Thrust Stand Monitor")
        self.setGeometry(100, 100, 1600, 900)
        
        # Serial reader
        self.serial_reader = SerialReader()
        
        # Data storage (keep all points; unbounded growth)
        self.max_points = None
        self.time_data = deque()             # elapsed seconds (numeric) for plotting
        self.thrust_data = deque()
        self.rpm_data = deque()
        self.temperature_data = deque()
        self.voltage_data = deque()
        self.current_data = deque()
        self.power_data = deque()
        self.throttle_data = deque()
        self.timestamp_data = deque()        # human-readable timestamps (HH:MM:SS.sss)
        # Full history arrays (not truncated)
        self.time_history = []
        self.thrust_history = []
        self.rpm_history = []
        self.temperature_history = []
        self.voltage_history = []
        self.current_history = []
        self.power_history = []
        self.throttle_history = []
        self.timestamp_history = []
        self.start_time = 0
        # Live plot decimation (display-only): seconds per displayed point
        self.display_step_s = 0.5
        # X-axis domain: True = throttle, False = time
        self.use_throttle_domain = False
        
        # Timer for updating plots
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        
        # Setup UI
        self.init_ui()
        # Theme state
        self.is_dark_mode = False
        
    def init_ui(self):
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout()
        main_widget.setLayout(main_layout)

        # Tabs
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # Live tab content
        live_tab = QWidget()
        live_layout = QVBoxLayout()
        live_tab.setLayout(live_layout)

        # Control panel
        control_panel = self.create_control_panel()
        live_layout.addWidget(control_panel)

        # Create horizontal layout for plots and live data
        content_layout = QHBoxLayout()
        
        # Plot area (left side) - use grid for square graphs
        plot_layout = QVBoxLayout()
        
        # Row 1: Thrust and RPM
        row1_layout = QHBoxLayout()
        # Row 2: Temperature and Voltage
        row2_layout = QHBoxLayout()
        # Row 3: Current and Power
        row3_layout = QHBoxLayout()
        # Row 4: Throttle (always vs time)
        row4_layout = QHBoxLayout()
        
        # Create individual plots with square aspect
        self.thrust_plot = self.create_plot("Thrust (grams)", "g", (255, 0, 0))
        self.rpm_plot = self.create_plot("RPM", "RPM", (0, 255, 0))
        self.temp_plot = self.create_plot("Temperature (°C)", "°C", (0, 0, 255))
        self.voltage_plot = self.create_plot("Voltage (V)", "V", (255, 165, 0))
        self.current_plot = self.create_plot("Current (A)", "A", (255, 0, 255))
        self.power_plot = self.create_plot("Power (W)", "W", (0, 0, 0))
        self.throttle_plot = self.create_plot("Throttle (%)", "%", (100, 150, 200))
        
        # Arrange plots in grid (2x3 + throttle)
        row1_layout.addWidget(self.thrust_plot['widget'])
        row1_layout.addWidget(self.rpm_plot['widget'])
        
        row2_layout.addWidget(self.temp_plot['widget'])
        row2_layout.addWidget(self.voltage_plot['widget'])
        
        row3_layout.addWidget(self.current_plot['widget'])
        row3_layout.addWidget(self.power_plot['widget'])
        
        # Throttle plot spans full width
        row4_layout.addWidget(self.throttle_plot['widget'])
        
        plot_layout.addLayout(row1_layout)
        plot_layout.addLayout(row2_layout)
        plot_layout.addLayout(row3_layout)
        plot_layout.addLayout(row4_layout)
        
        content_layout.addLayout(plot_layout, 3)  # 75% width
        
        # Live data display panel (right side)
        live_data_panel = self.create_live_data_panel()
        content_layout.addWidget(live_data_panel, 1)  # 25% width
        
        live_layout.addLayout(content_layout)

        self.tabs.addTab(live_tab, "Live")

        # History tab
        history_tab = self.create_history_tab()
        self.tabs.addTab(history_tab, "History")
        
        # Store plot references
        self.plots = {
            'thrust': self.thrust_plot,
            'rpm': self.rpm_plot,
            'temperature': self.temp_plot,
            'voltage': self.voltage_plot,
            'current': self.current_plot,
            'power': self.power_plot,
            'throttle': self.throttle_plot
        }
        
    def create_control_panel(self):
        """Create the control panel with port selection and checkboxes."""
        control_group = QGroupBox("Controls")
        control_layout = QHBoxLayout()
        
        # Port selection
        port_layout = QVBoxLayout()
        port_label = QLabel("Serial Port:")
        self.port_combo = QComboBox()
        self.refresh_ports()
        port_layout.addWidget(port_label)
        port_layout.addWidget(self.port_combo)
        
        refresh_btn = QPushButton("Refresh Ports")
        refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(refresh_btn)
        
        control_layout.addLayout(port_layout)
        
        # Connect/Disconnect buttons
        button_layout = QVBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        button_layout.addWidget(self.connect_btn)
        
        self.start_btn = QPushButton("Start Test")
        self.start_btn.clicked.connect(self.start_test)
        self.start_btn.setEnabled(False)
        button_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("Stop Test")
        self.stop_btn.clicked.connect(self.stop_test)
        self.stop_btn.setEnabled(False)
        button_layout.addWidget(self.stop_btn)
        
        self.stop_motor_btn = QPushButton("Stop Motor")
        self.stop_motor_btn.clicked.connect(self.stop_motor)
        self.stop_motor_btn.setEnabled(False)
        button_layout.addWidget(self.stop_motor_btn)
        
        self.procedure_btn = QPushButton("Run Auto Procedure")
        self.procedure_btn.clicked.connect(self.run_procedure)
        self.procedure_btn.setEnabled(False)
        button_layout.addWidget(self.procedure_btn)
        
        self.export_btn = QPushButton("Export to CSV")
        self.export_btn.clicked.connect(self.export_to_csv)
        self.export_btn.setEnabled(False)
        button_layout.addWidget(self.export_btn)
        
        self.autoscale_btn = QPushButton("Auto-Scale All")
        self.autoscale_btn.clicked.connect(self.autoscale_plots)
        # Match default sizing of other buttons (no custom size/style)
        button_layout.addWidget(self.autoscale_btn)

        # Theme toggle
        self.theme_toggle = QCheckBox("Dark Mode")
        self.theme_toggle.stateChanged.connect(self.toggle_theme)
        button_layout.addWidget(self.theme_toggle)
        
        # X-axis domain toggle
        self.domain_toggle = QCheckBox("X-Axis: Throttle")
        self.domain_toggle.stateChanged.connect(self.toggle_domain)
        button_layout.addWidget(self.domain_toggle)
        
        control_layout.addLayout(button_layout)

        # Test metadata inputs
        meta_layout = QVBoxLayout()
        meta_label = QLabel("Test Metadata:")
        meta_layout.addWidget(meta_label)
        self.motor_name_input = QLineEdit()
        self.motor_name_input.setPlaceholderText("Motor name/model")
        meta_layout.addWidget(QLabel("Motor:"))
        meta_layout.addWidget(self.motor_name_input)
        self.prop_type_input = QLineEdit()
        self.prop_type_input.setPlaceholderText("Propeller type/size")
        meta_layout.addWidget(QLabel("Propeller:"))
        meta_layout.addWidget(self.prop_type_input)
        control_layout.addLayout(meta_layout)
        
        # Checkboxes for plot visibility
        checkbox_layout = QVBoxLayout()
        checkbox_label = QLabel("Display Graphs:")
        checkbox_layout.addWidget(checkbox_label)
        
        self.checkboxes = {}
        measurements = ['thrust', 'rpm', 'temperature', 'voltage', 'current', 'power', 'throttle']
        labels = ['Thrust (g)', 'RPM', 'Temperature (°C)', 'Voltage (V)', 'Current (A)', 'Power (W)', 'Throttle (%)']
        
        for measure, label in zip(measurements, labels):
            cb = QCheckBox(label)
            cb.setChecked(True)
            cb.stateChanged.connect(lambda state, m=measure: self.toggle_plot(m, state))
            self.checkboxes[measure] = cb
            checkbox_layout.addWidget(cb)
        
        control_layout.addLayout(checkbox_layout)
        
        # Status label
        status_layout = QVBoxLayout()
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("font-weight: bold; font-size: 12pt;")
        status_layout.addWidget(self.status_label)
        control_layout.addLayout(status_layout)
        
        control_group.setLayout(control_layout)
        return control_group

    def apply_light_palette(self):
        app = QApplication.instance()
        app.setStyle("Fusion")
        palette = QPalette()
        app.setPalette(palette)
        app.setStyleSheet("")

    def apply_dark_palette(self):
        app = QApplication.instance()
        app.setStyle("Fusion")
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
        palette.setColor(QPalette.Base, QColor(35, 35, 35))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, QColor(220, 220, 220))
        palette.setColor(QPalette.ToolTipText, QColor(220, 220, 220))
        palette.setColor(QPalette.Text, QColor(220, 220, 220))
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
        palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
        palette.setColor(QPalette.Highlight, QColor(142, 45, 197).lighter())
        palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
        app.setPalette(palette)
        app.setStyleSheet("")

    def toggle_theme(self, state):
        self.is_dark_mode = state == 2
        app = QApplication.instance()
        if self.is_dark_mode:
            if qdarktheme is not None:
                app.setStyleSheet(qdarktheme.load_stylesheet("dark"))
            else:
                self.apply_dark_palette()
        else:
            if qdarktheme is not None:
                app.setStyleSheet(qdarktheme.load_stylesheet("light"))
            else:
                self.apply_light_palette()
        # Update plot widgets to dark/light backgrounds while keeping crisp text
        def _apply_plot_theme(plot_dict):
            is_dark = self.is_dark_mode
            bg = (20, 20, 20) if is_dark else 'w'
            # Use explicit RGB tuples for text colors to ensure they override stylesheet
            fg = (255, 255, 255) if is_dark else (0, 0, 0)
            w = plot_dict['widget']
            w.setBackground(bg)
            # Update title and axis labels using stored values with explicit colors
            title_text = plot_dict.get('title', '')
            y_label_text = plot_dict.get('y_label', '')
            w.setTitle(title_text, color=fg, size='12pt')
            w.setLabel('left', y_label_text, color=fg)
            w.setLabel('bottom', 'Time (s)', color=fg)
            # Force update of axis labels
            w.getAxis('left').setLabel(y_label_text, color=fg)
            # X-axis label depends on domain mode
            x_label = 'Throttle (%)' if self.is_dark_mode and getattr(self, 'use_throttle_domain', False) else 'Time (s)'
            if hasattr(self, 'use_throttle_domain') and self.use_throttle_domain:
                x_label = 'Throttle (%)'
            else:
                x_label = 'Time (s)'
            w.getAxis('bottom').setLabel(x_label, color=fg)
            # Update hover label color to match theme
            if 'label' in plot_dict:
                label_color = (255, 255, 255) if is_dark else (0, 0, 0)
                plot_dict['label'].setColor(label_color)
            # Force repaint
            w.update()
        for p in self.plots.values():
            _apply_plot_theme(p)
        # History plots too
        for p in [self.h_thrust_plot, self.h_rpm_plot, self.h_temp_plot, self.h_voltage_plot, self.h_current_plot, self.h_power_plot, self.h_throttle_plot]:
            _apply_plot_theme(p)

    def create_history_tab(self):
        """Create the history tab listing past CSV exports with graphs and raw data."""
        tab = QWidget()
        layout = QHBoxLayout()
        tab.setLayout(layout)

        # Left: file list and refresh
        left_layout = QVBoxLayout()
        self.history_refresh_btn = QPushButton("Refresh")
        self.history_refresh_btn.clicked.connect(self.refresh_history_files)
        left_layout.addWidget(self.history_refresh_btn)
        self.history_list = QListWidget()
        self.history_list.currentTextChanged.connect(self.load_history_file)
        left_layout.addWidget(self.history_list)

        layout.addLayout(left_layout, 1)

        # Right: plots and table
        right_layout = QVBoxLayout()

        # Plots
        plots_layout = QVBoxLayout()
        row1 = QHBoxLayout(); row2 = QHBoxLayout(); row3 = QHBoxLayout(); row4 = QHBoxLayout()
        self.h_thrust_plot = self.create_plot("Thrust (grams)", "g", (255, 0, 0))
        self.h_rpm_plot = self.create_plot("RPM", "RPM", (0, 255, 0))
        self.h_temp_plot = self.create_plot("Temperature (°C)", "°C", (0, 0, 255))
        self.h_voltage_plot = self.create_plot("Voltage (V)", "V", (255, 165, 0))
        self.h_current_plot = self.create_plot("Current (A)", "A", (255, 0, 255))
        self.h_power_plot = self.create_plot("Power (W)", "W", (0, 0, 0))
        self.h_throttle_plot = self.create_plot("Throttle (%)", "%", (100, 150, 200))
        row1.addWidget(self.h_thrust_plot['widget']); row1.addWidget(self.h_rpm_plot['widget'])
        row2.addWidget(self.h_temp_plot['widget']); row2.addWidget(self.h_voltage_plot['widget'])
        row3.addWidget(self.h_current_plot['widget']); row3.addWidget(self.h_power_plot['widget'])
        row4.addWidget(self.h_throttle_plot['widget'])  # Throttle spans full width
        plots_layout.addLayout(row1); plots_layout.addLayout(row2); plots_layout.addLayout(row3); plots_layout.addLayout(row4)
        right_layout.addLayout(plots_layout, 3)

        # Table
        self.history_table = QTableWidget()
        right_layout.addWidget(self.history_table, 2)

        layout.addLayout(right_layout, 3)

        # Initial scan
        self.refresh_history_files()

        return tab

    def refresh_history_files(self):
        """Scan for thrust_test_*.csv files and list them sorted by modified time."""
        pattern = os.path.join(os.getcwd(), "thrust_test_*.csv")
        files = glob.glob(pattern)
        files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        self.history_list.clear()
        for path in files:
            self.history_list.addItem(path)

    def load_history_file(self, path):
        """Load a CSV file, parse metadata/header, plot and display data table."""
        if not path:
            return
        times = []
        thrusts = []
        rpms = []
        temps = []
        volts = []
        currents = []
        powers = []
        throttles = []
        try:
            with open(path, 'r', newline='') as f:
                reader = csv.reader(f)
                header_found = False
                header = None
                throttle_idx = None
                for row in reader:
                    if not row:
                        continue
                    if not header_found:
                        # Look for the data header row (detect new or old format)
                        first = row[0].strip().lower()
                        if first.startswith('timestamp'):
                            header = row
                            header_found = True
                            # Find throttle column index if it exists
                            for i, h in enumerate(header):
                                if 'throttle' in h.lower():
                                    throttle_idx = i
                                    break
                            continue
                        elif first.startswith('time'):
                            header = row
                            header_found = True
                            # Find throttle column index if it exists
                            for i, h in enumerate(header):
                                if 'throttle' in h.lower():
                                    throttle_idx = i
                                    break
                            continue
                        else:
                            # ignore metadata rows
                            continue
                    # Parse data rows
                    try:
                        # New format: Timestamp, Elapsed (s), Thrust, RPM, Temp, Voltage, Current, Power, Throttle (optional)
                        if header and header[0].strip().lower().startswith('timestamp'):
                            ts_str = row[0].strip()
                            t = float(row[1]) if len(row) > 1 and row[1] != "" else None
                            th = float(row[2]) if len(row) > 2 and row[2] != "" else 0.0
                            r = float(row[3]) if len(row) > 3 and row[3] != "" else 0.0
                            te = float(row[4]) if len(row) > 4 and row[4] != "" else 0.0
                            v = float(row[5]) if len(row) > 5 and row[5] != "" else 0.0
                            c = float(row[6]) if len(row) > 6 and row[6] != "" else 0.0
                            p = float(row[7]) if len(row) > 7 and row[7] != "" else None
                            # Extract throttle: try column 8 first, or use throttle_idx if found
                            if throttle_idx is not None and len(row) > throttle_idx:
                                thr = float(row[throttle_idx]) if row[throttle_idx] != "" else 0.0
                            elif len(row) > 8 and row[8] != "":
                                thr = float(row[8])
                            else:
                                thr = 0.0
                        else:
                            # Old format: Time (s), Thrust, RPM, Temp, Voltage, Current, Power, Throttle (optional)
                            t = float(row[0])
                            th = float(row[1]) if len(row) > 1 and row[1] != "" else 0.0
                            r = float(row[2]) if len(row) > 2 and row[2] != "" else 0.0
                            te = float(row[3]) if len(row) > 3 and row[3] != "" else 0.0
                            v = float(row[4]) if len(row) > 4 and row[4] != "" else 0.0
                            c = float(row[5]) if len(row) > 5 and row[5] != "" else 0.0
                            p = float(row[6]) if len(row) > 6 and row[6] != "" else None
                            # Extract throttle: try column 7 first, or use throttle_idx if found
                            if throttle_idx is not None and len(row) > throttle_idx:
                                thr = float(row[throttle_idx]) if row[throttle_idx] != "" else 0.0
                            elif len(row) > 7 and row[7] != "":
                                thr = float(row[7])
                            else:
                                thr = 0.0
                            ts_str = None
                    except Exception:
                        continue

                    # Only append rows with a valid numeric time for plotting
                    if t is not None:
                        times.append(t)
                        thrusts.append(th)
                        rpms.append(r)
                        temps.append(te)
                        volts.append(v)
                        currents.append(c)
                        if p is not None:
                            powers.append(p)
                        throttles.append(thr)
        except Exception as e:
            QMessageBox.critical(self, "History Load Error", str(e))
            return

        # If no throttle data was found, create zeros
        if len(throttles) != len(times):
            throttles = [0.0] * len(times)
        
        time_array = np.array(times) if len(times) > 0 else np.array([])
        throttle_array = np.array(throttles) if len(throttles) > 0 else np.array([])
        
        def set_curve(plot, data, x_data):
            if len(x_data) > 0 and len(data) == len(x_data):
                plot['curve'].setData(x_data, np.array(data))
                if len(x_data) > 0:
                    plot['widget'].setXRange(max(0, x_data[0]), x_data[-1], padding=0.02)
            else:
                plot['curve'].setData([], [])
        
        # Use throttle or time based on domain setting
        x_axis_data = throttle_array if self.use_throttle_domain and len(throttle_array) > 0 else time_array
        
        set_curve(self.h_thrust_plot, thrusts, x_axis_data)
        set_curve(self.h_rpm_plot, rpms, x_axis_data)
        set_curve(self.h_temp_plot, temps, x_axis_data)
        set_curve(self.h_voltage_plot, volts, x_axis_data)
        set_curve(self.h_current_plot, currents, x_axis_data)
        # Power: use CSV column if present; otherwise compute Voltage * Current
        if len(powers) != len(times):
            powers = []
            if len(volts) == len(currents):
                try:
                    powers = (np.array(volts, dtype=float) * np.array(currents, dtype=float)).tolist()
                except Exception:
                    powers = []
        set_curve(self.h_power_plot, powers, x_axis_data)
        # Throttle plot always uses time as x-axis (for reference)
        set_curve(self.h_throttle_plot, throttles, time_array)

        # Update table
        cols = ['Time (s)', 'Thrust (g)', 'RPM', 'Temperature (°C)', 'Voltage (V)', 'Current (A)', 'Power (W)']
        self.history_table.clear()
        self.history_table.setColumnCount(len(cols))
        self.history_table.setHorizontalHeaderLabels(cols)
        self.history_table.setRowCount(len(times))
        for i in range(len(times)):
            pval = powers[i] if i < len(powers) else (volts[i] * currents[i] if i < len(volts) and i < len(currents) else 0)
            values = [f"{times[i]:.3f}", f"{thrusts[i]:.3f}", f"{rpms[i]:.1f}", f"{temps[i]:.2f}", f"{volts[i]:.3f}", f"{currents[i]:.3f}", f"{pval:.3f}"]
            for j, val in enumerate(values):
                self.history_table.setItem(i, j, QTableWidgetItem(val))
    
    def create_live_data_panel(self):
        """Create live data display panel."""
        data_group = QGroupBox("Live Data")
        data_layout = QVBoxLayout()
        data_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14pt;
                border: 2px solid #3498db;
                border-radius: 5px;
                margin-top: 10px;
                padding: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        
        # Create labels for each measurement
        label_style = """
            font-size: 16pt;
            font-weight: bold;
            padding: 10px;
            border-radius: 5px;
            margin: 5px;
        """

        # Timestamp label (real time)
        self.timestamp_label = QLabel("Time: ---")
        self.timestamp_label.setStyleSheet("font-size: 14pt; font-weight: bold; padding: 6px; margin: 4px;")
        data_layout.addWidget(self.timestamp_label)
        
        # Thrust
        self.thrust_value_label = QLabel("Thrust: --- g")
        self.thrust_value_label.setStyleSheet(label_style + "background-color: #ffebee; color: #c62828;")
        data_layout.addWidget(self.thrust_value_label)
        
        # RPM
        self.rpm_value_label = QLabel("RPM: --- RPM")
        self.rpm_value_label.setStyleSheet(label_style + "background-color: #e8f5e9; color: #2e7d32;")
        data_layout.addWidget(self.rpm_value_label)
        
        # Temperature
        self.temp_value_label = QLabel("Temp: --- °C")
        self.temp_value_label.setStyleSheet(label_style + "background-color: #e3f2fd; color: #1565c0;")
        data_layout.addWidget(self.temp_value_label)
        
        # Voltage
        self.voltage_value_label = QLabel("Voltage: --- V")
        self.voltage_value_label.setStyleSheet(label_style + "background-color: #fff3e0; color: #e65100;")
        data_layout.addWidget(self.voltage_value_label)
        
        # Current
        self.current_value_label = QLabel("Current: --- A")
        self.current_value_label.setStyleSheet(label_style + "background-color: #f3e5f5; color: #6a1b9a;")
        data_layout.addWidget(self.current_value_label)
        
        # Throttle
        self.throttle_value_label = QLabel("Throttle: --- %")
        self.throttle_value_label.setStyleSheet(label_style + "background-color: #e0f2f1; color: #00695c;")
        data_layout.addWidget(self.throttle_value_label)
        
        # Add spacer
        data_layout.addStretch()
        
        data_group.setLayout(data_layout)
        return data_group
    
    def create_plot(self, title, y_label, color):
        """Create a plot widget."""
        plot_widget = pg.PlotWidget()
        # Theme-aware plot styling: dark background for graphs in dark mode, keep text non-grey (white on dark)
        is_dark = getattr(self, 'is_dark_mode', False)
        bg = (20, 20, 20) if is_dark else 'w'
        # Use explicit RGB tuples for text colors
        fg = (255, 255, 255) if is_dark else (0, 0, 0)
        plot_widget.setBackground(bg)
        plot_widget.setTitle(title, color=fg, size='12pt')
        plot_widget.setLabel('left', y_label, color=fg)
        # X-axis label will be updated dynamically based on domain toggle
        plot_widget.setLabel('bottom', 'Time (s)', color=fg)
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        
        # Set minimum size for more square appearance
        plot_widget.setMinimumHeight(250)
        plot_widget.setMinimumWidth(400)
        
        # Set time axis to start at 0
        plot_widget.setXRange(0, 10, padding=0)
        
        # Enable mouse interactions for panning and zooming
        plot_widget.setMouseEnabled(x=True, y=True)  # Enable panning with left mouse drag
        vb = plot_widget.getViewBox()
        vb.setMouseMode(pg.ViewBox.PanMode)  # Default to pan mode (left drag to pan)
        # Enable right-click drag to zoom region
        vb.setMenuEnabled(True)  # Enable right-click menu (includes zoom options)
        # Mouse wheel zoom is enabled by default in PyQtGraph
        
        # Create plot curve
        curve = plot_widget.plot(pen=pg.mkPen(color=color, width=2))

        # Hover crosshair and value label
        vline = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen((150, 150, 150), style=pg.QtCore.Qt.DotLine))
        hline = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen((150, 150, 150), style=pg.QtCore.Qt.DotLine))
        value_label = pg.TextItem(color=(0, 0, 0))
        # Hover point marker
        marker = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(color[0], color[1], color[2], 200), pen=pg.mkPen('k', width=1))
        plot_widget.addItem(vline, ignoreBounds=True)
        plot_widget.addItem(hline, ignoreBounds=True)
        plot_widget.addItem(value_label)
        plot_widget.addItem(marker)
        vline.hide(); hline.hide(); value_label.hide(); marker.hide()

        def _on_mouse_moved(pos):
            if not plot_widget.sceneBoundingRect().contains(pos):
                vline.hide(); hline.hide(); value_label.hide(); marker.hide()
                return
            mouse_point = plot_widget.plotItem.vb.mapSceneToView(pos)
            x = mouse_point.x(); y = mouse_point.y()
            x_data, y_data = curve.getData()
            if x_data is None or y_data is None:
                vline.hide(); hline.hide(); value_label.hide(); marker.hide()
                return
            try:
                import numpy as _np
                x_data = _np.asarray(x_data, dtype=float)
                y_data = _np.asarray(y_data, dtype=float)
            except Exception:
                vline.hide(); hline.hide(); value_label.hide(); marker.hide()
                return
            if len(x_data) == 0 or len(x_data) != len(y_data):
                vline.hide(); hline.hide(); value_label.hide(); marker.hide()
                return
            # Find nearest data point by x
            idx = _np.searchsorted(x_data, x)
            cand_indices = [max(0, min(idx, len(x_data) - 1))]
            if idx > 0:
                cand_indices.append(idx - 1)
            # Choose closer by x distance
            i_best = min(cand_indices, key=lambda i: abs(x_data[i] - x))
            px = x_data[i_best]; py = y_data[i_best]
            # Only show if cursor is near the data point (within ~20 px)
            scene_pt = plot_widget.plotItem.vb.mapViewToScene(QPointF(px, py))
            dist = (scene_pt - pos).manhattanLength()
            if dist > 60:
                vline.hide(); hline.hide(); value_label.hide(); marker.hide()
                return
            vline.setPos(px); hline.setPos(py)
            value_label.setText(f"x={px:.3f}, y={py:.3f}")
            value_label.setPos(px, py)
            marker.setData([px], [py])
            vline.show(); hline.show(); value_label.show(); marker.show()

        hover_proxy = pg.SignalProxy(plot_widget.scene().sigMouseMoved, rateLimit=60, slot=lambda evt: _on_mouse_moved(evt[0]))
        
        return {
            'widget': plot_widget,
            'curve': curve,
            'visible': True,
            'vline': vline,
            'hline': hline,
            'label': value_label,
            'marker': marker,
            'hover_proxy': hover_proxy,
            'title': title,
            'y_label': y_label
        }
    
    def refresh_ports(self):
        """Refresh available serial ports."""
        self.port_combo.clear()
        ports = SerialReader.list_ports()
        if ports:
            self.port_combo.addItems(ports)
        else:
            self.port_combo.addItem("No ports available")
    
    def toggle_connection(self):
        """Connect or disconnect from Arduino."""
        if not self.serial_reader.is_connected:
            try:
                port = self.port_combo.currentText()
                if port == "No ports available":
                    QMessageBox.warning(self, "Error", "No serial ports available!")
                    return
                
                self.serial_reader.connect(port)
                self.connect_btn.setText("Disconnect")
                self.start_btn.setEnabled(True)
                self.stop_motor_btn.setEnabled(True)
                self.procedure_btn.setEnabled(True)
                self.status_label.setText(f"Status: Connected to {port}")
                self.status_label.setStyleSheet("color: green; font-weight: bold; font-size: 12pt;")
                
            except Exception as e:
                QMessageBox.critical(self, "Connection Error", str(e))
        else:
            self.stop_test()
            self.serial_reader.disconnect()
            self.connect_btn.setText("Connect")
            self.start_btn.setEnabled(False)
            self.stop_motor_btn.setEnabled(False)
            self.procedure_btn.setEnabled(False)
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 12pt;")
    
    def start_test(self):
        """Start data acquisition and plotting."""
        # Clear old data
        self.time_data.clear()
        self.thrust_data.clear()
        self.rpm_data.clear()
        self.temperature_data.clear()
        self.voltage_data.clear()
        self.current_data.clear()
        self.power_data.clear()
        self.throttle_data.clear()
        self.timestamp_data.clear()
        self.time_history = []
        self.thrust_history = []
        self.rpm_history = []
        self.temperature_history = []
        self.voltage_history = []
        self.current_history = []
        self.power_history = []
        self.throttle_history = []
        self.timestamp_history = []
        self.start_time = 0
        
        # Enable/disable buttons
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.export_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        
        # Start timer (update every 100ms)
        self.timer.start(100)
        self.status_label.setText("Status: Test Running")
        self.status_label.setStyleSheet("color: blue; font-weight: bold; font-size: 12pt;")
        # Record initial data point at t=0
        self.update_plots()
    
    def stop_test(self):
        """Stop data acquisition."""
        self.timer.stop()
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.connect_btn.setEnabled(True)
        
        # Enable export if we have data
        if len(self.time_history) > 0:
            self.export_btn.setEnabled(True)
        
        if self.serial_reader.is_connected:
            self.status_label.setText(f"Status: Connected (Test Stopped)")
            self.status_label.setStyleSheet("color: green; font-weight: bold; font-size: 12pt;")
    
    def stop_motor(self):
        """Stop the motor immediately."""
        if self.serial_reader.is_connected:
            self.serial_reader.send_command("s")
            self.status_label.setText("Status: Motor Stopped")
            self.status_label.setStyleSheet("color: orange; font-weight: bold; font-size: 12pt;")
    
    def run_procedure(self):
        """Run the automated throttle ramp procedure."""
        if self.serial_reader.is_connected:
            self.serial_reader.send_command("procedure")
            self.status_label.setText("Status: Running Auto Procedure")
            self.status_label.setStyleSheet("color: blue; font-weight: bold; font-size: 12pt;")
    
    def update_plots(self):
        """Read data and update all plots."""
        data = self.serial_reader.read_data()
        
        if data is None:
            return
        
        # Calculate elapsed seconds (numeric)
        if self.start_time == 0:
            import time as _tt
            self.start_time = _tt.time()
        
        import time as _tt
        elapsed = _tt.time() - self.start_time
        self.time_data.append(elapsed)
        self.time_history.append(elapsed)
        
        # Timestamp (human-readable) from SerialReader
        timestamp_str = data.get('timestamp') or datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.timestamp_data.append(timestamp_str)
        self.timestamp_history.append(timestamp_str)
        
        # Store data
        thrust_val = data.get('thrust', 0) or 0
        rpm_val = data.get('rpm', 0) or 0
        temp_val = data.get('temperature', 0) or 0
        voltage_val = data.get('voltage', 0) or 0
        current_val = data.get('current', 0) or 0
        power_val = data.get('power', 0) or (voltage_val * current_val)
        throttle_val = data.get('throttle', 0) or 0
        
        self.thrust_data.append(thrust_val)
        self.rpm_data.append(rpm_val)
        self.temperature_data.append(temp_val)
        self.voltage_data.append(voltage_val)
        self.current_data.append(current_val)
        self.power_data.append(power_val)
        self.throttle_data.append(throttle_val)
        self.thrust_history.append(thrust_val)
        self.rpm_history.append(rpm_val)
        self.temperature_history.append(temp_val)
        self.voltage_history.append(voltage_val)
        self.current_history.append(current_val)
        self.power_history.append(power_val)
        self.throttle_history.append(throttle_val)
        
        # Update live value labels (and timestamp at top)
        self.timestamp_label.setText(f"Time: {timestamp_str}")
        self.thrust_value_label.setText(f"Thrust: {thrust_val:.2f} g")
        self.rpm_value_label.setText(f"RPM: {rpm_val:.1f} RPM")
        self.temp_value_label.setText(f"Temp: {temp_val:.1f} °C")
        self.voltage_value_label.setText(f"Voltage: {voltage_val:.2f} V")
        self.current_value_label.setText(f"Current: {current_val:.3f} A")
        self.throttle_value_label.setText(f"Throttle: {throttle_val:.1f} %")
        
        # Update plots (display-only downsampling to reduce visual density)
        if self.use_throttle_domain:
            # Use throttle as x-axis
            throttle_array = np.array(self.throttle_data)
            
            # Sort by throttle for cleaner plots (remove duplicates at same throttle)
            def prepare_throttle_data(x_data, y_data):
                if len(x_data) == 0 or len(y_data) == 0:
                    return np.array([]), np.array([])
                # Create pairs and sort by throttle
                pairs = list(zip(x_data, y_data))
                # Remove duplicates at same throttle (keep last value)
                throttle_dict = {}
                for x, y in pairs:
                    throttle_dict[x] = y
                sorted_pairs = sorted(throttle_dict.items())
                if len(sorted_pairs) == 0:
                    return np.array([]), np.array([])
                x_sorted, y_sorted = zip(*sorted_pairs)
                return np.array(x_sorted), np.array(y_sorted)
            
            if self.plots['thrust']['visible']:
                thrust_array = np.array(self.thrust_data)
                x_vals, y_vals = prepare_throttle_data(throttle_array, thrust_array)
                self.plots['thrust']['curve'].setData(x_vals, y_vals)
            
            if self.plots['rpm']['visible']:
                rpm_array = np.array(self.rpm_data)
                x_vals, y_vals = prepare_throttle_data(throttle_array, rpm_array)
                self.plots['rpm']['curve'].setData(x_vals, y_vals)
            
            if self.plots['temperature']['visible']:
                temp_array = np.array(self.temperature_data)
                x_vals, y_vals = prepare_throttle_data(throttle_array, temp_array)
                self.plots['temperature']['curve'].setData(x_vals, y_vals)
            
            if self.plots['voltage']['visible']:
                volt_array = np.array(self.voltage_data)
                x_vals, y_vals = prepare_throttle_data(throttle_array, volt_array)
                self.plots['voltage']['curve'].setData(x_vals, y_vals)
            
            if self.plots['current']['visible']:
                curr_array = np.array(self.current_data)
                x_vals, y_vals = prepare_throttle_data(throttle_array, curr_array)
                self.plots['current']['curve'].setData(x_vals, y_vals)
            
            if self.plots['power']['visible']:
                power_array = np.array(self.power_data)
                x_vals, y_vals = prepare_throttle_data(throttle_array, power_array)
                self.plots['power']['curve'].setData(x_vals, y_vals)
            
            # Throttle plot always uses time as x-axis (for reference mapping)
            if self.plots['throttle']['visible']:
                time_array = np.array(self.time_data)
                throttle_array = np.array(self.throttle_data)
                dx, dy = decimate_by_time(time_array, throttle_array, self.display_step_s)
                self.plots['throttle']['curve'].setData(dx, dy)
            
            # Set throttle range (throttle plot uses time axis, so set it separately)
            if len(throttle_array) > 0:
                min_throttle = max(0, np.min(throttle_array))
                max_throttle = min(100, np.max(throttle_array))
                time_array = np.array(self.time_data)
                for plot_name, plot_info in self.plots.items():
                    if plot_info['visible']:
                        if plot_name == 'throttle':
                            # Throttle plot always uses time axis
                            if len(time_array) > 0:
                                plot_info['widget'].setXRange(max(0, time_array[0]), time_array[-1], padding=0.02)
                        else:
                            plot_info['widget'].setXRange(min_throttle, max_throttle, padding=0.02)
        else:
            # Use time as x-axis (original behavior)
            time_array = np.array(self.time_data)
            def decimate_by_time(x, y, step_s):
                if len(x) == 0:
                    return x, y
                dx = []
                dy = []
                last_t = None
                for xi, yi in zip(x, y):
                    if last_t is None or (xi - last_t) >= step_s:
                        dx.append(xi)
                        dy.append(yi)
                        last_t = xi
                    else:
                        # replace last sample in current bin with the latest value
                        dx[-1] = xi
                        dy[-1] = yi
                return np.array(dx), np.array(dy)
            
            if self.plots['thrust']['visible']:
                thrust_array = np.array(self.thrust_data)
                dx, dy = decimate_by_time(time_array, thrust_array, self.display_step_s)
                self.plots['thrust']['curve'].setData(dx, dy)
            
            if self.plots['rpm']['visible']:
                rpm_array = np.array(self.rpm_data)
                dx, dy = decimate_by_time(time_array, rpm_array, self.display_step_s)
                self.plots['rpm']['curve'].setData(dx, dy)
            
            if self.plots['temperature']['visible']:
                temp_array = np.array(self.temperature_data)
                dx, dy = decimate_by_time(time_array, temp_array, self.display_step_s)
                self.plots['temperature']['curve'].setData(dx, dy)
            
            if self.plots['voltage']['visible']:
                volt_array = np.array(self.voltage_data)
                dx, dy = decimate_by_time(time_array, volt_array, self.display_step_s)
                self.plots['voltage']['curve'].setData(dx, dy)
            
            if self.plots['current']['visible']:
                curr_array = np.array(self.current_data)
                dx, dy = decimate_by_time(time_array, curr_array, self.display_step_s)
                self.plots['current']['curve'].setData(dx, dy)
            
            if self.plots['power']['visible']:
                power_array = np.array(self.power_data)
                dx, dy = decimate_by_time(time_array, power_array, self.display_step_s)
                self.plots['power']['curve'].setData(dx, dy)
            
            # Throttle plot (always vs time for reference)
            if self.plots['throttle']['visible']:
                throttle_array = np.array(self.throttle_data)
                dx, dy = decimate_by_time(time_array, throttle_array, self.display_step_s)
                self.plots['throttle']['curve'].setData(dx, dy)
            
            # Keep time axis from going negative
            if len(time_array) > 0:
                min_time = max(0, time_array[0])
                max_time = max(10, time_array[-1])
                for plot_name, plot_info in self.plots.items():
                    if plot_info['visible']:
                        plot_info['widget'].setXRange(min_time, max_time, padding=0.02)
    
    def toggle_plot(self, measurement, state):
        """Show or hide a plot."""
        is_checked = state == 2  # Qt.Checked
        self.plots[measurement]['visible'] = is_checked
        self.plots[measurement]['widget'].setVisible(is_checked)
    
    def toggle_domain(self, state):
        """Toggle x-axis between time and throttle."""
        self.use_throttle_domain = state == 2  # Qt.Checked
        # Update all plot axis labels
        x_label = 'Throttle (%)' if self.use_throttle_domain else 'Time (s)'
        fg = (255, 255, 255) if self.is_dark_mode else (0, 0, 0)
        
        # Update live plots (throttle plot always uses time)
        for plot_name, plot_info in self.plots.items():
            if plot_name == 'throttle':
                # Throttle plot always uses time as x-axis
                plot_info['widget'].setLabel('bottom', 'Time (s)', color=fg)
                plot_info['widget'].getAxis('bottom').setLabel('Time (s)', color=fg)
            else:
                plot_info['widget'].setLabel('bottom', x_label, color=fg)
                plot_info['widget'].getAxis('bottom').setLabel(x_label, color=fg)
        
        # Update history plots (throttle plot always uses time)
        for plot_info in [self.h_thrust_plot, self.h_rpm_plot, self.h_temp_plot, 
                          self.h_voltage_plot, self.h_current_plot, self.h_power_plot]:
            plot_info['widget'].setLabel('bottom', x_label, color=fg)
            plot_info['widget'].getAxis('bottom').setLabel(x_label, color=fg)
        
        # Throttle plot always uses time as x-axis
        self.h_throttle_plot['widget'].setLabel('bottom', 'Time (s)', color=fg)
        self.h_throttle_plot['widget'].getAxis('bottom').setLabel('Time (s)', color=fg)
        
        # Trigger plot update to redraw with new domain
        if len(self.time_data) > 0:
            self.update_plots()
        
        # If a history file is currently loaded, reload it to update plots
        current_history = self.history_list.currentItem()
        if current_history:
            self.load_history_file(current_history.text())
    
    def autoscale_plots(self):
        """Auto-scale all visible plots to fit data perfectly."""
        if len(self.time_history) == 0:
            QMessageBox.information(self, "No Data", "No data to scale. Start a test first.")
            return
        
        # Auto-scale each visible plot
        data_map = {
            'thrust': self.thrust_history,
            'rpm': self.rpm_history,
            'temperature': self.temperature_history,
            'voltage': self.voltage_history,
            'current': self.current_history,
            'power': self.power_history,
            'throttle': self.throttle_history
        }
        
        if self.use_throttle_domain:
            # Use throttle for x-axis
            throttle_array = np.array(self.throttle_history)
            if len(throttle_array) > 0:
                min_x = max(0, np.min(throttle_array))
                max_x = min(100, np.max(throttle_array))
            else:
                min_x, max_x = 0, 100
        else:
            # Use time for x-axis
            time_array = np.array(self.time_history)
            min_x = max(0, time_array[0])  # Never go negative
            max_x = time_array[-1]
        
        for plot_name, plot_info in self.plots.items():
            if plot_info['visible'] and len(data_map[plot_name]) > 0:
                data_array = np.array(data_map[plot_name])
                
                # Throttle plot always uses time for x-axis
                if plot_name == 'throttle':
                    time_array = np.array(self.time_history)
                    if len(time_array) > 0:
                        plot_info['widget'].setXRange(max(0, time_array[0]), time_array[-1], padding=0.02)
                else:
                    # Set X range based on domain
                    plot_info['widget'].setXRange(min_x, max_x, padding=0.02)
                
                # Set Y range with padding
                min_val = np.min(data_array)
                max_val = np.max(data_array)
                
                # Add 5% padding on Y axis
                if max_val != min_val:
                    y_range = max_val - min_val
                    plot_info['widget'].setYRange(
                        min_val - 0.05 * y_range,
                        max_val + 0.05 * y_range,
                        padding=0
                    )
                else:
                    # If all values are the same, center with fixed range
                    plot_info['widget'].setYRange(min_val - 1, max_val + 1, padding=0)
        
        QMessageBox.information(self, "Auto-Scale", "All plots have been auto-scaled to fit data.")
    
    def export_to_csv(self):
        """Export collected data to CSV file."""
        if len(self.time_history) == 0:
            QMessageBox.warning(self, "No Data", "No data to export. Run a test first.")
            return
        
        # Generate default filename with date only (YYYY-MM-DD)
        date_str = datetime.now().strftime("%Y-%m-%d")
        default_filename = f"thrust_test_{date_str}.csv"
        
        # Open file dialog
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Export Test Data",
            default_filename,
            "CSV Files (*.csv);;All Files (*)"
        )
        
        if not filename:
            return  # User cancelled
        
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Metadata rows
                writer.writerow(['Motor', self.motor_name_input.text() or ''])
                writer.writerow(['Propeller', self.prop_type_input.text() or ''])
                writer.writerow(["Exported", datetime.now().strftime("%Y-%m-%d %H:%M:%S")])
                writer.writerow([])
                # Header — Timestamp (HH:MM:SS.sss), Elapsed (s), Thrust, RPM, Temp, Voltage, Current, Power, Throttle
                writer.writerow(['Timestamp (HH:MM:SS.sss)', 'Elapsed (s)', 'Thrust (g)', 'RPM', 'Temperature (°C)', 'Voltage (V)', 'Current (A)', 'Power (W)', 'Throttle (%)'])
                
                # Write data rows
                for i in range(len(self.time_history)):
                    throttle_val = self.throttle_history[i] if i < len(self.throttle_history) else 0.0
                    writer.writerow([
                        self.timestamp_history[i] if i < len(self.timestamp_history) else '',
                        f"{self.time_history[i]:.3f}",
                        f"{self.thrust_history[i]:.3f}",
                        f"{self.rpm_history[i]:.1f}",
                        f"{self.temperature_history[i]:.2f}",
                        f"{self.voltage_history[i]:.3f}",
                        f"{self.current_history[i]:.3f}",
                        f"{self.power_history[i]:.3f}",
                        f"{throttle_val:.1f}"
                    ])
            
            QMessageBox.information(self, "Export Successful", f"Data exported to:\n{filename}")
            
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export data:\n{str(e)}")
    
    def closeEvent(self, event):
        """Clean up when closing the application."""
        if self.serial_reader.is_connected:
            self.serial_reader.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = ThrustStandGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
