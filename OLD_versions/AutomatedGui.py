import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QCheckBox, QPushButton, QComboBox,
                             QLabel, QGroupBox, QMessageBox, QFileDialog, QLineEdit,
                             QTabWidget, QListWidget, QTableWidget, QTableWidgetItem)
import os
import glob
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
from collections import deque
import numpy as np
import csv
from datetime import datetime
from serial_reader import SerialReader


class ThrustStandGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Thrust Stand Monitor")
        self.setGeometry(100, 100, 1600, 900)

        # Serial reader
        self.serial_reader = SerialReader()

        # Data storage
        self.time_data = deque()
        self.thrust_data = deque()
        self.rpm_data = deque()
        self.temperature_data = deque()
        self.voltage_data = deque()
        self.current_data = deque()
        self.power_data = deque()
        self.throttle_data = deque()
        self.timestamp_data = deque()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)

        self.init_ui()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout()
        main_widget.setLayout(main_layout)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # ------------------ Live Tab ------------------
        live_tab = QWidget()
        live_layout = QVBoxLayout()
        live_tab.setLayout(live_layout)

        control_panel = self.create_control_panel()
        live_layout.addWidget(control_panel)

        content_layout = QHBoxLayout()
        plot_layout = QVBoxLayout()

        row1_layout = QHBoxLayout()
        row2_layout = QHBoxLayout()
        row3_layout = QHBoxLayout()

        self.thrust_plot = self.create_plot("Thrust (grams)", "g", (255, 0, 0))
        self.rpm_plot = self.create_plot("RPM", "RPM", (0, 255, 0))
        self.temp_plot = self.create_plot("Temperature (°C)", "°C", (0, 0, 255))
        self.voltage_plot = self.create_plot("Voltage (V)", "V", (255, 165, 0))
        self.current_plot = self.create_plot("Current (A)", "A", (255, 0, 255))
        self.power_plot = self.create_plot("Power (W)", "W", (0, 0, 0))

        row1_layout.addWidget(self.thrust_plot['widget'])
        row1_layout.addWidget(self.rpm_plot['widget'])
        row2_layout.addWidget(self.temp_plot['widget'])
        row2_layout.addWidget(self.voltage_plot['widget'])
        row3_layout.addWidget(self.current_plot['widget'])
        row3_layout.addWidget(self.power_plot['widget'])

        plot_layout.addLayout(row1_layout)
        plot_layout.addLayout(row2_layout)
        plot_layout.addLayout(row3_layout)

        content_layout.addLayout(plot_layout, 3)

        live_data_panel = self.create_live_data_panel()
        content_layout.addWidget(live_data_panel, 1)

        live_layout.addLayout(content_layout)

        self.tabs.addTab(live_tab, "Live")

        # ------------------ History Tab ------------------
        history_tab = self.create_history_tab()
        self.tabs.addTab(history_tab, "History")

        self.plots = {
            'thrust': self.thrust_plot,
            'rpm': self.rpm_plot,
            'temperature': self.temp_plot,
            'voltage': self.voltage_plot,
            'current': self.current_plot,
            'power': self.power_plot
        }

    # ------------------ Control Panel ------------------
    def create_control_panel(self):
        control_group = QGroupBox("Controls")
        control_layout = QHBoxLayout()

        # Serial port selection
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

        # Buttons
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
        self.autoscale_btn.setMinimumHeight(44)
        button_layout.addWidget(self.autoscale_btn)

        control_layout.addLayout(button_layout)

        # Test metadata
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

        # Graph checkboxes
        checkbox_layout = QVBoxLayout()
        checkbox_label = QLabel("Display Graphs:")
        checkbox_layout.addWidget(checkbox_label)
        self.checkboxes = {}
        measurements = ['thrust', 'rpm', 'temperature', 'voltage', 'current', 'power']
        labels = ['Thrust (g)', 'RPM', 'Temperature (°C)', 'Voltage (V)', 'Current (A)', 'Power (W)']
        for measure, label in zip(measurements, labels):
            cb = QCheckBox(label)
            cb.setChecked(True)
            cb.stateChanged.connect(lambda state, m=measure: self.toggle_plot(m, state))
            self.checkboxes[measure] = cb
            checkbox_layout.addWidget(cb)
        control_layout.addLayout(checkbox_layout)

        # Status
        status_layout = QVBoxLayout()
        self.status_label = QLabel("Status: Disconnected")
        status_layout.addWidget(self.status_label)
        control_layout.addLayout(status_layout)

        control_group.setLayout(control_layout)
        return control_group

    # ------------------ Live Data Panel ------------------
    def create_live_data_panel(self):
        data_group = QGroupBox("Live Data")
        data_layout = QVBoxLayout()
        label_style = "font-size:16pt;font-weight:bold;padding:10px;margin:5px;"
        self.timestamp_label = QLabel("Time: ---")
        data_layout.addWidget(self.timestamp_label)
        self.thrust_value_label = QLabel("Thrust: --- g"); self.thrust_value_label.setStyleSheet(label_style)
        self.rpm_value_label = QLabel("RPM: --- RPM"); self.rpm_value_label.setStyleSheet(label_style)
        self.temp_value_label = QLabel("Temp: --- °C"); self.temp_value_label.setStyleSheet(label_style)
        self.voltage_value_label = QLabel("Voltage: --- V"); self.voltage_value_label.setStyleSheet(label_style)
        self.current_value_label = QLabel("Current: --- A"); self.current_value_label.setStyleSheet(label_style)
        self.throttle_value_label = QLabel("Throttle: --- %"); self.throttle_value_label.setStyleSheet(label_style)
        for w in [self.thrust_value_label, self.rpm_value_label, self.temp_value_label,
                  self.voltage_value_label, self.current_value_label, self.throttle_value_label]:
            data_layout.addWidget(w)
        data_layout.addStretch()
        data_group.setLayout(data_layout)
        return data_group

    # ------------------ Plot creation ------------------
    def create_plot(self, title, y_label, color):
        plot_widget = pg.PlotWidget()
        plot_widget.setBackground('w')
        plot_widget.setTitle(title, color='k', size='12pt')
        plot_widget.setLabel('left', y_label, color='k')
        plot_widget.setLabel('bottom', 'Time (s)', color='k')
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        curve = plot_widget.plot(pen=pg.mkPen(color=color, width=2))
        return {'widget': plot_widget, 'curve': curve, 'visible': True}

    # ------------------ History Tab ------------------
    def create_history_tab(self):
        tab = QWidget()
        layout = QHBoxLayout()
        tab.setLayout(layout)
        left_layout = QVBoxLayout()
        self.history_refresh_btn = QPushButton("Refresh")
        self.history_refresh_btn.clicked.connect(self.refresh_history_files)
        left_layout.addWidget(self.history_refresh_btn)
        self.history_list = QListWidget()
        self.history_list.currentTextChanged.connect(self.load_history_file)
        left_layout.addWidget(self.history_list)
        layout.addLayout(left_layout, 1)

        right_layout = QVBoxLayout()
        self.h_thrust_plot = self.create_plot("Thrust (grams)", "g", (255, 0, 0))
        self.h_rpm_plot = self.create_plot("RPM", "RPM", (0, 255, 0))
        self.h_temp_plot = self.create_plot("Temperature (°C)", "°C", (0, 0, 255))
        self.h_voltage_plot = self.create_plot("Voltage (V)", "V", (255, 165, 0))
        self.h_current_plot = self.create_plot("Current (A)", "A", (255, 0, 255))
        self.h_power_plot = self.create_plot("Power (W)", "W", (0, 0, 0))

        plots_layout = QVBoxLayout()
        row1 = QHBoxLayout(); row2 = QHBoxLayout(); row3 = QHBoxLayout()
        row1.addWidget(self.h_thrust_plot['widget']); row1.addWidget(self.h_rpm_plot['widget'])
        row2.addWidget(self.h_temp_plot['widget']); row2.addWidget(self.h_voltage_plot['widget'])
        row3.addWidget(self.h_current_plot['widget']); row3.addWidget(self.h_power_plot['widget'])
        plots_layout.addLayout(row1); plots_layout.addLayout(row2); plots_layout.addLayout(row3)
        right_layout.addLayout(plots_layout, 3)

        self.history_table = QTableWidget()
        right_layout.addWidget(self.history_table, 2)
        layout.addLayout(right_layout, 3)
        self.refresh_history_files()
        return tab

    def refresh_history_files(self):
        pattern = os.path.join(os.getcwd(), "thrust_test_*.csv")
        files = glob.glob(pattern)
        files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        self.history_list.clear()
        for path in files:
            self.history_list.addItem(path)

    def load_history_file(self, path):
        if not path: return
        times, thrusts, rpms, temps, volts, currents, powers, throttles = [], [], [], [], [], [], [], []
        try:
            with open(path, 'r') as f:
                reader = csv.reader(f)
                header = next(reader, None)
                for row in reader:
                    if not row: continue
                    times.append(float(row[1]))
                    thrusts.append(float(row[2]))
                    rpms.append(float(row[3]))
                    temps.append(float(row[4]))
                    volts.append(float(row[5]))
                    currents.append(float(row[6]))
                    powers.append(float(row[7]))
                    throttles.append(float(row[8]))
        except Exception as e:
            QMessageBox.critical(self, "History Load Error", str(e))
            return
        # Update history plots
        for plot, data in zip([self.h_thrust_plot, self.h_rpm_plot, self.h_temp_plot,
                               self.h_voltage_plot, self.h_current_plot, self.h_power_plot],
                              [thrusts, rpms, temps, volts, currents, powers]):
            plot['curve'].setData(times, data)
        # Update table
        self.history_table.setColumnCount(9)
        self.history_table.setHorizontalHeaderLabels(['Timestamp','Time','Thrust','RPM','Temp','Volt','Current','Power','Throttle'])
        self.history_table.setRowCount(len(times))
        for i in range(len(times)):
            for j, val in enumerate([i, times[i], thrusts[i], rpms[i], temps[i], volts[i], currents[i], powers[i], throttles[i]]):
                self.history_table.setItem(i, j, QTableWidgetItem(str(val)))

    # ------------------ Serial / Test Controls ------------------
    def refresh_ports(self):
        self.port_combo.clear()
        ports = SerialReader.list_ports()
        if ports:
            self.port_combo.addItems(ports)
        else:
            self.port_combo.addItem("No ports available")

    def toggle_connection(self):
        if not self.serial_reader.is_connected:
            try:
                port = self.port_combo.currentText()
                if port == "No ports available":
                    QMessageBox.warning(self, "Warning", "No serial port selected!")
                    return
                self.serial_reader.connect(port)
                self.status_label.setText("Status: Connected")
                self.connect_btn.setText("Disconnect")
                self.start_btn.setEnabled(True)
                self.stop_btn.setEnabled(True)
                self.stop_motor_btn.setEnabled(True)
                self.procedure_btn.setEnabled(True)
                self.timer.start(100)
            except Exception as e:
                QMessageBox.critical(self, "Connection Error", str(e))
        else:
            self.serial_reader.disconnect()
            self.status_label.setText("Status: Disconnected")
            self.connect_btn.setText("Connect")
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.stop_motor_btn.setEnabled(False)
            self.procedure_btn.setEnabled(False)
            self.timer.stop()

    def start_test(self):
        for dq in [self.time_data, self.thrust_data, self.rpm_data, self.temperature_data,
                   self.voltage_data, self.current_data, self.power_data, self.throttle_data, self.timestamp_data]:
            dq.clear()
        self.timer.start(100)

    def stop_test(self):
        self.timer.stop()
        self.export_btn.setEnabled(True)

    def stop_motor(self):
        if self.serial_reader.is_connected:
            self.serial_reader.send_command("s")
            self.status_label.setText("Status: Motor Stopped")

    def run_procedure(self):
        self.serial_reader.send_command("procedure")

    # ------------------ Plot Updates ------------------
    def update_plots(self):
        data = self.serial_reader.read_data()
        if not data:
            return
        t = data.get('time',0); th = data.get('thrust',0); r = data.get('rpm',0)
        te = data.get('temperature',0); v = data.get('voltage',0); c = data.get('current',0)
        p = data.get('power',0); thr = data.get('throttle',0)
        self.time_data.append(t); self.thrust_data.append(th); self.rpm_data.append(r)
        self.temperature_data.append(te); self.voltage_data.append(v); self.current_data.append(c)
        self.power_data.append(p); self.throttle_data.append(thr); self.timestamp_data.append(datetime.now().strftime("%H:%M:%S"))
        # Update labels
        self.timestamp_label.setText(f"Time: {t:.2f} s")
        self.thrust_value_label.setText(f"Thrust: {th:.2f} g")
        self.rpm_value_label.setText(f"RPM: {r:.1f}")
        self.temp_value_label.setText(f"Temp: {te:.1f} °C")
        self.voltage_value_label.setText(f"Voltage: {v:.2f} V")
        self.current_value_label.setText(f"Current: {c:.2f} A")
        self.throttle_value_label.setText(f"Throttle: {thr:.1f} %")
        # Update plots
        for plot, data in zip([self.thrust_plot, self.rpm_plot, self.temp_plot, self.voltage_plot,
                               self.current_plot, self.power_plot],
                              [self.thrust_data, self.rpm_data, self.temperature_data,
                               self.voltage_data, self.current_data, self.power_data]):
            if plot['visible']:
                plot['curve'].setData(self.time_data, data)

    def toggle_plot(self, measure, state):
        visible = state == 2
        if measure in self.plots:
            self.plots[measure]['widget'].setVisible(visible)
            self.plots[measure]['visible'] = visible

    def autoscale_plots(self):
        for key in self.plots:
            self.plots[key]['widget'].enableAutoRange(True, True)

    def export_to_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV files (*.csv)")
        if not path:
            return
        headers = ['Timestamp','Time','Thrust','RPM','Temp','Voltage','Current','Power','Throttle']
        try:
            with open(path,'w',newline='') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                for i in range(len(self.time_data)):
                    writer.writerow([self.timestamp_data[i], self.time_data[i], self.thrust_data[i],
                                     self.rpm_data[i], self.temperature_data[i], self.voltage_data[i],
                                     self.current_data[i], self.power_data[i], self.throttle_data[i]])
            QMessageBox.information(self, "Export Complete", f"Data saved to {path}")
        except Exception as e:
            QMessageBox.critical(self, "Export Error", str(e))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = ThrustStandGUI()
    gui.show()
    sys.exit(app.exec_())
