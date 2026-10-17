# Older versions — not current

This folder is a passdown, not current state. Nothing in it is wired up or run anymore.

- `ThrustStand_Arduino.ino`, `ThrustStand_Arduino_Nano_Update.ino`, `ThrustStand_WebInterface.html` —
  earlier iterations of the BLE thrust stand sketch/web pair, before the pin remap and fixes that
  became `10_10_ThrustStand.ino` / `10_10_WebInterface.html` at the repo root.
- `thrust_gui.py`, `serial_reader.py`, `serial_reader_OLD.py`, `AutomatedGui.py`, `README_GUI.md`,
  `OLD_README.md`, `requirements.txt` — the original PyQt5/serial GUI stack, from before the
  project moved to the browser-based BLE interface.
- Everything else here (`AttemptVoltageSensor/`, `Calibration/`, `HallEffectSensor/`,
  `ThrustStandSensors/`, the extension-less test sketches, etc.) is single-sensor or ESP32-based
  exploration that predates the consolidated thrust stand sketch.

The active sketch and web interface are `10_10_ThrustStand.ino` and `10_10_WebInterface.html`
at the repository root.
