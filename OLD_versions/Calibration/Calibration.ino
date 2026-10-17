/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/


/*
   Simplified version of the HX711 calibration example.
   This version does NOT use EEPROM for saving calibration values.
*/


#include <HX711_ADC.h>


// Pins
const int HX711_dout = 0; // MCU > HX711 dout pin
const int HX711_sck = 1;  // MCU > HX711 sck pin


// HX711 constructor
HX711_ADC LoadCell(HX711_dout, HX711_sck);


unsigned long t = 0;


void setup() {
  Serial.begin(9600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");


  LoadCell.begin();
  // LoadCell.setReverseOutput(); // Uncomment to invert the output sign
  unsigned long stabilizingtime = 2000;
  bool _tare = true; // Perform tare at startup


  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } else {
    LoadCell.setCalFactor(1.0); // Default calibration value
    Serial.println("Startup complete");
  }


  while (!LoadCell.update());
  calibrate(); // Start calibration procedure
}


void loop() {
  static bool newDataReady = false;
  const int serialPrintInterval = 0;


  if (LoadCell.update()) newDataReady = true;


  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      Serial.print("Load cell output val: ");
      Serial.println(i);
      newDataReady = false;
      t = millis();
    }
  }


  // Serial commands
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();   // Tare
    else if (inByte == 'r') calibrate();         // Calibrate
    else if (inByte == 'c') changeCalFactor();   // Manually change calibration factor
  }


  if (LoadCell.getTareStatus()) {
    Serial.println("Tare complete");
  }
}


void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell on a level surface.");
  Serial.println("Remove all weight from the load cell.");
  Serial.println("Send 't' to tare.");


  bool _resume = false;
  while (!_resume) {
    LoadCell.update();
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 't') LoadCell.tareNoDelay();
    }
    if (LoadCell.getTareStatus()) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }


  Serial.println("Place a known mass on the load cell.");
  Serial.println("Then send the weight of this mass (e.g. 100.0) from Serial Monitor.");


  float known_mass = 0;
  _resume = false;
  while (!_resume) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }


  LoadCell.refreshDataSet();
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass);


  Serial.print("New calibration value: ");
  Serial.println(newCalibrationValue);
  Serial.println("Use this value as LoadCell.setCalFactor() in your sketch.");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from Serial Monitor.");
  Serial.println("To manually edit calibration, send 'c'.");
  Serial.println("***");
}


void changeCalFactor() {
  float oldCal = LoadCell.getCalFactor();
  Serial.println("***");
  Serial.print("Current calibration value: ");
  Serial.println(oldCal);
  Serial.println("Send new value from Serial Monitor (e.g. 696.0)");


  float newCal = 0;
  bool _resume = false;
  while (!_resume) {
    if (Serial.available() > 0) {
      newCal = Serial.parseFloat();
      if (newCal != 0) {
        Serial.print("New calibration value: ");
        Serial.println(newCal);
        LoadCell.setCalFactor(newCal);
        _resume = true;
      }
    }
  }
  Serial.println("Calibration factor updated.");
  Serial.println("***");
}
