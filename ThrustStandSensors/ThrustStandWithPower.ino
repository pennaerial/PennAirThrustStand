// Load cell, temperature sensor, and RPM measurement
#include <HX711_ADC.h>
#include <DHT.h>

// Pin Definitions
const int HX711_DOUT = 0; // Data output pin from HX711 load cell amplifier
const int HX711_SCK  = 1; // Clock input pin for HX711
const int DHTPIN     = 4; // Digital pin connected to DHT11 temperature/humidity sensor
const int HALL       = 5; // Digital pin connected to hall effect sensor

const int VOLTAGE_PIN = A2; // Analog pin for battery voltage
const int CURRENT_PIN = A1; // Analog pin for battery current

// ADC & Scaling Constants
const float ADC_REF  = 3.3;           // Reference voltage
const int   ADC_RES  = 1023;          // 1023 for 10-bit ADC
const float VOLTAGE_SCALE = 7.07;     // Calibration factor for voltage
const float CURRENT_SCALE = 14;       // Calibration factor for current
const float CURRENT_OFFSET = 0.618;      // Offset for current sensor

// Load Cell, DHT, and RPM initalization
DHT dht(DHTPIN, DHT11);
HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);

volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;
const int magnets = 8;                // Number of magnets per revolution
const unsigned long interval = 1000;  // Interval to calculate RPM (ms)

// Power Measurement Constants
const int NUM_SAMPLES = 100;       // Number of ADC readings to average
float voltage = 0;
float current = 0;

// Timing 
unsigned long t = 0; // For serial printing

void countPulse() {
  pulseCount++; 
}

float readVoltage() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += analogRead(VOLTAGE_PIN);
    float vAdc = (sum / (float)NUM_SAMPLES) * ADC_REF / ADC_RES;
    voltage = vAdc * VOLTAGE_SCALE;
    return voltage;
}

float readCurrent() {
    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) sum += analogRead(CURRENT_PIN);
    float iAdc = (sum / (float)NUM_SAMPLES) * ADC_REF / ADC_RES;
    current = (iAdc * CURRENT_SCALE) + CURRENT_OFFSET;
    return current;
}

void setup() {
  Serial.begin(9600);
  pinMode(HALL, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HALL), countPulse, FALLING);
  analogReadResolution(10);

  dht.begin();
  LoadCell.begin();

  Serial.println("\nStarting...");
  LoadCell.start(2000, true); // stabilizing time, tare

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check HX711 wiring!");
    while (1);
  } else {
    LoadCell.setCalFactor(1.0);
    Serial.println("Startup complete");
  }

  while (!LoadCell.update());
  // calibrate(); // Call manually with 'r' in Serial
}

void loop() {
  static bool newDataReady = false;
  const unsigned long serialPrintInterval = 100; // ms

  // Update load cell
  if (LoadCell.update()) newDataReady = true;

  // Print data if ready and interval elapsed
  if (newDataReady && millis() - t > serialPrintInterval) {
    newDataReady = false;
    t = millis();

    float voltage = readVoltage();
    float current = readCurrent();
    float power = voltage * current;

    // Battery readings
    Serial.print("Voltage: "); Serial.print(voltage, 2);
    Serial.print(" V | Current: "); Serial.print(current, 2);
    Serial.print(" A | Power: "); Serial.print(power, 2); Serial.println(" W");

    // Load cell
    float load = LoadCell.getData();
    Serial.print("Load cell: "); Serial.println(load);

    // Temperature
    float temp = dht.readTemperature();
    if (!isnan(temp)) {
      Serial.print("Temperature: "); Serial.println(temp);
    }

    // RPM
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval) {
      noInterrupts();
      unsigned long count = pulseCount;
      pulseCount = 0;
      interrupts();

      rpm = (count / (float)magnets) * (60000.0 / interval);
      Serial.print("RPM: "); Serial.println(rpm, 1);

      lastTime = currentTime;
    }

    Serial.println();
    delay(200);
  }

  // Serial commands
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
    else if (inByte == 'r') calibrate();
    else if (inByte == 'c') changeCalFactor();
  }

  if (LoadCell.getTareStatus()) {
    Serial.println("Tare complete");
  }
}

// Calibration Functions
void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration: place load cell level and empty. Send 't' to tare.");

  bool _resume = false;
  while (!_resume) {
    LoadCell.update();
    if (Serial.available() > 0 && Serial.read() == 't')
      LoadCell.tareNoDelay();
    if (LoadCell.getTareStatus()) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Place known mass, then send its weight (e.g. 100.0).");
  float known_mass = 0;
  _resume = false;
  while (!_resume) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass > 0) {
        Serial.print("Known mass: "); Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet();
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass);
  Serial.print("New calibration value: "); Serial.println(newCalibrationValue);
  LoadCell.setCalFactor(newCalibrationValue);

  Serial.println("*** Calibration complete ***");
}

void changeCalFactor() {
  float oldCal = LoadCell.getCalFactor();
  Serial.println("***");
  Serial.print("Current calibration value: "); Serial.println(oldCal);
  Serial.println("Send new value (e.g. 696.0):");

  float newCal = 0;
  bool _resume = false;
  while (!_resume) {
    if (Serial.available() > 0) {
      newCal = Serial.parseFloat();
      if (newCal > 0) {
        LoadCell.setCalFactor(newCal);
        Serial.print("New calibration value: "); Serial.println(newCal);
        _resume = true;
      }
    }
  }
  Serial.println("Calibration factor updated.");
  Serial.println("***");
}
