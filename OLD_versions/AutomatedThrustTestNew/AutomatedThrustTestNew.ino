#include <HX711_ADC.h>
#include <DHT.h>
#include <Servo.h>  // For ESC control

// --- Pin assignments ---
const int HX711_dout = 0;
const int HX711_sck = 1;
const int DHTPIN = 4;
const int hall = 5;
const int escPin = 9;  // ESC signal pin

// --- Global variables ---
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;
const int magnets = 8;
const unsigned long interval = 1000;

float temp;
unsigned long lastThrottleChange = 0;
float throttlePercent = 0;
bool motorRunning = false;
bool inProcedure = false;
bool rampingDown = false;

// --- Objects ---
DHT dht(DHTPIN, DHT11);
HX711_ADC LoadCell(HX711_dout, HX711_sck);
Servo esc;

// --- Flags ---
bool headerPrinted = false;

// --- Function prototypes ---
void countPulse();
void calibrate();
void handleSerialInput();
void updateESC();
void changeCalFactor();
void printData();

void setup() {
  Serial.begin(9600);
  pinMode(hall, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hall), countPulse, FALLING);

  dht.begin();
  LoadCell.begin();

  esc.attach(escPin);
  esc.writeMicroseconds(1000);  // Arm ESC
  Serial.println("Arming ESC... please wait 3 seconds");
  delay(3000);

  Serial.println("Starting...");
  LoadCell.start(2000, true);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("HX711 timeout, check wiring!");
    while (1);
  }

  LoadCell.setCalFactor(1.0);
  calibrate();

  Serial.println("ESC ready.");
  Serial.println("Enter a power % (0–100), 'procedure', 'c' to change cal factor, or 's' to stop:");
}

void loop() {
  static bool newDataReady = false;
  static unsigned long lastPrint = 0;
  const unsigned long serialPrintInterval = 100; // ms

  // Update load cell
  if (LoadCell.update()) newDataReady = true;

  // Handle serial commands from GUI
  if (Serial.available() > 0) handleSerialInput();

  // --- Motor data calculations ---
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    rpm = (count / (float)magnets) * (60000.0 / interval);
    temp = dht.readTemperature();
    lastTime = currentTime;
  }

  // --- Incremental ramp procedure ---
  if (inProcedure && !rampingDown && motorRunning && millis() - lastThrottleChange >= 5000) {
    throttlePercent += 5;
    if (throttlePercent > 100) throttlePercent = 100;
    updateESC();
    lastThrottleChange = millis();

    if (throttlePercent == 100) {
      delay(5000); // hold at 100%
      rampingDown = true;
      lastThrottleChange = millis();
    }
  }

  // --- Gradual ramp-down ---
  if (rampingDown && millis() - lastThrottleChange >= 500) {
    throttlePercent -= 5;
    if (throttlePercent <= 0) {
      throttlePercent = 0;
      updateESC();
      inProcedure = false;
      rampingDown = false;
      motorRunning = false;
    } else {
      updateESC();
      lastThrottleChange = millis();
    }
  }

  // --- Update ESC even if motorRunning is false ---
  updateESC();

  // --- Serial output for GUI ---
  if (millis() - lastPrint >= serialPrintInterval) {
    printData();
    lastPrint = millis();
    newDataReady = false;
  }
}

void countPulse() { pulseCount++; }

void updateESC() {
  int signal = map(throttlePercent, 0, 100, 1000, 2000);
  esc.writeMicroseconds(signal);
}

void calibrate() {
  Serial.println("*** Calibration ***");
  Serial.println("Place load cell empty and send 't' to tare.");

  bool _resume = false;
  while (!_resume) {
    LoadCell.update();
    if (Serial.available() > 0 && Serial.read() == 't')
      LoadCell.tareNoDelay();
    if (LoadCell.getTareStatus()) {
      Serial.println("Tare complete.");
      _resume = true;
    }
  }

  Serial.println("Place known mass and send its value (e.g. 100.0):");
  float known_mass = 0;
  while (known_mass == 0) {
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
    }
  }

  LoadCell.refreshDataSet();
  float newCal = LoadCell.getNewCalibration(known_mass);
  LoadCell.setCalFactor(newCal);
  Serial.print("Calibration complete. Factor = ");
  Serial.println(newCal);
  Serial.println("***");
}

void changeCalFactor() {
  float oldCal = LoadCell.getCalFactor();
  Serial.println("***");
  Serial.print("Current calibration value: ");
  Serial.println(oldCal);
  Serial.println("Send new value (e.g. 680.0):");

  float newCal = 0;
  bool _resume = false;
  while (!_resume) {
    if (Serial.available() > 0) {
      newCal = Serial.parseFloat();
      if (newCal > 0) {
        LoadCell.setCalFactor(newCal);
        Serial.print("New calibration value set to: ");
        Serial.println(newCal);
        _resume = true;
      }
    }
  }

  Serial.println("Calibration factor updated.");
  Serial.println("***");
}

void handleSerialInput() {
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.equalsIgnoreCase("s")) {
    throttlePercent = 0;
    updateESC();
    motorRunning = false;
    inProcedure = false;
    rampingDown = false;
    Serial.println("Motor stopped.");
    return;
  }

  if (input.equalsIgnoreCase("procedure")) {
    throttlePercent = 0;
    updateESC();
    motorRunning = true;
    inProcedure = true;
    rampingDown = false;
    lastThrottleChange = millis();
    return;
  }

  if (input.equalsIgnoreCase("c")) {
    changeCalFactor();
    return;
  }

  float val = input.toFloat();
  if (val >= 0 && val <= 100) {
    throttlePercent = val;
    motorRunning = (val > 0);
    inProcedure = false;
    rampingDown = false;
    updateESC();
  }
}

// --- Print structured CSV for GUI ---
void printData() {
  float thrust = LoadCell.getData();
  float voltage = analogRead(A0) * (5.0 / 1023.0); // placeholder
  float current = analogRead(A1) * (5.0 / 1023.0); // placeholder
  unsigned long ms = millis();

  // Print header once
  if (!headerPrinted) {
    Serial.println("time,thrust,rpm,temperature,voltage,current,power,throttle");
    headerPrinted = true;
  }

  float power = voltage * current;

  // Print values
  Serial.print(ms / 1000.0, 2); Serial.print(",");
  Serial.print(thrust, 2); Serial.print(",");
  Serial.print(rpm, 1); Serial.print(",");
  Serial.print(temp, 1); Serial.print(",");
  Serial.print(voltage, 2); Serial.print(",");
  Serial.print(current, 2); Serial.print(",");
  Serial.print(power, 2); Serial.print(",");
  Serial.print(throttlePercent, 1);
  Serial.println();
}
