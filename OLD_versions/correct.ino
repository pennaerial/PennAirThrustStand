#include <HX711_ADC.h>
#include <DHT.h>
#include <Servo.h>

// ===============================
// Pin Assignments
// ===============================
const int HX711_dout = 0;
const int HX711_sck  = 1;
const int DHTPIN     = 4;
const int hall       = 5;
const int escPin     = 9;

// ===============================
// Global Variables
// ===============================
volatile unsigned long pulseCount = 0;
unsigned long lastRPMCalc = 0;
float rpm = 0;
const int magnets = 8;
const unsigned long rpmInterval = 1000;

float temperature = 0;

// Throttle control
float throttlePercent = 0.0;
unsigned long lastThrottleChange = 0;
bool motorRunning = false;

// Auto-procedure state
bool inProcedure = false;
bool rampingDown = false;

// Flags
bool headerPrinted = false;

// ===============================
// Objects
// ===============================
DHT dht(DHTPIN, DHT11);
HX711_ADC LoadCell(HX711_dout, HX711_sck);
Servo esc;


// ======================================================
// INTERRUPT: RPM Pulse Counter
// ======================================================
void countPulse() {
  pulseCount++;
}


// ======================================================
// ESC Output (float-accurate PWM, no map())
// ======================================================
void updateESC() {
  throttlePercent = constrain(throttlePercent, 0.0, 100.0);
  int signal = (int)(1000 + (throttlePercent / 100.0) * 1000);
  esc.writeMicroseconds(signal);
}


// ======================================================
// Calibration routine (manual only)
// ======================================================
void calibrate() {
  Serial.println("*** Calibration Mode ***");
  Serial.println("1) Remove all weight and send 't' to tare.");

  bool done = false;
  while (!done) {
    LoadCell.update();
    if (Serial.available() > 0 && Serial.read() == 't') {
      LoadCell.tareNoDelay();
    }
    if (LoadCell.getTareStatus()) {
      Serial.println("Tare complete.");
      done = true;
    }
  }

  Serial.println("2) Place known mass and send its value (e.g. '100' for 100g).");

  float known_mass = 0;
  while (known_mass <= 0) {
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
    }
  }

  LoadCell.refreshDataSet();
  float newCal = LoadCell.getNewCalibration(known_mass);
  LoadCell.setCalFactor(newCal);

  Serial.print("Calibration done. New cal factor = ");
  Serial.println(newCal);
  Serial.println("***");
}


// ======================================================
// Change calibration factor manually
// ======================================================
void changeCalFactor() {
  Serial.println("*** Change Calibration Factor ***");
  Serial.print("Current Factor: ");
  Serial.println(LoadCell.getCalFactor());
  Serial.println("Send new factor:");

  float newCal = 0;
  while (newCal <= 0) {
    if (Serial.available() > 0) {
      newCal = Serial.parseFloat();
    }
  }

  LoadCell.setCalFactor(newCal);

  Serial.print("New calibration factor set: ");
  Serial.println(newCal);
  Serial.println("***");
}


// ======================================================
// Handle main serial input
// ======================================================
void handleSerialInput(String input) {
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
    Serial.println("Auto-throttle procedure started.");
    return;
  }

  if (input.equalsIgnoreCase("c")) {
    changeCalFactor();
    return;
  }

  // Manual throttle 0–100
  float val = input.toFloat();
  if (val >= 0 && val <= 100) {
    throttlePercent = val;
    motorRunning = (val > 0);
    inProcedure = false;
    rampingDown = false;
    updateESC();
  }
}


// ======================================================
// SETUP (Calibration removed from startup)
// ======================================================
void setup() {
  Serial.begin(9600);

  pinMode(hall, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hall), countPulse, FALLING);

  dht.begin();
  LoadCell.begin();

  esc.attach(escPin);
  esc.writeMicroseconds(1000);
  Serial.println("Arming ESC...");
  delay(3000);

  Serial.println("Initializing load cell...");
  LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("HX711 timeout. Check wiring!");
    while (1);
  }

  LoadCell.setCalFactor(100.0);  // placeholder default (user calibrates manually)

  Serial.println("Setup complete.");
  Serial.println("Commands:");
  Serial.println("   cal       → run calibration");
  Serial.println("   c         → change calibration factor");
  Serial.println("   procedure → auto throttle test");
  Serial.println("   s         → stop motor");
  Serial.println("   0–100     → manual throttle %");
}



// ======================================================
// Main Loop
// ======================================================
void loop() {
  static unsigned long lastPrint = 0;
  const unsigned long serialPrintInterval = 100;

  LoadCell.update();

  // Handle serial commands safely
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("cal")) {
      calibrate();
    } else {
      handleSerialInput(input);
    }
  }

  // RPM calculation (every 1 sec)
  if (millis() - lastRPMCalc >= rpmInterval) {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    rpm = (count / (float)magnets) * (60000.0 / rpmInterval);
    temperature = dht.readTemperature();
    lastRPMCalc = millis();
  }


  // ===============================
  // AUTO PROCEDURE: RAMP UP
  // ===============================
  if (inProcedure && !rampingDown) {
    if (millis() - lastThrottleChange >= 5000) {
      throttlePercent += 5;

      if (throttlePercent >= 100) {
        throttlePercent = 100;
        updateESC();
        Serial.println("At 100%, holding...");

        delay(5000);

        rampingDown = true;
      } else {
        updateESC();
      }

      lastThrottleChange = millis();
    }
  }

  // ===============================
  // AUTO PROCEDURE: RAMP DOWN
  // ===============================
  if (rampingDown) {
    if (millis() - lastThrottleChange >= 500) {
      throttlePercent -= 5;

      if (throttlePercent <= 0) {
        throttlePercent = 0;
        updateESC();
        motorRunning = false;
        inProcedure = false;
        rampingDown = false;

        Serial.println("Procedure complete.");
      } else {
        updateESC();
      }

      lastThrottleChange = millis();
    }
  }

  // ===============================
  // Periodic Output for GUI
  // ===============================
  if (millis() - lastPrint >= serialPrintInterval) {
    printData();
    lastPrint = millis();
  }
}


// ======================================================
// Print structured CSV for GUI
// ======================================================
void printData() {
  float thrust = LoadCell.getData();
  float voltage = analogRead(A0) * (5.0 / 1023.0);
  float current = analogRead(A1) * (5.0 / 1023.0);
  float power   = voltage * current;

  unsigned long ms = millis();

  if (!headerPrinted) {
    Serial.println("time,thrust,rpm,temperature,voltage,current,power,throttle");
    headerPrinted = true;
  }

  Serial.print(ms / 1000.0, 2); Serial.print(",");
  Serial.print(thrust, 2);      Serial.print(",");
  Serial.print(rpm, 1);          Serial.print(",");
  Serial.print(temperature, 1);  Serial.print(",");
  Serial.print(voltage, 2);      Serial.print(",");
  Serial.print(current, 2);      Serial.print(",");
  Serial.print(power, 2);        Serial.print(",");
  Serial.print(throttlePercent, 1);
  Serial.println();
}

