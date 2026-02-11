#include <HX711_ADC.h>
#include <DHT.h>
#include <Servo.h>

// ======================
// PIN DEFINITIONS
// ======================
const int HX711_DOUT = 0;
const int HX711_SCK  = 1;
const int DHTPIN     = 4;
const int HALL       = 5;
const int ESC_PIN    = 9;

const int VOLTAGE_PIN = A2;
const int CURRENT_PIN = A1;

// ======================
// ADC + SCALING
// ======================
const float ADC_REF = 3.3;
const int ADC_RES = 1023;
const int NUM_SAMPLES = 100;

const float VOLTAGE_SCALE = 12.3;
const float CURRENT_SCALE = 18.96;
const float CURRENT_OFFSET = 0.558;
const float VOLTAGE_OFFSET = -2.37;

// ======================
// THROTTLE PROCEDURE
// ======================
const int NUM_STEPS = 3;
float throttleSteps[NUM_STEPS] = {1.0, 5.0};
unsigned long stepDurations[NUM_STEPS] = {10000, 5000};

// ======================
// OBJECTS
// ======================
DHT dht(DHTPIN, DHT11);
HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);
Servo esc;

// ======================
// RPM
// ======================
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;
const int magnets = 8;
const unsigned long interval = 1000;

// ======================
// STATE
// ======================
float throttlePercent = 0.0;
bool motorRunning = false;
bool inProcedure = false;
unsigned long stepStartTime = 0;
int currentStep = 0;

// ======================
// LOGGING
// ======================
bool logging = false;
unsigned long lastLog = 0;
unsigned long startTime = 0;
const unsigned long LOG_INTERVAL = 200;

// ======================
// INTERRUPT
// ======================
void countPulse() {
  pulseCount++;
}

// ======================
// ESC OUTPUT
// ======================
void updateESC() {
  throttlePercent = constrain(throttlePercent, 0.0, 100.0);
  esc.writeMicroseconds(1000 + throttlePercent * 10);
}

// ======================
// SENSOR READS
// ======================
float readVoltage() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += analogRead(VOLTAGE_PIN);
  float vAdc = (sum / (float)NUM_SAMPLES) * ADC_REF / ADC_RES;
  return (vAdc * VOLTAGE_SCALE) + VOLTAGE_OFFSET;
}

float readCurrent() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += analogRead(CURRENT_PIN);
  float iAdc = (sum / (float)NUM_SAMPLES) * ADC_REF / ADC_RES;
  return (iAdc * CURRENT_SCALE) + CURRENT_OFFSET;
}

// ======================
// CALIBRATION FUNCTIONS
// ======================
void calibrate() {
  Serial.println("*** LOAD CELL CALIBRATION ***");
  Serial.println("Send 't' to tare (no load)");

  bool ready = false;
  while (!ready) {
    LoadCell.update();
    if (Serial.available() && Serial.read() == 't')
      LoadCell.tareNoDelay();
    if (LoadCell.getTareStatus()) ready = true;
  }

  Serial.println("Place known mass and send its value (grams)");

  float known_mass = 0;
  while (known_mass <= 0) {
    LoadCell.update();
    if (Serial.available())
      known_mass = Serial.parseFloat();
  }

  LoadCell.refreshDataSet();
  float cal = LoadCell.getNewCalibration(known_mass);
  LoadCell.setCalFactor(cal);

  Serial.print("Calibration factor set to: ");
  Serial.println(cal);
  Serial.println("*** DONE ***");
}

void changeCalFactor() {
  Serial.print("Current cal factor: ");
  Serial.println(LoadCell.getCalFactor());
  Serial.println("Send new calibration value:");

  float newCal = 0;
  while (newCal <= 0) {
    if (Serial.available())
      newCal = Serial.parseFloat();
  }

  LoadCell.setCalFactor(newCal);
  Serial.print("Updated cal factor: ");
  Serial.println(newCal);
}

// ======================
// SETUP
// ======================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL), countPulse, FALLING);

  dht.begin();
  LoadCell.begin();

  unsigned long stabilizingtime = 2000;
  bool tare = true;
  LoadCell.start(stabilizingtime, tare);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("HX711 timeout — check wiring");
    while (1);
  }

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000);
  Serial.println("Arming ESC...");
  delay(3000);

  Serial.println("READY");
  Serial.println("Commands:");
  Serial.println(" s = start logging");
  Serial.println(" e = stop logging");
  Serial.println(" procedure = throttle sweep");
  Serial.println(" x = motor stop");
  Serial.println(" t = tare load cell");
  Serial.println(" r = recalibrate load cell");
  Serial.println(" c = change calibration factor");
}

// ======================
// LOOP
// ======================
void loop() {
  LoadCell.update();

  // ----- PROCEDURE -----
  if (inProcedure && millis() - stepStartTime >= stepDurations[currentStep]) {
    currentStep++;
    if (currentStep >= NUM_STEPS) {
      throttlePercent = 0;
      updateESC();
      inProcedure = false;
    } else {
      throttlePercent = throttleSteps[currentStep];
      updateESC();
      stepStartTime = millis();
    }
  }

  // ----- LOGGING -----
  if (logging && millis() - lastLog >= LOG_INTERVAL) {
    lastLog = millis();

    float thrust = LoadCell.getData();
    float voltage = readVoltage();
    float current = readCurrent();
    float power = voltage * current;

    if (millis() - lastTime >= interval) {
      noInterrupts();
      unsigned long count = pulseCount;
      pulseCount = 0;
      interrupts();
      rpm = (count / (float)magnets) * (60000.0 / interval);
      lastTime = millis();
    }

    float t = (millis() - startTime) / 1000.0;

    Serial.print(t, 2); Serial.print(",");
    Serial.print(thrust, 3); Serial.print(",");
    Serial.print(voltage, 3); Serial.print(",");
    Serial.print(current, 3); Serial.print(",");
    Serial.print(power, 3); Serial.print(",");
    Serial.print(rpm, 1); Serial.print(",");
    Serial.println(throttlePercent, 1);
  }

  // ----- SERIAL -----
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "s") {
      logging = true;
      startTime = millis();
      Serial.println("time,thrust,voltage,current,power,rpm,throttle");
    } else if (cmd == "e") logging = false;
    else if (cmd == "procedure") {
      currentStep = 0;
      throttlePercent = throttleSteps[0];
      updateESC();
      stepStartTime = millis();
      inProcedure = true;
    } else if (cmd == "x") {
      throttlePercent = 0;
      updateESC();
      inProcedure = false;
    } else if (cmd == "t") LoadCell.tareNoDelay();
    else if (cmd == "r") calibrate();
    else if (cmd == "c") changeCalFactor();
  }
}