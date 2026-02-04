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

const float VOLTAGE_SCALE = 12.3; // Old: 10.57
const float CURRENT_SCALE = 18.96; // Old: 19.2
const float CURRENT_OFFSET = 0.558; // Old: 0.618
const float VOLTAGE_OFFSET = -2.37; // Old: 0

// ======================
// CUSTOM THROTTLE SEQUENCE
// ======================
const int NUM_STEPS = 3;
float throttleSteps[NUM_STEPS] = {100.0, 40.0, 100.0};
unsigned long stepDurations[NUM_STEPS] = {10000, 45000, 5000}; // ms

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
// ESC STATE
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
  int pwm = 1000 + (throttlePercent / 100.0) * 1000;
  esc.writeMicroseconds(pwm);
}

// ======================
// SENSOR READS
// ======================
float readVoltage() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++)
    sum += analogRead(VOLTAGE_PIN);

  float vAdc = (sum / (float)NUM_SAMPLES) * ADC_REF / ADC_RES;
  return (vAdc * VOLTAGE_SCALE) + VOLTAGE_OFFSET;
}

float readCurrent() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++)
    sum += analogRead(CURRENT_PIN);

  float iAdc = (sum / (float)NUM_SAMPLES) * ADC_REF / ADC_RES;
  return (iAdc * CURRENT_SCALE) + CURRENT_OFFSET;
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

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000);
  Serial.println("Arming ESC...");
  delay(3000);

  Serial.println("System ready");
  Serial.println("Commands:");
  Serial.println(" s = start logging");
  Serial.println(" e = stop logging");
  Serial.println(" procedure = run throttle sequence");
  Serial.println(" x = motor stop");
}

// ======================
// LOOP
// ======================
void loop() {
  LoadCell.update();

  // ===== THROTTLE SEQUENCE =====
  if (inProcedure) {
    if (millis() - stepStartTime >= stepDurations[currentStep]) {
      currentStep++;

      if (currentStep >= NUM_STEPS) {
        throttlePercent = 0;
        updateESC();
        inProcedure = false;
        motorRunning = false;
        Serial.println("Procedure complete");
      } else {
        throttlePercent = throttleSteps[currentStep];
        updateESC();
        stepStartTime = millis();
      }
    }
  }

  // ===== LOGGING =====
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

    Serial.print(t, 3); Serial.print(",");
    Serial.print(thrust, 3); Serial.print(",");
    Serial.print(voltage, 3); Serial.print(",");
    Serial.print(current, 3); Serial.print(",");
    Serial.print(power, 3); Serial.print(",");
    Serial.print(rpm, 1); Serial.print(",");
    Serial.println(throttlePercent, 1);
  }

  // ===== SERIAL =====
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "s") {
      logging = true;
      startTime = millis();
      lastLog = startTime;
      Serial.println("time,thrust,voltage,current,power,rpm,throttle");
    }
    else if (cmd == "e") {
      logging = false;
      Serial.println("LOGGING STOPPED");
    }
    else if (cmd == "procedure") {
      currentStep = 0;
      throttlePercent = throttleSteps[0];
      updateESC();
      stepStartTime = millis();
      inProcedure = true;
      motorRunning = true;
      Serial.println("Throttle sequence started");
    }
    else if (cmd == "x") {
      throttlePercent = 0;
      updateESC();
      motorRunning = false;
      inProcedure = false;
      Serial.println("Motor stopped");
    }
  }
}