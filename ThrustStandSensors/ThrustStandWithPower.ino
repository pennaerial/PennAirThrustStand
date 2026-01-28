#include <HX711_ADC.h>
#include <DHT.h>

// ======================
// PIN DEFINITIONS
// ======================
const int HX711_DOUT = 0;
const int HX711_SCK  = 1;
const int DHTPIN     = 4;
const int HALL       = 5;

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
// OBJECTS
// ======================
DHT dht(DHTPIN, DHT11);
HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);

// ======================
// RPM
// ======================
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;
const int magnets = 8;
const unsigned long interval = 1000;

// ======================
// LOGGING STATE
// ======================
bool logging = false;

// ======================
// TIMING
// ======================
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

  analogReadResolution(10);
  pinMode(HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL), countPulse, FALLING);

  dht.begin();
  LoadCell.begin();

  Serial.println("System ready");
  Serial.println("Commands: s=start, e=end");
}

// ======================
// LOOP
// ======================
void loop() {
  LoadCell.update();

  if (logging && millis() - lastLog >= LOG_INTERVAL) {
    lastLog = millis();

    float thrust = LoadCell.getData();
    float voltage = readVoltage();
    float current = readCurrent();
    float power = voltage * current;

    // time since logging started (seconds)
    float timestamp = (millis() - startTime) / 1000.0;

    // RPM
    if (millis() - lastTime >= interval) {
      noInterrupts();
      unsigned long count = pulseCount;
      pulseCount = 0;
      interrupts();

      rpm = (count / (float)magnets) * (60000.0 / interval);
      lastTime = millis();
    }

    // ===== CSV OUTPUT =====
    Serial.print(timestamp, 3);
    Serial.print(",");
    Serial.print(thrust, 3);
    Serial.print(",");
    Serial.print(voltage, 3);
    Serial.print(",");
    Serial.print(current, 3);
    Serial.print(",");
    Serial.print(power, 3);
    Serial.print(",");
    Serial.println(rpm, 1);
  }

  // ===== SERIAL COMMANDS =====
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 's') {
      logging = true;
      startTime = millis();  
      lastLog = startTime;   
      Serial.println("time_s,thrust,voltage,current,power,rpm");
    }
    else if (cmd == 'e') {
      logging = false;
      Serial.println("LOGGING STOPPED");
    }
    else if (cmd == 't') LoadCell.tareNoDelay();
    else if (cmd == 'r') calibrate();
    else if (cmd == 'c') changeCalFactor();
  }
}

// ======================
// CALIBRATION
// ======================
void calibrate() {
  Serial.println("Send 't' to tare");

  while (!LoadCell.getTareStatus()) {
    LoadCell.update();
    if (Serial.available() && Serial.read() == 't')
      LoadCell.tareNoDelay();
  }

  Serial.println("Send known mass:");
  float known_mass = 0;

  while (known_mass <= 0) {
    LoadCell.update();
    if (Serial.available())
      known_mass = Serial.parseFloat();
  }

  LoadCell.refreshDataSet();
  float newCal = LoadCell.getNewCalibration(known_mass);
  LoadCell.setCalFactor(newCal);

  Serial.print("New cal factor: ");
  Serial.println(newCal);
}

void changeCalFactor() {
  Serial.println("Send new calibration value:");

  float newCal = 0;
  while (newCal <= 0) {
    if (Serial.available())
      newCal = Serial.parseFloat();
  }

  LoadCell.setCalFactor(newCal);
  Serial.print("Calibration updated: ");
  Serial.println(newCal);
}
