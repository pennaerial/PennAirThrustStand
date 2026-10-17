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
unsigned long t = 0;
float throttlePercent = 0;
unsigned long lastThrottleChange = 0;
bool motorRunning = false;
bool inProcedure = false;
bool rampingDown = false;

// --- Objects ---
DHT dht(DHTPIN, DHT11);
HX711_ADC LoadCell(HX711_dout, HX711_sck);
Servo esc;

// --- Function prototypes ---
void countPulse();
void calibrate();
void handleSerialInput();
void updateESC();

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
  Serial.println("Enter a power % (0–100), 'procedure' for ramp test, or 's' to stop:");
}

void loop() {
  static bool newDataReady = false;
  const unsigned long serialPrintInterval = 100; // ms

  if (LoadCell.update()) newDataReady = true;
  if (Serial.available() > 0) handleSerialInput();

  // --- Print data only if motor is running ---
  if (motorRunning && newDataReady && millis() - t > serialPrintInterval) {
    newDataReady = false;
    t = millis();

    Serial.println();
    Serial.print("Load: "); Serial.println(LoadCell.getData());
    temp = dht.readTemperature();
    if (!isnan(temp)) {
      Serial.print("Temp: "); Serial.println(temp);
    }

    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval) {
      noInterrupts();
      unsigned long count = pulseCount;
      pulseCount = 0;
      interrupts();

      rpm = (count / (float)magnets) * (60000.0 / interval);
      Serial.print("RPM: ");
      Serial.println(rpm, 1);
      lastTime = currentTime;
    }
  }

  // --- Incremental up procedure ---
  if (inProcedure && !rampingDown && motorRunning && millis() - lastThrottleChange >= 5000) {
    throttlePercent += 5;
    if (throttlePercent > 100) throttlePercent = 100;
    updateESC();
    Serial.print("Throttle (%): ");
    Serial.println(throttlePercent);
    Serial.println("%");
    lastThrottleChange = millis();

    // Hold at 100% for 5 seconds, then start ramp down
    if (throttlePercent == 100) {
      Serial.println("Reached 100% — holding for 5 seconds...");
      delay(5000);
      rampingDown = true;
      Serial.println("Starting gradual ramp-down...");
      lastThrottleChange = millis();
    }
  }

  // --- Gradual ramp-down procedure ---
  if (rampingDown && millis() - lastThrottleChange >= 500) {
    throttlePercent -= 5;
    if (throttlePercent <= 0) {
      throttlePercent = 0;
      updateESC();
      Serial.print("[Ramp-Down] Throttle decreased to ");
      Serial.print(throttlePercent);
      Serial.println("%");
      inProcedure = false;
      rampingDown = false;
      motorRunning = false;
      Serial.println("Ramp-down complete. Motor stopped.");
      Serial.println("Enter a power % (0–100), 'procedure', or 's' to stop:");
      return;
    }
    updateESC();
    Serial.print("[Ramp-Down] Throttle decreased to ");
    Serial.print(throttlePercent);
    Serial.println("%");
    lastThrottleChange = millis();
  }
}

void countPulse() { pulseCount++; }

void handleSerialInput() {

  
  // Read characters IMMEDIATELY as they arrive (like calibrate)
  if (Serial.available() > 0) {
    char c = Serial.read();

    // --- Stop motor ---
    if (c == 's' || c == 'S') {
      throttlePercent = 0;
      updateESC();
      motorRunning = false;
      inProcedure = false;
      rampingDown = false;

      Serial.println("Motor stopped.");
      Serial.println("Enter power % (0–100), 'procedure', 'c', or 's':");
      return;
    }

    // --- Start automatic throttle procedure ---
    if (c == 'p' || c == 'P') {   // single-char like calibrate 't'
      Serial.println("Starting 5-second interval ramp procedure...");
      throttlePercent = 0;
      updateESC();
      motorRunning = true;
      inProcedure = true;
      rampingDown = false;
      lastThrottleChange = millis();

      Serial.println("Throttle at 0%, increasing by 5% every 5 seconds until 100%.");
      return;
    }

    // --- Calibration factor change ---
    if (c == 'c' || c == 'C') {
      changeCalFactor();
      return;
    }

    // --- If char is a digit, begin reading a number like calibrate() ---
    if (isDigit(c)) {
      String buffer = "";
      buffer += c;

      // read remaining digits or decimal point if they come
      delay(5);
      while (Serial.available() > 0) {
        char d = Serial.read();
        if (isDigit(d) || d == '.') {
          buffer += d;
        } else {
          break;
        }
      }

      float val = buffer.toFloat();
      if (val >= 0 && val <= 100) {
        throttlePercent = val;
        motorRunning = (val > 0);
        inProcedure = false;
        rampingDown = false;
        updateESC();

        Serial.print("Throttle (%): ");
        Serial.println(throttlePercent);
      } else {
        Serial.println("Invalid value. Must be 0–100.");
      }
      return;
    }

    // Unknown input
    Serial.println("Invalid input. Enter 0–100, 'p', 'c', or 's'.");
  }
}


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
