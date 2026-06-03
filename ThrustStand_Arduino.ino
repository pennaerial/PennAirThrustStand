#include <HX711_ADC.h>
#include <Servo.h>
#include <ArduinoBLE.h>

// ========================================================
// BLE UART SERVICE (Nordic UART Service UUIDs)
// ========================================================

BLEService uartService("6E400001-B5B3-F393-E0A9-E50E24DCCA9E");
// TX: Arduino → Web app (notify)
BLECharacteristic txChar("6E400003-B5B3-F393-E0A9-E50E24DCCA9E",
                         BLERead | BLENotify, 128);
// RX: Web app → Arduino (write)
BLECharacteristic rxChar("6E400002-B5B3-F393-E0A9-E50E24DCCA9E",
                         BLEWrite | BLEWriteWithoutResponse, 64);
// Internal BLE receive buffer — accumulates incoming bytes
String bleRxBuffer = "";

// ========================================================
// PINS
// ========================================================

const int HX711_DOUT  = 2;
const int HX711_SCK   = 3;
const int ESC_PIN     = 9;
const int VOLTAGE_PIN = A2;
const int CURRENT_PIN = A1;
// const int BATT_TEMP_PIN  = A3; // Commented out per request
const int BATT_VOLTAGE_PIN = A6; // MKR WiFi 1010 internal battery monitor

HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);
Servo esc;

// ========================================================
// CALIBRATION VALUES
// ========================================================

float LOAD_SCALE      = -273.0;
float VOLTAGE_SCALE   = 12.3;
float VOLTAGE_OFFSET  = 0.0;
float CURRENT_SCALE   = 18.96;
float CURRENT_OFFSET  = 0.558;
// float BATT_TEMP_OFFSET  = 0.0; // Commented out per request

// ========================================================
// STATE OF CHARGE (SOC) COULOMB COUNTING VARIABLES
// ========================================================
float batteryCapacity_mAh = 2200.0; // Default capacity configuration
float mAhConsumed         = 0.0;    // Accumulated capacity consumption
float currentSOC          = 100.0;  // Calculated State Of Charge percentage
unsigned long lastSOCUpdate = 0;
const unsigned long SOC_UPDATE_INTERVAL = 200; // Track in steps matching telemetry rates

// ========================================================
// SYSTEM SETTINGS
// ========================================================

const float ADC_REF  = 3.3;
const int   ADC_RES  = 1023;
const int   NUM_SAMPLES = 50;

// ========================================================
// LOGGING
// ========================================================

bool logging = false;
unsigned long lastLog  = 0;
unsigned long startTime = 0;
const unsigned long LOG_INTERVAL = 200;

// ========================================================
// ESC
// ========================================================

const int ESC_MIN = 1000;
const int ESC_MAX = 2000;

float throttlePercent = 0.0;
bool  motorRunning    = false;
bool  inMission       = false;

// ========================================================
// MISSION
// ========================================================

const int MAX_STEPS = 20;
float         missionThrottles[MAX_STEPS];
unsigned long missionDurations[MAX_STEPS];
int           totalMissionSteps = 0;
int           currentStep       = 0;
unsigned long stepStartTime     = 0;
unsigned long currentStepDuration = 0;

// ========================================================
// FORWARD DECLARATIONS
// ========================================================

void emergencyStop();
void printMenu();
float readCurrent();

// ========================================================
// BLE OUTPUT HELPERS
// ========================================================

void bleSend(const String& s) {
  if (!BLE.connected()) return;
  int len = s.length();
  int offset = 0;
  const int CHUNK = 20;
  while (offset < len) {
    int chunkLen = min(len - offset, CHUNK);
    txChar.writeValue((const uint8_t*)(s.c_str() + offset), chunkLen);
    offset += chunkLen;
  }
}

void blePrint(const String& s) {
  Serial.print(s);
  bleSend(s);
}

void blePrint(float v, int digits) {
  String s = String(v, digits);
  Serial.print(s);
  bleSend(s);
}

void blePrint(int v) {
  String s = String(v);
  Serial.print(s);
  bleSend(s);
}

void blePrint(char c) {
  Serial.print(c);
  bleSend(String(c));
}

void blePrintln(const String& s = "") {
  blePrint(s + "\n");
}

void blePrintln(float v, int digits) {
  blePrint(v, digits);
  blePrint("\n");
}

void blePrintln(int v) {
  blePrint(v);
  blePrint("\n");
}

// ========================================================
// BLE INPUT HELPERS
// ========================================================

void pollBLE() {
  BLE.poll();
  if (rxChar.written()) {
    int len = rxChar.valueLength();
    const uint8_t* data = rxChar.value();
    for (int i = 0; i < len; i++) {
      bleRxBuffer += (char)data[i];
    }
  }
}

bool inputAvailable() {
  pollBLE();
  return (Serial.available() > 0) || (bleRxBuffer.length() > 0);
}

char inputRead() {
  if (Serial.available() > 0) return (char)Serial.read();
  if (bleRxBuffer.length() > 0) {
    char c = bleRxBuffer[0];
    bleRxBuffer.remove(0, 1);
    return c;
  }
  return 0;
}

// ========================================================
// SMART, NON-BLOCKING SERIAL/BLE HELPERS
// ========================================================

void clearSerialBuffer() {
  delay(30);
  while (Serial.available() > 0) Serial.read();
  bleRxBuffer = "";
}

char waitForChar() {
  while (true) {
    LoadCell.update();
    pollBLE();

    if (inputAvailable()) {
      char c = inputRead();
      if (c == 'x' || c == 'X') {
        emergencyStop();
        return 'x';
      }
      if (c == '\n' || c == '\r') continue;

      clearSerialBuffer();
      return c;
    }
  }
}

float waitForFloat() {
  String inputString = "";
  while (true) {
    LoadCell.update();
    pollBLE();

    if (inputAvailable()) {
      char c = inputRead();
      if (c == 'x' || c == 'X') {
        emergencyStop();
        return 0.0;
      }
      if (c == '\n' || c == '\r') {
        if (inputString.length() > 0) {
          float val = inputString.toFloat();
          clearSerialBuffer();
          return val;
        }
      } else if (isDigit(c) || c == '.' || c == '-') {
        inputString += c;
      }
    }
  }
}

// ========================================================
// MOTOR & SENSOR FUNCTIONS
// ========================================================

void updateESC() {
  throttlePercent = constrain(throttlePercent, 0.0, 100.0);
  int signal = ESC_MIN;
  if (throttlePercent > 0.0) {
    signal = map((int)throttlePercent, 0, 100, ESC_MIN, ESC_MAX);
  }
  esc.writeMicroseconds(signal);
}

void emergencyStop() {
  throttlePercent = 0;
  updateESC();
  motorRunning = false;
  inMission    = false;
  logging      = false;
  blePrintln("\n*** EMERGENCY STOP TRIGGERED! MOTOR STOPPED ***");
}

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
  return ((iAdc - CURRENT_OFFSET) * CURRENT_SCALE);
}

float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += analogRead(BATT_VOLTAGE_PIN);
  float vAdc = (sum / (float)NUM_SAMPLES) * ADC_REF / ADC_RES;
  return vAdc * 2.0;
}

// COULOMB COUNTING CURRENT INTEGRATION METHOD
void updateSOC() {
  unsigned long currentTime = millis();
  if (lastSOCUpdate == 0) {
    lastSOCUpdate = currentTime;
    return;
  }

  unsigned long durationMs = currentTime - lastSOCUpdate;
  if (durationMs >= SOC_UPDATE_INTERVAL) {
    lastSOCUpdate = currentTime;
    float currentA = readCurrent();
    
    // Ignore fractional negative offset fluctuations when idling
    if (currentA < 0.0) currentA = 0.0;

    // Convert Amperes to Milliamperes, calculate hours elapsed, integrate consumption
    float mA = currentA * 1000.0;
    float hours = (float)durationMs / 3600000.0;
    mAhConsumed += (mA * hours);

    // Compute remaining capacity ratio
    currentSOC = 100.0 - ((mAhConsumed / batteryCapacity_mAh) * 100.0);
    currentSOC = constrain(currentSOC, 0.0, 100.0);
  }
}

void printMenu() {
  blePrintln("\n--- MAIN MENU ---");
  blePrintln(" x = EMERGENCY STOP");
  blePrintln(" 1 = Calibration Menu");
  blePrintln(" 2 = Run Single Motor Command");
  blePrintln(" 3 = Program Mission Sequence");
  blePrintln(" 4 = Run Programmed Mission");
  blePrintln(" s = Start manual logging");
  blePrintln(" e = Stop manual logging");
}

// ========================================================
// CALIBRATION ROUTINES
// ========================================================

void calibrateMenu() {
  blePrintln("\n--- CALIBRATION MENU ---");
  blePrintln(" L = Load Cell");
  blePrintln(" V = Voltage");
  blePrintln(" C = Current");
  blePrintln(" B = Battery Pack Capacity & SOC Setup");
  blePrintln(" E = ESC Limits");
  blePrintln(" O = Return to Main Menu");

  char choice = waitForChar();
  if (choice == 'L' || choice == 'l') {
    blePrintln("\n[LOAD CELL] Ensure tray is empty. Send 't' to tare.");
    while (waitForChar() != 't') {
      blePrintln("Invalid key. Send 't' to tare.");
    }

    blePrint("Taring...");
    LoadCell.tareNoDelay();
    while (!LoadCell.getTareStatus()) LoadCell.update();
    blePrintln(" Complete.");
    blePrintln("\nSelect Method:\n1 = Manual input calibration factor\n2 = Calibrate with known weight");
    int sub = 0;
    while (sub != 1 && sub != 2) {
      sub = (int)waitForFloat();
      if (sub != 1 && sub != 2) blePrintln("Invalid. Enter 1 or 2:");
    }

    if (sub == 1) {
      blePrintln("Enter known Load Cal Factor:");
      LOAD_SCALE = waitForFloat();
      LoadCell.setCalFactor(LOAD_SCALE);
    } else {
      blePrintln("Place weight. Enter its mass in grams (include mount weight if applicable):");
      float known_mass = waitForFloat();
      unsigned long stab = millis();
      while (millis() - stab < 1000) LoadCell.update();
      LoadCell.refreshDataSet();
      LOAD_SCALE = LoadCell.getNewCalibration(known_mass);
      LoadCell.setCalFactor(LOAD_SCALE);
    }
    blePrint("Load Cell Calibration Factor set to: ");
    blePrintln(LOAD_SCALE, 2);
  }
  else if (choice == 'V' || choice == 'v') {
    blePrintln("\n[VOLTAGE] Select Method:\n1 = Manual scale factor input\n2 = Compute from multi-meter reading");
    int sub = (int)waitForFloat();
    if (sub == 1) {
      blePrintln("Enter VOLTAGE_SCALE:");
      VOLTAGE_SCALE = waitForFloat();
    } else {
      blePrintln("Enter the exact voltage measured by your multi-meter:");
      float actualV = waitForFloat();
      float rawV = readVoltage() / VOLTAGE_SCALE;
      VOLTAGE_SCALE = actualV / rawV;
    }
    blePrint("Voltage Scale set to: ");
    blePrintln(VOLTAGE_SCALE, 4);
  }
  else if (choice == 'C' || choice == 'c') {
    blePrintln("\n[CURRENT] Ensure motor is OFF. Taring resting current...");
    long sum = 0;
    for (int i = 0; i < 100; i++) {
      sum += analogRead(CURRENT_PIN);
      delay(2);
    }
    CURRENT_OFFSET = (sum / 100.0) * ADC_REF / ADC_RES;
    blePrint("Resting Current offset set to: ");
    blePrintln(CURRENT_OFFSET, 4);

    blePrintln("\nSelect Method:\n1 = Manual current scale input\n2 = Calibrate using steady running load");
    int sub = (int)waitForFloat();
    if (sub == 1) {
      blePrintln("Enter CURRENT_SCALE:");
      CURRENT_SCALE = waitForFloat();
    } else {
      blePrintln("Run steady load. Enter measured current value in Amps:");
      float actualI = waitForFloat();
      float rawI = (analogRead(CURRENT_PIN) * ADC_REF / ADC_RES) - CURRENT_OFFSET;
      if (rawI == 0) rawI = 0.001;
      CURRENT_SCALE = actualI / rawI;
    }
    blePrint("Current Scale set to: ");
    blePrintln(CURRENT_SCALE, 4);
  }
  else if (choice == 'B' || choice == 'b') {
    blePrintln("\n[BATTERY CONFIG] Enter Battery Capacity in mAh:");
    float inputCap = waitForFloat();
    if (inputCap > 0.0) {
      batteryCapacity_mAh = inputCap;
      blePrint("Capacity registered: ");
      blePrint((int)batteryCapacity_mAh);
      blePrintln(" mAh");
    }
    blePrintln("Reset remaining State of Charge (SOC) to 100%? (y/n)");
    char confirm = waitForChar();
    if (confirm == 'y' || confirm == 'Y') {
      mAhConsumed = 0.0;
      currentSOC  = 100.0;
      blePrintln("Coulomb counter reset. Battery calibrated to 100% SOC.");
    }
  }
  /* // --- TEMPERATURE CONFIGURATION COMMENTED OUT ---
  else if (choice == 'T' || choice == 't') {
    ...
  }
  */
  else if (choice == 'E' || choice == 'e') {
    blePrintln("\n[ESC CAL] !!! REMOVE PROPELLER FOR SAFETY !!!\nSend 'y' to confirm and begin throttle range programming.");
    if (waitForChar() == 'y') {
      blePrintln("Setting high signal (MAX). PLUG IN BATTERY NOW. Wait for double beep, then send 'd'.");
      esc.writeMicroseconds(ESC_MAX);
      while (waitForChar() != 'd');
      esc.writeMicroseconds(ESC_MIN);
      blePrintln("Setting low signal (MIN). Wait for operational initialization tones. Done.");
    }
  }

  printMenu();
}

// ========================================================
// MISSION PROGRAMMING
// ========================================================

void programMission() {
  blePrintln("How many phases in this mission? (Max 20)");
  totalMissionSteps = (int)waitForFloat();
  if (totalMissionSteps > MAX_STEPS) totalMissionSteps = MAX_STEPS;
  for (int i = 0; i < totalMissionSteps; i++) {
    blePrint("Step ");
    blePrint(i + 1);
    blePrintln(" - Enter Throttle % (0-100):");
    missionThrottles[i] = waitForFloat();

    blePrint("Step ");
    blePrint(i + 1);
    blePrintln(" - Enter Duration (seconds):");
    missionDurations[i] = (unsigned long)(waitForFloat() * 1000);
  }
  blePrintln("Mission successfully stored.");
  printMenu();
}

// ========================================================
// MOTOR RUN TRIGGER
// ========================================================

void triggerMotorRun(float throttle, unsigned long durationMs, bool isMissionRun) {
  throttlePercent   = throttle;
  currentStepDuration = durationMs;
  stepStartTime     = millis();
  motorRunning      = true;
  inMission         = isMissionRun;
  if (!logging) {
    logging   = true;
    startTime = millis();
    blePrintln("time,thrust,voltage,current,power,throttle,soc,stand");
  }
  updateESC();
}

// ========================================================
// SETUP
// ========================================================

void setup() {
  Serial.begin(115200);
  unsigned long serialWait = millis();
  while (!Serial && millis() - serialWait < 3000);

  LoadCell.begin();
  LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("HX711 Error: Check data/clock wire connections.");
    while (1);
  }
  LoadCell.setCalFactor(LOAD_SCALE);
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(ESC_MIN);
  Serial.println("Arming ESC...");
  delay(3000);
  if (!BLE.begin()) {
    Serial.println("BLE init failed! Check board.");
    while (1);
  }

  BLE.setLocalName("ThrustStand");
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(txChar);
  uartService.addCharacteristic(rxChar);
  BLE.addService(uartService);
  BLE.advertise();

  Serial.println("BLE advertising as 'ThrustStand'.");
  Serial.println("SYSTEM INITIALIZATION SUCCESSFUL.");
  printMenu();
}

// ========================================================
// LOOP
// ========================================================

void loop() {
  LoadCell.update();
  pollBLE();
  updateSOC(); // Non-blocking capacity tracker

  if (inputAvailable()) {
    char cmd = inputRead();
    if (cmd == 'x' || cmd == 'X') {
      emergencyStop();
      clearSerialBuffer();
    }
    else if (cmd == '1' && !motorRunning) {
      clearSerialBuffer();
      calibrateMenu();
    }
    else if (cmd == '2' && !motorRunning) {
      clearSerialBuffer();
      blePrintln("Enter Throttle %:");
      float t = waitForFloat();
      blePrintln("Enter Duration (seconds):");
      float d = waitForFloat() * 1000;
      blePrintln("Starting Motor...");
      triggerMotorRun(t, (unsigned long)d, false);
    }
    else if (cmd == '3' && !motorRunning) {
      clearSerialBuffer();
      programMission();
    }
    else if (cmd == '4' && !motorRunning) {
      clearSerialBuffer();
      if (totalMissionSteps > 0) {
        blePrintln("Starting Mission Sequence...");
        currentStep = 0;
        triggerMotorRun(missionThrottles[0], missionDurations[0], true);
      } else {
        blePrintln("Error: No mission profile data programmed yet.");
      }
    }
    else if ((cmd == 's' || cmd == 'S') && !logging) {
      clearSerialBuffer();
      logging   = true;
      startTime = millis();
      blePrintln("time,thrust,voltage,current,power,throttle,soc,stand");
    }
    else if ((cmd == 'e' || cmd == 'E') && logging) {
      clearSerialBuffer();
      logging = false;
      blePrintln("Logging stopped. Output saved.");
      printMenu();
    }
  }

  if (motorRunning) {
    if (millis() - stepStartTime >= currentStepDuration) {
      if (inMission) {
        currentStep++;
        if (currentStep >= totalMissionSteps) {
          throttlePercent = 0;
          updateESC();
          motorRunning = false;
          inMission    = false;
          logging      = false;
          blePrintln("\nMission Complete. Operational limits safe.");
          printMenu();
        } else {
          throttlePercent     = missionThrottles[currentStep];
          currentStepDuration = missionDurations[currentStep];
          updateESC();
          stepStartTime = millis();
        }
      } else {
        throttlePercent = 0;
        updateESC();
        motorRunning = false;
        logging      = false;
        blePrintln("\nMotor Run Complete.");
        printMenu();
      }
    }
  }

  if (logging && (millis() - lastLog >= LOG_INTERVAL)) {
    lastLog = millis();
    float thrust  = LoadCell.getData();
    float voltage = readVoltage();
    float current = readCurrent();
    float power   = voltage * current;
    float battVoltage = readBatteryVoltage();
    float t       = (millis() - startTime) / 1000.0;

    // Generated unified 8-column data line matching modifications
    String row = String(t, 2) + "," +
                 String(thrust, 3) + "," +
                 String(voltage, 3) + "," +
                 String(current, 3) + "," +
                 String(power, 3) + "," +
                 String(throttlePercent, 1) + "," +
                 String(currentSOC, 1) + "," +
                 String(battVoltage, 2);

    blePrintln(row);
  }
}
