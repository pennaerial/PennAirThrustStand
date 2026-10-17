const int sensorPin = A0;
const int zeroButton = 7;

// Change this based on your module:
// ACS712 5A: 185 mV/A
// ACS712 20A: 100 mV/A
// ACS712 30A: 66 mV/A
float mVperAmp = 66;  // <--- SET THIS for your module

float zeroOffset = 512;  // mid-point default for no current

void setup() {
  Serial.begin(9600);
  pinMode(zeroButton, INPUT_PULLUP);

  Serial.println("ACS712 Current Sensor Ready");
  Serial.println("Press button or type 'z' in Serial Monitor to zero.");
}

void loop() {
  // Serial zero command
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      calibrateZero();
    }
  }

  // Button zero
  if (digitalRead(zeroButton) == LOW) {
    calibrateZero();
  }

  // Read current
  float sensorValue = analogRead(sensorPin);
  float voltage = (sensorValue * 5.0) / 1023.0;    // ADC → Voltage
  float offsetVoltage = (zeroOffset * 5.0) / 1023.0;
  float voltageDiff = voltage - offsetVoltage;     // difference from zero
  float current = (voltageDiff * 1000) / mVperAmp; // mA → A

  Serial.print("Current: ");
  Serial.print(current, 3);
  Serial.println(" A");

  delay(100);
}

void calibrateZero() {
  Serial.println("Zeroing... keep wires open, no current flowing.");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    sum += analogRead(sensorPin);
    delay(2);
  }
  zeroOffset = sum / 500.0;
  Serial.print("Zero Offset Set To: ");
  Serial.println(zeroOffset);
}
