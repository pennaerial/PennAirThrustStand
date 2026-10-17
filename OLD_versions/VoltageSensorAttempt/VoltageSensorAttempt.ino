const int sensorPin = A2;
float sensorValue = 0;
float voltage = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue = analogRead(sensorPin);

  // Convert ADC value (0-1023) to voltage (0-5V)
  float sensedVoltage = (sensorValue * 5.0) / 1023.0;

  // Convert 0-5V range to 0-25V range of the measured input
  voltage = sensedVoltage * 5.0;  // Because module divides by 5

  Serial.print("Measured Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  delay(500);
}