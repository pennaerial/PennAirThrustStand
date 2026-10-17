// Measure current with ACS712
// and print on Serial Monitor
const int nSamples = 1000;
const float vcc = 5.0;
const int adcMax = 1023;

const float sens = 0.185;  // 5A
//const float sens = 0.100;  // 20A
//const float sens = 0.66;  // 30A

float avg() {
  float val = 0;
  for (int i = 0; i < nSamples; i++) {
    val += analogRead(A0);
    delay(1);
  }
  return val / adcMax / nSamples;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  float cur = (vcc / 2 - vcc * avg()) / sens;
  Serial.print("Current:");
  Serial.println(cur);
}
