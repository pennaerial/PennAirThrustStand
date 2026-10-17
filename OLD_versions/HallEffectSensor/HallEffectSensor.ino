//hall effect sensor
const int hall = 5;           // Hall sensor pin
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;

const int magnets = 8;        // Number of magnets per revolution
const unsigned long interval = 1000; // Interval to calculate RPM (ms)

// Interrupt Service Routine (must be defined before use)
void countPulse() {
  pulseCount++;
}

void setup() {
  pinMode(hall, INPUT_PULLUP);  // Use pull-up for stability
  Serial.begin(9600);

  // Attach interrupt to hall sensor pin
  attachInterrupt(digitalPinToInterrupt(hall), countPulse, FALLING);
}

void loop() {
  unsigned long currentTime = millis();

  // Calculate RPM every interval
  if (currentTime - lastTime >= interval) {
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    // Compute RPM
    rpm = (count / (float)magnets) * (60000.0 / interval);

    Serial.print("RPM: ");
    Serial.println(rpm);

    lastTime = currentTime;
  }
}




/* Basic Sensing Magnet*/
/*
const int hall = 5;
int hallVal;


void setup() {
  pinMode(hall, INPUT);
  Serial.begin(9600);

}

void loop() {
  hallVal = digitalRead(hall);
  Serial.println(hallVal);
  
}
*/
