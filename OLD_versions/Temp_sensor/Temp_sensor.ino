/*
DHT sensor with serial monitor
*/

#include <DHT.h>
#define DHTPIN 4

DHT dht(DHTPIN, DHT11);
float h;
float t;
void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);
  h = dht.readHumidity();
  t = dht.readTemperature();
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  Serial.print("Temperature: ");
  Serial.println(t);

  delay(100);
  
  Serial.print("Humidity: ");
  Serial.println(h);
}