#include <DHT.h>

#define DHTPIN 2        // Pin connected to DHT22 data pin
#define DHTTYPE DHT22   // Specify DHT22 sensor type

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

void setup() {
  Serial.begin(9600);     // Start serial communication at 9600 baud
  dht.begin();            // Initialize the DHT sensor
}

void loop() {
  // Read temperature and humidity
  float temperatureC = dht.readTemperature();  // Temperature in Celsius
  float humidity = dht.readHumidity();         // Humidity in %

  // Check if readings are valid
  if (isnan(temperatureC) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    // Send temperature and humidity as a comma-separated string
    Serial.print(temperatureC);
    Serial.print(",");
    Serial.println(humidity);
  }
  
  delay(2000); // Update every 2 seconds (DHT22 is slow, minimum 2s interval)
}