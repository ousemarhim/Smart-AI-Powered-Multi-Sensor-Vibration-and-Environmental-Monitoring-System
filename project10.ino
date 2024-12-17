// Include Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include "DHT.h"
#include <WiFi.h>
#include <HTTPClient.h>

// Define Sensor Pins and Constants
#define DHTPIN 15
#define DHTTYPE DHT22
#define SW420_PIN 13
#define BUZZER_PIN 27

// ThingSpeak Settings
const char* ssid = "iPhone de ousama";
const char* password = "baboucha43210";
String apiKey = "8RWXR51E23QZ8Z28";
const char* server = "http://api.thingspeak.com/update";

// Sensor Objects
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
DHT dht(DHTPIN, DHTTYPE);

// Variables
float xAccel, yAccel, zAccel;
float temperature, humidity, pressure;
bool shockDetected = false;

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected!");

  // Initialize Sensors
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println("BMP180 initialization failed!");
    while (1);
  }

  dht.begin();

  // Set up IO pins
  pinMode(SW420_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  // Read MPU6050 Data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  xAccel = a.acceleration.x;
  yAccel = a.acceleration.y;
  zAccel = a.acceleration.z;

  // Read BMP180 Data
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0; // Convert to hPa

  // Read DHT22 Data
  humidity = dht.readHumidity();
  float tempDHT = dht.readTemperature();
  if (!isnan(tempDHT)) {
    temperature = tempDHT; // Override BMP temp with DHT if valid
  }

  // Read SW-420 Shock Sensor
  shockDetected = digitalRead(SW420_PIN);

  // Trigger Buzzer on Shock
  if (shockDetected) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Print Sensor Data to Serial Monitor
  Serial.println("--- Sensor Data ---");
  Serial.print("X: "); Serial.println(xAccel);
  Serial.print("Y: "); Serial.println(yAccel);
  Serial.print("Z: "); Serial.println(zAccel);
  Serial.print("Temperature: "); Serial.println(temperature);
  Serial.print("Humidity: "); Serial.println(humidity);
  Serial.print("Pressure: "); Serial.println(pressure);
  Serial.print("Shock Detected: "); Serial.println(shockDetected);

  // Send Data to ThingSpeak
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(server) + "?api_key=" + apiKey +
                 "&field1=" + String(xAccel) +
                 "&field2=" + String(yAccel) +
                 "&field3=" + String(zAccel) +
                 "&field4=" + String(temperature) +
                 "&field5=" + String(humidity) +
                 "&field6=" + String(pressure) +
                 "&field7=" + String(shockDetected);

    http.begin(url);
    int httpCode = http.GET();
    if (httpCode > 0) {
      Serial.println("Data sent to ThingSpeak: " + url);
    } else {
      Serial.println("Error in HTTP request.");
    }
    http.end();
  }

  // Wait before next reading
  delay(2000);
}
