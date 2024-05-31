#include "secret.h"
#include <BlynkSimpleWifi.h>
#include <Blynk.h>
#include <WiFiS3.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

// Motor Pins
#define M1F 6
#define M1R 4
#define M2F 5
#define M2R 3

// GPS
const int RXPin = 1;
const int TXPin = 0;
const int GPSBaud = 9600;
SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps;

float LONG = 0.0;
float LAT = 0.0;
float speed = 0.0;

// WiFi
#define BLYNK_PRINT Serial
BlynkTimer timer;

// TOF10120 sensor
const int TOF_ADDRESS = 0x52;
float distance;

// SERVO
Servo servo;

int sensorCounter = 0;
bool newGPSData = false;

// Function to connect to WiFi
void connectToWiFi() {
  Serial.print("Connecting to ");
  Serial.println(wifiCredentials[0].ssid);
  WiFi.begin(wifiCredentials[0].ssid, wifiCredentials[0].password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nConnection failed, check your credentials or reset the board.");
  }
}

// Function to read GPS data
void readGPS() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      newGPSData = true;
    }
  }

  if (newGPSData && gps.location.isValid()) {
    LAT = gps.location.lat();
    LONG = gps.location.lng();
    speed = gps.speed.knots();

    // Debug output
    Serial.print("Latitude: ");
    Serial.print(LAT, 6);
    Serial.print(", Longitude: ");
    Serial.print(LONG, 6);
    Serial.print(", Speed: ");
    Serial.println(speed);

    // Send GPS data to Blynk
    Blynk.virtualWrite(V4, String(LAT, 6) + "," + String(LONG, 6));  // Send latitude and longitude together
    Blynk.virtualWrite(V1, speed);  // Send speed

    newGPSData = false;
  } else {
    Serial.println("Waiting for valid GPS signal...");
  }
}

// Function to read TOF sensor data
void readTOFSensor() {
  Wire.beginTransmission(TOF_ADDRESS);
  Wire.write(0x00);  // Request the distance measurement
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("I2C Error: ");
    Serial.println(error);
    Serial.println("Failed to read from TOF sensor");
    return;
  }

  Wire.requestFrom(TOF_ADDRESS, 2);  // Request 2 bytes from the sensor
  if (Wire.available() == 2) {
    uint16_t distanceRaw = Wire.read();  // Read the first byte
    distanceRaw |= Wire.read() << 8;  // Read the second byte and combine

    distance = distanceRaw / 10.0;  // Convert to cm and store in the float variable

    Serial.print("Distance: ");
    Serial.println(distance);

    // Send distance data to Blynk
    Blynk.virtualWrite(V5, distance);
  } else {
    Serial.println("Failed to read from TOF sensor");
  }
}

// Function to read sensors alternatively
void readSensors() {
  if (sensorCounter % 2 == 0) {
    readGPS();
  } else {
    readTOFSensor();
  }

  // Increment the counter
  sensorCounter++;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial started at 115200");
  gpsSerial.begin(GPSBaud);
  Serial.println("GPS Serial started at 9600");
  connectToWiFi();
  Blynk.begin(BLYNK_AUTH_TOKEN, wifiCredentials[0].ssid, wifiCredentials[0].password);

  pinMode(M1F, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M1R, OUTPUT);
  pinMode(M2R, OUTPUT);
  digitalWrite(M1F, LOW);
  digitalWrite(M2F, LOW);
  digitalWrite(M1R, LOW);
  digitalWrite(M2R, LOW);

  servo.attach(11);

  Blynk.virtualWrite(V2, 0);

  // Setup timers for GPS and ultrasonic sensor
  timer.setInterval(2000L, readSensors); // Read sensors every 2 seconds

  Wire.begin();  // Initialize I2C communication

  Serial.println("Setup completed");
}

void loop() {
  Blynk.run();
  timer.run(); // Run the timer
}

BLYNK_WRITE(V0) {
  int relayState = param.asInt();
  digitalWrite(M1F, relayState);
  digitalWrite(M2F, relayState);
}

BLYNK_WRITE(V2) {
  int sliderValue = param.asInt();  // Get value from Blynk slider
  int servoCommand = map(sliderValue, 0, 180, 0, 180);  // Map value to servo range
  servo.write(servoCommand);
}

BLYNK_WRITE(V3) {
  int relayState = param.asInt();
  digitalWrite(M1R, relayState);
  digitalWrite(M2R, relayState);
}
