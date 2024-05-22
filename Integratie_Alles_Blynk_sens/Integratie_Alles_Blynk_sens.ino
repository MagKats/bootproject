#include "secret.h"
#include <BlynkSimpleWifi.h>
#include <Blynk.h>
#include <WiFiS3.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

#define M1F 6
#define M1R 4
#define M2F 5
#define M2R 3

#define BLYNK_PRINT Serial
BlynkTimer timer; 

// GPS
#define PowerGPS 2
const int RXPin = 8;
const int TXPin = 7;
const int GPSBaud = 9600;
SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps;

float LONG = 0.0;
float LAT = 0.0;
float speed = 0.0;

// TOF10120 sensor
const int TOF_ADDRESS = 0x52;
float distance;

// SERVO
Servo servo;

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

void readGPS() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated()) {
    LAT = gps.location.lat();
    LONG = gps.location.lng();
    speed = gps.speed.knots();

    Serial.print("Latitude: ");
    Serial.println(LAT, 6);  // Print latitude with 6 decimals
    Serial.print("Longitude: ");
    Serial.println(LONG, 6);  // Print longitude with 6 decimals
    Serial.print("Speed (knots): ");
    Serial.println(speed, 2);  // Print speed with 2 decimals

    // Send GPS data to Blynk
    Blynk.virtualWrite(V4, LAT, LONG);
    Blynk.virtualWrite(V1, speed);

    // Debugging
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  }
}

void readTOF() {
  Wire.beginTransmission(TOF_ADDRESS);
  Wire.write(0x00);  // Request the distance measurement
  Wire.endTransmission();

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

  pinMode(PowerGPS, OUTPUT);
  digitalWrite(PowerGPS, HIGH);

  servo.attach(11);

  Blynk.virtualWrite(V2, 0);

  // Setup timers for GPS and ultrasonic sensor
//  timer.setInterval(10000L, readGPS); // Read GPS every 10 seconds
  timer.setInterval(3000L, readTOF); // Read ultrasonic sensor every 3 seconds

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
