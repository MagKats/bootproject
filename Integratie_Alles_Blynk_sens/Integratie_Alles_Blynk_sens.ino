#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""

#include <BlynkSimpleWifi.h>
#include <Blynk.h>
#include <WiFiS3.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "Arduino_LED_Matrix.h"

#define M1F 6
#define M1R 4
#define M2F 5
#define M2R 3

// Buzzer
/*const byte speakerPin=A5;
unsigned long lastPeriodStart;
const int onDuration=1000;
const int periodDuration=6000;*/

#define BLYNK_PRINT Serial
BlynkTimer timer; 

// GPS
const int RXPin = 8;
const int TXPin = 7;
const int GPSBaud = 9600;
SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps;

float LONG = 0.0;
float LAT = 0.0;
float speed = 0.0;

// Ultrasone sensor
const int trigPin = 9;
const int echoPin = 10;
long duration;
float distance;

// SERVO
Servo servo;

// Matrix
ArduinoLEDMatrix matrix;

struct WiFiCredential {
  const char* ssid;
  const char* password;
};

// WiFi credentials
const WiFiCredential wifiCredentials[] = {
  {"", ""}  // Update these with your actual WiFi credentials
};

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
    /*if (millis()-lastPeriodStart>=periodDuration)
      {
        lastPeriodStart+=periodDuration;
        tone(speakerPin,960, onDuration); // play 550 Hz tone in background for 'onDuration'
      }*/

  } else {
    Serial.println("\nConnection failed, check your credentials or reset the board.");
  }
}

void readGPS() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      LAT = gps.location.lat();
      LONG = gps.location.lng();
      speed = gps.speed.kmph();

      Serial.print("Latitude: ");
      Serial.println(LAT, 6);  // Print latitude met 6 decimalen
      Serial.print("Longitude: ");
      Serial.println(LONG, 6);  // Print longitude met 6 decimalen
      Serial.print("Speed (knots): ");
      Serial.println(speed, 2);  // Print speed met 2 decimalen

      // Stuur GPS-data naar Blynk
      Blynk.virtualWrite(V4, LAT);
      Blynk.virtualWrite(V5, LONG);
      Blynk.virtualWrite(V1, speed);

      // | DEBUG |
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
    }
  }
}

void readUltrasonicSensor() {
  // Zorg ervoor dat de trigPin laag is
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Zet de trigPin hoog gedurende 10 microseconden
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Lees de echoPin, bereken de reistijd en de afstand
  duration = pulseIn(echoPin, HIGH);
  //distance = duration * 0.034 / 2;
  distance = duration * 0.1482 / 2;

  // Print de afstand naar de seriële monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  // Stuur de afstand naar Blynk op V5 met 2 decimalen nauwkeurigheid
  Blynk.virtualWrite(V5, String(distance, 2).toFloat());
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial started at 115200");
  gpsSerial.begin(GPSBaud);
  Serial.println("GPS Serial started at 9600");
  connectToWiFi();
  Blynk.begin(BLYNK_AUTH_TOKEN, wifiCredentials[0].ssid, wifiCredentials[0].password);
  matrix.begin();

  pinMode(M1F, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M1R, OUTPUT);
  pinMode(M2R, OUTPUT);
  digitalWrite(M1F, LOW);
  digitalWrite(M2F, LOW);
  digitalWrite(M1R, LOW);
  digitalWrite(M2R, LOW);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(speakerPin, OUTPUT);

  servo.attach(11);

  Blynk.virtualWrite(V2, 0);

  // Setup timers om de GPS-data en de ultrasone sensor te lezen
  timer.setInterval(10000L, readGPS);
  timer.setInterval(2000L, readUltrasonicSensor);  // Lees elke 2 seconden de ultrasone sensor

  Serial.println("Setup completed");
}

void loop() {
  Blynk.run();
  timer.run(); // Laat de timer draaien
}

BLYNK_WRITE(V0) {
  int relayState = param.asInt();
  digitalWrite(M1F, relayState);
  digitalWrite(M2F, relayState);
}

BLYNK_WRITE(V2) {
  int sliderValue = param.asInt();  // Waarde ophalen van Blynk slider
  int servoCommand = map(sliderValue, 0, 180, 0, 180);  // Waarde mappen naar servo bereik
  servo.write(servoCommand);
}

BLYNK_WRITE(V3) {
  int relayState = param.asInt();
  digitalWrite(M1R, relayState);
  digitalWrite(M2R, relayState);
}
