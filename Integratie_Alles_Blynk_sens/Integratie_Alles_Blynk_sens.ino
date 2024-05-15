#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""

#include <BlynkSimpleWifi.h>
#include <Blynk.h>
#include <WiFiS3.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#define M1F 6
#define M1R 4
#define M2F 5
#define M2R 3

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

// SERVO
Servo servo;

struct WiFiCredential {
  const char* ssid;
  const char* password;
};

// WiFi credentials
const WiFiCredential wifiCredentials[] = {
  {"MagNet", "123iot123"}  // Update these with your actual WiFi credentials
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
      speed = gps.speed.knots();

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

  // Setup een timer om elke 10 seconden de GPS-data te lezen
  timer.setInterval(10000L, readGPS);

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
