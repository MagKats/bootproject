#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "DEV_Config.h"
#include "L76X.h"

const int rxPin = 12;
const int txPin = 14;

SoftwareSerial gpsSerial(rxPin, txPin);
TinyGPSPlus gps;

float LONG = 0.0;
float LAT = 0.0;
float speed = 0.0;

void readGPS() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      LAT = gps.location.lat();
      LONG = gps.location.lng();
      speed = gps.speed.knots();

      Serial.print("Latitude: ");
      Serial.println(LAT, 6);  // Print latitude with 6 decimal places
      Serial.print("Longitude: ");
      Serial.println(LONG, 6);  // Print longitude with 6 decimal places
      Serial.print("Speed (knots): ");
      Serial.println(speed, 2);  // Print speed with 2 decimal places
            //Serial.print("Satellites: ");
      //Serial.println(gps.satellites.value());
    }
  }
}

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  DEV_Set_Baudrate(115200);
  L76X_Send_Command(SET_NMEA_OUTPUT);
  L76X_Send_Command(SET_NMEA_BAUDRATE_9600);
  DEV_Delay_ms(500);

  L76X_Send_Command("9600");
  DEV_Set_Baudrate(9600);
  DEV_Delay_ms(500);
  L76X_Send_Command(SET_NMEA_OUTPUT);
}

void loop() {
  readGPS();
}
