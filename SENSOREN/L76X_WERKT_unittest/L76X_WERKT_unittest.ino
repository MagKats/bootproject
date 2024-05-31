#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "DEV_Config.h"
#include "L76X.h"

const int rxPin = 0;
const int txPin = 1;

SoftwareSerial gpsSerial(rxPin, txPin);
TinyGPSPlus gps;

void readGPS() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
        Serial.print(c);
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

    if (gps.location.isUpdated()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6); // Print latitude with 6 decimal places
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6); // Print longitude with 6 decimal places
        Serial.print("Speed (knots): ");
        Serial.println(gps.speed.knots(), 2); // Print speed with 2 decimal places
        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());
    }
}
