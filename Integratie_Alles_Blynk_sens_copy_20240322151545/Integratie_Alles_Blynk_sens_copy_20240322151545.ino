#define BLYNK_TEMPLATE_ID "TMPL5aALXjp2f"
#define BLYNK_TEMPLATE_NAME "SCHIP CONTROLE"
#define BLYNK_AUTH_TOKEN "uO-RsP0GDFLPK8S9MKLGdS8HW1Nf4gO1"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#define RELAY_PIN 25


//------------------------------------ WIFI

void connectToWiFi() {
  int numNetworks = sizeof(wifiCredentials) / sizeof(wifiCredentials[0]);
  for (int i = 0; i < numNetworks; i++) {
    Serial.print("Connecting to ");
    Serial.println(wifiCredentials[i].ssid);
    WiFi.begin(wifiCredentials[i].ssid, wifiCredentials[i].password);
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
      break;
    } else {
      Serial.println("\nConnection failed!");
    }
  }
}

void setup() {
  Serial.begin(9600);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Blynk.virtualWrite(V2, 0);
}

void loop() {
  Blynk.run();
}

BLYNK_WRITE(V0) {
  int relayState = param.asInt(); // Get the state of the virtual pin
  if (relayState == HIGH) {
    // Turn on the relay when the button is pressed
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    // Turn off the relay when the button is released
    digitalWrite(RELAY_PIN, LOW);
  }
}

// Function to handle the slider value change event
BLYNK_WRITE(V2) {
  int sliderValue = param.asInt(); // Get the value from the slider
  // Map the slider value (0-1023) to the range of the relay (0-1)
  int relayState = map(sliderValue, 0, 1023, 0, 1);
  // Update the relay state accordingly
  digitalWrite(RELAY_PIN, relayState);
}
