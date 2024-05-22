#define BLYNK_TEMPLATE_ID "TMPL5C4B1dg3j"
#define BLYNK_TEMPLATE_NAME "Boot Project Rev4"
#define BLYNK_AUTH_TOKEN "5u0JzlNEczJ-npDnxfmGmSe15lzVdBPW"

struct WiFiCredential {
  const char* ssid;
  const char* password;
};

const WiFiCredential wifiCredentials[] = {
  {"MagNet", "123iot123"}  // Update these with your actual WiFi credentials
};
