

/****************************************
 * Include Libraries
 ****************************************/
#include <UbidotsESP8266.h>
#include <ESP8266WiFi.h>

/****************************************
 * Define Constants
 ****************************************/
namespace {
  const char * WIFISSID = "Nokia 3"; // Assign your WiFi SSID
  const char * PASSWORD = "password"; // Assign your WiFi password
  const char * TOKEN = "BBFF-3Sx9h51erTRYO9Jvw7mvqvbHRWLUYH"; // Assign your Ubidots TOKEN
}

Ubidots client(TOKEN);

/****************************************
 * Main Functions
 ****************************************/
void setup() {
  Serial.begin(115200);
  client.wifiConnection(WIFISSID, PASSWORD);
}

void loop() {
  client.readData(); // Reads the command from the logger
  delay(1000);
}
