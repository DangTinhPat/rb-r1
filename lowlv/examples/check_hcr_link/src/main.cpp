#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <cstring>
#include "wifi_secrets.h"

// UART0 connects to Luckfox. Serial is the ESP32-S3 native USB console.
char line[32]{};
unsigned used = 0;
bool overflow = false;
WiFiServer console(8023);
WiFiClient client;
bool network_ready = false;

void logLine(const char* text) {
  Serial.println(text);
  if (client.connected()) client.println(text);
}

void setup() {
  Serial.begin(115200);
  Serial0.begin(921600, SERIAL_8N1, 44, 43);  // RX44, TX43
  Serial.println("UART check ready: waiting for hello");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("rb1-link");
  WiFi.setAutoReconnect(true);
  WiFi.begin(CHECK_WIFI_SSID, CHECK_WIFI_PASSWORD);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!network_ready) {
      ArduinoOTA.setHostname("rb1-link");
      ArduinoOTA.setPassword(CHECK_OTA_PASSWORD);
      ArduinoOTA.begin();
      console.begin();
      console.setNoDelay(true);
      network_ready = true;
    }
    ArduinoOTA.handle();
    if (console.hasClient()) {
      client.stop();
      client = console.accept();
      client.setNoDelay(true);
      client.println("UART check ready: hello -> im here");
    }
  }
  while (Serial0.available() > 0) {
    const char c = static_cast<char>(Serial0.read());
    if (c == '\r') continue;
    if (c == '\n') {
      line[used] = '\0';
      if (!overflow && std::strcmp(line, "hello") == 0) {
        logLine("LF -> ESP: hello");
        Serial0.print("im here\n");
        logLine("ESP -> LF: im here");
      }
      used = 0;
      overflow = false;
    } else if (used < sizeof(line) - 1) {
      line[used++] = c;
    } else {
      overflow = true;
    }
  }
  delay(1);
}
