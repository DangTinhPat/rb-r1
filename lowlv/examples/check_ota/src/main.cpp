// Kiểm tra Wi-Fi, OTA và truyền nhận hai chiều. Chỉ để xác thực kết nối, không điều khiển gì.
//
//  - Nối Wi-Fi (mạng kiểm thử) ở chế độ station.
//  - ArduinoOTA: nạp firmware qua mạng (pio run -e check_ota_air -t upload).
//  - TCP cổng 8023: máy tính gửi "henlo" thì ESP trả lại "henlo"; ESP tự gửi "henlo from esp32 #N" mỗi 2 giây;
//    gõ chữ vào Serial Monitor thì ESP chuyển sang máy tính.

#include <Arduino.h>
#include <cstring>
#include <ArduinoOTA.h>
#include <WiFi.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#error "Thiếu include/wifi_secrets.h: sao chép từ include/wifi_secrets.example.h rồi điền thông tin mạng"
#endif

namespace {

constexpr char kHostname[] = "rb1-check";
constexpr std::uint16_t kTcpPort = 8023;
constexpr std::uint32_t kHeartbeatMs = 2000;
constexpr std::uint32_t kStatusMs = 5000;
constexpr std::size_t kLineMax = 128;

WiFiServer server(kTcpPort);
WiFiClient client;
bool services_started = false;
std::uint32_t heartbeat_count = 0;

char tcp_line[kLineMax];
std::size_t tcp_len = 0;
char serial_line[kLineMax];
std::size_t serial_len = 0;

void startServices() {
  ArduinoOTA.setHostname(kHostname);
  ArduinoOTA.setPassword(CHECK_OTA_PASSWORD);
  ArduinoOTA.onStart([] { Serial.println("OTA: bat dau nap"); });
  ArduinoOTA.onEnd([] { Serial.println("\nOTA: xong, khoi dong lai"); });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    Serial.printf("OTA: %u%%\r", done * 100U / total);
  });
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA: loi %u\n", static_cast<unsigned>(error)); });
  ArduinoOTA.begin();
  server.begin();
  server.setNoDelay(true);
  services_started = true;
  Serial.printf("San sang: %s.local  ip=%s  tcp=%u\n", kHostname, WiFi.localIP().toString().c_str(),
                static_cast<unsigned>(kTcpPort));
}

void handleTcpLine(const char* text) {
  Serial.printf("[RX tcp] %s\n", text);
  if (std::strcmp(text, "henlo") == 0) {
    client.print("henlo\n");
  } else {
    client.printf("echo: %s\n", text);
  }
}

void pollTcp() {
  if (server.hasClient()) {
    WiFiClient incoming = server.accept();
    if (client && client.connected()) {
      incoming.print("busy\n");
      incoming.stop();
    } else {
      client = incoming;
      tcp_len = 0;
      Serial.printf("TCP: may tinh %s ket noi\n", client.remoteIP().toString().c_str());
      client.print("henlo from esp32\n");
    }
  }
  if (!client || !client.connected()) return;
  while (client.available() > 0) {
    const int c = client.read();
    if (c == '\n') {
      tcp_line[tcp_len] = '\0';
      if (tcp_len > 0) handleTcpLine(tcp_line);
      tcp_len = 0;
    } else if (c != '\r' && tcp_len + 1 < kLineMax) {
      tcp_line[tcp_len++] = static_cast<char>(c);
    }
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == '\n') {
      serial_line[serial_len] = '\0';
      if (serial_len > 0) {
        if (client && client.connected()) {
          client.printf("%s\n", serial_line);
          Serial.printf("[TX tcp] %s\n", serial_line);
        } else {
          Serial.println("Chua co may tinh nao ket noi TCP");
        }
      }
      serial_len = 0;
    } else if (c != '\r' && serial_len + 1 < kLineMax) {
      serial_line[serial_len++] = static_cast<char>(c);
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\ncheck_ota: Wi-Fi + OTA + henlo");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(kHostname);
  WiFi.setAutoReconnect(true);
  WiFi.begin(CHECK_WIFI_SSID, CHECK_WIFI_PASSWORD);
  Serial.printf("Dang noi Wi-Fi '%s'...\n", CHECK_WIFI_SSID);
}

void loop() {
  static std::uint32_t last_heartbeat = 0;
  static std::uint32_t last_status = 0;
  const std::uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    if (!services_started) startServices();
    ArduinoOTA.handle();
    pollTcp();
    if (client && client.connected() && now - last_heartbeat >= kHeartbeatMs) {
      last_heartbeat = now;
      client.printf("henlo from esp32 #%lu\n", static_cast<unsigned long>(++heartbeat_count));
    }
  }
  pollSerial();

  if (now - last_status >= kStatusMs) {
    last_status = now;
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("wifi ok  ssid=%s  ip=%s  rssi=%d dBm  tcp_client=%s\n", WiFi.SSID().c_str(),
                    WiFi.localIP().toString().c_str(), WiFi.RSSI(), (client && client.connected()) ? "yes" : "no");
    } else {
      Serial.println("wifi chua noi duoc");
    }
  }
  delay(2);
}
