// Kiểm tra IMU MPU6500: đọc gia tốc và con quay qua SPI đều 500 Hz, gửi lên máy tính.
// Dữ liệu là THÔ trong hệ trục của chip (chưa xoay sang hệ trục robot); phía máy tính áp bản đồ trục.
//
// Kiến trúc: một task riêng đọc IMU đúng chu kỳ 2 ms và ghi vào hàng đợi; vòng loop() lấy từ hàng đợi để gửi đi.
// Nhờ vậy việc gửi qua Wi-Fi bị nghẽn không làm mất mẫu và không làm lệch mốc thời gian (nếu đọc và gửi chung một
// vòng thì mỗi lần Wi-Fi nghẽn là mất một đoạn mẫu, khiến góc xoay bị đánh giá thấp và yaw trôi).
//
// Hai đường ra, cùng một định dạng dòng:
//   - UART0 (cáp USB), 921600 baud.
//   - Wi-Fi: máy chủ TCP cổng 8023 (một máy tính mỗi lúc). Máy tính nối bằng socket://<IP>:8023.
// Có ArduinoOTA nên nạp lại firmware được qua Wi-Fi (pio run -e check_imu_air -t upload).
//
// Dòng gửi lên:   $IMU,<ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
//   ms: thời gian từ lúc bật, mili giây có phần lẻ micro giây, lấy ngay sau khi đọc cảm biến;
//   a*: gia tốc m/s^2; g*: vận tốc góc rad/s.
// Dòng bắt đầu bằng '#' là thông báo (có thông báo số mẫu bị bỏ), máy tính bỏ qua.

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#error "Thiếu include/wifi_secrets.h: sao chép từ include/wifi_secrets.example.h rồi điền thông tin mạng"
#endif

namespace {

// Chân IMU trên PCB (giống firmware chính).
constexpr int kCsPin = 10;
constexpr int kMosiPin = 11;
constexpr int kSclkPin = 12;
constexpr int kMisoPin = 13;
constexpr int kIntPin = 14;

constexpr char kHostname[] = "rb1-imu";
constexpr std::uint16_t kTcpPort = 8023;
constexpr std::uint32_t kPeriodMs = 2;      // 500 Hz, bằng tần số mẫu nội của chip
constexpr std::size_t kQueueDepth = 1024;   // khoảng 2 giây dữ liệu
constexpr float kGravity = 9.80665F;
constexpr float kDegToRad = 0.017453292519943295F;
constexpr float kAccelScale = kGravity / 16384.0F;  // +-2 g
constexpr float kGyroScale = kDegToRad / 65.5F;     // +-500 deg/s

const SPISettings kConfigSpi{1'000'000U, MSBFIRST, SPI_MODE0};
const SPISettings kReadSpi{8'000'000U, MSBFIRST, SPI_MODE0};

struct RawSample {
  std::int64_t t_us;        // esp_timer_get_time() ngay sau khi đọc xong
  std::int16_t accel[3];
  std::int16_t gyro[3];
};

WiFiServer server(kTcpPort);
WiFiClient client;
QueueHandle_t sample_queue = nullptr;
bool services_started = false;
std::uint8_t who_am_i = 0;
bool imu_ok = false;
std::atomic<std::uint32_t> queue_drops{0};   // hàng đợi đầy (loop không kịp lấy)
std::uint32_t net_drops = 0;                 // Wi-Fi không còn chỗ gửi
std::uint32_t reported_drops = 0;

void writeRegister(std::uint8_t address, std::uint8_t value) {
  SPI.beginTransaction(kConfigSpi);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(address & 0x7FU);
  SPI.transfer(value);
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
}

std::uint8_t readRegister(std::uint8_t address) {
  SPI.beginTransaction(kConfigSpi);
  digitalWrite(kCsPin, LOW);
  SPI.transfer(address | 0x80U);
  const std::uint8_t value = SPI.transfer(0U);
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
  return value;
}

bool beginImu() {
  pinMode(kCsPin, OUTPUT);
  digitalWrite(kCsPin, HIGH);
  pinMode(kIntPin, INPUT);
  SPI.begin(kSclkPin, kMisoPin, kMosiPin, kCsPin);
  writeRegister(0x6B, 0x80);  // Reset
  delay(100);
  writeRegister(0x6A, 0x10);  // Tắt giao diện I2C
  writeRegister(0x6B, 0x01);  // PLL theo con quay X
  writeRegister(0x19, 0x01);  // 1 kHz / (1 + 1) = 500 Hz
  writeRegister(0x1A, 0x03);  // Lọc thông thấp số của con quay
  writeRegister(0x1B, 0x08);  // Con quay +-500 deg/s
  writeRegister(0x1C, 0x00);  // Gia tốc +-2 g
  writeRegister(0x1D, 0x03);  // Lọc thông thấp số của gia tốc
  delay(10);
  who_am_i = readRegister(0x75);
  return who_am_i == 0x70U || who_am_i == 0x71U;
}

void readImu(RawSample& s) {
  std::uint8_t request[15]{};
  std::uint8_t response[15]{};
  request[0] = 0x3BU | 0x80U;
  SPI.beginTransaction(kReadSpi);
  digitalWrite(kCsPin, LOW);
  SPI.transferBytes(request, response, sizeof request);
  digitalWrite(kCsPin, HIGH);
  SPI.endTransaction();
  s.t_us = esp_timer_get_time();
  const std::uint8_t* b = response + 1;
  const auto be16 = [&](int i) {
    return static_cast<std::int16_t>((static_cast<std::uint16_t>(b[i]) << 8U) | b[i + 1]);
  };
  for (int i = 0; i < 3; ++i) {
    s.accel[i] = be16(2 * i);
    s.gyro[i] = be16(8 + 2 * i);  // bỏ qua nhiệt độ ở byte 6-7
  }
}

// Đọc cảm biến đúng chu kỳ, không phụ thuộc Wi-Fi hay cổng serial. Chạy ở lõi 1 với mức ưu tiên cao hơn loop().
void sensorTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kPeriodMs));
    RawSample s{};
    readImu(s);
    if (xQueueSend(sample_queue, &s, 0) != pdTRUE) queue_drops.fetch_add(1);
  }
}

void printHeader(Print& out) {
  out.printf("# check_imu who_am_i=0x%02X %s\n", who_am_i, imu_ok ? "OK" : "KHONG DUNG (can 0x70 hoac 0x71)");
}

void startServices() {
  WiFi.setSleep(false);  // tắt tiết kiệm năng lượng để luồng dữ liệu không bị dồn cục
  ArduinoOTA.setHostname(kHostname);
  ArduinoOTA.setPassword(CHECK_OTA_PASSWORD);
  ArduinoOTA.onStart([] { Serial.println("# OTA: bat dau nap"); });
  ArduinoOTA.onEnd([] { Serial.println("# OTA: xong, khoi dong lai"); });
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("# OTA: loi %u\n", static_cast<unsigned>(error)); });
  ArduinoOTA.begin();
  server.begin();
  server.setNoDelay(true);
  services_started = true;
  Serial.printf("# san sang: %s ip=%s tcp=%u\n", kHostname, WiFi.localIP().toString().c_str(),
                static_cast<unsigned>(kTcpPort));
}

void pollClient() {
  if (server.hasClient()) {
    WiFiClient incoming = server.accept();
    if (client && client.connected()) {
      incoming.print("# busy\n");
      incoming.stop();
    } else {
      client = incoming;
      client.setNoDelay(true);
      printHeader(client);
    }
  }
  // Máy tính không gửi gì cần xử lý: đọc bỏ để bộ đệm nhận không đầy.
  while (client && client.connected() && client.available() > 0) client.read();
}

// Gom các dòng $IMU thành gói rồi mới gửi: ít lần gọi mạng hơn và mỗi gói TCP mang nhiều mẫu. Chờ mạng ở đây chỉ làm
// loop() chậm lại, không làm mất mẫu vì việc đọc cảm biến nằm ở task riêng và có hàng đợi đệm.
char batch[1536];
std::size_t batch_len = 0;

void flushBatch() {
  if (batch_len == 0) return;
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(batch);
  Serial.write(bytes, batch_len);
  if (client && client.connected()) {
    if (client.write(bytes, batch_len) != batch_len) ++net_drops;
  }
  batch_len = 0;
}

void emit(const RawSample& s) {
  const std::int64_t ms = s.t_us / 1000;
  const unsigned frac = static_cast<unsigned>(s.t_us % 1000);
  char line[112];
  const int n = std::snprintf(line, sizeof line, "$IMU,%lld.%03u,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n",
                              static_cast<long long>(ms), frac, s.accel[0] * kAccelScale, s.accel[1] * kAccelScale,
                              s.accel[2] * kAccelScale, s.gyro[0] * kGyroScale, s.gyro[1] * kGyroScale,
                              s.gyro[2] * kGyroScale);
  if (n <= 0) return;
  if (batch_len + static_cast<std::size_t>(n) > sizeof batch) flushBatch();
  std::memcpy(batch + batch_len, line, static_cast<std::size_t>(n));
  batch_len += static_cast<std::size_t>(n);
}

}  // namespace

void setup() {
  Serial.begin(921600);
  delay(300);
  imu_ok = beginImu();
  printHeader(Serial);
  sample_queue = xQueueCreate(kQueueDepth, sizeof(RawSample));
  xTaskCreatePinnedToCore(sensorTask, "imu", 4096, nullptr, 5, nullptr, 1);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(kHostname);
  WiFi.setAutoReconnect(true);
  WiFi.begin(CHECK_WIFI_SSID, CHECK_WIFI_PASSWORD);
}

void loop() {
  static std::uint32_t last_report_ms = 0;

  if (WiFi.status() == WL_CONNECTED) {
    if (!services_started) startServices();
    ArduinoOTA.handle();
    pollClient();
  }

  RawSample s;
  int budget = 128;  // chừa thời gian cho OTA và Wi-Fi giữa các lần lấy
  while (budget-- > 0 && xQueueReceive(sample_queue, &s, 0) == pdTRUE) emit(s);
  flushBatch();

  const std::uint32_t now = millis();
  if (now - last_report_ms >= 5000) {
    last_report_ms = now;
    const std::uint32_t drops = queue_drops.load() + net_drops;
    if (drops != reported_drops) {
      reported_drops = drops;
      const char* where = "# bo mau: hang doi=%lu mang=%lu\n";
      if (client && client.connected()) client.printf(where, static_cast<unsigned long>(queue_drops.load()),
                                                      static_cast<unsigned long>(net_drops));
    }
  }
  delay(1);
}
