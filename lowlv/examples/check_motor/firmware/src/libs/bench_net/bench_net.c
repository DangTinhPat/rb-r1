/**
 * @file bench_net.c
 * @brief Wi-Fi access for the bench (BENCH_OTA builds only):
 *   - TCP 8023: the console protocol of main.c, after "AUTH <key>"
 *   - HTTP 8081 (header X-OTA-Key): GET /status, POST /ota, POST /reboot
 * Before any flash write or reboot the motors are verified OFF by reading
 * register 0x18 of all six drivers back; the bench then stays locked (no
 * commands, no bus traffic) until the next reboot.
 * Credentials live in bench_secrets.h (see bench_secrets.example.h).
 */
#ifdef BENCH_OTA
#include "bench_net.h"

#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "bench_secrets.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define CONSOLE_PORT 8023
#define HTTP_PORT 8081
#define AUTH_TIMEOUT_US 5000000
#define OFF_VERIFY_TIMEOUT_US 3000000
#define MIN_APP_SIZE 288  // smaller than an ESP image header + app descriptor

typedef struct {
  char text[768];
} net_line_t;

static leg_t **s_legs;
static unsigned s_count;

static atomic_bool s_locked, s_client_ready;
static QueueHandle_t s_tx;

bool bench_net_locked(void) { return atomic_load(&s_locked); }

void bench_net_line(const char *line) {
  if (!s_tx || !atomic_load(&s_client_ready)) {
    return;
  }
  net_line_t msg;
  snprintf(msg.text, sizeof(msg.text), "%s", line);
  (void)xQueueSend(s_tx, &msg, 0);  // a slow GUI never blocks the motor workers
}

// ---------------- Maintenance lock ----------------
static bool all_off(void) {
  if (!bench_net_locked() || s_count != 2) {
    return false;
  }
  for (unsigned i = 0; i < s_count; i++) {
    if (!s_legs[i] || !leg_ota_ready(s_legs[i])) {
      return false;
    }
  }
  return true;
}

// Locks the bench and waits for every leg to confirm its motors are OFF.
static bool prepare_flash(void) {
  atomic_store(&s_locked, true);
  for (unsigned i = 0; i < s_count; i++) {
    if (s_legs[i]) leg_prepare_ota(s_legs[i]);
  }
  const int64_t end = esp_timer_get_time() + OFF_VERIFY_TIMEOUT_US;
  while (!all_off() && esp_timer_get_time() < end) {
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return all_off();
}

// ---------------- HTTP ----------------
static bool authorized(httpd_req_t *req) {
  char value[160];
  const size_t len = httpd_req_get_hdr_value_len(req, "X-OTA-Key");
  if (len != strlen(BENCH_ACCESS_KEY) || len >= sizeof(value) ||
      httpd_req_get_hdr_value_str(req, "X-OTA-Key", value, sizeof(value)) != ESP_OK ||
      memcmp(value, BENCH_ACCESS_KEY, len)) {
    httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "authentication required");
    return false;
  }
  return true;
}

static esp_err_t status_get(httpd_req_t *req) {
  if (!authorized(req)) {
    return ESP_OK;
  }
  const esp_app_desc_t *app = esp_app_get_description();
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char mac_s[18], sha[65];
  snprintf(mac_s, sizeof(mac_s), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3],
           mac[4], mac[5]);
  for (unsigned i = 0; i < 32; i++) {
    snprintf(sha + i * 2, 3, "%02x", app->app_elf_sha256[i]);
  }

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "firmware", "rb-r1-check-motor");
  cJSON_AddStringToObject(root, "project", app->project_name);
  cJSON_AddStringToObject(root, "elf_sha256", sha);
  cJSON_AddStringToObject(root, "mac", mac_s);
  cJSON_AddStringToObject(root, "partition", esp_ota_get_running_partition()->label);
  cJSON_AddBoolToObject(root, "maintenance", bench_net_locked());
  cJSON_AddBoolToObject(root, "off_confirmed", all_off());
  cJSON_AddNumberToObject(root, "uptime_ms", esp_timer_get_time() / 1000);
  cJSON_AddNumberToObject(root, "reset_reason", esp_reset_reason());
  cJSON_AddNumberToObject(root, "free_heap", esp_get_free_heap_size());
  cJSON *motors = cJSON_AddArrayToObject(root, "motors");
  for (unsigned l = 0; l < s_count; l++) {
    leg_view_t view = {0};
    if (s_legs[l]) leg_view(s_legs[l], &view);
    for (unsigned j = 0; j < LEG_JOINTS; j++) {
      cJSON *motor = cJSON_CreateObject();
      cJSON_AddItemToArray(motors, motor);
      cJSON_AddNumberToObject(motor, "index", l * LEG_JOINTS + j);
      cJSON_AddStringToObject(motor, "config_error", view.j[j].config_error ? view.j[j].config_error : "");
      cJSON_AddNumberToObject(motor, "response_level", view.j[j].response_level);
      cJSON_AddBoolToObject(motor, "off_confirmed", view.j[j].off_confirmed);
      cJSON_AddBoolToObject(motor, "online", view.j[j].online);
      cJSON_AddBoolToObject(motor, "enabled", view.j[j].enabled);
      cJSON_AddNumberToObject(motor, "position_deg", view.j[j].pos_deg);
      cJSON_AddNumberToObject(motor, "miss", view.j[j].miss);
      cJSON_AddNumberToObject(motor, "polls", view.j[j].polls);
      cJSON_AddNumberToObject(motor, "retries", view.j[j].retries);
      cJSON_AddNumberToObject(motor, "bus_errors", view.bus_errors);
      cJSON_AddNumberToObject(motor, "bus_echo_errors", view.bus_echo_errors);
      cJSON_AddNumberToObject(motor, "bus_timeout_errors", view.bus_timeout_errors);
      cJSON_AddNumberToObject(motor, "bus_frame_errors", view.bus_frame_errors);
      cJSON_AddNumberToObject(motor, "bus_partial_replies", view.bus_partial_errors);
      cJSON_AddNumberToObject(motor, "bus_late_replies", view.bus_late_replies);
    }
  }

  cJSON *buses = cJSON_AddArrayToObject(root, "buses");
  const int64_t now = esp_timer_get_time();
  for (unsigned l = 0; l < s_count; l++) {
    leg_view_t view = {0};
    if (s_legs[l]) leg_view(s_legs[l], &view);
    const jb_capture_t *c = &view.bus_last_bad;
    cJSON *bus = cJSON_CreateObject();
    cJSON_AddItemToArray(buses, bus);
    cJSON_AddNumberToObject(bus, "leg", l);
    cJSON_AddNumberToObject(bus, "cycle_us_max", view.cycle_us_max);
    cJSON_AddNumberToObject(bus, "sync_recovered", view.bus_sync_recovered);
    if (c->kind) {
      char tx[JB_CAPTURE_LEN * 3 + 1] = "", rx[JB_CAPTURE_LEN * 3 + 1] = "";
      for (unsigned i = 0; i < c->tx_len; i++) snprintf(tx + i * 3, 4, "%02X ", c->tx[i]);
      for (unsigned i = 0; i < c->rx_len; i++) snprintf(rx + i * 3, 4, "%02X ", c->rx[i]);
      char kind[2] = {c->kind, 0};
      cJSON *bad = cJSON_AddObjectToObject(bus, "last_bad");
      cJSON_AddStringToObject(bad, "kind", kind);
      cJSON_AddStringToObject(bad, "tx", tx);
      cJSON_AddStringToObject(bad, "rx", rx);
      cJSON_AddNumberToObject(bad, "age_ms", (now - c->at_us) / 1000);
    }
  }

  char *body = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);
  if (!body) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "allocation failed");
  }
  httpd_resp_set_type(req, "application/json");
  const esp_err_t result = httpd_resp_sendstr(req, body);
  cJSON_free(body);
  return result;
}

static void reboot_task(void *arg) {
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_restart();
}

static esp_err_t reboot_post(httpd_req_t *req) {
  if (!authorized(req)) {
    return ESP_OK;
  }
  if (!prepare_flash()) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "motors OFF not verified; locked");
  }
  httpd_resp_sendstr(req, "rebooting OFF\n");
  xTaskCreate(reboot_task, "ota-reboot", 2048, NULL, 3, NULL);
  return ESP_OK;
}

// After any failure past prepare_flash() the bench stays locked until reboot.
static esp_err_t ota_post(httpd_req_t *req) {
  if (!authorized(req)) {
    return ESP_OK;
  }
  const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
  if (!next || req->content_len < MIN_APP_SIZE || req->content_len > next->size) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid app size");
  }
  if (!prepare_flash()) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "six motor OFF readbacks not verified; locked");
  }
  esp_ota_handle_t ota;
  esp_err_t err = esp_ota_begin(next, req->content_len, &ota);
  if (err != ESP_OK) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "ota begin failed; locked");
  }

  char buf[4096];
  int remaining = req->content_len;
  while (remaining > 0) {
    const int n = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? sizeof(buf) : remaining);
    if (n <= 0) {
      esp_ota_abort(ota);
      return httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "upload incomplete; locked");
    }
    err = esp_ota_write(ota, buf, n);
    if (err != ESP_OK) {
      esp_ota_abort(ota);
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid app; locked");
    }
    remaining -= n;
  }

  // Validates the image and its embedded hash before the boot slot is switched.
  err = esp_ota_end(ota);
  if (err != ESP_OK) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "app validation failed; locked");
  }
  if (esp_ota_set_boot_partition(next) != ESP_OK) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "boot selection failed; locked");
  }
  httpd_resp_sendstr(req, "OK verified app; rebooting OFF\n");
  xTaskCreate(reboot_task, "ota-reboot", 2048, NULL, 3, NULL);
  return ESP_OK;
}

// ---------------- TCP console ----------------
// One client at a time. The first line must be "AUTH <key>" within 5 s; after
// that every line goes to s_command and queued output lines are sent back.
static void tcp_task(void *arg) {
  const int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  const int yes = 1;
  setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  const struct sockaddr_in address = {.sin_family = AF_INET,
                                      .sin_port = htons(CONSOLE_PORT),
                                      .sin_addr.s_addr = htonl(INADDR_ANY)};
  if (listener < 0 || bind(listener, (const struct sockaddr *)&address, sizeof(address)) ||
      listen(listener, 1)) {
    if (listener >= 0) close(listener);
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    const int client = accept(listener, NULL, NULL);
    if (client < 0) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    bool auth = false, alive = true;
    char line[128];
    size_t used = 0;
    net_line_t out = {0};
    size_t sent = 0, length = 0;
    xQueueReset(s_tx);
    const int64_t auth_deadline = esp_timer_get_time() + AUTH_TIMEOUT_US;

    while (alive) {
      fd_set rd;
      FD_ZERO(&rd);
      FD_SET(client, &rd);
      struct timeval wait = {.tv_usec = 20000};
      const int ready = select(client + 1, &rd, NULL, NULL, &wait);
      if (ready < 0 || (!auth && esp_timer_get_time() > auth_deadline)) {
        break;
      }

      if (ready > 0) {
        char bytes[128];
        const int n = recv(client, bytes, sizeof(bytes), 0);
        if (n <= 0) break;
        for (int i = 0; i < n && alive; i++) {
          const char ch = bytes[i];
          if (ch == '\r') continue;
          if (ch != '\n') {
            if (used + 1 < sizeof(line)) {
              line[used++] = ch;
            } else {
              alive = false;  // over-long line
            }
            continue;
          }
          line[used] = '\0';
          used = 0;
          if (auth) {
            bench_net_line("#E read-only console: control via LF UART only\n");
          } else if (strcmp(line, "AUTH " BENCH_ACCESS_KEY) == 0) {
            auth = true;
            atomic_store(&s_client_ready, true);
            bench_net_line("#NET authenticated 3in1-ota\n");

          } else {
            alive = false;
          }
        }
      }

      if (auth) {
        if (sent == length && xQueueReceive(s_tx, &out, 0) == pdTRUE) {
          length = strlen(out.text);
          sent = 0;
        }
        if (sent < length) {
          const int n = send(client, out.text + sent, length - sent, MSG_DONTWAIT);
          if (n > 0) {
            sent += n;
          } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            alive = false;
          }
        }
      }
    }
    atomic_store(&s_client_ready, false);
    close(client);
  }
}

// ---------------- Wi-Fi ----------------
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
  if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
    esp_wifi_connect();
  }
  if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    const ip_event_got_ip_t *event = data;
    ESP_LOGI("bench-net", "IP " IPSTR "; console %d; OTA %d", IP2STR(&event->ip_info.ip),
             CONSOLE_PORT, HTTP_PORT);
  }
}

void bench_net_start(leg_t **legs, unsigned count) {
  s_legs = legs;
  s_count = count;

  s_tx = xQueueCreate(16, sizeof(net_line_t));
  ESP_ERROR_CHECK(s_tx ? ESP_OK : ESP_ERR_NO_MEM);

  // Never erase existing NVS automatically when moving between firmware versions.
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *netif = esp_netif_create_default_wifi_sta();
  ESP_ERROR_CHECK(esp_netif_set_hostname(netif, "lf-mcu"));
  const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));

  wifi_config_t config = {0};
  snprintf((char *)config.sta.ssid, sizeof(config.sta.ssid), "%s", BENCH_WIFI_SSID);
  snprintf((char *)config.sta.password, sizeof(config.sta.password), "%s", BENCH_WIFI_PASSWORD);
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));  // power save adds latency to the console

  httpd_config_t http = HTTPD_DEFAULT_CONFIG();
  http.server_port = HTTP_PORT;
  http.stack_size = 8192;
  http.recv_wait_timeout = 10;
  http.send_wait_timeout = 5;
  httpd_handle_t server;
  ESP_ERROR_CHECK(httpd_start(&server, &http));
  const httpd_uri_t routes[] = {
      {.uri = "/status", .method = HTTP_GET, .handler = status_get},
      {.uri = "/ota", .method = HTTP_POST, .handler = ota_post},
      {.uri = "/reboot", .method = HTTP_POST, .handler = reboot_post},
  };
  for (unsigned i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
  }
  xTaskCreatePinnedToCore(tcp_task, "bench-tcp", 4096, NULL, 4, NULL, 0);
}
#endif  // BENCH_OTA
