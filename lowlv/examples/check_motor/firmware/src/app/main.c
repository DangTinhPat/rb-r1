/** Standalone Lumi-Quantum motor bench adapted for LF UART0, not HCR1. */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <inttypes.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "leg.h"
#include "bench_safety.h"
#ifdef BENCH_OTA
#include "bench_net.h"
#endif
#define LEGS 2
#define MOTORS (LEGS * LEG_JOINTS)
#define TELEMETRY_PERIOD_MS 50


// Firmware start gains. gui.py replaces them when a joint comes online, and
// leg.c clamps any request to the Kp_max/Kd_max read back from that driver.
// min/max_deg = the driver's whole int16 position range (+-2 output turns); only
// the lag guard stops a joint at a mechanical end stop. Narrow them to the real
// travel once it is known.
#define JOINT_RANGE_DEG 719.0f
// Wheel cap 950 rpm = BL4012 no-load speed at 12 V (rated 400: above it, a
// loaded wheel may overheat). Same ceiling as the web GUI.
#define WHEEL_CFG(id, dir) {"wheel", id, JOINT_WHEEL, 0.0f, 0.0f, 950.0f, 600.0f, 0.0f, 0.0f, dir}
#define KNEE_CFG(id, dir) \
  {"knee", id, JOINT_POSITION, 10.0f, 0.036f, 30.0f, 120.0f, -JOINT_RANGE_DEG, JOINT_RANGE_DEG, dir}
#define HIP_CFG(id, dir) \
  {"hip", id, JOINT_POSITION, 10.0f, 0.036f, 30.0f, 120.0f, -JOINT_RANGE_DEG, JOINT_RANGE_DEG, dir}

// IDs and directions checked on the robot: knee = ID3, hip = ID2.
static const leg_cfg_t LEG_CFG[LEGS] = {
    {"leg1", UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18,
     {WHEEL_CFG(1, -1), KNEE_CFG(3, +1), HIP_CFG(2, -1)}},
    {"leg2", UART_NUM_2, GPIO_NUM_15, GPIO_NUM_16,
     {WHEEL_CFG(1, +1), KNEE_CFG(3, -1), HIP_CFG(2, +1)}},
};


static leg_t *s_legs[LEGS];
static volatile bool s_resend_gains = true;
static const bool s_telemetry = true;
static uint32_t s_sequence;
static void output(const char *format, ...) {
  char line[768]; va_list args; va_start(args, format);
  int n = vsnprintf(line, sizeof(line), format, args); va_end(args);
  if (n < 0 || n >= sizeof(line)) return;
  uart_write_bytes(UART_NUM_0, line, n);
#ifdef BENCH_OTA
  bench_net_line(line);
#endif
}
static bool all_ready(bool home) {
  for (int l = 0; l < LEGS; l++) {
    if (!s_legs[l]) return false;
    leg_view_t v; leg_view(s_legs[l], &v);
    for (int j = 0; j < LEG_JOINTS; j++) {
      joint_view_t *m = &v.j[j];
      if (!m->online || m->status || !m->feedback_us ||
          esp_timer_get_time() - m->feedback_us >= 30000) return false;
      if (home ? !m->homed : (!m->off_confirmed || m->enabled)) return false;
    }
  }
  return true;
}
static bool number(const char *s, float *v) {
  char *end; errno = 0; *v = strtof(s, &end);
  return *s && !*end && !errno && isfinite(*v);
}
static bool uint32(const char *s, uint32_t *v) {
  if (!*s || *s == '-' || *s == '+') return false;
  char *end; errno = 0; unsigned long x = strtoul(s, &end, 10);
  if (*end || errno || x > UINT32_MAX) return false;
  *v = x; return true;
}
static void stop_all(void) {
  bench_trip();
  for (int l = 0; l < LEGS; l++) if (s_legs[l]) {
    leg_forget_home(s_legs[l]);
    for (int j = 0; j < LEG_JOINTS; j++) leg_enable(s_legs[l], j, false);
  }
}
// All commands come from UART, never Wi-Fi. C seq command / P seq.
static void submit_command(char *line) {
  if (!strcmp(line, "HELLO")) { stop_all(); s_sequence = 0; output("#HELLO MOTOR_BENCH_1\n"); return; }
  if (!strcmp(line, "STOP")) { stop_all(); output("#E STOP latched\n"); return; }
  char *args[8], *save; int n = 0;
  for (char *p = strtok_r(line, " ", &save); p; p = strtok_r(NULL, " ", &save)) {
    if (n == 8) return;
    args[n++] = p;
  }
  if (n < 2) return;
  uint32_t seq;
  if (!uint32(args[1], &seq) || !seq || seq <= s_sequence) return;
  if (!strcmp(args[0], "P") && n == 2) {
    s_sequence = seq; bench_renew(esp_timer_get_time()); return;
  }
  if (strcmp(args[0], "C") || n < 3) return;
  s_sequence = seq;
  const char *error = "bad command";
  char *cmd = args[2]; float a = 0, b = 0, c = 0;
  for (int i = 3; i < n; i++) {
    float *v = i == 3 ? &a : i == 4 ? &b : &c;
    if (i > 5 || !number(args[i], v)) goto done;
  }
#ifdef BENCH_OTA
  if (bench_net_locked()) { error = "OTA locked"; goto done; }
#endif
  if (!strcmp(cmd, "STOP") && n == 3) { stop_all(); error = NULL; goto done; }
  if (!strcmp(cmd, "GG") && n == 3) { s_resend_gains = true; error = NULL; goto done; }
  if (!strcmp(cmd, "RESET") && n == 3) {
    if (!all_ready(false)) { error = "need fresh six motor OFF readbacks"; goto done; }
    for (int l = 0; l < LEGS; l++) leg_forget_home(s_legs[l]);
    bench_reset(esp_timer_get_time()); error = NULL; goto done;
  }
  if (!bench_live(esp_timer_get_time())) { error = "fault latched: RESET then MOTOR_HOME"; goto done; }
  if (!strcmp(cmd, "MOTOR_HOME") && n == 3) {
    if (!all_ready(false)) { error = "HOME requires six fresh OFF motors"; goto done; }
    for (int l = 0; l < LEGS; l++) leg_home(s_legs[l]);
    error = NULL; goto done;
  }
  if ((!strcmp(cmd, "ON") || !strcmp(cmd, "OFF")) && n == 4 && a == floorf(a) && a > 0 && a <= 63) {
    bool on = !strcmp(cmd, "ON");
    if (on && !all_ready(true)) { error = "MOTOR_HOME required / stale motor"; goto done; }
    for (int m = 0; m < MOTORS; m++) if (((unsigned)a & (1u << m)) && s_legs[m/3])
      leg_enable(s_legs[m/3], m%3, on);
    error = NULL; goto done;
  }
  if (a < 0 || a >= MOTORS || a != floorf(a)) goto done;
  int m = (int)a; leg_t *leg = s_legs[m / 3]; int j = m % 3;
  if (!leg) goto done;
  leg_view_t view; leg_view(leg, &view);
  if (!strcmp(cmd, "G") && n == 6 && j != 0 && b >= 0 && b <= 65.535f && c >= 0 && c <= 65.535f) {
    leg_gains(leg, j, b, c); error = NULL; goto done;
  }
  if (!view.j[j].enabled || !all_ready(true)) { error = "motor OFF / not homed / stale feedback"; goto done; }
  if (!strcmp(cmd, "W") && n == 5 && j == 0 && fabsf(b) <= 950) {
    leg_wheel(leg, j, b); error = NULL;
  } else if (!strcmp(cmd, "MOVE") && n == 6 && j != 0 && fabsf(b) <= 360 && c > 0 && c <= 30) {
    leg_move(leg, j, b, c); error = NULL;
  } else if (!strcmp(cmd, "JOG") && n == 6 && j != 0 && b == floorf(b) && fabsf(b) <= 1 && c > 0 && c <= 30) {
    leg_jog(leg, j, (int)b, c); error = NULL;
  }
done:
  if (error) output("#A %" PRIu32 " REJECT %s\n", seq, error);
  else output("#A %" PRIu32 " QUEUED\n", seq);
}
static void console_task(void *arg) {
  char line[96]; size_t used = 0; bool overflow = false;
  for (;;) {
    uint8_t ch;
    if (uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(1)) != 1) continue;
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[used] = 0;
      if (!overflow) submit_command(line);
      used = 0; overflow = false;
    } else if (ch < 32 || ch > 126 || used + 1 >= sizeof(line)) overflow = true;
    else if (!overflow) line[used++] = ch;
  }
}
static void watchdog_task(void *arg) {
  for (;;) { bench_live(esp_timer_get_time()); vTaskDelay(pdMS_TO_TICKS(1)); }
}
static void telemetry_task(void *arg) {
  static char buf[640];
  float sent_gains[MOTORS][4] = {0};
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));

    leg_view_t v[LEGS] = {0};
    for (int l = 0; l < LEGS; l++) {
      if (s_legs[l]) leg_view(s_legs[l], &v[l]);
    }

    unsigned off = 0, homed = 0;
    for (int i = 0; i < MOTORS; i++) {
      joint_view_t *j = &v[i / LEG_JOINTS].j[i % LEG_JOINTS];
      if (j->off_confirmed) off |= 1u << i;
      if (j->homed) homed |= 1u << i;
    }
    output("#S %d %u %u\n", bench_fault(), homed, off);
    if (s_telemetry) {
      int len = snprintf(buf, sizeof(buf), "#T %u %u %u %u", (unsigned)v[0].cycle_us,
                         (unsigned)v[1].cycle_us, (unsigned)v[0].bus_errors,
                         (unsigned)v[1].bus_errors);
      for (int i = 0; i < MOTORS && len < (int)sizeof(buf); i++) {
        const joint_view_t *j = &v[i / LEG_JOINTS].j[i % LEG_JOINTS];
        len += snprintf(buf + len, sizeof(buf) - len, " %d,%d,%d,%02X,%.1f,%.1f,%.1f,%u",
                        j->online, j->enabled, j->moving, j->status, j->pos_deg,
                        j->vel_rpm, j->target, (unsigned)j->miss);
      }
      output("%s\n", buf);
    }

    const bool all = s_resend_gains;
    s_resend_gains = false;
    for (int i = 0; i < MOTORS; i++) {
      const joint_view_t *j = &v[i / LEG_JOINTS].j[i % LEG_JOINTS];
      const float g[4] = {j->kp, j->kd, j->kp_max, j->kd_max};
      if (all || memcmp(g, sent_gains[i], sizeof(g)) != 0) {
        memcpy(sent_gains[i], g, sizeof(g));
        output("#G %d %.3f %.3f %.3f %.3f\n", i, g[0], g[1], g[2], g[3]);
      }
    }
  }
}


void app_main(void) {
  const uart_config_t uart = {.baud_rate=921600, .data_bits=UART_DATA_8_BITS,
    .parity=UART_PARITY_DISABLE, .stop_bits=UART_STOP_BITS_1,
    .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .source_clk=UART_SCLK_DEFAULT};
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 8192, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  bench_trip();
  for (int l = 0; l < LEGS; l++) s_legs[l] = leg_start(&LEG_CFG[l], 1);
  xTaskCreatePinnedToCore(console_task, "lf-uart", 4096, NULL, 10, NULL, 0);
  xTaskCreatePinnedToCore(watchdog_task, "lease", 2048, NULL, 12, NULL, 0);
  xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 4, NULL, 0);
#ifdef BENCH_OTA
  bench_net_start(s_legs, LEGS);
#endif
  (void)esp_ota_mark_app_valid_cancel_rollback();
}
