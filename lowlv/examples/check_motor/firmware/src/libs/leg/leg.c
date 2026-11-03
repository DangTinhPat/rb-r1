/**
 * @file leg.c
 * @brief One task per leg: configures its three motors over the bus, runs the
 *        trajectory/MIT/wheel loops every CONTROL_PERIOD_MS, and publishes a
 *        snapshot (leg_view) for telemetry. Public calls only queue requests.
 */
#include "leg.h"
#include "bench_safety.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "leg";

#define BUS_BAUD 1000000
// Per cycle: two MIT joints (~0.8 ms each with the default 500 us reply delay)
// and one wheel transaction. A slow or missing wheel reply stretches the cycle.
#define CONTROL_PERIOD_MS 4
#define MISS_LIMIT 25  // consecutive failed cycles before a motor is declared lost
#define CONFIG_RETRY_US 1000000
#define JOG_TIMEOUT_US 300000  // host must refresh a held jog faster than this
// While moving, the trajectory may lead the shaft by this much before the
// motion is cancelled (obstacle, end stop, or too little Kp for the load).
#define LAG_LIMIT_DEG 15.0f
// BL4012 ignores a speed written right after torque enable (needs >= 20 ms).
#define WHEEL_SETTLE_US 25000
// The BL4012 answers later the more it is asked (a wheel flooded
// with speed writes replied ever later), and a late reply on this shared bus
// collides with the joints' frames. So the wheel gets at most one transaction
// per slot, and an unchanged speed is only re-sent as a keepalive.
#define WHEEL_SLOT_US 10000
#define WHEEL_KEEPALIVE_US 100000

#define CPS_PER_RPM ((float)JB_COUNTS_PER_REV / 60.0f)

typedef struct {
  uint32_t generation;
  int enable;  // +1 on, -1 off, 0 none
  bool jog_new;
  int jog_dir;
  float jog_rpm;
  int64_t jog_us;
  bool move_new;
  float move_deg;
  float move_rpm;
  bool wheel_new;
  float wheel_rpm;
  bool gains_new;
  float kp, kd;
} joint_req_t;

typedef struct {
  const joint_cfg_t *cfg;
  uint32_t generation;
  const char *config_error;
  int response_level;
  bool configured;
  bool off_confirmed, homed;
  float home_deg;
  bool enabled;
  int64_t next_config_us;
  float kp_req, kd_req;
  uint16_t kp_raw, kd_raw, kp_max, kd_max;

  joint_reply_t fb;
  int64_t fb_us;
  float vel_rpm;
  uint8_t last_status;
  uint32_t miss_streak, miss_total;
  uint32_t polls;    // liveness transactions: every joint cycle, every wheel read
  uint32_t retries;  // exchanges that failed once and were sent again

  float target;    // deg (position) or driver-frame RPM (wheel)
  float traj_pos;  // counts
  float traj_vel;  // counts/s (position) or commanded RPM (wheel)
  int jog_dir;     // driver frame
  float jog_rpm;
  int64_t jog_us;
  float move_rpm;  // speed cap of a MOVE, 0 = cfg max_rpm

  float wheel_pos_deg;  // BL4012: 0..360 within one turn
  int64_t engaged_us;
  int64_t wheel_slot_us;   // start of the last wheel transaction
  int64_t wheel_write_us;  // last acknowledged speed write
  int16_t wheel_sent_rpm;  // meaningful while wheel_sent
  bool wheel_sent;
  bool wheel_wrote_last;   // reads and writes alternate when both are due
} joint_t;

struct leg {
  const leg_cfg_t *cfg;
  joint_bus_t bus;
  joint_t j[LEG_JOINTS];
  uint32_t cycle_us, cycle_us_max;
  portMUX_TYPE lock;
  joint_req_t req[LEG_JOINTS];
  leg_view_t view;
  bool home_request, forget_home;
  uint32_t home_generation;
#ifdef BENCH_OTA
  bool maintenance, ota_ready;
#endif
};

static float counts_to_deg(float c) { return c * 360.0f / JB_COUNTS_PER_REV; }
static float deg_to_counts(float d) { return d * JB_COUNTS_PER_REV / 360.0f; }

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int16_t sat16(float v) { return (int16_t)lroundf(clampf(v, -32768.0f, 32767.0f)); }

static uint16_t gain_raw(float v, uint16_t max) {
  const long raw = lroundf(fmaxf(v, 0.0f) * 1000.0f);
  return (uint16_t)(raw < max ? raw : max);
}

static bool is_wheel(const joint_t *j) { return j->cfg->kind == JOINT_WHEEL; }

// ---------------- Feedback ----------------
static void note_status(leg_t *leg, joint_t *j, uint8_t status) {
  if (status != j->last_status) {
    ESP_LOGW(TAG, "%s %s id%u status 0x%02X -> 0x%02X", leg->cfg->name, j->cfg->name,
             j->cfg->id, j->last_status, status);
    j->last_status = status;
  }
  j->fb.status = status;
  j->miss_streak = 0;
}

// Joint replies carry the position; speed comes from its differences.
static void on_joint_reply(leg_t *leg, joint_t *j, const joint_reply_t *r) {
  const int64_t now = esp_timer_get_time();
  if (j->fb_us) {
    const float dt = (now - j->fb_us) * 1e-6f;
    const int16_t dpos = (int16_t)(r->pos - j->fb.pos);
    if (dt > 0.0f) {
      j->vel_rpm += 0.2f * (dpos / dt / CPS_PER_RPM - j->vel_rpm);
    }
  }
  note_status(leg, j, r->status);
  j->fb = *r;
  j->fb_us = now;
}

static void apply_gains(joint_t *j) {
  if (is_wheel(j)) {
    j->kp_raw = j->kd_raw = 0;
    return;
  }
  j->kp_raw = gain_raw(j->kp_req, j->kp_max);
  j->kd_raw = gain_raw(j->kd_req, j->kd_max);
}

static esp_err_t set_torque(leg_t *leg, joint_t *j, uint8_t on) {
  joint_reply_t r;
  const jb_format_t fmt = is_wheel(j) ? JB_FMT_WHEEL : JB_FMT_JOINT;
  esp_err_t err = bus_write(&leg->bus, fmt, j->cfg->id, JB_REG_TORQUE_ENABLE, &on, 1, &r);
  // BL4012 may apply a torque write without sending a write ACK.
  // Recover ON only by reading the actual torque-enable register, in this lease.
  if (err == ESP_ERR_TIMEOUT && on && is_wheel(j)) {
    uint8_t torque = 255;
    err = bus_read(&leg->bus, fmt, j->cfg->id, JB_REG_TORQUE_ENABLE, 1, &torque, &r);
    if (err == ESP_OK && (torque != on || r.status ||
        !bench_current(leg->bus.motion_generation, esp_timer_get_time()))) err = ESP_FAIL;
  }
  if (err == ESP_OK) {
    if (is_wheel(j)) {
      note_status(leg, j, r.status);
    } else {
      on_joint_reply(leg, j, &r);
    }
  }
  return err == ESP_OK && r.status ? ESP_FAIL : err;
}

// ---------------- BL4012 wheel ----------------
static esp_err_t wheel_read_present(leg_t *leg, joint_t *j) {
  uint8_t d[6];
  joint_reply_t r;
  esp_err_t err =
      bus_read(&leg->bus, JB_FMT_WHEEL, j->cfg->id, JB_WHEEL_REG_PRESENT, sizeof(d), d, &r);
  if (err == ESP_OK) {
    note_status(leg, j, r.status);
    j->wheel_pos_deg = (uint16_t)(d[0] | (d[1] << 8)) * 360.0f / 1024.0f;
    j->vel_rpm = (int16_t)(d[2] | (d[3] << 8));
    j->fb_us = esp_timer_get_time();
  }
  return err;
}

static esp_err_t wheel_goal(leg_t *leg, joint_t *j, const uint8_t d[2], joint_reply_t *r) {
  esp_err_t err = bus_write(&leg->bus, JB_FMT_WHEEL, j->cfg->id, JB_WHEEL_REG_SPEED, d, 2, r);
  if (err == ESP_ERR_TIMEOUT) {
    uint8_t goal[2] = {0};
    err = bus_read(&leg->bus, JB_FMT_WHEEL, j->cfg->id, JB_WHEEL_REG_SPEED, 2, goal, r);
    if (err == ESP_OK && (memcmp(goal, d, 2) ||
        !bench_current(leg->bus.motion_generation, esp_timer_get_time()))) err = ESP_FAIL;
  }
  return err == ESP_OK && r->status ? ESP_FAIL : err;
}

static esp_err_t wheel_write_speed(leg_t *leg, joint_t *j, int16_t rpm, int64_t now) {
  const uint8_t d[2] = {(uint8_t)rpm, (uint8_t)((uint16_t)rpm >> 8)};
  joint_reply_t r;
  j->wheel_sent = false;  // unacknowledged: send again in the next write slot
  esp_err_t err = wheel_goal(leg, j, d, &r);
  if (err == ESP_OK) {
    note_status(leg, j, r.status);
    j->wheel_sent = true;
    j->wheel_sent_rpm = rpm;
    j->wheel_write_us = now;
  }
  return err;
}

static void run_wheel(joint_t *j, float dt) {
  const float dv = j->cfg->max_accel * dt;
  j->traj_vel += clampf(j->target - j->traj_vel, -dv, dv);
}

// At most one wheel transaction per WHEEL_SLOT_US. A slot carries a speed write
// when the speed changed or the keepalive is due, otherwise a state read; when
// both are due they alternate, so feedback still comes every other slot.
// Only the reads count for liveness (a write's reply is not required).
static esp_err_t wheel_cycle(leg_t *leg, joint_t *j, int64_t now, float dt) {
  leg->bus.motion_generation = j->generation;
  if (j->enabled) {
    run_wheel(j, dt);
  }
  if (now - j->wheel_slot_us < WHEEL_SLOT_US) {
    return ESP_OK;
  }
  j->wheel_slot_us = now;

  if (j->enabled && !j->wheel_wrote_last && now - j->engaged_us >= WHEEL_SETTLE_US) {
    const int16_t rpm = sat16(j->traj_vel);
    if (!j->wheel_sent || rpm != j->wheel_sent_rpm ||
        now - j->wheel_write_us >= WHEEL_KEEPALIVE_US) {
      j->wheel_wrote_last = true;
      if (!bench_live(esp_timer_get_time())) return ESP_FAIL;
      return wheel_write_speed(leg, j, rpm, now);
    }
  }
  j->wheel_wrote_last = false;
  j->polls++;
  if (wheel_read_present(leg, j) == ESP_OK) {
    return ESP_OK;
  }
  j->retries++;  // a read changes nothing on the wheel: ask once more
  return wheel_read_present(leg, j);
}

// ---------------- Configuration ----------------
static bool confirm_off(leg_t *leg, joint_t *j);
static bool configure_wheel(leg_t *leg, joint_t *j) {
  j->config_error = "wheel feedback read failed";
  if (wheel_read_present(leg, j) != ESP_OK) return false;
  j->config_error = "wheel OFF readback failed";
  // Wheel writes can omit their ACK. Only a clean OFF register readback is proof.
  return confirm_off(leg, j);
}

static bool configure_joint(leg_t *leg, joint_t *j) {
  const uint8_t id = j->cfg->id;
  joint_reply_t r;
  uint8_t level, lim[4];
  const uint8_t mode = JB_MODE_MIT;
  j->config_error = "joint ping failed";
  if (bus_ping(&leg->bus, JB_FMT_JOINT, id, &r) != ESP_OK) return false;
  on_joint_reply(leg, j, &r);
  j->config_error = "joint response-level read failed";
  if (bus_read(&leg->bus, JB_FMT_JOINT, id, JB_REG_RESPONSE_LEVEL, 1, &level, &r) != ESP_OK) return false;
  j->response_level = level;
  if (level != 2) {
    // Configure while physically OFF. The old reply level may suppress this
    // write's ACK, so require a clean readback of the new level instead.
    j->config_error = "joint OFF readback before reply-level setup failed";
    if (!confirm_off(leg, j)) return false;
    const uint8_t all_replies = 2;
    j->config_error = "joint response-level setup/readback failed";
    (void)bus_write(&leg->bus, JB_FMT_JOINT, id, JB_REG_RESPONSE_LEVEL, &all_replies, 1, &r);
    if (bus_read(&leg->bus, JB_FMT_JOINT, id, JB_REG_RESPONSE_LEVEL, 1, &level, &r) != ESP_OK ||
        r.status || level != 2) return false;
    j->response_level = level;
  }
  j->config_error = "joint gain-limits read failed";
  if (bus_read(&leg->bus, JB_FMT_JOINT, id, JB_REG_KP_MAX, 4, lim, &r) != ESP_OK) return false;
  j->config_error = "joint OFF readback failed";
  if (!confirm_off(leg, j)) return false;
  j->config_error = "joint MIT mode write failed";
  if (bus_write(&leg->bus, JB_FMT_JOINT, id, JB_REG_CONTROL_MODE, &mode, 1, &r) != ESP_OK || r.status) return false;
  j->kp_max = (uint16_t)(lim[0] | (lim[1] << 8));
  j->kd_max = (uint16_t)(lim[2] | (lim[3] << 8));
  j->fb_us = 0;
  j->vel_rpm = 0.0f;
  j->last_status = r.status;
  on_joint_reply(leg, j, &r);
  return true;
}

static bool configure(leg_t *leg, joint_t *j) {
  if (!(is_wheel(j) ? configure_wheel(leg, j) : configure_joint(leg, j))) {
    return false;
  }
  apply_gains(j);
  j->enabled = false;
  j->jog_dir = 0;
  j->configured = true;
  j->config_error = NULL;
  return true;
}

// ---------------- Enable / disable ----------------
static void engage(leg_t *leg, joint_t *j, uint32_t generation) {
  if (!j->homed || !j->off_confirmed || !bench_current(generation, esp_timer_get_time())) return;
  j->generation = leg->bus.motion_generation = generation;
  if (j->miss_streak) {
    ESP_LOGE(TAG, "%s %s: not responding, stays off", leg->cfg->name, j->cfg->name);
    return;
  }
  j->traj_vel = 0.0f;
  j->jog_dir = 0;
  j->move_rpm = 0.0f;
  if (is_wheel(j)) {
    // Clear a previous session's speed before torque enable, with register readback.
    uint8_t zero[2] = {0}, goal[2] = {255, 255}; joint_reply_t reply;
    if (wheel_goal(leg, j, zero, &reply) != ESP_OK ||
        reply.status || bus_read(&leg->bus, JB_FMT_WHEEL, j->cfg->id, JB_WHEEL_REG_SPEED, 2, goal, &reply) != ESP_OK ||
        reply.status || goal[0] || goal[1] || !bench_live(esp_timer_get_time())) {
      bench_trip(); return;
    }
    j->off_confirmed = false; // even a lost enable ACK may have energized the driver
    if (set_torque(leg, j, 1) != ESP_OK) {
      ESP_LOGE(TAG, "%s %s: torque enable not acknowledged, stays off", leg->cfg->name,
               j->cfg->name);
      bench_trip();
      return;
    }
    j->engaged_us = esp_timer_get_time();
    j->target = 0.0f;
    j->wheel_sent = false;
    j->wheel_wrote_last = false;
  } else {
    // Start at the measured pose: closing the loop must not move anything.
    // The first MIT block also switches the driver torque on.
    j->traj_pos = j->fb.pos;
    j->target = counts_to_deg(j->fb.pos);
  }
  j->off_confirmed = false;
  j->enabled = true;
  ESP_LOGI(TAG, "%s %s: torque ON", leg->cfg->name, j->cfg->name);
}

static bool confirm_off(leg_t *leg, joint_t *j) {
  uint8_t torque = 255;
  joint_reply_t r;
  const jb_format_t fmt = is_wheel(j) ? JB_FMT_WHEEL : JB_FMT_JOINT;
  j->enabled = false;
  j->jog_dir = 0;
  j->traj_vel = 0;
  j->target = is_wheel(j) ? 0 : counts_to_deg(j->fb.pos);
  // A missing ACK never becomes an OFF confirmation.
  (void)set_torque(leg, j, 0);
  esp_err_t read = bus_read(&leg->bus, fmt, j->cfg->id, JB_REG_TORQUE_ENABLE, 1, &torque, &r);
  j->off_confirmed = read == ESP_OK && !r.status && torque == 0;
  return j->off_confirmed;
}
static void disengage(leg_t *leg, joint_t *j, const char *why) {
  if (!confirm_off(leg, j)) bench_trip();
  ESP_LOGW(TAG, "%s %s: OFF requested (%s), confirmed=%d", leg->cfg->name, j->cfg->name,
           why, j->off_confirmed);
}

// ---------------- Joint motion (every control period) ----------------
// Accel- and speed-limited approach to the target; brakes so it stops on it.
static void step_position(joint_t *j, float dt, float vmax_rpm) {
  const float target = deg_to_counts(j->target);
  const float vmax = vmax_rpm * CPS_PER_RPM;
  const float amax = j->cfg->max_accel * CPS_PER_RPM;
  const float d = target - j->traj_pos;

  const float v_des = copysignf(fminf(vmax, sqrtf(2.0f * amax * fabsf(d))), d);
  j->traj_vel += clampf(v_des - j->traj_vel, -amax * dt, amax * dt);
  const float step = j->traj_vel * dt;
  if (fabsf(step) >= fabsf(d)) {
    j->traj_pos = target;
    j->traj_vel = 0.0f;
  } else {
    j->traj_pos += step;
  }
}

// Release of a jog: aim at the point where the current speed brakes to zero.
static void jog_stop(joint_t *j) {
  const float amax = j->cfg->max_accel * CPS_PER_RPM;
  const float brake = j->traj_vel * fabsf(j->traj_vel) / (2.0f * amax);
  j->jog_dir = 0;
  j->target = clampf(counts_to_deg(j->traj_pos + brake), j->cfg->min_deg, j->cfg->max_deg);
}

static void run_position(leg_t *leg, joint_t *j, int64_t now, float dt) {
  const joint_cfg_t *c = j->cfg;
  if (j->jog_dir && now - j->jog_us > JOG_TIMEOUT_US) {
    jog_stop(j);
  }

  float vmax = j->move_rpm > 0.0f ? fminf(j->move_rpm, c->max_rpm) : c->max_rpm;
  if (j->jog_dir) {
    const float here = counts_to_deg(j->traj_pos);
    const float limit = j->jog_dir > 0 ? c->max_deg : c->min_deg;
    if ((limit - here) * j->jog_dir <= 0.0f) {
      jog_stop(j);  // already at (or outside) the limit in that direction
    } else {
      j->target = limit;
      vmax = fminf(j->jog_rpm, c->max_rpm);
    }
  }
  step_position(j, dt, vmax);

  const float lag = counts_to_deg(j->traj_pos - j->fb.pos);
  if ((j->traj_vel != 0.0f || j->jog_dir) && fabsf(lag) > LAG_LIMIT_DEG) {
    ESP_LOGW(TAG, "%s %s: shaft %.1f deg behind, motion stopped", leg->cfg->name, c->name, lag);
    bench_trip();
    j->jog_dir = 0;
    j->traj_pos = j->fb.pos;
    j->traj_vel = 0.0f;
    j->target = counts_to_deg(j->fb.pos);
  }
}

// A frame the driver did not take (e.g. hit by noise while it drives the motor)
// is sent once more in the same cycle; an MIT block repeated is the same
// setpoint, so this is safe. `retries` keeps the raw loss visible.
static esp_err_t joint_cycle(leg_t *leg, joint_t *j, int64_t now, float dt) {
  leg->bus.motion_generation = j->generation;
  joint_reply_t r;
  esp_err_t err = ESP_FAIL;
  j->polls++;
  joint_mit_cmd_t cmd = {0};
  if (j->enabled) {
    run_position(leg, j, now, dt);
    cmd = (joint_mit_cmd_t){.kp = j->kp_raw,
                            .kd = j->kd_raw,
                            .pos = sat16(j->traj_pos),
                            .rpm = sat16(j->traj_vel / CPS_PER_RPM),
                            .tau = 0};
  }
  for (int attempt = 0; attempt < 2 && err != ESP_OK; attempt++) {
    if (attempt) {
      j->retries++;
    }
    if (j->enabled && !bench_live(esp_timer_get_time())) return ESP_FAIL;
    err = j->enabled ? bus_write_mit(&leg->bus, j->cfg->id, &cmd, &r)
                     : bus_ping(&leg->bus, JB_FMT_JOINT, j->cfg->id, &r);
  }
  if (err == ESP_OK) {
    on_joint_reply(leg, j, &r);
  }
  return err;
}

// ---------------- Leg task ----------------
static void apply_request(leg_t *leg, joint_t *j, const joint_req_t *r) {
  if (r->generation != bench_generation()) return;
  if (r->gains_new) {
    j->kp_req = r->kp;
    j->kd_req = r->kd;
    apply_gains(j);
  }
  if (r->enable > 0 && !j->enabled && bench_live(esp_timer_get_time())) {
    engage(leg, j, r->generation);
  } else if (r->enable < 0 && j->enabled) {
    disengage(leg, j, "operator");
  }
  if (!j->enabled) {
    return;
  }

  if (is_wheel(j)) {
    if (r->wheel_new) {
      j->target = clampf(r->wheel_rpm * j->cfg->dir, -j->cfg->max_rpm, j->cfg->max_rpm);
    }
    return;
  }
  if (r->jog_new) {
    if (r->jog_dir) {
      j->jog_dir = r->jog_dir * j->cfg->dir;
      j->jog_rpm = r->jog_rpm;
      j->jog_us = r->jog_us;
    } else if (j->jog_dir) {
      jog_stop(j);
    }
  }
  if (r->move_new) {
    if (j->jog_dir) {
      jog_stop(j);
    }
    j->target = clampf(j->target + r->move_deg * j->cfg->dir, j->cfg->min_deg, j->cfg->max_deg);
    j->move_rpm = r->move_rpm;
  }
}

static void publish_view(leg_t *leg) {
  leg_view_t v = {
      .cycle_us = leg->cycle_us,
      .cycle_us_max = leg->cycle_us_max,
      .bus_errors = leg->bus.err_timeout + leg->bus.err_echo + leg->bus.err_frame,
      .bus_echo_errors = leg->bus.err_echo,
      .bus_timeout_errors = leg->bus.err_timeout,
      .bus_frame_errors = leg->bus.err_frame,
      .bus_partial_errors = leg->bus.err_partial,
      .bus_late_replies = leg->bus.err_late,
      .bus_sync_recovered = leg->bus.sync_recovered,
      .bus_last_bad = leg->bus.last_bad,
  };
  for (int i = 0; i < LEG_JOINTS; i++) {
    const joint_t *j = &leg->j[i];
    const bool wheel = is_wheel(j);
    const float wheel_sign = wheel ? j->cfg->dir : 1.0f;  // wheel speeds: forward > 0
    v.j[i] = (joint_view_t){
        .config_error = j->config_error, .response_level = j->response_level,
        .off_confirmed = j->off_confirmed,
        .homed = j->homed, .home_deg = j->home_deg, .feedback_us = j->fb_us,
        .online = j->configured,  // dropped after MISS_LIMIT consecutive misses
        .enabled = j->enabled,
        .moving = j->traj_vel != 0.0f || j->jog_dir != 0,
        .status = j->fb.status,
        .pos_deg = wheel ? j->wheel_pos_deg : counts_to_deg(j->fb.pos),
        .vel_rpm = j->vel_rpm * wheel_sign,
        .target = j->target * wheel_sign,
        .kp = j->kp_raw / 1000.0f,
        .kd = j->kd_raw / 1000.0f,
        .kp_max = j->kp_max / 1000.0f,
        .kd_max = j->kd_max / 1000.0f,
        .miss = j->miss_total,
        .polls = j->polls,
        .retries = j->retries,
    };
  }
  taskENTER_CRITICAL(&leg->lock);
  leg->view = v;
  taskEXIT_CRITICAL(&leg->lock);
}

#ifdef BENCH_OTA
// OTA maintenance: torque off on every motor, then read register 0x18 back.
// True only if all of them report 0 with a clean status.
static bool verify_all_off(leg_t *leg) {
  bool verified = true;
  for (int i = 0; i < LEG_JOINTS; i++) {
    joint_t *j = &leg->j[i];
    const jb_format_t fmt = is_wheel(j) ? JB_FMT_WHEEL : JB_FMT_JOINT;
    joint_reply_t reply;
    uint8_t torque = 255;
    (void)set_torque(leg, j, 0);
    const esp_err_t err =
        bus_read(&leg->bus, fmt, j->cfg->id, JB_REG_TORQUE_ENABLE, 1, &torque, &reply);
    verified &= err == ESP_OK && torque == 0 && reply.status == 0;
    j->enabled = false;
    j->jog_dir = 0;
    j->traj_vel = 0.0f;
  }
  return verified;
}
#endif

static void leg_step(leg_t *leg, int64_t t0, float dt) {
  joint_req_t req[LEG_JOINTS];
  taskENTER_CRITICAL(&leg->lock);
#ifdef BENCH_OTA
  const bool maintenance = leg->maintenance;
  const bool ota_ready = leg->ota_ready;
#endif
  const bool home = leg->home_request, forget = leg->forget_home;
  const uint32_t home_generation = leg->home_generation;
  leg->home_request = leg->forget_home = false;
  memcpy(req, leg->req, sizeof(req));
  for (int i = 0; i < LEG_JOINTS; i++) {
    leg->req[i].enable = 0;
    leg->req[i].jog_new = false;
    leg->req[i].move_new = false;
    leg->req[i].wheel_new = false;
    leg->req[i].gains_new = false;
  }
  taskEXIT_CRITICAL(&leg->lock);

#ifdef BENCH_OTA
  if (maintenance) {
    // Latched until reboot: verify OFF once, then stop all bus traffic.
    if (!ota_ready) {
      const bool verified = verify_all_off(leg);
      publish_view(leg);
      taskENTER_CRITICAL(&leg->lock);
      leg->ota_ready = verified;
      taskEXIT_CRITICAL(&leg->lock);
    }
    return;
  }
#endif

  for (int i = 0; i < LEG_JOINTS; i++) {
    joint_t *j = &leg->j[i];
    if (forget) j->homed = false;
    bool live = bench_live(esp_timer_get_time());
    if (!live) { j->homed = false; memset(&req[i], 0, sizeof(req[i])); }
    if ((!live || !j->configured) && (j->enabled || !j->off_confirmed) &&
        (j->configured || t0 >= j->next_config_us)) {
      confirm_off(leg, j);
    }
    if (!j->configured) {
      if (t0 >= j->next_config_us && !configure(leg, j)) {
        j->next_config_us = t0 + CONFIG_RETRY_US;
      }
      continue;
    }
    if (home && bench_current(home_generation, esp_timer_get_time()) && live && !j->enabled && j->off_confirmed && j->fb_us &&
        esp_timer_get_time() - j->fb_us < 30000 && !j->fb.status) {
      j->home_deg = is_wheel(j) ? j->wheel_pos_deg : counts_to_deg(j->fb.pos);
      j->homed = true;
    }
    // Recheck after a potentially blocking configuration/previous motor exchange.
    if (!bench_live(esp_timer_get_time())) {
      j->homed = false;
      memset(&req[i], 0, sizeof(req[i]));
      if (j->enabled || !j->off_confirmed) confirm_off(leg, j);
    }
    if (j->enabled && !bench_current(j->generation, esp_timer_get_time())) {
      bench_trip(); disengage(leg, j, "stale enable session");
    }
    apply_request(leg, j, &req[i]);

    const esp_err_t err = is_wheel(j) ? wheel_cycle(leg, j, t0, dt) : joint_cycle(leg, j, t0, dt);
    if (err == ESP_OK) {
      if (j->enabled && j->fb.status) {
        bench_trip();
        disengage(leg, j, "driver protection cut torque");
      }
      continue;
    }
    if (j->enabled) { bench_trip(); disengage(leg, j, "bus failure"); }
    j->miss_total++;
    if (++j->miss_streak >= MISS_LIMIT) {
      ESP_LOGE(TAG, "%s %s id%u lost (%u cycles without reply)", leg->cfg->name, j->cfg->name,
               j->cfg->id, (unsigned)j->miss_streak);
      j->configured = false;
      j->enabled = false;
      j->next_config_us = t0 + CONFIG_RETRY_US;
    }
  }

  leg->cycle_us = (uint32_t)(esp_timer_get_time() - t0);
  if (leg->cycle_us > leg->cycle_us_max) {
    leg->cycle_us_max = leg->cycle_us;
  }
  publish_view(leg);
}
static void leg_task(void *arg) {
  leg_t *leg = arg;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    leg_step(leg, esp_timer_get_time(), CONTROL_PERIOD_MS / 1000.0f);
  }
}

// ---------------- Public API ----------------
static bool allow_motion(uint32_t generation) { return bench_current(generation, esp_timer_get_time()); }

leg_t *leg_start(const leg_cfg_t *cfg, int core) {
  leg_t *leg = calloc(1, sizeof(*leg));
  if (!leg) {
    return NULL;
  }
  leg->cfg = cfg;
  portMUX_INITIALIZE(&leg->lock);
  for (int i = 0; i < LEG_JOINTS; i++) {
    leg->j[i].cfg = &cfg->joint[i];
    leg->j[i].kp_req = cfg->joint[i].kp;
    leg->j[i].kd_req = cfg->joint[i].kd;
  }
  if (cfg->unused_rx_pin != GPIO_NUM_NC) {
    gpio_reset_pin(cfg->unused_rx_pin);
    gpio_set_direction(cfg->unused_rx_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(cfg->unused_rx_pin, GPIO_FLOATING);
  }
  if (joint_bus_init(&leg->bus, cfg->uart, cfg->data_pin, BUS_BAUD) != ESP_OK) {
    free(leg);
    return NULL;
  }
  leg->bus.allow_motion = allow_motion;
  publish_view(leg);
  ESP_LOGI(TAG, "%s: UART%d on GPIO%d @ %d bps, period %d ms", cfg->name, cfg->uart,
           cfg->data_pin, BUS_BAUD, CONTROL_PERIOD_MS);
  xTaskCreatePinnedToCore(leg_task, cfg->name, 4096, leg, 20, NULL, core);
  return leg;
}

static bool valid(int joint) { return joint >= 0 && joint < LEG_JOINTS; }

void leg_enable(leg_t *leg, int joint, bool on) {
  if (!valid(joint)) return;
  taskENTER_CRITICAL(&leg->lock);
  leg->req[joint].generation = bench_generation();
  leg->req[joint].enable = on ? 1 : -1;
  taskEXIT_CRITICAL(&leg->lock);
}

void leg_jog(leg_t *leg, int joint, int dir, float rpm) {
  if (!valid(joint)) return;
  const int64_t now = esp_timer_get_time();
  taskENTER_CRITICAL(&leg->lock);
  joint_req_t *r = &leg->req[joint];
  r->generation = bench_generation();
  r->jog_new = true;
  r->jog_dir = dir > 0 ? 1 : (dir < 0 ? -1 : 0);
  r->jog_rpm = fabsf(rpm);
  r->jog_us = now;
  taskEXIT_CRITICAL(&leg->lock);
}

void leg_move(leg_t *leg, int joint, float delta_deg, float rpm) {
  if (!valid(joint)) return;
  taskENTER_CRITICAL(&leg->lock);
  joint_req_t *r = &leg->req[joint];
  r->generation = bench_generation();
  r->move_deg = r->move_new ? r->move_deg + delta_deg : delta_deg;  // clicks in one cycle add up
  r->move_rpm = fabsf(rpm);
  r->move_new = true;
  taskEXIT_CRITICAL(&leg->lock);
}

void leg_wheel(leg_t *leg, int joint, float rpm) {
  if (!valid(joint)) return;
  taskENTER_CRITICAL(&leg->lock);
  leg->req[joint].generation = bench_generation();
  leg->req[joint].wheel_new = true;
  leg->req[joint].wheel_rpm = rpm;
  taskEXIT_CRITICAL(&leg->lock);
}

void leg_gains(leg_t *leg, int joint, float kp, float kd) {
  if (!valid(joint)) return;
  taskENTER_CRITICAL(&leg->lock);
  leg->req[joint].generation = bench_generation();
  leg->req[joint].gains_new = true;
  leg->req[joint].kp = kp;
  leg->req[joint].kd = kd;
  taskEXIT_CRITICAL(&leg->lock);
}

void leg_halt_motion(leg_t *leg) {
  const int64_t now = esp_timer_get_time();
  taskENTER_CRITICAL(&leg->lock);
  for (int i = 0; i < LEG_JOINTS; i++) {
    leg->req[i].jog_new = true;
    leg->req[i].jog_dir = 0;
    leg->req[i].jog_us = now;
    leg->req[i].wheel_new = true;
    leg->req[i].wheel_rpm = 0.0f;
  }
  taskEXIT_CRITICAL(&leg->lock);
}

void leg_view(leg_t *leg, leg_view_t *out) {
  taskENTER_CRITICAL(&leg->lock);
  *out = leg->view;
  taskEXIT_CRITICAL(&leg->lock);
}

#ifdef BENCH_OTA
void leg_prepare_ota(leg_t *leg) {
  taskENTER_CRITICAL(&leg->lock);
  leg->maintenance = true;
  memset(leg->req, 0, sizeof(leg->req));
  taskEXIT_CRITICAL(&leg->lock);
}

bool leg_ota_ready(leg_t *leg) {
  taskENTER_CRITICAL(&leg->lock);
  const bool ready = leg->maintenance && leg->ota_ready;
  taskEXIT_CRITICAL(&leg->lock);
  return ready;
}
#endif

void leg_home(leg_t *leg) {
  taskENTER_CRITICAL(&leg->lock);
  leg->home_request = true; leg->home_generation = bench_generation();
  taskEXIT_CRITICAL(&leg->lock);
}
void leg_forget_home(leg_t *leg) {
  taskENTER_CRITICAL(&leg->lock);
  leg->forget_home = true; leg->home_request = false; memset(leg->req, 0, sizeof(leg->req));
  taskEXIT_CRITICAL(&leg->lock);
}
