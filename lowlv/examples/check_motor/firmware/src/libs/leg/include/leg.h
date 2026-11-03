/**
 * @file leg.h
 * @brief One leg = one single-wire bus with a BL4012 wheel and two MIT joints,
 *        driven by its own task. Every motor is enabled and commanded alone.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "joint_bus.h"

#define LEG_JOINTS 3

// JOINT_POSITION: joint driver in MIT mode.
// JOINT_WHEEL: Kpower BL4012 wheel, speed control via 0x18/0x20 (it has no MIT).
typedef enum { JOINT_WHEEL, JOINT_POSITION } joint_kind_t;

typedef struct {
  const char *name;
  uint8_t id;
  joint_kind_t kind;
  float kp;         // N·m/rad, clamped to the driver's Kp_max (wheel: unused)
  float kd;         // N·m·s/rad, clamped to the driver's Kd_max (wheel: unused)
  float max_rpm;    // wheel: command limit; position joint: jog/move speed ceiling
  float max_accel;  // RPM/s
  float min_deg;    // position joint: allowed target range in the driver frame
  float max_deg;
  int8_t dir;       // +1/-1: host fold / forward -> driver direction
} joint_cfg_t;

typedef struct {
  const char *name;
  uart_port_t uart;
  gpio_num_t data_pin;
  gpio_num_t unused_rx_pin;  // PCB RX pad tied to DATA: kept floating, or GPIO_NUM_NC
  joint_cfg_t joint[LEG_JOINTS];
} leg_cfg_t;

typedef struct {
  const char *config_error;
  int response_level;
  bool online;
  bool off_confirmed;  // torque-enable register read back as zero
  bool homed;
  float home_deg;
  int64_t feedback_us;
  bool enabled;
  bool moving;
  uint8_t status;
  float pos_deg;  // driver frame; wheel: 0..360 within one turn
  float vel_rpm;  // wheel: forward > 0
  float target;   // deg (position joint) or RPM (wheel, forward > 0)
  float kp, kd, kp_max, kd_max;
  uint32_t miss;   // liveness transactions without a valid reply
  uint32_t polls;    // liveness transactions attempted (miss / polls = loss rate)
  uint32_t retries;  // first attempts that failed (retries / polls = raw loss)
} joint_view_t;

typedef struct {
  joint_view_t j[LEG_JOINTS];
  uint32_t cycle_us;
  uint32_t cycle_us_max;
  uint32_t bus_errors;
  uint32_t bus_echo_errors, bus_timeout_errors, bus_frame_errors;
  uint32_t bus_partial_errors;  // timeouts where the reply was cut short
  uint32_t bus_late_replies;    // replies that arrived after their window
  uint32_t bus_sync_recovered;  // replies whose lost first 0xFF was restored
  jb_capture_t bus_last_bad;
} leg_view_t;

typedef struct leg leg_t;

leg_t *leg_start(const leg_cfg_t *cfg, int core);

// Requests are applied by the leg task on its next cycle (<= one period).
void leg_enable(leg_t *leg, int joint, bool on);
// dir: +1 = fold, -1 = extend, 0 = stop (mapped through joint_cfg_t.dir).
// Must be repeated while held; the joint stops by itself if refreshes stop.
void leg_jog(leg_t *leg, int joint, int dir, float rpm);
// Move by delta_deg from the current target (> 0 = fold, < 0 = extend) at rpm.
void leg_move(leg_t *leg, int joint, float delta_deg, float rpm);
// rpm > 0 = forward (mapped through joint_cfg_t.dir).
void leg_wheel(leg_t *leg, int joint, float rpm);
void leg_gains(leg_t *leg, int joint, float kp, float kd);
// Wheels to 0 RPM and jogs stopped; torque state is left unchanged.
void leg_halt_motion(leg_t *leg);
void leg_view(leg_t *leg, leg_view_t *out);

#ifdef BENCH_OTA
// Sticky until reboot; the owning worker verifies OFF then stops all bus I/O.
void leg_prepare_ota(leg_t *leg);
bool leg_ota_ready(leg_t *leg);
#endif

// Capture the current OFF pose, no movement and no encoder-register write.
void leg_home(leg_t *leg);
void leg_forget_home(leg_t *leg);
