#include "bench_safety.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t deadline;
static bool fault = true;
static uint32_t generation = 1;
void bench_trip(void) {
  taskENTER_CRITICAL(&lock); fault = true; deadline = 0; taskEXIT_CRITICAL(&lock);
}
bool bench_fault(void) {
  taskENTER_CRITICAL(&lock); bool f = fault; taskEXIT_CRITICAL(&lock); return f;
}
bool bench_live(int64_t now) {
  taskENTER_CRITICAL(&lock);
  if (!fault && (!deadline || now >= deadline)) { fault = true; deadline = 0; }
  bool live = !fault;
  taskEXIT_CRITICAL(&lock); return live;
}
void bench_renew(int64_t now) {
  taskENTER_CRITICAL(&lock);
  // Expiry is sticky even when the watchdog task has not run yet.
  if (!fault && deadline && now >= deadline) fault = true;
  if (!fault) deadline = now + BENCH_TTL_US;
  taskEXIT_CRITICAL(&lock);
}
void bench_reset(int64_t now) {
  taskENTER_CRITICAL(&lock); generation++; fault = false; deadline = now + BENCH_TTL_US; taskEXIT_CRITICAL(&lock);
}

uint32_t bench_generation(void) {
  taskENTER_CRITICAL(&lock); uint32_t g = generation; taskEXIT_CRITICAL(&lock); return g;
}
bool bench_current(uint32_t g, int64_t now) {
  taskENTER_CRITICAL(&lock);
  if (!fault && (!deadline || now >= deadline)) { fault = true; deadline = 0; }
  bool valid = !fault && g == generation;
  taskEXIT_CRITICAL(&lock); return valid;
}
