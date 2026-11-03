#pragma once

namespace gateway::task_schedule {
// All control tasks share core 1; Wi-Fi/console remain outside this schedule.
inline constexpr int core = 1;
inline constexpr unsigned service_priority = 20;
// UART consumes telemetry and delivers TARGETs for both workers. Ready motor
// work must not starve it; the watchdog/service must still preempt all I/O.
inline constexpr unsigned uart_priority = 19;
inline constexpr unsigned motor_priority = 18;
static_assert(service_priority > uart_priority && uart_priority > motor_priority,
              "Watchdog > HCR UART > motor workers on the shared control core");
}  // namespace gateway::task_schedule
