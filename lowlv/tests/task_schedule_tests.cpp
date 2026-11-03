#include "gateway/core/TaskSchedule.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <deque>

// Deterministic ready-task model, NOT an ESP/FreeRTOS timing measurement.
// Two continuously ready workers represent bus-processing CPU contention.
// Service wakes at 2 kHz; UART drains work every 1 ms. Motor TX masks IRQs
// for 200 us at a time (a 20-byte motor frame at 1 Mbaud). Even a higher
// priority task must wait for that section to end.
struct Result {
  unsigned max_tx_age=0, stale=0, sent=0, service_lateness=0;
  std::array<unsigned,2> motor_work{};
};
Result simulate(unsigned uart_priority) {
  using namespace gateway::task_schedule;
  constexpr unsigned quantum=10, duration=50000;
  Result out;
  std::deque<unsigned> packets;
  unsigned service_work=0, uart_work=0, service_release=0, next_uart=0;
  unsigned wire_remaining=0, wire_side=0, turn=0;
  for(unsigned now=0;now<duration;now+=quantum) {
    if(now%500==0) { assert(!service_work);service_work=50;service_release=now; }
    if(!uart_work && now>=next_uart) uart_work=50;
    if(wire_remaining) {
      wire_remaining-=quantum;out.motor_work[wire_side]+=quantum;continue;
    }
    const bool motor_busy=out.motor_work[0]<6000 || out.motor_work[1]<6000;
    // Motor workers run a 12 ms aggregate CPU burst, then block on their bus.
    const unsigned motor_ready_priority=motor_busy?motor_priority:0;
    if(service_work && service_priority>=uart_priority && service_priority>=motor_ready_priority) {
      if(service_work==50) out.service_lateness=std::max(out.service_lateness,now-service_release);
      service_work-=quantum;
      if(!service_work && service_release%5000==0) packets.push_back(now);
    } else if(uart_work && uart_priority>=motor_ready_priority) {
      uart_work-=quantum;
      if(!uart_work) {
        for(auto stamp:packets) {
          const auto age=now-stamp;out.max_tx_age=std::max(out.max_tx_age,age);
          if(age>10000) ++out.stale;else ++out.sent;
        }
        packets.clear();next_uart=now+1000;
      }
    } else if(motor_busy) {
      wire_side=(turn++)%2;
      if(out.motor_work[wire_side]>=6000) wire_side=1-wire_side;
      wire_remaining=200-quantum;out.motor_work[wire_side]+=quantum;
    }
  }
  return out;
}
int main() {
  const auto former=simulate(17);
  assert(former.stale>0 && former.max_tx_age>10000);
  const auto current=simulate(gateway::task_schedule::uart_priority);
  std::printf("model UART max queued age: former=%u us current=%u us\n",
              former.max_tx_age,current.max_tx_age);
  std::fflush(stdout);
  assert(current.stale==0 && current.sent==10);
  assert(current.max_tx_age<2000);
  assert(current.service_lateness<=200);
  assert(current.motor_work[0]==6000 && current.motor_work[1]==6000);
  std::puts("PASS: ready motor workers cannot starve UART; watchdog preempts UART; both workers make progress (host model only)");
}
