#pragma once
#include <stdbool.h>
#include <stdint.h>
// Independent bench lease; never used by the production HCR1 runtime.
#define BENCH_TTL_US 20000
void bench_trip(void);
bool bench_fault(void);
bool bench_live(int64_t now);
// Called only by the UART parser for a strictly increasing heartbeat sequence.
void bench_renew(int64_t now);
void bench_reset(int64_t now);  // caller must first verify every motor OFF

uint32_t bench_generation(void);
bool bench_current(uint32_t generation, int64_t now);
