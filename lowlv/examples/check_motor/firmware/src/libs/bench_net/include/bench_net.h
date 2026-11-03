/**
 * @file bench_net.h
 * @brief Wi-Fi side of the esp32-s3-3in1-ota build (compiled only with BENCH_OTA).
 */
#pragma once

#include <stdbool.h>

#include "leg.h"

// Starts Wi-Fi (station), the authenticated TCP console (8023) and the HTTP
// status/OTA server (8081). TCP is read-only; motor commands are UART-only.
void bench_net_start(leg_t **legs, unsigned count);
// Queues one line for the TCP client; never blocks the caller.
void bench_net_line(const char *line);
// True once an OTA/reboot request locked the bench; stays true until reboot.
bool bench_net_locked(void);
