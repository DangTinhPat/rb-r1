/* Test fixture only; not the production MCU state machine. */
#ifndef HCR_LINK_H
#define HCR_LINK_H
#include "hcr_protocol.h"
/* Call from a single UART task; publish the resulting target atomically to the servo task. */
typedef struct {
  uint32_t epoch, tx_seq, rx_seq, applied_seq, last_telemetry_seq;
  uint64_t last_telemetry_us, deadline_us;
  int connected, have_rx, enabled, latched_fault;
  HcrTarget target;
} HcrLink;

/* Process LF frame; returns reply length (HELLO_ACK), or 0.
   New HELLO always disables outputs; never re-arm automatically on reboot. */
static inline size_t hcr_link_receive(HcrLink *l, const HcrFrame *f, uint64_t now, uint8_t *reply,
                                      size_t cap) {
  if (f->type == HCR_HELLO && f->size == 12 && hcr_u16(f->payload + 4) == 200 &&
      hcr_u16(f->payload + 6) == 100 && hcr_u32(f->payload + 8) == HCR_SCHEMA) {
    uint32_t epoch = hcr_u32(f->payload);
    if (epoch != l->epoch || !l->connected) {
      memset(l, 0, sizeof(*l));
      l->epoch = epoch;
      l->connected = 1;
    }
    return hcr_hello_ack(reply, cap, l->tx_seq++, now, l->epoch);
  }
  if (l->connected && f->type == HCR_CAL_IMU && f->size == 8 &&
      hcr_u32(f->payload) == l->epoch &&
      (!l->have_rx || hcr_newer(f->seq,l->rx_seq))) {
    uint32_t result=HCR_CAL_IMU_ACCEPTED;
    if (l->enabled) result=HCR_CAL_IMU_NOT_OFF;
    else if (l->latched_fault) result=HCR_CAL_IMU_REFUSED;
    else if (hcr_u32(f->payload+4)!=l->last_telemetry_seq ||
             now-l->last_telemetry_us>30000) result=HCR_CAL_IMU_FEEDBACK;
    l->rx_seq=f->seq; l->have_rx=1;
    uint8_t payload[12]; hcr_put32(payload,l->epoch); hcr_put32(payload+4,f->seq);
    hcr_put32(payload+8,result);
    return hcr_encode(reply,cap,HCR_CAL_IMU_ACK,l->tx_seq++,now,payload,sizeof payload);
  }
  HcrTarget t;
  if (!l->connected || !hcr_target(f, &t) || t.epoch != l->epoch)
    return 0;
  if (l->have_rx && !hcr_newer(f->seq, l->rx_seq))
    return 0;
  if (t.profile != HCR_DISABLED) {
    if (l->latched_fault)
      return 0;
    // Don't accept targets referring to unsent or >30 ms old telemetry.
    if (hcr_newer(t.source_seq, l->last_telemetry_seq) ||
        (uint32_t)(l->last_telemetry_seq - t.source_seq) > 8 || now - l->last_telemetry_us > 30000)
      return 0;
  }
  l->target = t;
  l->rx_seq = f->seq;
  l->have_rx = 1;
  l->deadline_us = now + HCR_TARGET_TTL_US;
  l->enabled = t.profile != HCR_DISABLED;
  return 0;
}
/* The servo task calls this every tick. Also check physical E-stop and driver faults. */
static inline int hcr_link_live(HcrLink *l, uint64_t now) {
  if (!l->enabled)
    return 0;
  if (now >= l->deadline_us) {
    l->enabled = 0;
    l->latched_fault = 1;
    return 0;
  }
  return 1;
}
/* Call only after the servo task has actually applied the corresponding target. */
static inline void hcr_link_applied(HcrLink *l, uint32_t target_seq) {
  l->applied_seq = target_seq;
}

static inline size_t hcr_link_telemetry(HcrLink *l, uint8_t *out, size_t cap, uint64_t sample_us,
                                        uint32_t flags, const float values[29]) {
  if (!l->connected)
    return 0;
  l->last_telemetry_seq = l->tx_seq++;
  l->last_telemetry_us = sample_us;
  if (l->latched_fault)
    flags |= HCR_DRIVER_FAULT;
  return hcr_telemetry(out, cap, l->last_telemetry_seq, sample_us, l->epoch, flags, l->applied_seq,
                       values);
}
/* Send on boot/reboot when no fresh LF session has arrived; retry at 500 ms. */
static inline size_t hcr_request_session(uint8_t *out, size_t cap, uint64_t now) {
  uint8_t p[12];
  hcr_put32(p, 0);
  hcr_put16(p + 4, 200);
  hcr_put16(p + 6, 100);
  hcr_put32(p + 8, HCR_SCHEMA);
  return hcr_encode(out, cap, HCR_HELLO, 0, now, p, 12);
}
#endif
