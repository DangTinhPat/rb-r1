#include "reference_mcu_link.h"
#include <stdio.h>
#include <assert.h>
int main(int argc, char **argv) {
  (void)argv;
  if (argc > 1) {
    uint8_t b[280];
    size_t n = hcr_hello_ack(b, sizeof b, 42, 123456, 0x10203040);
    return fwrite(b, 1, n, stdout) == n ? 0 : 1;
  }
  HcrParser parser = {0};
  HcrFrame f;
  int c;
  while ((c = getchar()) != EOF)
    if (hcr_feed(&parser, (uint8_t)c, &f)) {
      HcrTarget t;
      if (hcr_target(&f, &t)) {
        HcrLink link = {0};
        link.connected = 1;
        link.epoch = t.epoch;
        link.last_telemetry_seq = t.source_seq;
        link.last_telemetry_us = 1000;
        uint8_t reply[280];
        hcr_link_receive(&link, &f, 1000, reply, sizeof reply);
        assert(hcr_link_live(&link, 20999));
        assert(!hcr_link_live(&link, 21000));
        assert(link.latched_fault);
        f.seq++;
        hcr_link_receive(&link, &f, 22000, reply, sizeof reply);
        assert(!hcr_link_live(&link, 22000));
        printf("%u %u %u %.9g %.9g\n", f.seq - 1, t.epoch, t.profile, t.target[0],
               t.feedforward[3]);
      }
    }
  return 0;
}
