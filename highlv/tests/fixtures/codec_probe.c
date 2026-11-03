#include "reference_mcu_link.h"
#include <stdio.h>
#include <assert.h>
int main(int argc, char **argv) {
  if (argc > 1 && strcmp(argv[1], "diagnostics") == 0) {
    uint8_t b[280]; uint32_t values[HCR_EXECUTION_DIAG_WORDS];
    for(unsigned i=0;i<HCR_EXECUTION_DIAG_WORDS;++i) values[i]=i;
    values[0]=7; values[1]=HCR_EXECUTION_DIAG_VERSION;
    size_t n=hcr_execution_diagnostics(b,sizeof b,9,123456,values);
    return fwrite(b,1,n,stdout)==n ? 0 : 1;
  }
  if (argc > 1 && strcmp(argv[1], "home") == 0) {
    uint8_t b[280], payload[12];
    hcr_put32(payload, 7); hcr_put32(payload+4, 1);
    size_t n = hcr_encode(b, sizeof b, HCR_ENCODER_HOME, 3, 3, payload, 8);
    if (fwrite(b, 1, n, stdout) != n) return 1;
    hcr_put32(payload+4, 3); hcr_put32(payload+8, HCR_HOME_APPLIED);
    n = hcr_encode(b, sizeof b, HCR_ENCODER_HOME_ACK, 2, 100001, payload, 12);
    return fwrite(b, 1, n, stdout) == n ? 0 : 1;
  }
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
