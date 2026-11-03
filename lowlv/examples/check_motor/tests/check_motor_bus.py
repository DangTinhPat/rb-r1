#!/usr/bin/env python3
"""Host checks of the actual joint_bus.c with mocked ESP-IDF UART/GPIO only.

No board/UART is opened. Golden MIT vectors follow section 7 of the supplied
HTML specification; its first TX checksum is corrected from 0x64 to 0x69.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

STUB = r'''
#ifndef ESP_TEST_STUB_H
#define ESP_TEST_STUB_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef int esp_err_t; typedef int uart_port_t; typedef int gpio_num_t;
typedef unsigned TickType_t;
typedef struct {int unused;} uart_dev_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define taskENTER_CRITICAL(m) ((void)(m))
#define taskEXIT_CRITICAL(m) ((void)(m))
extern uart_dev_t UART_HW;
#define UART_LL_GET_HW(num) ((void)(num), &UART_HW)
int64_t esp_timer_get_time(void);
static inline void esp_rom_delay_us(uint32_t us){(void)us;}
bool uart_ll_is_tx_idle(uart_dev_t *);
uint32_t uart_ll_get_txfifo_len(uart_dev_t *);
void uart_ll_write_txfifo(uart_dev_t *, const uint8_t *, uint32_t);
void uart_ll_txfifo_rst(uart_dev_t *);
typedef struct {int baud_rate,data_bits,parity,stop_bits,flow_ctrl,source_clk;} uart_config_t;
#define ESP_OK 0
#define ESP_ERR_TIMEOUT 1
#define ESP_ERR_INVALID_RESPONSE 2
#define ESP_ERR_INVALID_SIZE 3
#define ESP_ERR_INVALID_ARG 4
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define UART_PIN_NO_CHANGE -1
#define GPIO_MODE_INPUT_OUTPUT 3
#define GPIO_PULLUP_ONLY 1
#define SIG_GPIO_OUT_IDX 0
#define SOC_UART_TX_PIN_IDX 0
#define UART_PERIPH_SIGNAL(u,s) ((unsigned)(u)+(unsigned)(s))
#define pdMS_TO_TICKS(x) (x)
#define ESP_RETURN_ON_ERROR(expr,tag,...) do {(void)(tag);int e=(expr);if(e)return e;}while(0)
extern int GPIO;
static inline void esp_rom_gpio_connect_out_signal(int a,unsigned b,bool c,bool d){(void)a;(void)b;(void)c;(void)d;}
static inline void gpio_ll_output_disable(int *a,int b){(void)a;(void)b;}
static inline int uart_driver_install(int a,int b,int c,int d,void *e,int f){(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;return 0;}
static inline int uart_param_config(int a,const uart_config_t *b){(void)a;(void)b;return 0;}
static inline int uart_set_pin(int a,int b,int c,int d,int e){(void)a;(void)b;(void)c;(void)d;(void)e;return 0;}
static inline int gpio_set_direction(int a,int b){(void)a;(void)b;return 0;}
static inline int gpio_set_level(int a,int b){(void)a;(void)b;return 0;}
static inline int gpio_set_pull_mode(int a,int b){(void)a;(void)b;return 0;}
static inline int uart_set_rx_timeout(int a,int b){(void)a;(void)b;return 0;}
int uart_flush_input(uart_port_t);
int uart_read_bytes(uart_port_t,void *,size_t,TickType_t);
#endif
'''

HARNESS = r'''
#include "joint_bus.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int GPIO;
uart_dev_t UART_HW;
static unsigned char incoming[128], response[64], sent[64];
static size_t used, at, response_len, sent_len;
static int corrupt_echo, tx_result, tx_stuck;
static int64_t clock_us;
static bool deny_motion(uint32_t g){(void)g;return false;}
int64_t esp_timer_get_time(void){return clock_us+=50;}
int uart_flush_input(int p){(void)p;used=at=0;return 0;}
bool uart_ll_is_tx_idle(uart_dev_t *hw){(void)hw;return !tx_stuck;}
uint32_t uart_ll_get_txfifo_len(uart_dev_t *hw){(void)hw;return 128;}
void uart_ll_txfifo_rst(uart_dev_t *hw){(void)hw;tx_stuck=0;}
void uart_ll_write_txfifo(uart_dev_t *hw,const uint8_t *data,uint32_t n){
  (void)hw;assert(n<=sizeof sent);memcpy(sent,data,n);sent_len=n;
  memcpy(incoming,data,n);if(corrupt_echo)incoming[0]^=1;
  memcpy(incoming+n,response,response_len);used=n+response_len;tx_stuck=tx_result!=0;
}
int uart_read_bytes(int p,void *data,size_t n,TickType_t t){
  (void)p;(void)t;if(n>used-at)n=used-at;memcpy(data,incoming+at,n);at+=n;return (int)n;
}
static void prepare(const unsigned char *bytes,size_t n){
  assert(n<=sizeof response);if(n)memcpy(response,bytes,n);response_len=n;
  corrupt_echo=tx_result=0;
}
static void same(const unsigned char *expected,size_t n){assert(sent_len==n&&!memcmp(sent,expected,n));}
int main(void){
  joint_bus_t b;assert(joint_bus_init(&b,1,17,1000000)==ESP_OK);joint_reply_t r;
  // Manufacturer HTML section 7.1: checksum in its TX example is wrong.
  const unsigned char mit_rx[]={0xff,0xff,1,7,0,0,0x10,0x0d,3,0x1a,0xbd};
  const unsigned char mit_tx[]={0xff,0xff,1,0x0d,3,0x1a,0x10,0x27,0x24,0,0,0x10,0,0,0,0,0x69};
  joint_mit_cmd_t cmd={10000,36,4096,0,0};prepare(mit_rx,sizeof mit_rx);
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_OK);same(mit_tx,sizeof mit_tx);
  assert(r.pos==4096&&r.status==0);
  // Section 7.2: position-only write, correct documented checksum and position.
  const unsigned char pos_rx[]={0xff,0xff,1,7,0,0x40,0x1f,5,3,0x1e,0x72};
  const unsigned char pos_tx[]={0xff,0xff,1,5,3,0x1e,0,0x20,0xb8};
  const unsigned char position[]={0,0x20};prepare(pos_rx,sizeof pos_rx);
  assert(bus_write(&b,JB_FMT_JOINT,1,0x1e,position,2,&r)==ESP_OK);
  same(pos_tx,sizeof pos_tx);assert(r.pos==8000);
  // Wheel format has no embedded joint position before the echoed instruction.
  const unsigned char wheel_rx[]={0xff,0xff,1,0x0b,0,4,2,0x24,0,2,0xc8,0,0xf6,0xff,0x0a};
  const unsigned char wheel_tx[]={0xff,0xff,1,4,2,0x24,6,0xce};
  unsigned char out[48];prepare(wheel_rx,sizeof wheel_rx);
  assert(bus_read(&b,JB_FMT_WHEEL,1,0x24,6,out,&r)==ESP_OK);same(wheel_tx,sizeof wheel_tx);
  assert(out[0]==0&&out[1]==2&&out[2]==0xc8&&out[3]==0&&out[4]==0xf6&&out[5]==0xff);
  // Timeout / bad echo / bad checksum are distinct errors, not successful apply.
  prepare(NULL,0);assert(bus_ping(&b,JB_FMT_JOINT,1,&r)==ESP_ERR_TIMEOUT);assert(b.err_timeout==1);
  prepare(mit_rx,sizeof mit_rx);corrupt_echo=1;
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_ERR_INVALID_RESPONSE);assert(b.err_echo==1);
  prepare(mit_rx,sizeof mit_rx);response[response_len-1]^=1;
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_ERR_INVALID_RESPONSE);assert(b.err_frame==1);
  prepare(mit_rx,sizeof mit_rx);tx_result=ESP_ERR_TIMEOUT;
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_ERR_TIMEOUT);assert(b.err_timeout==2);
  // A reply cut short is a timeout and is also counted as partial.
  prepare(mit_rx,5);assert(bus_write_mit(&b,1,&cmd,&r)==ESP_ERR_TIMEOUT);
  assert(b.err_timeout==3&&b.err_partial==1);
  // Bytes still arriving after a failed exchange are drained and counted late.
  prepare(NULL,0);assert(bus_ping(&b,JB_FMT_JOINT,1,&r)==ESP_ERR_TIMEOUT);
  const uint32_t late=b.err_late;
  memcpy(incoming+used,mit_rx,sizeof mit_rx);used+=sizeof mit_rx;
  prepare(mit_rx,sizeof mit_rx);assert(bus_write_mit(&b,1,&cmd,&r)==ESP_OK);assert(b.err_late==late+1);
  // A broadcast has TX echo, but no motor reply. Oversized reads are rejected.
  const unsigned char off=0;prepare(NULL,0);
  assert(bus_write(&b,JB_FMT_JOINT,0xfe,0x18,&off,1,&r)==ESP_OK);
  assert(bus_read(&b,JB_FMT_JOINT,0xfe,0x24,2,out,&r)==ESP_ERR_INVALID_ARG);
  assert(bus_read(&b,JB_FMT_JOINT,1,0x24,sizeof out,out,&r)==ESP_ERR_INVALID_SIZE);
  // Captured on the robot: the wheel reply lost its first FF; checksum still valid.
  const unsigned char wheel_lost_sync[]={0xff,1,0x0b,0,4,2,0x24,0x2e,3,0,0,0xfb,0xff,0x9e};
  prepare(wheel_lost_sync,sizeof wheel_lost_sync);
  assert(bus_read(&b,JB_FMT_WHEEL,1,0x24,6,out,&r)==ESP_OK&&b.sync_recovered==1);
  assert(out[0]==0x2e&&out[1]==3&&out[4]==0xfb&&out[5]==0xff);
  // One lost FF plus a corrupted byte must still be rejected by the checksum.
  unsigned char bad_sync[sizeof wheel_lost_sync];memcpy(bad_sync,wheel_lost_sync,sizeof bad_sync);
  bad_sync[8]^=1;prepare(bad_sync,sizeof bad_sync);
  assert(bus_read(&b,JB_FMT_WHEEL,1,0x24,6,out,&r)==ESP_ERR_INVALID_RESPONSE);
  // Final FIFO guard rejects expired motion, while OFF remains available.
  b.allow_motion=deny_motion;sent_len=0;
  prepare(mit_rx,sizeof mit_rx);
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_ERR_TIMEOUT && !sent_len);
  prepare(NULL,0);assert(bus_write(&b,JB_FMT_JOINT,0xfe,0x18,&off,1,&r)==ESP_OK);
  puts("PASS: actual joint_bus.c; MIT/position/wheel frames, echo, reply offset, checksum, timeout, broadcast, late reply and lost sync byte");
  return 0;
}
'''


def main():
    # The example checksum can be verified without depending on the codec under test.
    documented = bytes.fromhex('01 0D 03 1A 10 27 24 00 00 10 00 00 00 00')
    assert (~sum(documented) & 0xff) == 0x69
    with tempfile.TemporaryDirectory(prefix='bl4012e-codec-') as directory:
        build = Path(directory)
        (build/'esp_stub.h').write_text(STUB)
        for name in ['driver/gpio.h','driver/uart.h','esp_err.h','esp_check.h',
                     'esp_rom_gpio.h','esp_rom_sys.h','esp_timer.h','freertos/FreeRTOS.h','hal/gpio_ll.h','hal/uart_ll.h',
                     'soc/gpio_sig_map.h','soc/uart_periph.h']:
            target = build/name
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text('#include "esp_stub.h"\n')
        (build/'probe.c').write_text(HARNESS)
        binary = build/'probe'
        subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror',
                        '-fsanitize=address,undefined','-g','-I'+str(build),
                        '-I'+str(ROOT/'firmware/src/libs/joint_bus/include'), str(ROOT/'firmware/src/libs/joint_bus/joint_bus.c'),
                        str(build/'probe.c'),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
    print('PASS: HTML MIT example checksum 0x64 is a documentation error; expected 0x69')


if __name__ == '__main__':
    main()
