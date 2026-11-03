#!/usr/bin/env python3
"""Compile actual bench parser, lease and leg worker with mocked SDK and buses.
No hardware, networking or flash. Run under ASan/UBSan.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1] / 'firmware/src'
STUB = r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>
typedef int uart_port_t; typedef int gpio_num_t; typedef int esp_err_t;
typedef int portMUX_TYPE; typedef unsigned TickType_t;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portMUX_INITIALIZE(p) (*(p)=0)
#define taskENTER_CRITICAL(p) ((void)(p))
#define taskEXIT_CRITICAL(p) ((void)(p))
#define pdMS_TO_TICKS(ms) (ms)
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM -2
#define ESP_ERR_TIMEOUT -3
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_ERROR_CHECK(x) assert((x)==0)
#define UART_NUM_0 0
#define UART_NUM_1 1
#define UART_NUM_2 2
#define GPIO_NUM_15 15
#define GPIO_NUM_16 16
#define GPIO_NUM_17 17
#define GPIO_NUM_18 18
#define GPIO_NUM_NC -1
#define GPIO_MODE_INPUT 0
#define GPIO_FLOATING 0
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define UART_PIN_NO_CHANGE -1
typedef struct {int baud_rate,data_bits,parity,stop_bits,flow_ctrl,source_clk;} uart_config_t;
int64_t esp_timer_get_time(void);
int uart_write_bytes(int,const void *,size_t);
static inline int uart_read_bytes(int p,void *b,size_t n,unsigned t){return 0;}
static inline void gpio_reset_pin(int p){}
static inline void gpio_set_direction(int p,int d){}
static inline void gpio_set_pull_mode(int p,int d){}
static inline void vTaskDelay(unsigned t){}
static inline void vTaskDelayUntil(unsigned *t,unsigned d){}
static inline unsigned xTaskGetTickCount(void){return 0;}
static inline int xTaskCreatePinnedToCore(void (*f)(void*),const char *n,unsigned s,void *a,unsigned p,void *h,int c){return 1;}
static inline int uart_driver_install(int a,int b,int c,int d,void *e,int f){return 0;}
static inline int uart_param_config(int a,const uart_config_t *b){return 0;}
static inline int uart_set_pin(int a,int b,int c,int d,int e){return 0;}
static inline int esp_ota_mark_app_valid_cancel_rollback(void){return 0;}
'''
HARNESS = r'''
#include <stdio.h>
#include <string.h>
#include "leg.h"
static int64_t now = 1000000;
static bool torque[3][4];
static int bad_torque_read, lost_speed, reply_level[3][4];
static unsigned char speed_goal[3][4][2];
static int lost_off, lost_enable, bad_zero, mit_count, fail_mit, read_status;
static char output_text[8192];
int64_t esp_timer_get_time(void){return now;}
int uart_write_bytes(int p,const void *b,size_t n){
  size_t at=strlen(output_text); assert(at+n<sizeof(output_text));memcpy(output_text+at,b,n);output_text[at+n]=0;return n;
}
esp_err_t joint_bus_init(joint_bus_t *b,int u,int p,int baud){b->uart=u;return 0;}
esp_err_t bus_ping(joint_bus_t *b,jb_format_t f,uint8_t id,joint_reply_t *r){*r=(joint_reply_t){.pos=100,.status=read_status};return 0;}
esp_err_t bus_read(joint_bus_t *b,jb_format_t f,uint8_t id,uint8_t reg,uint8_t n,uint8_t *out,joint_reply_t *r){
  memset(out,0,n);*r=(joint_reply_t){.pos=100,.status=read_status};
  if(reg==JB_REG_TORQUE_ENABLE){if(bad_torque_read && torque[b->uart][id])return ESP_FAIL;out[0]=torque[b->uart][id];}
  if(reg==JB_REG_RESPONSE_LEVEL)out[0]=reply_level[b->uart][id];
  if(reg==JB_WHEEL_REG_SPEED)memcpy(out,speed_goal[b->uart][id],2);
  if(reg==JB_WHEEL_REG_SPEED && bad_zero)out[0]=1;
  return 0;
}
esp_err_t bus_write(joint_bus_t *b,jb_format_t f,uint8_t id,uint8_t reg,const uint8_t *v,uint8_t n,joint_reply_t *r){
  *r=(joint_reply_t){.pos=100,.status=read_status};
  if(reg==JB_REG_TORQUE_ENABLE){torque[b->uart][id]=*v;if((*v && lost_enable)||(!*v && lost_off))return ESP_ERR_TIMEOUT;}
  if(reg==JB_REG_RESPONSE_LEVEL){reply_level[b->uart][id]=*v;return ESP_ERR_TIMEOUT;}
  if(reg==JB_WHEEL_REG_SPEED){memcpy(speed_goal[b->uart][id],v,2);if(lost_speed)return ESP_ERR_TIMEOUT;}
  return 0;
}
esp_err_t bus_write_mit(joint_bus_t *b,uint8_t id,const joint_mit_cmd_t *c,joint_reply_t *r){
  mit_count++;torque[b->uart][id]=1;*r=(joint_reply_t){.pos=100,.status=read_status};
  if(fail_mit){now+=21000;return -1;}return 0;
}
#include "bench_safety.c"
#include "leg.c"
#include "main.c"
static leg_t legs[2];
static void init(void){
  memset(legs,0,sizeof legs);memset(torque,0,sizeof torque);output_text[0]=0;
  bad_torque_read=lost_speed=0;memset(reply_level,0,sizeof reply_level);memset(speed_goal,0,sizeof speed_goal);
  lost_off=0;read_status=bad_zero=lost_enable=fail_mit=mit_count=0;
  bench_reset(now);s_sequence=0;
  for(int l=0;l<2;l++){
    leg_t *p=&legs[l];p->cfg=&LEG_CFG[l];p->bus.uart=l+1;s_legs[l]=p;
    for(int j=0;j<3;j++){
      p->j[j].cfg=&LEG_CFG[l].joint[j];p->j[j].configured=true;
      p->j[j].off_confirmed=p->j[j].homed=true;
      p->j[j].fb_us=now;p->j[j].fb.pos=100;
    }
    publish_view(p);
  }
}
static void cmd(const char *s){char buf[128];strcpy(buf,s);submit_command(buf);}
int main(void){
  // Exact lease boundary, sticky expiry even when heartbeat arrives before watchdog.
  init();assert(bench_live(now+19999));assert(!bench_live(now+20000));
  bench_renew(now+20001);assert(bench_fault());
  init();bench_renew(now+20000);assert(bench_fault());
  // Expired LF cannot emit an MIT frame; both workers request verified OFF.
  init();engage(&legs[0],&legs[0].j[1],bench_generation());assert(legs[0].j[1].enabled);
  now+=20000;assert(joint_cycle(&legs[0],&legs[0].j[1],now,.004f)!=0);assert(!mit_count);
  leg_step(&legs[0],now,.004f);leg_step(&legs[1],now,.004f);
  assert(!legs[0].j[1].enabled && legs[0].j[1].off_confirmed && !legs[1].j[2].homed);
  // Enable ACK lost AFTER driver energized: worker must retry OFF, never believe old OFF.
  init();lost_enable=1;bad_torque_read=1;engage(&legs[0],&legs[0].j[0],bench_generation());
  assert(torque[1][1] && bench_fault() && !legs[0].j[0].off_confirmed);
  bad_torque_read=0;leg_step(&legs[0],now,.004f);assert(!torque[1][1] && legs[0].j[0].off_confirmed);
  // A driver that omits OFF-write ACK is still verified by clean torque=0 readback.
  init();lost_off=1;assert(confirm_off(&legs[0],&legs[0].j[0]));
  assert(configure_wheel(&legs[0],&legs[0].j[0]));
  // Reply level 1 -> 2 is configured only while OFF and must read back as 2.
  init();reply_level[1][3]=1;assert(configure_joint(&legs[0],&legs[0].j[1]));
  assert(reply_level[1][3]==2 && !torque[1][3] && legs[0].j[1].response_level==2);
  // Missing wheel write ACK can recover via exact register readback, no false OFF.
  init();lost_enable=lost_speed=1;engage(&legs[0],&legs[0].j[0],bench_generation());
  assert(legs[0].j[0].enabled && torque[1][1] && !bench_fault());
  assert(wheel_write_speed(&legs[0],&legs[0].j[0],30,now)==ESP_OK);
  assert(speed_goal[1][1][0]==30);
  // Wheel goal must read back zero before enable.
  init();bad_zero=1;engage(&legs[0],&legs[0].j[0],bench_generation());assert(bench_fault() && !torque[1][1]);
  // A blocking failed first MIT exchange cannot retry an expired target.
  init();engage(&legs[0],&legs[0].j[1],bench_generation());fail_mit=1;
  assert(joint_cycle(&legs[0],&legs[0].j[1],now,.004f)!=0);assert(mit_count==1 && bench_fault());
  // Parser rejects fractional indices, nonfinite / extra values, wrong joint kind.
  const char *bad[]={"C 1 ON 1.5","C 2 W 0 nan","C 3 W 0 1 STOP","C 4 MOVE 0 1 1","C 5 JOG 2 .5 1"};
  init();for(unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++){output_text[0]=0;cmd(bad[i]);assert(strstr(output_text,"REJECT"));}
  // Stale sequence does not extend TTL. STOP cannot be undone by a heartbeat.
  init();cmd("P 10");now+=19000;cmd("P 9");now+=1000;assert(!bench_live(now));
  cmd("STOP");cmd("P 11");assert(bench_fault());
  // RESET removes old queued enables; ARM requires a newly captured HOME.
  init();leg_enable(&legs[0],1,true);cmd("C 1 RESET");assert(strstr(output_text,"QUEUED"));
  leg_step(&legs[0],now,.004f);leg_step(&legs[1],now,.004f);
  assert(!legs[0].j[1].enabled && !legs[0].j[1].homed);
  output_text[0]=0;cmd("C 2 ON 2");assert(strstr(output_text,"REJECT"));
  cmd("C 3 MOTOR_HOME");leg_step(&legs[0],now,.004f);leg_step(&legs[1],now,.004f);
  output_text[0]=0;cmd("C 4 ON 2");assert(strstr(output_text,"QUEUED"));
  leg_step(&legs[0],now,.004f);assert(legs[0].j[1].enabled);
  // A worker-local copy from before RESET cannot enable a motor afterward.
  init();leg_enable(&legs[0],1,true);joint_req_t old=legs[0].req[1];
  bench_reset(now);apply_request(&legs[0],&legs[0].j[1],&old);assert(!legs[0].j[1].enabled);
  // Nonzero driver status trips the global latch and both legs end OFF.
  init();engage(&legs[0],&legs[0].j[1],bench_generation());
  read_status=4;leg_step(&legs[0],now,.004f);assert(bench_fault() && !legs[0].j[1].enabled);
  read_status=0;leg_step(&legs[1],now,.004f);assert(!legs[1].j[1].homed);
  puts("PASS: actual bench lease, UART command parser and leg workers (mock motor buses)");
}
'''
with tempfile.TemporaryDirectory(prefix='motor-control-test-') as name:
    root = Path(name)
    (root/'stub.h').write_text(STUB)
    for name in ['driver/gpio.h','driver/uart.h','esp_err.h','esp_log.h','esp_timer.h',
                 'esp_ota_ops.h','freertos/FreeRTOS.h','freertos/task.h']:
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include "stub.h"\n')
    (root/'test.c').write_text(HARNESS)
    subprocess.run(['cc','-std=gnu11','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer',
                    '-no-pie','-I',str(root),'-I',str(ROOT/'libs/leg/include'),
                    '-I',str(ROOT/'libs/joint_bus/include'),'-I',str(ROOT/'libs/leg'),
                    '-I',str(ROOT/'app'), str(root/'test.c'),'-lm','-o',str(root/'check')],check=True)
    subprocess.run([str(root/'check')],check=True)
