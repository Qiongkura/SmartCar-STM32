/**
 * hcsr04.h —— 超声波测距 (TRIG=PB11, ECHO=PB4)
 *
 * ECHO 用外部中断 + DWT 微秒计时测脉宽, 不占用 CPU 等待, 不影响控制周期。
 * 调用方式: 每 100ms 左右调一次 hcsr04_start(), 然后随时读 hcsr04_distance_cm()。
 */
#ifndef __HCSR04_H
#define __HCSR04_H

#include <stdbool.h>

void  hcsr04_init(void);
void  hcsr04_start(void);            /* 发起一次测量 */
bool  hcsr04_ready(void);            /* 本次测量是否已完成 */
float hcsr04_distance_raw_cm(void);  /* 未加比例/零点补偿, 用于标定; 无效或过期返回 -1 */
float hcsr04_distance_cm(void);      /* 最近一次测完、经过补偿的距离; 无效或过期返回 -1 */
float hcsr04_distance_median_cm(void); /* 最近 3 次取中位数, 抗跳变, 推荐用这个 */
void  hcsr04_reset(void);            /* 清掉 ready 标志, 准备下次测量 */
void  hcsr04_tick(void);             /* 每 10ms 调一次, 处理无回波超时 */

#endif
