/**
 * vdd.h —— 用 STM32 内部的参考电压测量自己的供电电压 (VDD)
 *
 * F103 芯片内部有一个出厂校准过的参考电压 (VREFINT, 约 1.20V) 和一个
 * 12 位 ADC, 测出 VREFINT 的 ADC 读数就能反推 VDD:
 *     VDD = 3.3V × 厂校准值 / 实测读数
 *
 * 用途: 排查"电池供电时电压掉下去"这类问题 —— 屏幕上直接显示电压,
 * 来回切换供电就能看出掉没掉, 不用反复拿万用表量。
 */
#ifndef __VDD_H
#define __VDD_H

#include <stdint.h>

void  vdd_init(void);
float vdd_read(void);      /* 返回 VDD, 单位 V; 失败返回 0 */

/* 采一次指定 ADC 通道的原始读数 (0~4095), 给供电波形记录器 (pwrlog.c) 用 */
uint16_t vdd_read_raw(uint32_t channel);

#endif
