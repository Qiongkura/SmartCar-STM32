/**
 * pwrlog.h —— 供电波形记录器 (专门用来查"电池供电时 3.3V 塌陷")
 *
 * 为什么需要它: 插着 ST-Link 调试时, ST-Link 的 3.3V 会给整块板供电,
 * 把"12V→5V→3.3V"这条可疑链路整个绕开 —— 什么问题都看不出来。
 * 正确做法是 ST-Link **只接 SWCLK/SWDIO/GND**(不接 3.3V), 板子用电池供电,
 * 然后让固件自己把供电电压录下来, 再用 SWD 把数据读出来分析。
 *
 * 用调试器触发 (和 bt_test_req 一个套路), 正常运行完全不参与:
 *     mww <pwrlog_period_us> 2000      # 采样间隔, 单位微秒
 *     mww <pwrlog_n> 512               # 采多少个点
 *     mww <pwrlog_req> 1               # 开始记录 (阻塞约 n*period)
 *     ... 等记录完 (pwrlog_done == 1) ...
 *     dump_image vref.bin <pwrlog_vref> 1024
 *
 * 记录的是 **VREFINT 的原始 ADC 读数**: 读数越大 = VDD 越低 (反比)。
 * 换算: VDD(mV) = 1200 * 4096 / raw   (用的是典型值 1.20V, 有 ±3% 误差)
 *
 * 每个点的 bit15 是芯片自带欠压检测器(PVD, 阈值 2.9V)的状态:
 *     1 = 这一瞬间 VDD 已经跌破 2.9V。这是硬件比较器的结论, 比换算值更可靠。
 */
#ifndef __PWRLOG_H
#define __PWRLOG_H

#include <stdint.h>

#define PWRLOG_N 512

extern volatile uint8_t  pwrlog_req;         /* 写 1 触发一次记录 */
extern volatile uint8_t  pwrlog_done;        /* 1 = 记录完成 */
extern volatile uint32_t pwrlog_period_us;   /* 采样间隔 us (默认 2000) */
extern volatile uint16_t pwrlog_n;           /* 本次记录点数 (<=512) */
extern volatile uint8_t  pwrlog_aux_en;      /* 1 = 同时采 PA0/PA1 (要先接分压电阻!) */

/* 记录结果 */
extern uint16_t pwrlog_vref[PWRLOG_N];       /* VREFINT 原始读数 | bit15=PVD 报警 */
extern uint16_t pwrlog_aux0[PWRLOG_N];       /* PA0 原始读数 (ADC_IN0) */
extern uint16_t pwrlog_aux1[PWRLOG_N];       /* PA1 原始读数 (ADC_IN1) */
extern volatile uint16_t pwrlog_count;       /* 实际记录点数 */
extern volatile uint32_t pwrlog_elapsed_ms;  /* 整段记录花了多少毫秒 */
extern volatile uint32_t pwrlog_pvd_low;     /* 记录期间 PVD 报警的点数 */
extern volatile uint32_t pwrlog_bad;         /* ADC 采样失败(读到 0)的点数, 不参与最值 */
extern volatile uint32_t pwrlog_lo_mv;       /* 最低电压 (mV, 由最大原始读数换算) */
extern volatile uint32_t pwrlog_hi_mv;       /* 最高电压 (mV, 由最小原始读数换算) */
extern volatile uint32_t pwrlog_csr_before;  /* 记录前的 RCC_CSR (复位标志) */
extern volatile uint32_t pwrlog_csr_after;   /* 记录后的 RCC_CSR: 若出现
                                                PORRSTF(bit1)/PINRSTF(bit2) 说明
                                                期间真的欠压复位过 */

void pwrlog_init(void);   /* 开机调用: 使能 PVD */
void pwrlog_run(void);    /* 执行一次记录 (阻塞, 期间车必须是停的) */

#endif
