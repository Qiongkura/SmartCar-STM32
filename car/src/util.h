/**
 * util.h —— 微秒延时和计时 (用 Cortex-M3 的 DWT 周期计数器, 72MHz)
 */
#ifndef __UTIL_H
#define __UTIL_H

#include <stdint.h>

void     util_init(void);       /* 使能 DWT, 必须在 main 开头调用一次 */
uint32_t util_micros(void);     /* 开机以来的微秒数 (约 71.6 分钟回绕, 只适合看大致时间) */

/* 需要算"两件事之间隔了多久"时用这两个, 不要用 util_micros() 相减:
   CYCCNT 是 32 位周期计数, 每 2^32/72MHz ≈ 59.65 秒回绕一次;
   util_micros() 已经除过 72, 它的回绕点和 CYCCNT 对不上,
   所以先相减再除才不会在回绕时算出错误的间隔。 */
uint32_t util_cycles(void);                        /* 读原始周期计数 */
uint32_t util_cycles_diff_us(uint32_t start);      /* 距 start 过了多少微秒 (自动处理回绕) */
void     delay_us(uint32_t us);

#endif
