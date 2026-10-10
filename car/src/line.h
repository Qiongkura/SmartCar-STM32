/**
 * line.h —— 8 路循迹传感器 (10P 座, 1~8 脚)
 *
 * 位置误差 line_error(): 线在车正中间时约等于 0, 线偏左为负、偏右为正, 范围 ±1。
 * 全灭(丢线)时返回上一次的值, 由上层决定怎么处理。
 */
#ifndef __LINE_H
#define __LINE_H

#include <stdbool.h>
#include <stdint.h>

void    line_init(void);
void    line_update(void);
uint8_t line_bits(void);      /* 每一位代表一路, bit0 = 最左那一路, 1=压线 */
uint8_t line_read_raw(void);  /* 直接读电平 (中断里快速扫描用) */
int     line_count(void);     /* 压线的路数 */
float   line_error(void);     /* -1.0 ~ +1.0 */
bool    line_lost(void);
extern uint8_t line_prev_count;   /* 上一拍压线路数 */      /* 一路都没压线 */

#endif
