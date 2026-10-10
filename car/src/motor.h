/**
 * motor.h —— TB6612 电机驱动 + 编码器读取
 *
 * 逻辑上只分左右两侧: MOT_LEFT / MOT_RIGHT
 * 到底哪一侧是物理的"电机A", 由 board.h 里的 MOTOR_A_IS_LEFT 决定。
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

#define MOT_LEFT   0
#define MOT_RIGHT  1

void motor_init(void);

/* duty: -1.0 ~ +1.0, 正数 = 车往前开 */
void motor_set_duty(int side, float duty);

void motor_coast(int side);   /* 断开: 轮子自由滑行 */
void motor_brake(int side);   /* 短接刹车 */

/* 读取并清空"上次调用以来"的编码器增量 (带符号, 前进为正) */
int16_t motor_read_delta(int side);

/* 硬件计数器原始值 (16位环形, 不清零不转向修正)。
   调试页显示用: 手转轮子看两个编码器每圈计数是否一致 —— 核对左右标定。 */
uint16_t motor_raw_cnt(int side);

#endif
