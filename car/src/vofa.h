/**
 * vofa.h —— 模式四: 通过蓝牙串口向 VOFA+ 发送速度曲线
 *
 * 用 VOFA+ 的 FireWater 协议(纯文本): "目标速度,实际速度\n"
 * VOFA+ 里选 FireWater, 波特率与 board.h 的 BT_BAUD 一致即可。
 */
#ifndef __VOFA_H
#define __VOFA_H

void vofa_init(void);
void vofa_send2(float a, float b);        /* 两个通道 */
void vofa_send3(float a, float b, float c);
void vofa_send4(float a, float b, float c, float d);
void vofa_send_str(const char *s);        /* 发一行文本, 调试用 */

#endif
