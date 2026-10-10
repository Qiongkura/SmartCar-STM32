/**
 * main.h —— 公共声明
 */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* 外设句柄 (main.c 中定义) */
extern I2C_HandleTypeDef  hi2c1;   /* OLED: I2C1 重映射 PB8/PB9 */
extern UART_HandleTypeDef huart1;  /* 蓝牙 HC-06: PA9/PA10 */
extern TIM_HandleTypeDef  htim1;   /* 电机 PWM: PA8(CH1) PA11(CH4) */
extern TIM_HandleTypeDef  htim2;   /* 编码器B: PA0/PA1 */
extern TIM_HandleTypeDef  htim4;   /* 编码器A: PB6/PB7 */

void Error_Handler(void);

/* 运行时改蓝牙串口波特率 (模式四现场切换用, 不用重新烧录) */
void bt_set_baud(uint32_t baud);

/* 给蓝牙模块发 AT 指令并收回应 (阻塞式, 最多等 timeout_ms 毫秒)。
   返回 true 表示收到回应, 回应内容写进 resp。 */
bool bt_at_cmd(uint32_t baud, const char *cmd, char *resp, uint8_t resp_len, uint32_t timeout_ms);

/* ---- 串口接收 (模式七 蓝牙遥控用) ---- */
#define BT_RX_FAULT_UART     1u
#define BT_RX_FAULT_OVERFLOW 2u
#define BT_RX_FAULT_REARM    4u
#define BT_RX_FAULT_RESTART  8u
void bt_rx_service(void);
uint32_t bt_rx_take_fault(void);
int bt_rx_get_timed(uint8_t *out, uint32_t *ms);
void bt_rx_start(void);            /* 开中断接收 */
void bt_rx_stop(void);             /* 关中断接收 (AT 自检期间用) */
int  bt_rx_get(uint8_t *out);      /* 取一个收到的字节; 返回 0 = 没有新数据 */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
