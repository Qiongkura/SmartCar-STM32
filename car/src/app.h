/**
 * app.h —— 应用层: 6 个模式的调度、按键交互、OLED 界面
 */
#ifndef __APP_H
#define __APP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  MODE_HEADING = 1,   /* 模式一 方向保持与自动纠偏 */
  MODE_TURN    = 2,   /* 模式二 指定角度转向 */
  MODE_DIST    = 3,   /* 模式三 里程计 */
  MODE_VOFA    = 4,   /* 模式四 速度曲线 (VOFA+) */
  MODE_LINE    = 5,   /* 模式五 循线行驶 N 圈 */
  MODE_OBJECT  = 6,   /* 模式六 物体计数与测距 */
  MODE_RC      = 7,   /* 模式七 蓝牙遥控 (断联自动停车) */
  MODE_DEBUG   = 8    /* 调试页 (长按 K1 进入, 标定用; 不占模式编号) */
} car_mode_t;

/* 短按 K1 循环的模式个数 (调试页不在其中, 它靠长按 K1 进出) */
#define MODE_COUNT 7

void app_init(void);
void app_loop(void);          /* 在主循环里反复调用 */
void app_idle_tasks(void);    /* 空闲任务: 落盘事件日志 */
uint8_t app_is_vofa_mode(void); /* 是否模式四(VOFA, 串口走文本) */

#endif
