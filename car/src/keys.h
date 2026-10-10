/**
 * keys.h —— 5 个板载按键 (低电平按下, 内部上拉)
 *
 * 事件类型:
 *   SHORT  —— 松手时产生 (按下时间 < 800ms)
 *   LONG   —— 按住超过 800ms 时产生一次
 *   REPEAT —— 继续按住后每 150ms 产生一次 (用来连续加减数值)
 */
#ifndef __KEYS_H
#define __KEYS_H

#include <stdbool.h>

typedef enum {
  KEY_MODE = 0,   /* K1: 切换模式 */
  KEY_UP,         /* K2: + */
  KEY_DOWN,       /* K3: - */
  KEY_SET,        /* K4: 方向 / 设置 */
  KEY_RUN,        /* K5: 启动/停止, 长按 = 陀螺重新校准 */
  KEY_COUNT
} key_id_t;

typedef enum {
  KEY_EV_NONE = 0,
  KEY_EV_SHORT,
  KEY_EV_LONG,
  KEY_EV_REPEAT
} key_event_t;

void keys_init(void);
void keys_scan(void);                    /* 每 10ms 调用一次 */
bool keys_pressed(key_id_t k);           /* 当前是否按下(已消抖) */
key_event_t keys_take_event(key_id_t k); /* 取事件, 取完自动清掉 */

#endif
