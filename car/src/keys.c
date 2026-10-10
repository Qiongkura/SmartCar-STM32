/**
 * keys.c —— 5 个按键 (低电平按下, 内部上拉)
 *
 * 消抖用"积分滤波"而不是"连续相同才算数":
 *   低电平给计数器加分, 高电平减分, 累计到阈值才改变状态。
 *   按键抖动时计数器在中间徘徊, 状态不变 —— 对接触不良/飞线按键最稳。
 *
 * 事件: SHORT(松手时产生) / LONG(按住 1 秒产生一次) / REPEAT(之后每 0.3 秒一次)
 *
 * 每个键还记了诊断计数 (按了几次 / 最近按了多久 / 连加几次),
 * 出问题时可以用调试器读出来, 判断是按键硬件问题还是固件问题。
 */
#include "keys.h"
#include "board.h"

#define FILTER_MAX    6     /* 积分计数器上限 (6 次扫描 = 60ms) */
#define FILTER_UP     3     /* 低电平累计到这算"按下" (30ms, 原50ms太钝) */
#define FILTER_DN     1     /* 高电平累计到这算"松开" (留迟滞防抖) */
#define LONG_MS       1000  /* 按住超过它才算长按 */
#define REPEAT_MS     300   /* 长按之后每隔这么久重复一次 */
#define LOCKOUT_MS    120   /* 松开后锁定期 (原350ms导致连按没反应; 120ms仍可压抖动) */

typedef struct {
  GPIO_TypeDef *port;
  uint16_t      pin;
  uint8_t       stable;     /* 消抖后的电平: 0=按下 */
  uint8_t       cnt;        /* 积分滤波计数器 */
  uint32_t      press_ms;
  bool          long_sent;
  uint32_t      last_repeat;
  uint32_t      lock_until;
  key_event_t   event;
  /* --- 诊断用 --- */
  uint16_t      press_cnt;     /* 累计被按下多少次 */
  uint16_t      last_held_ms;  /* 最近一次按了多久(ms) */
  uint16_t      repeat_cnt;    /* 累计连加了多少次 */
} key_t;

/* 诊断日志: 记录每个键最近 8 次按下的时刻和持续时间。
   用调试器读这两个数组, 就能看出"一次物理按下"被固件记成了几次、间隔多少毫秒:
   间隔几十毫秒的连续几条 = 接触抖动; 间隔几百毫秒以上 = 人真的按了多次。 */
uint16_t key_log_t[KEY_COUNT][8];   /* 按下时刻 (HAL_GetTick 的低 16 位) */
uint16_t key_log_d[KEY_COUNT][8];   /* 这次按了多久 (ms) */
uint8_t  key_log_i[KEY_COUNT];      /* 写指针 */

static key_t keys[KEY_COUNT] = {
    {KEY_MODE_PORT, KEY_MODE_PIN, 1, 0, 0, false, 0, 0, KEY_EV_NONE, 0, 0, 0},
    {KEY_UP_PORT,   KEY_UP_PIN,   1, 0, 0, false, 0, 0, KEY_EV_NONE, 0, 0, 0},
    {KEY_DOWN_PORT, KEY_DOWN_PIN, 1, 0, 0, false, 0, 0, KEY_EV_NONE, 0, 0, 0},
    {KEY_SET_PORT,  KEY_SET_PIN,  1, 0, 0, false, 0, 0, KEY_EV_NONE, 0, 0, 0},
    {KEY_RUN_PORT,  KEY_RUN_PIN,  1, 0, 0, false, 0, 0, KEY_EV_NONE, 0, 0, 0},
};

void keys_init(void) {
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;      /* 按键另一端接 GND, 靠内部上拉 */
  g.Speed = GPIO_SPEED_FREQ_LOW;

  g.Pin = KEY_MODE_PIN;                    /* K1: PA12 */
  HAL_GPIO_Init(KEY_MODE_PORT, &g);
  g.Pin = KEY_UP_PIN;                      /* K2: PB10 */
  HAL_GPIO_Init(KEY_UP_PORT, &g);
  g.Pin = KEY_DOWN_PIN | KEY_SET_PIN | KEY_RUN_PIN;   /* K3/K4/K5: PC13/14/15 */
  HAL_GPIO_Init(GPIOC, &g);
}

void keys_scan(void) {
  uint32_t now = HAL_GetTick();

  for (int i = 0; i < KEY_COUNT; i++) {
    key_t *k = &keys[i];
    uint8_t raw = (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_RESET) ? 0 : 1;

    /* 积分滤波: 低电平加分, 高电平减分 */
    if (raw == 0) {
      if (k->cnt < FILTER_MAX) k->cnt++;
    } else {
      if (k->cnt > 0) k->cnt--;
    }

    if (k->stable == 1) {                    /* 当前: 松开 */
      if (k->cnt >= FILTER_UP) {
        if (now < k->lock_until) {
          k->cnt = FILTER_UP - 1;            /* 锁定窗口内, 先不认这次按下 */
        } else {
          k->cnt = FILTER_MAX;
          k->stable = 0;
          k->press_ms = now;
          k->long_sent = false;
          k->last_repeat = now;
          if (k->press_cnt < 65535) k->press_cnt++;
        }
      }
    } else {                                 /* 当前: 按下 */
      if (k->cnt <= FILTER_DN) {
        uint32_t held = now - k->press_ms;
        k->cnt = 0;
        k->stable = 1;
        k->last_held_ms = (held > 65535) ? 65535 : (uint16_t)held;
        /* 记一条日志 */
        uint8_t li = key_log_i[i];
        key_log_t[i][li] = (uint16_t)(k->press_ms & 0xFFFF);
        key_log_d[i][li] = k->last_held_ms;
        key_log_i[i] = (uint8_t)((li + 1) & 7);
        if (!k->long_sent) k->event = KEY_EV_SHORT;
        k->lock_until = now + LOCKOUT_MS;
      } else {
        uint32_t held = now - k->press_ms;
        if (!k->long_sent && held >= LONG_MS) {
          k->event = KEY_EV_LONG;
          k->long_sent = true;
          k->last_repeat = now;
        } else if (k->long_sent && (now - k->last_repeat) >= REPEAT_MS) {
          k->event = KEY_EV_REPEAT;
          k->last_repeat = now;
          if (k->repeat_cnt < 65535) k->repeat_cnt++;
        }
      }
    }
  }
}

bool keys_pressed(key_id_t k) {
  return (k < KEY_COUNT) && (keys[k].stable == 0);
}

key_event_t keys_take_event(key_id_t k) {
  if (k >= KEY_COUNT) return KEY_EV_NONE;
  key_event_t e = keys[k].event;
  keys[k].event = KEY_EV_NONE;
  return e;
}
