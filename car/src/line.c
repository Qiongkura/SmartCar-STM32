/**
 * line.c —— 8 路循迹读取
 *
 * 引脚顺序(对应 10P 座的 1~8 脚): PA2 PA3 PA4 PA5 PA6 PA7 PB0 PB1
 * 如果模块装反了(左右颠倒), 把 board.h 里的 LINE_REVERSE_ORDER 改成 1。
 * 如果模块是"压线输出高", 把 LINE_ACTIVE_LEVEL 改成 1。
 */
#include "line.h"
#include "board.h"

static GPIO_TypeDef *const line_port[8] = {
    GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB};
static const uint16_t line_pin[8] = {
    GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5,
    GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_0, GPIO_PIN_1};

static uint8_t bits = 0;
static float   error = 0.0f;
static bool    lost = true;
static uint8_t line_last_count = 0;
uint8_t        line_prev_count = 0;   /* 上一拍压线路数 */

void line_init(void) {
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
          GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &g);
}

/* 直接读 8 路电平 (不做误差计算): 给 1kHz 边沿扫描中断用 */
uint8_t line_read_raw(void) {
  uint8_t b = 0;
  for (int i = 0; i < 8; i++) {
    int idx = LINE_REVERSE_ORDER ? (7 - i) : i;
    uint8_t lv = (HAL_GPIO_ReadPin(line_port[idx], line_pin[idx]) == GPIO_PIN_SET) ? 1 : 0;
    if (lv == LINE_ACTIVE_LEVEL) b |= (uint8_t)(1u << i);
  }
  return b;
}

void line_update(void) {
  uint8_t b = line_read_raw();
  bits = b;

  int n = 0;
  float sum = 0.0f;
  for (int i = 0; i < 8; i++) {
    if (b & (1u << i)) {
      sum += (float)i;
      n++;
    }
  }
  line_prev_count = line_last_count;          /* 上一拍路数 (穿越捕捉的进入门) */
  line_last_count = (uint8_t)n;
  lost = (n == 0);
  if (n > 0) {
    float pos = sum / (float)n;          /* 0 ~ 7 */
    /* 探头中心相对车心差 LINE_SENSOR_OFFSET_CM: 不加这个偏置, 控制器
       会以为线在正中间, 实际车心固定偏一边(实测偏左 1cm)。
       按 e 的量纲折算: e=1 对应 3.5 个探头间距 */
    error = (pos - 3.5f) / 3.5f + LINE_SENSOR_OFFSET_CM / (3.5f * LINE_SENSOR_PITCH_CM);
    if (error >  1.0f) error =  1.0f;
    if (error < -1.0f) error = -1.0f;
  }
}

uint8_t line_bits(void)  { return bits; }
int     line_count(void) { int n = 0; for (int i = 0; i < 8; i++) if (bits & (1u << i)) n++; return n; }
float   line_error(void) { return error; }
bool    line_lost(void)  { return lost; }
