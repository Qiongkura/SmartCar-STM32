/** HC-SR04: PB11 trigger, PB4 echo, DWT edge timestamps. */
#include "hcsr04.h"
#include "board.h"
#include "util.h"

#define SONAR_HIST SONAR_MEDIAN_N
#if SONAR_HIST != 3 && SONAR_HIST != 5
#error "SONAR_MEDIAN_N must be 3 or 5"
#endif
#define ECHO_TIMEOUT_MS 60u
#define MIN_ECHO_US ((uint32_t)(2.0f * SONAR_US_PER_CM))
#define MAX_ECHO_US ((uint32_t)(400.0f * SONAR_US_PER_CM))

typedef enum { ECHO_IDLE, ECHO_WAIT_RISE, ECHO_WAIT_FALL } echo_state_t;
static volatile echo_state_t echo_state = ECHO_IDLE;
static volatile uint16_t sonar_hist[SONAR_HIST];
static volatile uint8_t sonar_hist_i;
static volatile uint32_t t_rise_cyc;
static volatile uint32_t result_ms;
static volatile uint16_t last_us;
static volatile bool done;
static uint32_t trig_ms;

/* Called from EXTI, or from a short interrupt-protected timeout section. */
static void finish_measurement(uint32_t width) {
  uint16_t valid = (width >= MIN_ECHO_US && width <= MAX_ECHO_US)
                 ? (uint16_t)width : 0;
  uint32_t now = HAL_GetTick();
  echo_state = ECHO_IDLE;

  /* 上一笔结果已经过期(超过 SONAR_STALE_MS 没有新测量): 先把旧窗口清空,
     否则新样本会和几百毫秒前的旧值混在一起取中位数 —— 那个旧窗口
     不能因为来了新样本就"复活"。 */
  if (result_ms != 0 && (uint32_t)(now - result_ms) > SONAR_STALE_MS) {
    for (uint8_t i = 0; i < SONAR_HIST; ++i) sonar_hist[i] = 0;
    sonar_hist_i = 0;
  }

  last_us = valid;                 /* Invalid echoes must replace the old result. */
  sonar_hist[sonar_hist_i] = valid;
  sonar_hist_i = (uint8_t)((sonar_hist_i + 1) % SONAR_HIST);
  result_ms = now;
  done = true;                    /* Publish after the complete sample is stored. */
}

void hcsr04_init(void) {
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  echo_state = ECHO_IDLE;
  done = false;
  last_us = 0;
  sonar_hist_i = 0;
  result_ms = 0;
  for (uint8_t i = 0; i < SONAR_HIST; ++i) sonar_hist[i] = 0;

  g.Pin = TRIG_PIN;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TRIG_PORT, &g);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  g.Pin = ECHO_PIN;
  g.Mode = GPIO_MODE_IT_RISING_FALLING;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &g);
  __HAL_GPIO_EXTI_CLEAR_IT(ECHO_PIN);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void hcsr04_start(void) {
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  if (echo_state != ECHO_IDLE || done) {
    __set_PRIMASK(irq);
    return;
  }
  /* Arm before triggering. A timestamp of zero is valid, including at wrap. */
  __HAL_GPIO_EXTI_CLEAR_IT(ECHO_PIN);
  trig_ms = HAL_GetTick();
  echo_state = ECHO_WAIT_RISE;
  __set_PRIMASK(irq);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
  delay_us(12);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

bool hcsr04_ready(void) { return done; }

/* Uncompensated distance: use this for two-point calibration. */
float hcsr04_distance_raw_cm(void) {
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  uint16_t width = last_us;
  uint32_t stamp = result_ms;
  __set_PRIMASK(irq);
  if (!width || (uint32_t)(HAL_GetTick() - stamp) > SONAR_STALE_MS) return -1.0f;
  return (float)width / SONAR_US_PER_CM;
}

float hcsr04_distance_cm(void) {
  float raw = hcsr04_distance_raw_cm();
  return raw < 0.0f ? -1.0f : raw * SONAR_SCALE + SONAR_OFFSET_CM;
}

float hcsr04_distance_median_cm(void) {
  uint16_t v[SONAR_HIST];
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  uint32_t stamp = result_ms;
  for (uint8_t i = 0; i < SONAR_HIST; ++i) v[i] = sonar_hist[i];
  __set_PRIMASK(irq);
  if ((uint32_t)(HAL_GetTick() - stamp) > SONAR_STALE_MS) return -1.0f;

  for (uint8_t i = 1; i < SONAR_HIST; ++i) {
    uint16_t key = v[i];
    int j = (int)i - 1;
    while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; --j; }
    v[j + 1] = key;
  }
  uint16_t median = v[SONAR_HIST / 2];
  if (!median) return -1.0f;       /* A majority of invalid samples clears history. */
  return (float)median / SONAR_US_PER_CM * SONAR_SCALE + SONAR_OFFSET_CM;
}

void hcsr04_reset(void) { done = false; }

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (pin != ECHO_PIN) return;
  GPIO_PinState level = HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN);
  if (echo_state == ECHO_WAIT_RISE && level == GPIO_PIN_SET) {
    t_rise_cyc = util_cycles();
    echo_state = ECHO_WAIT_FALL;
  } else if (echo_state == ECHO_WAIT_FALL && level == GPIO_PIN_RESET) {
    finish_measurement(util_cycles_diff_us(t_rise_cyc));
  }
}

void hcsr04_tick(void) {
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  if (echo_state != ECHO_IDLE && (uint32_t)(HAL_GetTick() - trig_ms) > ECHO_TIMEOUT_MS)
    finish_measurement(0);
  __set_PRIMASK(irq);
}
