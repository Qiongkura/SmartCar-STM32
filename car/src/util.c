#include "util.h"
#include "stm32f1xx_hal.h"

#define CPU_MHZ 72U

void util_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t util_micros(void) {
  return DWT->CYCCNT / CPU_MHZ;
}

uint32_t util_cycles(void) {
  return DWT->CYCCNT;
}

/* 先做 32 位无符号差值(回绕自动正确), 再换算成微秒 */
uint32_t util_cycles_diff_us(uint32_t start) {
  return (DWT->CYCCNT - start) / CPU_MHZ;
}

void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * CPU_MHZ;
  while ((DWT->CYCCNT - start) < ticks) {
  }
}
