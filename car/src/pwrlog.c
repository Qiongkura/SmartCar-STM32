#include "pwrlog.h"
#include "board.h"
#include "util.h"
#include "vdd.h"

/* ================= 可调参数 ================= */
#define PWRLOG_DEF_PERIOD_US  2000    /* 默认 2ms 一个点 -> 512 点约 1 秒 */
#define PWRLOG_MIN_PERIOD_US  60      /* 一次 ADC 转换约 30us, 再快没意义 */
/* =========================================== */

volatile uint8_t  pwrlog_req = 0;
volatile uint8_t  pwrlog_done = 0;
volatile uint32_t pwrlog_period_us = PWRLOG_DEF_PERIOD_US;
volatile uint16_t pwrlog_n = PWRLOG_N;
volatile uint8_t  pwrlog_aux_en = 0;

uint16_t pwrlog_vref[PWRLOG_N];
uint16_t pwrlog_aux0[PWRLOG_N];
uint16_t pwrlog_aux1[PWRLOG_N];
volatile uint16_t pwrlog_count = 0;
volatile uint32_t pwrlog_elapsed_ms = 0;
volatile uint32_t pwrlog_pvd_low = 0;
volatile uint32_t pwrlog_bad = 0;
volatile uint32_t pwrlog_lo_mv = 0;
volatile uint32_t pwrlog_hi_mv = 0;
volatile uint32_t pwrlog_csr_before = 0;
volatile uint32_t pwrlog_csr_after = 0;

void pwrlog_init(void) {
  /* 芯片自带的可编程电压检测器 (PVD): 门槛选最高档 2.9V。
     3.3V 轨正常时它不报警, 一旦掉破 2.9V, PWR_CR 的 PVDO 位就是 1 ——
     这是硬件比较器的结论, 不受我们那个 ±3% 换算误差影响。 */
  PWR_PVDTypeDef pvd = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  pvd.PVDLevel = PWR_PVDLEVEL_7;   /* 这一版 HAL 里 7 = 最高档 2.9V (PWR_CR_PLS_2V9) */
  pvd.Mode = PWR_PVD_MODE_NORMAL;
  HAL_PWR_ConfigPVD(&pvd);
  HAL_PWR_EnablePVD();
}

/* 采辅助通道时要把 PA0/PA1 从"编码器输入"改成"模拟输入";
   采完再改回输入模式 (编码器就是把它当输入用的)。
   注意: 这两个脚在正式装车后接的是编码器 A/B 相, 只有面包板上空着的时候才能用。 */
static void aux_pins(int analog) {
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  g.Mode = analog ? GPIO_MODE_ANALOG : GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);
}

void pwrlog_run(void) {
  uint16_t n = pwrlog_n;
  if (n == 0 || n > PWRLOG_N) n = PWRLOG_N;

  uint32_t period = pwrlog_period_us;
  if (period < PWRLOG_MIN_PERIOD_US) period = PWRLOG_MIN_PERIOD_US;
  if (period > 200000u) period = 200000u;

  pwrlog_done = 0;

  /* 先记下复位标志再清掉: 记录完之后如果又出现 PORRSTF, 说明这段
     时间里真的发生过欠压复位 (芯片掉到约 2.0V 以下会自己重启) */
  pwrlog_csr_before = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  if (pwrlog_aux_en) aux_pins(1);

  uint32_t lo = 0xFFFFFFFFu, hi = 0, low_cnt = 0, bad_cnt = 0;
  uint32_t t0 = HAL_GetTick();

  for (uint16_t i = 0; i < n; i++) {
    uint32_t t_start = util_cycles();

    uint16_t v = vdd_read_raw(ADC_CHANNEL_VREFINT);
    uint16_t raw12 = (uint16_t)(v & 0x0FFFu);
    if (raw12 == 0) {
      bad_cnt++;                          /* 采样失败(读到 0): 单独标记, 不参与最值 */
    } else {
      if (raw12 > hi) hi = raw12;
      if (raw12 < lo) lo = raw12;
    }
    /* PVD 是硬件比较器, 和 ADC 采样成不成功无关, 单独判 */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != 0) {
      low_cnt++;
      v |= 0x8000u;                       /* bit15 = 这一瞬间已经跌破 2.9V */
    }
    pwrlog_vref[i] = v;

    if (pwrlog_aux_en) {
      pwrlog_aux0[i] = vdd_read_raw(ADC_CHANNEL_0);   /* PA0 */
      pwrlog_aux1[i] = vdd_read_raw(ADC_CHANNEL_1);   /* PA1 */
    }

    while (util_cycles_diff_us(t_start) < period) { }   /* 等到下一个采样点 */
  }

  pwrlog_count = n;
  pwrlog_elapsed_ms = HAL_GetTick() - t0;
  pwrlog_pvd_low = low_cnt;
  pwrlog_bad = bad_cnt;
  /* 原始读数 -> mV: VDD = 1.2V × 4096 / raw —— 读数越小, 电压越高。
     所以"最高电压"要用最小的原始读数算, "最低电压"要用最大的算。
     (之前这两行写反了) */
  pwrlog_hi_mv = (lo != 0xFFFFFFFFu) ? (1200u * 4096u / lo) : 0;
  pwrlog_lo_mv = hi ? (1200u * 4096u / hi) : 0;

  if (pwrlog_aux_en) aux_pins(0);

  pwrlog_csr_after = RCC->CSR;
  pwrlog_done = 1;
}
