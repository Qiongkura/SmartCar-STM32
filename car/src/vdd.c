#include "vdd.h"
#include "stm32f1xx_hal.h"

/* 说明: 标准 STM32F103 的信息区 0x1FFFF7BA 存有 VREFINT 出厂校准值,
   但这块板上的芯片读出来是全 0xFF (信息区为空, 常见于兼容/国产芯片),
   所以改用数据手册的典型值 1.20V 来反推。精度约 ±0.1V ——
   判断"电池供电时电压掉没掉"完全够用。 */
#define VREFINT_MV_NOMINAL 1200U

static ADC_HandleTypeDef hadc1;

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();
  }
}

void vdd_init(void) {
  RCC_PeriphCLKInitTypeDef pclk = {0};
  pclk.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  pclk.AdcClockSelection = RCC_ADCPCLK2_DIV6;      /* 72MHz/6 = 12MHz, ADCCLK 上限 14MHz */
  HAL_RCCEx_PeriphCLKConfig(&pclk);

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) return;
  HAL_ADCEx_Calibration_Start(&hadc1);
}

/* 采一次指定通道的原始读数 (0~4095)。
   供电波形记录器 (pwrlog.c) 要连续采几千次, 所以单独拆出来。 */
uint16_t vdd_read_raw(uint32_t channel) {
  ADC_ChannelConfTypeDef ch = {0};
  ch.Channel = channel;
  ch.Rank = ADC_REGULAR_RANK_1;
  ch.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;   /* 内部参考源内阻大, 采样要够长 */
  if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) return 0;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return 0;
  }
  uint32_t raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return (uint16_t)raw;
}

float vdd_read(void) {
  uint16_t raw = vdd_read_raw(ADC_CHANNEL_VREFINT);
  if (raw == 0) return 0.0f;
  /* VDD = VREFINT / (raw / 4096) = 1.2V * 4096 / raw */
  return (float)VREFINT_MV_NOMINAL / 1000.0f * 4096.0f / (float)raw;
}
