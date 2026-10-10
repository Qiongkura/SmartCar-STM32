#include "stm32f1xx_hal.h"
#include <stdint.h>

UART_HandleTypeDef huart1;

/* SWD 直接读这些变量即可判断车载蓝牙是否真的回数据。 */
const volatile char BT_TEST_ID[] __attribute__((used)) = "BT-TEST-01";
volatile uint32_t bt_tx_count = 0;
volatile uint32_t bt_frame_count = 0;
volatile uint32_t bt_scan_pass = 0;
volatile uint32_t bt_probe_baud = 0;
volatile uint32_t bt_hit_baud = 0;
volatile uint32_t bt_hit_count = 0;
volatile uint32_t bt_ok_count = 0;
volatile uint32_t bt_rx_count = 0;
volatile uint32_t bt_last_byte = 0;
volatile uint32_t bt_line_idle_high = 0;
volatile uint32_t bt_scan_index = 0;
volatile uint8_t  bt_resp_len = 0;
volatile char     bt_resp[64];

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void Error_Handler(void);

void SysTick_Handler(void) { HAL_IncTick(); }

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef g = {0};
  if (huart->Instance != USART1) return;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* PA9 = MCU TX -> HC-06 RXD */
  g.Pin = GPIO_PIN_9;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &g);

  /* PA10 = MCU RX <- HC-06 TXD */
  g.Pin = GPIO_PIN_10;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1) return;
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
  __HAL_RCC_USART1_CLK_DISABLE();
}

static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void SystemClock_Config(void) {
  /* 面包板裸 STM32 可能没有外部 8MHz 晶振，测试程序用内部 HSI 运行，
     这样只接 MCU + HC-05 也能启动 USART1。 */
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_OFF;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void clear_rx(void) {
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
    (void)huart1.Instance->DR;
  }
}

static uint32_t wait_response(uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();
  uint32_t n = 0;
  uint8_t saw_o = 0;

  while ((HAL_GetTick() - start) < timeout_ms) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
      uint8_t c = (uint8_t)(huart1.Instance->DR & 0xffu);
      bt_last_byte = c;
      bt_rx_count++;
      if (c == 'O') saw_o = 1;
      if (saw_o && c == 'K') bt_ok_count++;
      if (n < sizeof(bt_resp) - 1u) {
        bt_resp[n++] = (char)((c >= 32u && c < 127u) ? c : '.');
        bt_resp[n] = 0;
      }
    }
  }
  return n;
}

static uint32_t probe_baud(uint32_t baud) {
  static const uint8_t at[] = {'A', 'T'};
  static const uint8_t crlf[] = {'\r', '\n'};
  uint32_t n;

  HAL_UART_DeInit(&huart1);
  huart1.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
  clear_rx();

  bt_probe_baud = baud;
  bt_resp_len = 0;
  bt_resp[0] = 0;

  /* HC-06 常用裸 AT；兼容模块不回时再补 CRLF。 */
  HAL_UART_Transmit(&huart1, (uint8_t *)at, sizeof(at), 100);
  n = wait_response(120);
  if (n == 0) {
    HAL_UART_Transmit(&huart1, (uint8_t *)crlf, sizeof(crlf), 100);
    n = wait_response(180);
  }
  bt_resp_len = (uint8_t)(n > 63u ? 63u : n);
  return n;
}

int main(void) {
  /* 从机数据模式测试: 安全的停止帧, 不会让主机驱动车辆。 */
  static const uint8_t stop_frame[5] = {0xAA, 0x55, 0x05, 0x00, 0xFA};

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();       /* HC-06/HC-05 数据模式固定 9600 */
  bt_probe_baud = 9600u;

  for (;;) {
    HAL_UART_Transmit(&huart1, (uint8_t *)stop_frame, sizeof(stop_frame), 50);
    bt_tx_count += sizeof(stop_frame);
    bt_frame_count++;
    bt_line_idle_high = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET);

    uint32_t start = HAL_GetTick();
    uint32_t got = 0;
    while ((HAL_GetTick() - start) < 70u) {
      if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint8_t c = (uint8_t)(huart1.Instance->DR & 0xffu);
        bt_last_byte = c;
        bt_rx_count++;
        got++;
        if (bt_resp_len < sizeof(bt_resp) - 1u) {
          bt_resp[bt_resp_len++] = (char)((c >= 32u && c < 127u) ? c : '.');
          bt_resp[bt_resp_len] = 0;
        }
      }
    }
    if (got) {
      bt_hit_count++;             /* 收到数据的窗口数 */
      bt_hit_baud = 9600u;
    }
    bt_scan_pass++;
    HAL_Delay(30);
  }
}

static void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}
