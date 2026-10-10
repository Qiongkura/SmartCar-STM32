/**
 * main.c —— 第十八届单片机协会大二技术考核 · 智能小车
 *
 * 芯片: STM32F103C8T6 (核心板插在"单协二面小车"底板上)
 * 外设分配 (全部按 PCB 实际网表):
 *   OLED    : I2C1 重映射  PB8=SCL  PB9=SDA
 *   MPU6050 : 软件 I2C     PA15=SCL PB3=SDA   (JTAG 脚, 已关 JTAG)
 *   超声波  : TRIG=PB11    ECHO=PB4          (JTAG 脚)
 *   蓝牙    : USART1       PA9=TX   PA10=RX
 *   电机A   : PWM=PA11(TIM1_CH4) AIN1=PB13 AIN2=PB12  编码器=PB6/PB7(TIM4)
 *   电机B   : PWM=PA8 (TIM1_CH1) BIN1=PB14 BIN2=PB15  编码器=PA0/PA1(TIM2)
 *   按键×5  : PB10 PC13 PC14 PC15 PA12 (低电平按下)
 *   循迹×8  : PA2 PA3 PA4 PA5 PA6 PA7 PB0 PB1
 */
#include "main.h"

/* 烧录前身份核对: 电脑端先从 SWD 读这串字符, 不是 CAR-01 就拒绝烧录
   (发生过把小车固件烧进遥控器的事故, 这就是防再犯的) */
const volatile char FW_ID[] __attribute__((used)) = "CAR-01";

#include "app.h"
#include "board.h"
#include "util.h"
#include "vdd.h"
#include <string.h>

I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim1;      /* 电机 PWM */
TIM_HandleTypeDef  htim2;      /* 编码器 B */
TIM_HandleTypeDef  htim4;      /* 编码器 A */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

int main(void) {
  HAL_Init();
  if (FW_ID[0] != 'C') for (;;);   /* 永不成立: 防止链接器丢弃 FW_ID (烧录身份核对用) */
  SystemClock_Config();
  util_init();                  /* DWT 微秒计时 */
  MX_GPIO_Init();               /* 里面会关掉 JTAG, 必须在用 PA15/PB3/PB4 之前 */
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  vdd_init();                  /* 供电电压自测 (调试页显示) */

  app_init();                   /* 屏幕、按键、电机、编码器、陀螺、超声波 */

  while (1) {
    /* 蓝牙心跳: 每 200ms 发 1 字节 0xA5 (所有模式都发)。
       遥控器端超过 1.5s 收不到就判定断链, 自动驱动 KEY 重连模块。
       HAL_UART_Transmit 忙时直接返回 BUSY 跳过本拍, 不阻塞主循环。 */
    static uint32_t t_hb = 0;
    uint32_t now = HAL_GetTick();
    if (now - t_hb >= 200) {
      t_hb = now;
      if (!app_is_vofa_mode()) {       /* VOFA 走文本流, 二进制心跳会混进数据 */
        uint8_t hb = 0xA5;
        (void)HAL_UART_Transmit(&huart1, &hb, 1, 1);
      }
    }
    bt_rx_service();
    app_loop();
    app_idle_tasks();     /* 跑完/停车后把事件日志落 Flash (掉电不丢) */
  }
}

/* ================= 时钟: 外部 8MHz -> 72MHz ================= */
static void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.HSIState = RCC_HSI_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;    /* 36MHz */
  clk.APB2CLKDivider = RCC_HCLK_DIV1;    /* 72MHz */
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void HAL_MspInit(void) {
  __HAL_RCC_AFIO_CLK_ENABLE();           /* 重映射/关 JTAG 都要先开这个时钟 */
  __HAL_RCC_PWR_CLK_ENABLE();
}

/* ================= GPIO: 关 JTAG ================= */
static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /* 关键: PA15/PB3 要给 MPU6050 当软件 I2C, PB4 要给超声波 ECHO,
     这三个脚复位后是 JTAG 的 JTDI/JTDO/NJTRST, 不关掉 JTAG 就永远是复用功能。
     这里只关 JTAG, SWD (PA13/PA14) 保留, 所以还能用 ST-Link 下载调试。 */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

/* ================= OLED 用的硬件 I2C1 (PB8/PB9) ================= */

/* I2C 总线恢复: 如果上次复位时正好打断了 I2C 传输, 从机会一直把 SDA 拉低
   等时钟脉冲, 这时任何传输都会超时。开机先用 GPIO 手动打 9 个时钟脉冲
   把从机放开, 再补一个 STOP 条件。这是 I2C 的标准恢复流程。 */
static void i2c1_bus_recovery(void) {
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;      /* SCL / SDA */
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &g);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  delay_us(20);

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) {  /* SDA 被拉低 */
    for (int i = 0; i < 9; i++) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
      delay_us(10);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
      delay_us(10);
    }
    /* 补一个 STOP: SCL 为高时 SDA 由低跳到高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_us(10);
  }
}

static void MX_I2C1_Init(void) {
  i2c1_bus_recovery();                  /* 先清总线, 再交给硬件 I2C */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
  GPIO_InitTypeDef g = {0};
  if (hi2c->Instance != I2C1) return;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /* PCB 上 OLED 接的是 PB8/PB9, 也就是 I2C1 的重映射引脚 */
  __HAL_AFIO_REMAP_I2C1_ENABLE();
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode = GPIO_MODE_AF_OD;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &g);
}

/* ================= 蓝牙 HC-06 串口 ================= */
static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = BT_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef g = {0};
  if (huart->Instance != USART1) return;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  /* PA9 = TX: 单片机输出, 用复用推挽 */
  g.Pin = GPIO_PIN_9;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &g);

  /* PA10 = RX: 对单片机来说是输入! 必须配成输入模式(推荐上拉)。
     如果按 AF_PP 配, 输出驱动器会被使能并把这条线拉低,
     模块发回来的数据永远进不来 —— 之前"蓝牙无应答"就是这个原因。 */
  g.Pin = GPIO_PIN_10;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);

  /* VOFA+ 的数据是中断方式发的 (HAL_UART_Transmit_IT), 必须打开串口中断,
     否则数据只会停在缓冲区里发不出去 (实测踩过这个坑)。 */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ================= 串口接收 (模式七 蓝牙遥控用) =================
 * 用中断收字节, 存进环形缓冲, 上层用 bt_rx_get() 取。
 * UART 错误或缓冲溢出时丢弃整段残留，并通知上层停车、重新找帧头。
 * 每个字节记录中断接收时间，积压旧指令不能刷新遥控保活时间。
 */
#define BT_RX_BUF 256u
static volatile uint8_t bt_rx_buf[BT_RX_BUF];
static volatile uint32_t bt_rx_ms[BT_RX_BUF];
static volatile uint16_t bt_rx_head = 0, bt_rx_tail = 0;
static uint8_t bt_rx_byte;
static volatile bool bt_rx_on = false;
static volatile uint32_t bt_rx_fault = 0;
volatile uint32_t bt_uart_error_count = 0;
volatile uint32_t bt_uart_last_error = 0;
volatile uint32_t bt_rx_overflow_count = 0;
volatile uint32_t bt_rx_rearm_fail_count = 0;

static void bt_rx_rearm(void) {
  if (bt_rx_on && HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1) != HAL_OK) {
    ++bt_rx_rearm_fail_count;
    bt_rx_fault |= BT_RX_FAULT_REARM;
  }
}

void bt_rx_start(void) {
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  bt_rx_on = false;
  HAL_UART_AbortReceive(&huart1);
  __HAL_UART_CLEAR_OREFLAG(&huart1);  /* SR then DR also clears FE/NE/PE */
  bt_rx_head = bt_rx_tail = 0;
  bt_rx_fault |= BT_RX_FAULT_RESTART;
  bt_rx_on = true;
  bt_rx_rearm();
  __set_PRIMASK(irq);
}

void bt_rx_stop(void) {
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  bt_rx_on = false;
  HAL_UART_AbortReceive(&huart1);
  bt_rx_head = bt_rx_tail = 0;
  __set_PRIMASK(irq);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1 || !bt_rx_on) return;
  /* HAL invokes this BEFORE ErrorCallback on ORE/FE/NE with RXNE.
   * Receive_IT would clear ErrorCode: never rearm or publish that byte. */
  if (huart->ErrorCode != HAL_UART_ERROR_NONE) return;
  uint16_t next = (uint16_t)((bt_rx_head + 1u) % BT_RX_BUF);
  if (next == bt_rx_tail) {
    ++bt_rx_overflow_count;
    bt_rx_fault |= BT_RX_FAULT_OVERFLOW;
    bt_rx_tail = bt_rx_head;  /* queue has a hole: discard the whole fragment */
  } else {
    bt_rx_buf[bt_rx_head] = bt_rx_byte;
    bt_rx_ms[bt_rx_head] = HAL_GetTick();
    bt_rx_head = next;
  }
  bt_rx_rearm();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1 || !bt_rx_on) return;
  ++bt_uart_error_count;
  bt_uart_last_error = huart->ErrorCode;
  bt_rx_fault |= BT_RX_FAULT_UART;
  HAL_UART_AbortReceive(huart); /* FE/NE may still be BUSY_RX */
  __HAL_UART_CLEAR_OREFLAG(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  bt_rx_head = bt_rx_tail = 0;
  bt_rx_rearm();
}

void bt_rx_service(void) {
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  if (bt_rx_on && huart1.RxState != HAL_UART_STATE_BUSY_RX) {
    HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    bt_rx_head = bt_rx_tail = 0;
    bt_rx_fault |= BT_RX_FAULT_REARM;
    bt_rx_rearm();
  }
  __set_PRIMASK(irq);
}

uint32_t bt_rx_take_fault(void) {
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  uint32_t fault = bt_rx_fault;
  bt_rx_fault = 0;
  if (fault) bt_rx_head = bt_rx_tail = 0;
  __set_PRIMASK(irq);
  return fault;
}

int bt_rx_get_timed(uint8_t *out, uint32_t *ms) {
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  if (bt_rx_tail == bt_rx_head) { __set_PRIMASK(irq); return 0; }
  *out = bt_rx_buf[bt_rx_tail];
  *ms = bt_rx_ms[bt_rx_tail];
  bt_rx_tail = (uint16_t)((bt_rx_tail + 1u) % BT_RX_BUF);
  __set_PRIMASK(irq);
  return 1;
}
int bt_rx_get(uint8_t *out) { uint32_t ms; return bt_rx_get_timed(out, &ms); }

/* 现场改蓝牙波特率: HC-06 出厂是 9600, 很多人会改成 115200。
   测评时不允许重新烧录, 所以留一个按键切换的口子 (模式四里 K4 长按)。 */
void bt_set_baud(uint32_t baud) {
  if (huart1.Init.BaudRate == baud) return;
  bool restart_rx = bt_rx_on;
  bt_rx_stop();
  HAL_UART_DeInit(&huart1);
  huart1.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
  if (restart_rx) bt_rx_start();
}

/* ================= 给蓝牙模块发 AT 指令 =================
 * 板上的串口 (PA9/PA10) 直接连着蓝牙模块的 TXD/RXD, 所以固件可以直接跟它对话:
 *   "AT" 测试模块在不在、"AT+RESET" 让它复位重新广播、"AT+BAUD8" 改波特率。
 * 注意: 这个函数是阻塞的, 只在调试页手动触发, 不要在控制循环里调用。
 */
bool bt_at_cmd(uint32_t baud, const char *cmd, char *resp, uint8_t resp_len, uint32_t timeout_ms) {
  uint8_t rl = 0;
  const uint8_t crlf[2] = {13, 10};      /* 模块要求命令以 CRLF 结尾 */

  bt_set_baud(baud);
  /* AT 的应答这里是自己轮询 DR 收的, 必须先把中断接收停掉 ——
     否则中断服务程序会先把字节读走, 轮询永远读到空, 一直"无应答"。 */
  bt_rx_stop();

  /* 清掉接收缓冲里的残留 */
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
    (void)huart1.Instance->DR;
  }

  /* HC-06 命令模式通常要求裸 AT, 不带 CR/LF；部分兼容模块又要求 CR/LF。
     先发裸命令，短暂无回应再补发 CR/LF，两种模块都能自检。 */
  HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);

  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 120u && rl < (uint8_t)(resp_len - 1)) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
      char c = (char)(huart1.Instance->DR & 0xFF);
      if (c != 13 && c != 10 && c >= 32 && c < 127) resp[rl++] = c;
      t0 = HAL_GetTick();
    }
  }
  if (rl == 0) {
    HAL_UART_Transmit(&huart1, (uint8_t *)crlf, 2, 100);
  }
  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < timeout_ms && rl < (uint8_t)(resp_len - 1)) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
      char c = (char)(huart1.Instance->DR & 0xFF);
      if (c != 13 && c != 10 && c >= 32 && c < 127) resp[rl++] = c;
      t0 = HAL_GetTick();
    }
  }
  resp[rl] = 0;
  bt_rx_start();                          /* 恢复中断接收 */
  return rl > 0;
}

/* ================= 电机 PWM: TIM1 CH1(PA8) + CH4(PA11), 20kHz ================= */
static void MX_TIM1_Init(void) {
  TIM_OC_InitTypeDef oc = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;                 /* 72MHz */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_MAX - 1;          /* 72MHz/3600 = 20kHz */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
  GPIO_InitTypeDef g = {0};
  if (htim->Instance != TIM1) return;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  g.Pin = GPIO_PIN_8 | GPIO_PIN_11;         /* PA8=CH1, PA11=CH4 */
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &g);
}

/* ================= 编码器: TIM4(PB6/PB7) + TIM2(PA0/PA1) =================
 * 注意不要把 TIM2 重映射到 PA15/PB3, 那两个脚给 MPU6050 用了。
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim) {
  GPIO_InitTypeDef g = {0};
  /* 编码器输入脚用上拉: 电机没插时引脚悬空会拾取噪声, 数出一堆假脉冲,
     上拉以后悬空就是稳定的高电平, 不会再有假计数。
     (编码器输出是推挽的, 上拉不影响正常计数) */
  if (htim->Instance == TIM4) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
  } else if (htim->Instance == TIM2) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);
  }
}

/* ================= 中断 ================= */
void SysTick_Handler(void) {
  HAL_IncTick();
}

/* 超声波 ECHO (PB4) 的上升/下降沿 */
void EXTI4_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/* 蓝牙串口中断: 模式四的数据靠它发出去 */
void USART1_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart1);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
