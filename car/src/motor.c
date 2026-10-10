#include "motor.h"
#include "board.h"

/* 逻辑侧(左/右) -> 物理通道(A/B) */
#if MOTOR_A_IS_LEFT
  #define CH_OF_LEFT   0
  #define CH_OF_RIGHT  1
#else
  #define CH_OF_LEFT   1
  #define CH_OF_RIGHT  0
#endif

static const uint8_t out_rev[2] = {MOTOR_A_OUT_REVERSE, MOTOR_B_OUT_REVERSE};
static const uint8_t enc_rev[2] = {ENCODER_A_REVERSE, ENCODER_B_REVERSE};

static uint16_t enc_last[2] = {0, 0};

static GPIO_TypeDef *in1_port[2] = {MOTA_IN1_PORT, MOTB_IN1_PORT};
static uint16_t      in1_pin[2]  = {MOTA_IN1_PIN,  MOTB_IN1_PIN};
static GPIO_TypeDef *in2_port[2] = {MOTA_IN2_PORT, MOTB_IN2_PORT};
static uint16_t      in2_pin[2]  = {MOTA_IN2_PIN,  MOTB_IN2_PIN};

static void set_pwm(uint8_t ch, uint16_t compare) {
  /* 电机A = TIM1_CH4 (PA11), 电机B = TIM1_CH1 (PA8) */
  if (ch == 0) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, compare);
  else         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
}

void motor_init(void) {
  /* 方向脚 */
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin = MOTA_IN1_PIN | MOTA_IN2_PIN | MOTB_IN1_PIN | MOTB_IN2_PIN;
  HAL_GPIO_Init(GPIOB, &g);

  /* PWM: TIM1 CH1(PA8) 和 CH4(PA11), 20kHz */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  set_pwm(0, 0);
  set_pwm(1, 0);

  /* 编码器: TIM2 = 电机A (PA0/PA1), TIM4 = 电机B (PB6/PB7)
     HAL_TIM_Encoder_Init 会用 htim->Init 里的参数配置时基, 所以先填好,
     Period 给 0xFFFF 用满 16 位计数范围。 */
  TIM_Encoder_InitTypeDef e = {0};
  e.EncoderMode = TIM_ENCODERMODE_TI12;   /* 4 倍频 */
  e.IC1Polarity = TIM_ICPOLARITY_RISING;
  e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  e.IC1Prescaler = TIM_ICPSC_DIV1;
  e.IC1Filter = 10;
  e.IC2Polarity = TIM_ICPOLARITY_RISING;
  e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  e.IC2Prescaler = TIM_ICPSC_DIV1;
  e.IC2Filter = 10;

  htim4.Instance = TIM4;                  /* 必须赋, 否则 MspInit 里判断不成立, 时钟开不了 */
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Encoder_Init(&htim4, &e);

  htim2.Instance = TIM2;
  htim2.Init = htim4.Init;
  HAL_TIM_Encoder_Init(&htim2, &e);

  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  enc_last[0] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  enc_last[1] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

void motor_set_duty(int side, float duty) {
  uint8_t ch = (side == MOT_LEFT) ? CH_OF_LEFT : CH_OF_RIGHT;
  if (duty > 1.0f)  duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  if (out_rev[ch]) duty = -duty;

  /* 占空比很小的时候直接当 0, 避免电机嗡嗡响 */
  uint16_t cmp = (uint16_t)((duty < 0 ? -duty : duty) * (float)PWM_MAX + 0.5f);
  if (cmp < 8) cmp = 0;

  if (duty > 0.0001f) {
    HAL_GPIO_WritePin(in1_port[ch], in1_pin[ch], GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2_port[ch], in2_pin[ch], GPIO_PIN_RESET);
  } else if (duty < -0.0001f) {
    HAL_GPIO_WritePin(in1_port[ch], in1_pin[ch], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port[ch], in2_pin[ch], GPIO_PIN_SET);
  } else {
    /* 0: 两脚都低 = 断开, 轮子滑行 */
    HAL_GPIO_WritePin(in1_port[ch], in1_pin[ch], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port[ch], in2_pin[ch], GPIO_PIN_RESET);
  }
  set_pwm(ch, cmp);
}

void motor_coast(int side) { motor_set_duty(side, 0.0f); }

void motor_brake(int side) {
  uint8_t ch = (side == MOT_LEFT) ? CH_OF_LEFT : CH_OF_RIGHT;
  HAL_GPIO_WritePin(in1_port[ch], in1_pin[ch], GPIO_PIN_SET);
  HAL_GPIO_WritePin(in2_port[ch], in2_pin[ch], GPIO_PIN_SET);
  set_pwm(ch, PWM_MAX);
}

int16_t motor_read_delta(int side) {
  uint8_t ch = (side == MOT_LEFT) ? CH_OF_LEFT : CH_OF_RIGHT;
  /* 电机A 的编码器插座接的是 PA0/PA1 = TIM2; 电机B 接的是 PB6/PB7 = TIM4。
     这是逐个插座核对 PCB 网表得到的 (两个插座器件名相同, 不能想当然)。 */
  uint16_t now = (ch == 0) ? (uint16_t)__HAL_TIM_GET_COUNTER(&htim2)
                           : (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  /* 16 位环形差值: 不清零计数器, 只和上次的值作差, 自动处理回绕 */
  int16_t d = (int16_t)(now - enc_last[ch]);
  enc_last[ch] = now;
  if (enc_rev[ch]) d = (int16_t)(-d);
  return d;
}

uint16_t motor_raw_cnt(int side) {
  uint8_t ch = (side == MOT_LEFT) ? CH_OF_LEFT : CH_OF_RIGHT;
  return (ch == 0) ? (uint16_t)__HAL_TIM_GET_COUNTER(&htim2)
                   : (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
}
