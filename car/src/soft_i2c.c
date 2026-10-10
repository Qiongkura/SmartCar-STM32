#include "soft_i2c.h"
#include "board.h"
#include "util.h"

/* 约 100kHz: 半周期 5us */
#define I2C_DELAY() delay_us(5)

#define SCL_H() HAL_GPIO_WritePin(MPU_SCL_PORT, MPU_SCL_PIN, GPIO_PIN_SET)
#define SCL_L() HAL_GPIO_WritePin(MPU_SCL_PORT, MPU_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H() HAL_GPIO_WritePin(MPU_SDA_PORT, MPU_SDA_PIN, GPIO_PIN_SET)
#define SDA_L() HAL_GPIO_WritePin(MPU_SDA_PORT, MPU_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ() HAL_GPIO_ReadPin(MPU_SDA_PORT, MPU_SDA_PIN)

void soft_i2c_init(void) {
  GPIO_InitTypeDef g = {0};
  /* PA15 和 PB3 是 JTAG 脚, 调用前 main.c 里已经关掉 JTAG。
     注意: 这两个脚在**不同端口**上, 必须分开初始化 ——
     如果拿同一个 Pin 掩码 (PIN15|PIN3) 去初始化两个端口,
     会连带把 PA3(循迹输入) 和 PB15(电机B方向) 也改成开漏输出, 那两个功能就废了。 */
  g.Mode = GPIO_MODE_OUTPUT_OD;      /* 开漏, 靠上拉电阻拉高 */
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin = MPU_SCL_PIN;               /* 只初始化 PA15 */
  HAL_GPIO_Init(MPU_SCL_PORT, &g);
  g.Pin = MPU_SDA_PIN;               /* 只初始化 PB3 */
  HAL_GPIO_Init(MPU_SDA_PORT, &g);
  SCL_H();
  SDA_H();
}

static void i2c_start(void) {
  SDA_H();
  SCL_H();
  I2C_DELAY();
  SDA_L();
  I2C_DELAY();
  SCL_L();
  I2C_DELAY();
}

static void i2c_stop(void) {
  SDA_L();
  SCL_H();
  I2C_DELAY();
  SDA_H();
  I2C_DELAY();
}

/* 返回 0 = 收到应答 */
static int i2c_write_byte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 0x80) SDA_H(); else SDA_L();
    b <<= 1;
    I2C_DELAY();
    SCL_H();
    I2C_DELAY();
    SCL_L();
  }
  SDA_H();                 /* 释放 SDA 读应答 */
  I2C_DELAY();
  SCL_H();
  I2C_DELAY();
  int ack = SDA_READ();    /* 0 = ACK */
  SCL_L();
  I2C_DELAY();
  return ack;
}

static uint8_t i2c_read_byte(int ack_next) {
  uint8_t b = 0;
  SDA_H();
  for (uint8_t i = 0; i < 8; i++) {
    b <<= 1;
    SCL_H();
    I2C_DELAY();
    if (SDA_READ()) b |= 1;
    SCL_L();
    I2C_DELAY();
  }
  if (ack_next) SDA_L(); else SDA_H();   /* 0=ACK, 1=NACK */
  I2C_DELAY();
  SCL_H();
  I2C_DELAY();
  SCL_L();
  SDA_H();
  I2C_DELAY();
  return b;
}

int soft_i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val) {
  i2c_start();
  if (i2c_write_byte(dev << 1)) { i2c_stop(); return 1; }
  if (i2c_write_byte(reg))      { i2c_stop(); return 2; }
  if (i2c_write_byte(val))      { i2c_stop(); return 3; }
  i2c_stop();
  return 0;
}

int soft_i2c_read_regs(uint8_t dev, uint8_t reg, uint8_t *buf, uint16_t len) {
  i2c_start();
  if (i2c_write_byte(dev << 1)) { i2c_stop(); return 1; }
  if (i2c_write_byte(reg))      { i2c_stop(); return 2; }
  i2c_start();                              /* 重复起始 */
  if (i2c_write_byte((dev << 1) | 1)) { i2c_stop(); return 3; }
  for (uint16_t i = 0; i < len; i++) {
    buf[i] = i2c_read_byte(i < (uint16_t)(len - 1));
  }
  i2c_stop();
  return 0;
}

int soft_i2c_probe(uint8_t dev) {
  i2c_start();
  int ack = i2c_write_byte(dev << 1);
  i2c_stop();
  return ack;
}
