/**
 * soft_i2c.h —— 软件 I2C (只给 MPU6050 用: SCL=PA15, SDA=PB3)
 *
 * 为什么用软件 I2C: 这两个脚上没有任何硬件 I2C 外设能到
 * (I2C1 默认脚 PB6/PB7 给了编码器、重映射脚 PB8/PB9 给了 OLED,
 *  I2C2 的 PB10/PB11 给了按键和超声波), 只能软件模拟。
 */
#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include <stdint.h>

void soft_i2c_init(void);

/* 返回值: 0 = 成功, 非 0 = 从机没应答 */
int  soft_i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val);
int  soft_i2c_read_regs(uint8_t dev, uint8_t reg, uint8_t *buf, uint16_t len);
int  soft_i2c_probe(uint8_t dev);   /* 探测设备是否存在 */

#endif
