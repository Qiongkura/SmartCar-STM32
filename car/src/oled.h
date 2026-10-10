/**
 * oled.h —— SSD1306 128x64 OLED (I2C1 重映射: PB8=SCL, PB9=SDA)
 *
 * 带"脏页"记录: 只有内容变化的页才会通过 I2C 重发,
 * 避免每帧都刷 1KB 把控制周期拖长。
 */
#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#define OLED_WIDTH   128
#define OLED_HEIGHT  64

void OLED_Init(void);
uint8_t OLED_Addr(void);             /* 探测到的实际地址 (0x78 或 0x7A) */
extern uint8_t oled_ok;              /* 1 = 屏应答了 */
void OLED_Clear(void);
void OLED_ClearLine(uint8_t y);      /* 清掉 y 开始的那一行(16 像素高) */
void OLED_Refresh(void);             /* 只刷新有改动的页 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t on);
void OLED_DrawChar(uint8_t x, uint8_t y, char ch);
void OLED_DrawString(uint8_t x, uint8_t y, const char *str);
void OLED_Printf(uint8_t x, uint8_t y, const char *fmt, ...);

#endif
