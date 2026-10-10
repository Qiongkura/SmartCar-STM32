/**
 * oled.h —— SSD1306 128x64 OLED (硬件 I2C1 重映射: PB8=SCL, PB9=SDA)
 * 裸寄存器实现, 不依赖 HAL 句柄。
 *
 * 带"影子页"比对: 刷新时只把内容真变了的那几页过 I2C,
 * 重复画同样的文字不产生任何总线流量, 不拖慢 20Hz 发送周期。
 * 开机探测不到屏时所有读写直接跳过, 遥控器照常发帧。
 */
#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_USE_SH1106 0     /* 1.3 寸屏 (SH1106) 改成 1, 列偏移 +2 */

void OLED_Init(void);
void OLED_Poll(void);                /* 主循环每圈调用: 屏拔掉/掉电后自动恢复 */
uint8_t OLED_Addr(void);             /* 探测到的实际地址 (0x78 或 0x7A) */
extern uint8_t oled_ok;              /* 1 = 屏应答了 */
void OLED_Clear(void);
void OLED_ClearLine(uint8_t y);      /* 清掉 y 开始的那一行(16 像素高) */
void OLED_Refresh(void);             /* 刷新所有有改动的页 (开机/换页用) */
void OLED_RefreshChunk(void);        /* 主循环每圈调用: 最多刷一页, 不卡发送 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t on);
void OLED_DrawChar(uint8_t x, uint8_t y, char ch);
void OLED_DrawString(uint8_t x, uint8_t y, const char *str);  /* ASCII 8x16 */
void OLED_Printf(uint8_t x, uint8_t y, const char *fmt, ...);

#endif
