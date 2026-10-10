#include "oled.h"
#include "oledfont.h"
#include "font_cn.h"
#include "board.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 显存: 8 页 x 128 列, 1 字节 = 竖着 8 个像素, bit0 在最上面 */
static uint8_t gram[OLED_HEIGHT / 8][OLED_WIDTH];

/* 实际使用的从机地址。模块有 0x3C 和 0x3D 两种, 开机自动探测,
   省得因为地址不对整屏不亮还找不到原因。 */
uint8_t oled_addr = OLED_I2C_ADDR;
uint8_t oled_ok = 0;          /* 探测到屏幕 = 1 (调试时读这个变量就知道屏认到没有) */
static uint8_t dirty = 0xFF;      /* 每一位对应一页, 1 = 这页需要重发 */

#if OLED_USE_SH1106
  #define COL_OFFSET 2
#else
  #define COL_OFFSET 0
#endif

static void write_cmd(const uint8_t *cmd, uint16_t len) {
  HAL_I2C_Mem_Write(&hi2c1, oled_addr, 0x00, I2C_MEMADD_SIZE_8BIT,
                    (uint8_t *)cmd, len, OLED_I2C_TIMEOUT);
}

void OLED_Init(void) {
  static const uint8_t init_cmds[] = {
      0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
      0x8D, 0x14,              /* 电荷泵, 不开整屏不亮 */
      0x20, 0x02,              /* 页寻址 */
      0xA1, 0xC8,              /* 扫描方向 */
      0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40,
      0xA4, 0xA6, 0xAF};
  HAL_Delay(50);

  /* 先看 0x3C 在不在, 不在就试 0x3D */
  if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDR, 2, 30) == HAL_OK) {
    oled_ok = 1;
  } else if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDR_ALT, 2, 30) == HAL_OK) {
    oled_addr = OLED_I2C_ADDR_ALT;
    oled_ok = 1;
  }

  write_cmd(init_cmds, sizeof(init_cmds));
  memset(gram, 0, sizeof(gram));
  dirty = 0xFF;
  OLED_Refresh();
}

uint8_t OLED_Addr(void) { return oled_addr; }

void OLED_Clear(void) {
  memset(gram, 0, sizeof(gram));
  dirty = 0xFF;
}

void OLED_ClearLine(uint8_t y) {
  /* 一行文字高 16 像素, 在 SSD1306 里跨两页 (一页 8 像素), 两页都要清。
     只清一页的话下半截会留着上一次的内容, 看起来就是文字重叠、像卡住一样。 */
  uint8_t page = (uint8_t)(y >> 3);
  for (uint8_t p = page; p < (uint8_t)(page + 2) && p < OLED_HEIGHT / 8; p++) {
    memset(&gram[p][0], 0, OLED_WIDTH);
    dirty |= (uint8_t)(1u << p);
  }
}

void OLED_Refresh(void) {
  for (uint8_t page = 0; page < OLED_HEIGHT / 8; page++) {
    if (!(dirty & (1u << page))) continue;
    uint8_t addr[3] = {(uint8_t)(0xB0 | page),
                       (uint8_t)(COL_OFFSET & 0x0F),
                       (uint8_t)(0x10 | ((COL_OFFSET >> 4) & 0x0F))};
    write_cmd(addr, sizeof(addr));
    HAL_I2C_Mem_Write(&hi2c1, oled_addr, 0x40, I2C_MEMADD_SIZE_8BIT,
                      &gram[page][0], OLED_WIDTH, OLED_I2C_TIMEOUT);
  }
  dirty = 0;
}

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t on) {
  if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
  uint8_t page = (uint8_t)(y >> 3);
  if (on) gram[page][x] |= (uint8_t)(1u << (y & 7));
  else    gram[page][x] &= (uint8_t)~(1u << (y & 7));
  dirty |= (uint8_t)(1u << page);
}

void OLED_DrawChar(uint8_t x, uint8_t y, char ch) {
  if (ch < 0x20 || ch > 0x7E) ch = '?';
  const uint8_t *g = OLED_F8x16[(uint8_t)(ch - 0x20)];
  for (uint8_t row = 0; row < 16; row++) {
    uint8_t b = g[row];
    for (uint8_t col = 0; col < 8; col++) {
      if (b & (0x80 >> col)) OLED_DrawPoint((uint8_t)(x + col), (uint8_t)(y + row), 1);
    }
  }
}

/* 取下一个 UTF-8 字符的 Unicode 码点, 并让指针前进 */
static uint32_t utf8_next(const char **p) {
  const uint8_t *s = (const uint8_t *)*p;
  uint32_t cp = s[0];
  int extra;
  if (cp < 0x80) { (*p)++; return cp; }
  if ((cp & 0xE0) == 0xC0)      { cp &= 0x1F; extra = 1; }
  else if ((cp & 0xF0) == 0xE0) { cp &= 0x0F; extra = 2; }
  else if ((cp & 0xF8) == 0xF0) { cp &= 0x07; extra = 3; }
  else { (*p)++; return 0xFFFD; }
  for (int i = 0; i < extra; i++) {
    uint8_t c = s[1 + i];
    if ((c & 0xC0) != 0x80) { *p += 1 + i; return 0xFFFD; }
    cp = (cp << 6) | (c & 0x3F);
  }
  *p += 1 + extra;
  return cp;
}

static const uint8_t *cn_lookup(uint32_t cp) {
  for (int i = 0; i < CN_FONT_COUNT; i++) {
    if (cn_code[i] == cp) return cn_bitmap[i];
    if (cn_code[i] > cp) break;          /* 表是按码点升序排的 */
  }
  return NULL;
}

static void draw_glyph16(uint8_t x, uint8_t y, const uint8_t *bits) {
  for (uint8_t row = 0; row < 16; row++) {
    uint16_t w = (uint16_t)(((uint16_t)bits[row * 2] << 8) | bits[row * 2 + 1]);
    for (uint8_t col = 0; col < 16; col++) {
      if (w & (0x8000u >> col)) OLED_DrawPoint((uint8_t)(x + col), (uint8_t)(y + row), 1);
    }
  }
}

/* 支持中英文混排: ASCII 占 8 像素宽, 汉字占 16 像素宽, 都是 16 像素高 */
void OLED_DrawString(uint8_t x, uint8_t y, const char *str) {
  while (*str) {
    uint32_t cp = utf8_next(&str);
    if (cp < 0x80) {
      OLED_DrawChar(x, y, (char)cp);
      x = (uint8_t)(x + 8);
    } else {
      const uint8_t *g = cn_lookup(cp);
      if (g) {
        draw_glyph16(x, y, g);
        x = (uint8_t)(x + 16);
      }
      /* 字库里没有的字就不画, 换字后重新跑 tools/genfont_cn.py 即可 */
    }
    if (x > OLED_WIDTH - 8) break;
  }
}

void OLED_Printf(uint8_t x, uint8_t y, const char *fmt, ...) {
  char buf[40];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  OLED_DrawString(x, y, buf);
}
