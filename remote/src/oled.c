/**
 * oled.c —— SSD1306 128x64 OLED 驱动 (裸寄存器 + 软件 I2C, 从 SmartCar 项目移植)
 *
 * PB8=SCL / PB9=SDA, GPIO 开漏位翻转实现 I2C (≈200kHz)。
 * 为什么不用硬件 I2C1 外设: 这块板上外设版"地址应答正常、多字节数据传输
 * 必然卡死"(TXE/BTF 等不到, 原因未明), 而用 SWD 手工位翻转整条链路全通,
 * 于是果断换软实现——和小车 MPU6050 的软 I2C 同一套思路。
 *
 * 带"影子页"比对: 刷新时只把内容真变了的那几页过 I2C, 重复画同样的文字
 * 不产生任何总线流量。屏没接/拔掉时所有写直接跳过, 遥控器照常发帧。
 */
#include "oled.h"
#include "oledfont.h"
#include "stm32f1xx.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 显存: 8 页 x 128 列, 1 字节 = 竖着 8 个像素, bit0 在最上面 */
static uint8_t gram[OLED_HEIGHT / 8][OLED_WIDTH];
/* 已上屏的影子页: Refresh 逐页比对, 内容没变的页连 I2C 都不走 */
static uint8_t shadow[OLED_HEIGHT / 8][OLED_WIDTH];

/* 模块有 0x3C 和 0x3D 两种地址, 开机自动探测 */
#define OLED_ADDR_3C 0x78      /* 0x3C << 1 */
#define OLED_ADDR_3D 0x7A      /* 0x3D << 1 */

uint8_t oled_addr = OLED_ADDR_3C;
uint8_t oled_ok   = 0;         /* 探测到屏 = 1 (调试时读这个变量就知道屏认到没有) */
static uint8_t wr_fail = 0;    /* 任何一次写失败都置位, 触发 OLED_Poll 自愈 */

#if OLED_USE_SH1106
  #define COL_OFFSET 2
#else
  #define COL_OFFSET 0
#endif

extern uint32_t millis(void);  /* main.c 的 SysTick 毫秒计 */

/* ---------------- 软件 I2C (GPIO 位翻转, 开漏) ---------------- */

#define SCL_H()  (GPIOB->BSRR = (1u << 8))
#define SCL_L()  (GPIOB->BRR  = (1u << 8))
#define SDA_H()  (GPIOB->BSRR = (1u << 9))
#define SDA_L()  (GPIOB->BRR  = (1u << 9))
#define SDA_IN() ((GPIOB->IDR >> 9) & 1u)

static void i2c_dly(void) { for (volatile int i = 0; i < 14; i++) {} }   /* ≈1.2µs → SCL ≈350kHz */

static void i2c_start(void) {
  SDA_H(); SCL_H(); i2c_dly();
  SDA_L(); i2c_dly();
  SCL_L(); i2c_dly();
}

static void i2c_stop(void) {
  SCL_L(); SDA_L(); i2c_dly();
  SCL_H(); i2c_dly();
  SDA_H(); i2c_dly();
}

/* 发一个字节, 返回 0=ACK 1=NACK */
static uint8_t i2c_tx(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    if ((b >> i) & 1) SDA_H(); else SDA_L();
    i2c_dly();
    SCL_H(); i2c_dly();
    SCL_L();
  }
  SDA_H();                 /* 释放 SDA 给从机回 ACK */
  i2c_dly();
  SCL_H(); i2c_dly();
  uint8_t nack = (uint8_t)SDA_IN();
  SCL_L();
  return nack;
}

/* SSD1306 每次传输 = [地址] + [控制字节] + 数据; 控制字节 0x00=命令 0x40=显存
   整笔传输失败自动重试 3 次: 屏上电瞬间可能 NACK, 单次失败会让页内容残缺 */
static uint8_t oled_write(uint8_t ctrl, const uint8_t *d, uint16_t n) {
  for (uint8_t t = 0; t < 3; t++) {
    uint8_t fail;
    i2c_start();
    fail = i2c_tx(oled_addr);
    if (!fail) fail = i2c_tx(ctrl);
    for (uint16_t i = 0; i < n && !fail; i++) fail = i2c_tx(d[i]);
    i2c_stop();
    if (!fail) return 1;
    wr_fail = 1;
  }
  return 0;
}

/* 只看地址在不在总线上 */
static uint8_t i2c_probe(uint8_t addr) {
  uint8_t fail;
  i2c_start();
  fail = i2c_tx(addr);
  i2c_stop();
  return (uint8_t)!fail;
}

/* PB8/PB9 = 通用开漏输出, 空闲放高; 顺手解卡:
   上次复位前的传输可能把从机卡在字节中途 (SDA 钳死在低),
   标准解法 (NXP AN10216) 是打最多 9 个时钟把它顶出去, 再手动补个 STOP */
static void i2c_bus_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  GPIOB->CRH = (GPIOB->CRH & ~0xFFu) | 0x77u;
  GPIOB->BSRR = (1u << 8) | (1u << 9);
  i2c_dly();
  for (int i = 0; i < 9 && !SDA_IN(); i++) {
    SCL_L(); i2c_dly();
    SCL_H(); i2c_dly();
  }
  if (!SDA_IN()) {
    SCL_L(); SDA_L(); i2c_dly();
    SCL_H(); i2c_dly();
    SDA_H(); i2c_dly();
  }
}

/* ---------------- SSD1306 ---------------- */

static const uint8_t init_cmds[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
    0x8D, 0x14,              /* 电荷泵, 不开整屏不亮 */
    0x20, 0x02,              /* 页寻址 */
    0xA1, 0xC8,              /* 扫描方向 */
    0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40,
    0xA4, 0xA6, 0xAF};

/* 探测屏并重新初始化; 成功后把影子标记为失效, 下次 Refresh 全量重发
   (屏掉过电的话显存是空的, 影子里的"已上屏内容"不能再信) */
static void oled_setup(void) {
  if (i2c_probe(oled_addr) || i2c_probe(OLED_ADDR_3C) || i2c_probe(OLED_ADDR_3D)) {
    oled_ok = 1;
    oled_write(0x00, init_cmds, sizeof(init_cmds));
    memset(shadow, 0xFF, sizeof(shadow));
  } else {
    oled_ok = 0;
  }
}

void OLED_Init(void) {
  i2c_bus_init();
  uint32_t t0 = millis();
  while (millis() - t0 < 50) {}            /* 上电稳定 */

  oled_addr = OLED_ADDR_3C;
  oled_setup();
  memset(gram, 0, sizeof(gram));
  OLED_Refresh();
}

/* 自愈心跳: 主循环每圈调一次。
   - 屏被拔掉/掉电 (焊接时碰线很常见) → 写失败标志置位 → 重新探测+初始化;
   - 开机时屏不在、后来插上 → oled_ok=0 时每 500ms 重新探测一次。
   插回去半秒内自动恢复, 不用复位遥控器; 屏正常时零总线开销。 */
void OLED_Poll(void) {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 500) return;
  last = now;
  if (oled_ok) {
    if (!wr_fail) return;
    wr_fail = 0;
  }
  oled_setup();
}

uint8_t OLED_Addr(void) { return oled_addr; }

void OLED_Clear(void) {
  memset(gram, 0, sizeof(gram));
}

void OLED_ClearLine(uint8_t y) {
  /* 一行文字高 16 像素, 在 SSD1306 里跨两页 (一页 8 像素), 两页都要清。
     只清一页的话下半截会留着上一次的内容, 看起来就是文字重叠、像卡住一样。 */
  uint8_t page = (uint8_t)(y >> 3);
  for (uint8_t p = page; p < (uint8_t)(page + 2) && p < OLED_HEIGHT / 8; p++) {
    memset(&gram[p][0], 0, OLED_WIDTH);
  }
}

void OLED_Refresh(void) {
  if (!oled_ok) return;
  for (uint8_t page = 0; page < OLED_HEIGHT / 8; page++) {
    /* 只看内容和上屏影子是否一致。不能拿 dirty 位参与判定:
       ui_draw 每次重画同样的文字也会把所有页标脏, 那样整屏每次
       全量重发, 会拖慢 20Hz 发送循环 (实测踩坑) */
    if (memcmp(gram[page], shadow[page], OLED_WIDTH) == 0) continue;
    uint8_t addr[3] = {(uint8_t)(0xB0 | page),
                       (uint8_t)(COL_OFFSET & 0x0F),
                       (uint8_t)(0x10 | ((COL_OFFSET >> 4) & 0x0F))};
    /* 写失败时影子保持旧值, 下个 UI 周期 (100ms 后) 自动重发这页;
       绝不能在失败时也更新影子, 否则残缺的页永远不再重发 (实测踩坑:
       开机时失败的页把 Hello 残影和状态文字叠在一起, 再也不刷新) */
    if (!oled_write(0x00, addr, sizeof(addr))) continue;
    if (!oled_write(0x40, &gram[page][0], OLED_WIDTH)) continue;
    memcpy(shadow[page], gram[page], OLED_WIDTH);
  }
}

/* 分页流水刷新: 主循环每圈调一次, 每次最多发 1 页 (~4ms)。
   整屏 8 页连续刷会阻塞主循环 ~35ms, 把 30Hz 的遥控发送卡出可见延迟
   (摇杆动得花 -> 屏幕变化大 -> 延迟忽高忽低); 拆成一页一调后,
   发送帧总是能在页间隙里发出去, 最大阻塞降到一个页的时间。 */
static uint8_t rr_page = 0;

void OLED_RefreshChunk(void) {
  if (!oled_ok) return;
  for (uint8_t i = 0; i < OLED_HEIGHT / 8; i++) {
    uint8_t page = (uint8_t)((rr_page + i) % (OLED_HEIGHT / 8));
    if (memcmp(gram[page], shadow[page], OLED_WIDTH) == 0) continue;
    uint8_t addr[3] = {(uint8_t)(0xB0 | page),
                       (uint8_t)(COL_OFFSET & 0x0F),
                       (uint8_t)(0x10 | ((COL_OFFSET >> 4) & 0x0F))};
    if (!oled_write(0x00, addr, sizeof(addr))) return;
    if (!oled_write(0x40, &gram[page][0], OLED_WIDTH)) return;
    memcpy(shadow[page], gram[page], OLED_WIDTH);
    rr_page = (uint8_t)((page + 1) % (OLED_HEIGHT / 8));
    return;                                /* 本圈只发一页 */
  }
}

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t on) {
  if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
  uint8_t page = (uint8_t)(y >> 3);
  if (on) gram[page][x] |= (uint8_t)(1u << (y & 7));
  else    gram[page][x] &= (uint8_t)~(1u << (y & 7));
}

void OLED_DrawChar(uint8_t x, uint8_t y, char ch) {
  if (ch < 0x20 || ch > 0x7E) ch = '?';
  const uint8_t *g = OLED_F8x16[(uint8_t)(ch - 0x20)];
  for (uint8_t row = 0; row < 16; row++) {
    uint8_t b = g[row];
    for (uint8_t col = 0; col < 8; col++) {
      /* 整块 8x16 都要写, 0 位也必须清 0: 只点墨不清除的话, 上一个字符
         在这些位置留下的笔画会永远留着, 数字一变就叠成一团 (实测踩坑) */
      OLED_DrawPoint((uint8_t)(x + col), (uint8_t)(y + row),
                     (uint8_t)((b >> (7 - col)) & 1u));
    }
  }
}

/* ASCII 8x16, 超出屏幕右缘就停 */
void OLED_DrawString(uint8_t x, uint8_t y, const char *str) {
  while (*str) {
    OLED_DrawChar(x, y, *str++);
    x = (uint8_t)(x + 8);
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
