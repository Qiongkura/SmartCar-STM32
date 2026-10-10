/**
 * 单协二面遥控器 —— 双摇杆发射端 (STM32F103C8T6, 裸寄存器实现)
 *
 * 排线 (面包板/底板):
 *   左摇杆 X=A0  Y=A1  SW=B10     右摇杆 X=A2  Y=A3  SW=B11  (按下=低)
 *   按键 STOP=A4  MODE=A5 (按下=低。短按切模式, 长按进出测试页)
 *   USART1: PA9=TX -> 蓝牙RXD   PA10=RX <- 蓝牙TXD   9600 8N1
 *   PC13 = 板载LED, 每次发送翻转    OLED = SSD1306 PB8=SCL PB9=SDA (软件 I2C)
 *
 * 三种驾驶模式 (MODE 短按循环切换, 屏幕显示当前模式):
 *   M1 TANK  坦克:  左摇杆Y = 左轮前进/后退, 右摇杆Y = 右轮前进/后退
 *   M2 PAD   手柄:  左摇杆Y = 前进/后退,     右摇杆X = 左转/右转
 *   M3 SOLO  单杆:  左摇杆 = 前后+左右 (原行为), 右摇杆空置
 * 协议: M2/M3 发混控帧 AA 55 06 LIN ROT XOR; M1 坦克发专用坦克帧
 *   AA 55 07 WL WR XOR —— 左右轮直控、不经混控 (一侧轮静止另一侧转的
 *   大差速, 混控通道量程表达不了; 需要车端新版固件认识 0x07)。
 * 停车条件: STOP 键 / 任一摇杆按下 / 该模式所需轴断线。
 *
 * 力度档位: [左摇杆按键]单击 减 10%, [右摇杆按键]单击 加 10%, 范围 10~100%。
 *   调整后屏幕底部显示 1.5 秒 SPD xxx%; 右上角小条常显。上电默认 100% (不记忆)。
 *   摇杆按键不再是急停键 (急停 = STOP 键 / 松杆 / 蓝牙断联)。
 * 测试页 (长按 MODE 进/出): 左右摇杆状态小盘 + 蓝牙 TX/RX 计数。
 * 屏幕: 开机 Hello 0.8 秒 -> 主界面 (10Hz, 只刷变化的页)。
 * 方向反了改下面四个 FLIP 开关重新编译。
 *
 * 蓝牙配对: 按住 MODE 上电 = 全自动配对主机模式 (固件自己拉 KEY, 无需跳线):
 * 搜索车端蓝牙并绑定, 配一次永久生效。屏幕显示每一步, 详情读 dbg_prov。
 * 蓝牙自动重连: 车端每 200ms 发心跳, 1.5s 收不到判定断链 -> 固件拉 KEY
 * 进 AT 模式发 AT+RESET, 模块自动回连绑定地址 (dbg_recov 记次数)。
 * 接线: 蓝牙 KEY/EN 接 PA8 (A8 脚), 不要再接 3.3V 跳线!
 */
#include "stm32f1xx.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

/* 烧录前身份核对: 电脑端先从 SWD 读这串字符, 不是 REMOTE-01 就拒绝烧录
   (发生过把小车固件烧进遥控器的事故, 这就是防再犯的) */
const volatile char FW_ID[] __attribute__((used)) = "REMOTE-01";

#define FLIP_LX 0        /* 左摇杆X 取反 */
#define FLIP_LY 0        /* 左摇杆Y 取反 */
#define FLIP_RX 1        /* 右摇杆X取反：右摇杆物理旋转180度 */
#define FLIP_RY 1        /* 右摇杆Y取反：右摇杆物理旋转180度 */


#define DEADZONE 12      /* ±100 量程下, 小于它当 0 */
#define FULL     1800.0f /* 距中心多少 ADC counts 算满杆 (12位ADC) */
#define SEND_MS  30      /* 发送周期 33Hz (跟手优先; 从 50ms 降下来, 车端照单全收) */
#define UI_MS    100     /* 状态页刷新周期 10Hz */
#define MODE_HOLD_MS 1000 /* MODE 按住多久算长按 (测试页) */

/* 蓝牙 KEY/EN 脚 (PA8): 高 = AT 模式, 低 = 数据模式 */
#define KEY_H()  (GPIOA->BSRR = (1u << 8))
#define KEY_L()  (GPIOA->BRR  = (1u << 8))

/* SWD 调试口可见的实时状态 (电脑可以用 openocd 读这些排方向) */
volatile uint16_t dbg_rawx, dbg_rawy, dbg_cx, dbg_cy;          /* 左摇杆 */
volatile uint16_t dbg_rawrx, dbg_rawry, dbg_crx, dbg_cry;      /* 右摇杆 */
volatile int      dbg_lin, dbg_rot;
volatile int      dbg_wl, dbg_wr;    /* 坦克模式的左/右轮命令 */
volatile int8_t   dbg_lxu, dbg_lyu, dbg_rxu, dbg_ryu;  /* 归一化 -100~100 */
volatile uint8_t  dbg_stop, dbg_mode, dbg_sw, dbg_sw2, dbg_sent;
volatile uint8_t  dbg_mode_sel, dbg_test;      /* 当前模式 1/2/3, 测试页标志 */
/* 轴有效性: 开机标定的中心必须落在量程中部, 为 0/贴边 = 断线或虚焊 */
volatile uint8_t  dbg_xok, dbg_yok, dbg_rxok, dbg_ryok;
/* 蓝牙链路观察: 发帧计数 + 收到字节计数 (模式七车端不回传, RX 一般恒 0) */
volatile uint32_t rx_count;
volatile uint32_t rx_ms;
/* 蓝牙自配对状态 (SWD 可读): 0=正常 1=配对中 2=成功 3=失败
   dbg_prov_err = 失败步骤码, dbg_bt_* = 搜到的车端蓝牙地址 */
volatile uint8_t  dbg_prov, dbg_prov_err, dbg_recov;   /* dbg_recov: 自动重连次数 */
volatile uint16_t dbg_bt_nap, dbg_bt_uap;
volatile uint32_t dbg_bt_lap;
volatile char dbg_cap[256];   /* 配对时 AT+INQ 的原始回复, SWD 调试用 */
volatile char dbg_rn[64];
static volatile uint32_t tx_count;   /* 累计发出的帧数, 屏幕上显示 */
static volatile uint8_t  g_spd = 100; /* 力度档位 10~100% (RAM, 上电默认 100) */
static volatile uint32_t g_spd_ms;    /* 最近一次调速时刻: 底部临时显示 SPD */

static volatile uint32_t g_ms = 0;

void SysTick_Handler(void) { g_ms++; }
uint32_t millis(void) { return g_ms; }   /* oled.c 延时用 */

static void clock_init(void) {
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY)) {}
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
  /* PLLSRC 必须置 1 (PLL 输入=HSE): 漏了它 PLL 挂在默认 HSI/2=4MHz 上,
     4x9=36MHz, 全板半速、USART 波特率减半 (实测踩坑)。 */
  RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC |
              RCC_CFGR_PPRE1_DIV2;                     /* APB1=36MHz */
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY)) {}
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}

static void gpio_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                  RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
  /* PA0/PA1/PA2/PA3 = 摇杆模拟输入 (CNF=00 MODE=00 模拟模式) */
  GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
                  GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |
                  GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |
                  GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
  /* PA4/PA5 = 按键输入上拉 (CNF=10 MODE=00, ODR=1) */
  GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4 |
                  GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
  GPIOA->CRL |=  GPIO_CRL_CNF4_1 | GPIO_CRL_CNF5_1;
  GPIOA->ODR |=  (1 << 4) | (1 << 5);
  /* PB10/PB11 = 左右摇杆 SW 输入上拉 */
  GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10 |
                  GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
  GPIOB->CRH |=  GPIO_CRH_CNF10_1 | GPIO_CRH_CNF11_1;
  GPIOB->ODR |=  (1 << 10) | (1 << 11);
  /* PC13 = LED, 开漏输出 */
  GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
  GPIOC->CRH |=  GPIO_CRH_MODE13_0 | GPIO_CRH_CNF13_0;
  /* PA9 = USART1 TX 复用推挽50MHz; PA10 = RX 浮空输入
     注意 CNF=01 才是浮空输入: 清完位不置 CNF10_0 会落在模拟输入 (00),
     数字输入缓冲被关, 串口永远收不到字节 (实测踩坑, AT 无响应的真凶) */
  GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9 |
                  GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
  GPIOA->CRH |=  GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1 |
                 GPIO_CRH_CNF10_0;
  /* PA8 = 蓝牙 KEY/EN (推挽输出, 默认低 = 数据模式) */
  GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
  GPIOA->CRH |=  GPIO_CRH_MODE8_1;                  /* 2MHz 推挽输出 */
  GPIOA->BRR  =  (1 << 8);
  /* PB8/PB9 交给 oled.c 的软件 I2C (GPIO 开漏位翻转), 这里不配 */
}

static void usart1_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  /* PCLK2=72MHz, 9600: USARTDIV = 72e6/(16*9600) = 468.75 -> 468+12/16 */
  USART1->BRR = (468 << 4) | 12;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void adc1_init(void) {
  RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;          /* ADCCLK = 12MHz */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->CR2 = ADC_CR2_ADON;
  for (volatile int i = 0; i < 10000; i++) {} /* 上电稳定 */
  ADC1->CR2 |= ADC_CR2_RSTCAL;
  while (ADC1->CR2 & ADC_CR2_RSTCAL) {}
  ADC1->CR2 |= ADC_CR2_CAL;
  while (ADC1->CR2 & ADC_CR2_CAL) {}
  /* F1 软件启动走触发机制: 触发源选 SWSTART(EXTSEL=111) 并使能触发,
     adc_read 里置 SWSTART 才真正开转 (EXTTRIG=0 时 SWSTART 无效,
     EOC 永远不来, 实测卡死在这)。 */
  ADC1->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL;
}

/* 单次采样, 71.5 周期采样时间 */
static uint16_t adc_read(uint8_t ch) {
  ADC1->SQR1 = 0;                             /* 1 个转换 */
  ADC1->SQR3 = ch;
  if (ch < 10) {
    ADC1->SMPR2 |= (7u << (ch * 3));          /* 71.5 cycles */
  } else {
    ADC1->SMPR1 |= (7u << ((ch - 10) * 3));
  }
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC1->SR & ADC_SR_EOC)) {}
  return (uint16_t)ADC1->DR;
}

static uint16_t adc_avg(uint8_t ch, int n) {
  uint32_t s = 0;
  for (int i = 0; i < n; i++) s += adc_read(ch);
  return (uint16_t)(s / n);
}

/* 把 ADC 偏差换算成 -100~100, 带死区 */
static int to_units(int delta) {
  int v = (int)(delta * (100.0f / FULL) + (delta >= 0 ? 0.5f : -0.5f));
  if (v >  100) v =  100;
  if (v < -100) v = -100;
  if (v < DEADZONE && v > -DEADZONE) v = 0;
  return v;
}

static void avoid_header_payload(int *a, int *b) {
  /* 车端解析器把 payload 内的 AA 55 保留作重新同步标记，
     避免精确组合 (-86,+85) 产生歧义帧。偏 1 个量化单位，用户几乎无感。 */
  if (*a == -86 && *b == 85) {
    *b = 84;
  }
}

static void send_frame(uint8_t lin, uint8_t rot) {
  int a = (int8_t)lin;
  int b = (int8_t)rot;
  avoid_header_payload(&a, &b);
  lin = (uint8_t)(int8_t)a;
  rot = (uint8_t)(int8_t)b;
  uint8_t f[6];
  f[0] = 0xAA; f[1] = 0x55; f[2] = 0x06; f[3] = lin; f[4] = rot;
  f[5] = (uint8_t)(f[0] ^ f[1] ^ f[2] ^ f[3] ^ f[4]);
  while (!(USART1->SR & USART_SR_TXE)) {}
  for (int i = 0; i < 6; i++) {
    USART1->DR = f[i];
    while (!(USART1->SR & USART_SR_TXE)) {}
  }
  GPIOC->ODR ^= (1 << 13);                    /* LED 翻转 = 在发 */
  dbg_sent++;
  tx_count++;
}

/* 坦克帧: AA 55 07 WL WR XOR, WL/WR 为带符号轮速 (-100~100),
   车端直控左右轮 (一侧 0 = 该轮钉在原地) */
static void send_tank_frame(uint8_t wl, uint8_t wr) {
  int a = (int8_t)wl;
  int b = (int8_t)wr;
  avoid_header_payload(&a, &b);
  wl = (uint8_t)(int8_t)a;
  wr = (uint8_t)(int8_t)b;
  uint8_t f[6];
  f[0] = 0xAA; f[1] = 0x55; f[2] = 0x07; f[3] = wl; f[4] = wr;
  f[5] = (uint8_t)(f[0] ^ f[1] ^ f[2] ^ f[3] ^ f[4]);
  while (!(USART1->SR & USART_SR_TXE)) {}
  for (int i = 0; i < 6; i++) {
    USART1->DR = f[i];
    while (!(USART1->SR & USART_SR_TXE)) {}
  }
  GPIOC->ODR ^= (1 << 13);                    /* LED 翻转 = 在发 */
  dbg_sent++;
  tx_count++;
}

/* ---------- 界面 ----------
 * 主界面: 左边 = 状态盘。M1 坦克: 左右轮两条竖直条形图(上正下负);
 *          M2/M3: 十字盘显示本模式实际发出的指令 (推哪个杆点怎么走即车怎么走)
 *         右边 = 模式名 / 数值 / TX 计数 / 状态词 (异常时闪烁报警)
 * 测试页 (长按 MODE 进/出): 两个小盘 = 左右摇杆实时位置, 底行蓝牙 TX/RX 计数
 * 校准口诀: 每个盘对应各自摇杆, 上推点向上/右推点向右; 反了改对应 FLIP 重烧 */

static void ui_hline(uint8_t x0, uint8_t x1, uint8_t y, uint8_t on) {
  for (uint8_t x = x0; x <= x1; x++) OLED_DrawPoint(x, y, on);
}
static void ui_vline(uint8_t x, uint8_t y0, uint8_t y1, uint8_t on) {
  for (uint8_t y = y0; y <= y1; y++) OLED_DrawPoint(x, y, on);
}

/* 十字盘: 显示当前模式实际发出的指令向量 (vx=转向来源, vy=前后来源) */
static void ui_stick(int vx, int vy) {
  const uint8_t cx = 24, cy = 24, half = 20;   /* 中心 (24,24), 最大偏移 ±20px */
  for (uint8_t y = 1; y <= 46; y++)            /* 清整个盘面 */
    for (uint8_t x = 1; x <= 46; x++)
      OLED_DrawPoint(x, y, 0);
  ui_hline(1, 46, 1, 1);                       /* 外框 */
  ui_hline(1, 46, 46, 1);
  ui_vline(1, 1, 46, 1);
  ui_vline(46, 1, 46, 1);
  for (uint8_t d = 4; d <= 43; d += 3) {       /* 中心虚线十字 */
    OLED_DrawPoint(d, cy, 1);
    OLED_DrawPoint(cx, d, 1);
  }
  int px = cx + (int)((int32_t)vx * half / 100);   /* 右推 = 点向右 */
  int py = cy - (int)((int32_t)vy * half / 100);   /* 上推 = 点向上 */
  for (uint8_t dy = 0; dy < 5; dy++)           /* 5x5 位置点 */
    for (uint8_t dx = 0; dx < 5; dx++)
      OLED_DrawPoint((uint8_t)(px - 2 + dx), (uint8_t)(py - 2 + dy), 1);
}

/* 坦克模式: 一条竖直条形图表示一个轮子 (-100 下 ~ +100 上), x0 左边界 */
static void ui_bar(uint8_t x0, uint8_t w, int v) {
  const uint8_t top = 2, bot = 57, mid = 30;
  ui_hline(x0, (uint8_t)(x0 + w - 1), top, 1);
  ui_hline(x0, (uint8_t)(x0 + w - 1), bot, 1);
  ui_vline(x0, top, bot, 1);
  ui_vline((uint8_t)(x0 + w - 1), top, bot, 1);
  for (uint8_t y = top + 1; y < bot; y++)      /* 内部清空 */
    for (uint8_t x = x0 + 1; x < (uint8_t)(x0 + w - 1); x++)
      OLED_DrawPoint(x, y, 0);
  ui_hline(x0 + 1, (uint8_t)(x0 + w - 2), mid, 1);   /* 零点线 */
  int len = v > 100 ? 100 : (v < -100 ? -100 : v);
  len = len * 26 / 100;                        /* ±26px 满程 */
  if (len > 0) {
    for (int y = 0; y < len; y++) ui_hline(x0 + 3, (uint8_t)(x0 + w - 4), (uint8_t)(mid - 1 - y), 1);
  } else if (len < 0) {
    for (int y = 0; y < -len; y++) ui_hline(x0 + 3, (uint8_t)(x0 + w - 4), (uint8_t)(mid + 1 + y), 1);
  }
}

static void ui_tank(void) {
  ui_bar(6, 18, dbg_wl);                       /* 左条 = 左轮 */
  ui_bar(28, 18, dbg_wr);                      /* 右条 = 右轮 */
}

/* 测试页小盘: 左上角 (x0,y0), 边长 s; 点按 -100~100 归一值走 */
static void ui_mini_pad(uint8_t x0, uint8_t y0, uint8_t s, int vx, int vy) {
  for (uint8_t y = y0; y < (uint8_t)(y0 + s); y++)
    for (uint8_t x = x0; x < (uint8_t)(x0 + s); x++)
      OLED_DrawPoint(x, y, (uint8_t)(x == x0 || x == (uint8_t)(x0 + s - 1) ||
                                     y == y0 || y == (uint8_t)(y0 + s - 1)));
  uint8_t c = (uint8_t)(s / 2);
  for (uint8_t d = 3; d < s - 3; d += 3) {
    OLED_DrawPoint((uint8_t)(x0 + d), (uint8_t)(y0 + c), 1);
    OLED_DrawPoint((uint8_t)(x0 + c), (uint8_t)(y0 + d), 1);
  }
  int r = s / 2 - 3;
  int px = x0 + c + (int)((int32_t)vx * r / 100);
  int py = y0 + c - (int)((int32_t)vy * r / 100);
  for (uint8_t dy = 0; dy < 3; dy++)
    for (uint8_t dx = 0; dx < 3; dx++)
      OLED_DrawPoint((uint8_t)(px - 1 + dx), (uint8_t)(py - 1 + dy), 1);
}

static void ui_test(void) {
  ui_mini_pad(2, 0, 32, dbg_lxu, dbg_lyu);
  ui_mini_pad(40, 0, 32, dbg_rxu, dbg_ryu);
  OLED_DrawString(16, 32, "L");
  OLED_DrawString(54, 32, "R");
  /* 蓝牙状态: LINK = 近期收到车端心跳 (车在线且链路通); 
     TXOK = 只发不回 (车没上电或没连上) */
  int link = (rx_count > 0) && ((millis() - rx_ms) < 1500);
  OLED_DrawString(84, 32, link ? "LINK" : "TXOK");
  OLED_Printf(0, 48, "TX %05u SPD%03u",
              (unsigned)(tx_count % 100000u), (unsigned)g_spd);
}

/* 力度档位小条: 右上角, 常显当前力度 (避免只在进行中才看得到) */
static void ui_spd_bar(void) {
  const uint8_t x0 = 110, y0 = 3, x1 = 126, y1 = 13;
  ui_hline(x0, x1, y0, 1);
  ui_hline(x0, x1, y1, 1);
  ui_vline(x0, y0, y1, 1);
  ui_vline(x1, y0, y1, 1);
  uint8_t w = (uint8_t)((x1 - x0 - 1) * g_spd / 100);
  for (uint8_t y = y0 + 1; y < y1; y++) {
    for (uint8_t x = x0 + 1; x <= (uint8_t)(x0 + w); x++) OLED_DrawPoint(x, y, 1);
    for (uint8_t x = (uint8_t)(x0 + 1 + w); x < x1; x++)  OLED_DrawPoint(x, y, 0);
  }
}

static void ui_draw(void) {
  if (dbg_test) { ui_test(); return; }
  ui_spd_bar();
  if (dbg_mode_sel == 1) {
    ui_tank();
  } else if (dbg_mode_sel == 2) {
    ui_stick(dbg_rxu, dbg_lyu);   /* 手柄: 点=右杆X(转向)+左杆Y(前后) */
  } else {
    ui_stick(dbg_lxu, dbg_lyu);   /* 单杆: 点=左杆X+左杆Y */
  }
  OLED_DrawString(52, 0, dbg_mode_sel == 1 ? "M1 TANK" :
                          dbg_mode_sel == 2 ? "M2 PAD " : "M3 SOLO");
  if (dbg_mode_sel == 1) {
    OLED_Printf(52, 16, "L %+04d", dbg_wl);
    OLED_Printf(52, 32, "R %+04d", dbg_wr);
  } else {
    OLED_Printf(52, 16, "FWD %+04d", dbg_lin);
    OLED_Printf(52, 32, "TRN %+04d", dbg_rot);
  }
  /* 状态词 (固定 8 字宽防残影; OK 时让位给 TX 计数) */
  char st[9] = "OK      ";
  if (dbg_mode_sel == 1) {
    if (!dbg_yok)       strcpy(st, "LYER    ");   /* 左杆Y 断线 */
    else if (!dbg_ryok) strcpy(st, "RYER    ");   /* 右杆Y 断线 */
  } else if (dbg_mode_sel == 2) {
    if (!dbg_yok)       strcpy(st, "LYER    ");
    else if (!dbg_rxok) strcpy(st, "RXER    ");
  } else {
    if (!dbg_xok)       strcpy(st, "LXER    ");
    else if (!dbg_yok)  strcpy(st, "LYER    ");
  }
  if (st[0] == 'O' && dbg_stop) strcpy(st, "STOP    ");
  if ((millis() - g_spd_ms) < 1500) {              /* 刚调速: 底部显示力度 1.5s */
    snprintf(st, sizeof st, "SPD %03u%%", (unsigned)g_spd);
  } else if (st[0] == 'O') {
    snprintf(st, sizeof st, "TX %05u", (unsigned)(tx_count % 100000u));
  } else if ((millis() / 400) & 1) {
    strcpy(st, "        ");                        /* 报警闪烁 */
  }
  OLED_DrawString(52, 48, st);
}

/* ---------- 蓝牙自配对 (按住 MODE 上电触发) ----------
 * HC-05 出厂是从机, "谁发起连接"记录在模块自身固件里, 只能通过模块串口的
 * AT 命令改写 —— 这段代码代替电脑: 按住 MODE 上电 (KEY 跳线需接高, 模块灯
 * 慢闪), 自动完成 配对码/波特率/搜索车端/绑定/主模式, 屏幕显示每一步。
 * 配一次永久生效, 之后上电自动回连小车。 */
static char bt_addr[15];      /* "xxxx,xx,xxxxxx" 共14字符 */

static void usart1_set_baud(uint32_t baud) {
  uint32_t m = 72000000u / (16u * baud);
  uint32_t rem = 72000000u - m * 16u * baud;
  uint32_t f = (rem * 16u + 8u * baud) / (16u * baud);
  USART1->BRR = (uint32_t)((m << 4) | (f & 0xFu));
}

static void at_send(const char *s) {
  while (*s) {
    while (!(USART1->SR & USART_SR_TXE)) {}
    USART1->DR = (uint8_t)*s++;
  }
  while (!(USART1->SR & USART_SR_TXE)) {}
  USART1->DR = '\r';
  while (!(USART1->SR & USART_SR_TXE)) {}
  USART1->DR = '\n';
}

/* 发命令并原样收集回复 (INQ/RNAME 这类多行响应用) */
static uint16_t at_capture(const char *cmd, char *buf, uint16_t size, uint32_t ms) {
  uint16_t n = 0;
  uint32_t t0 = millis();
  while (USART1->SR & USART_SR_RXNE) (void)USART1->DR;
  at_send(cmd);
  buf[0] = 0;
  while (millis() - t0 < ms && n < size - 1) {
    if (USART1->SR & USART_SR_RXNE) {
      buf[n++] = (char)USART1->DR;
      buf[n] = 0;
    }
  }
  return n;
}

/* 发一条 AT 命令并等回复: 0=超时 1=OK 2=ERROR */
static uint8_t at_exchange(const char *cmd, uint32_t ms) {
  char r[64];
  uint16_t n = 0;
  uint32_t t0 = millis();
  while (USART1->SR & USART_SR_RXNE) (void)USART1->DR;   /* 清残留 */
  at_send(cmd);
  r[0] = 0;
  while (millis() - t0 < ms) {
    if (USART1->SR & USART_SR_RXNE) {
      char c = (char)USART1->DR;
      if (n < sizeof r - 1) { r[n++] = c; r[n] = 0; }
      if (strstr(r, "OK")) return 1;
      if (strstr(r, "ERROR")) return 2;
    }
  }
  return 0;
}

static uint32_t hex_n(const char *s, uint8_t n) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < n; i++) {
    char c = s[i];
    v = v * 16u + (uint32_t)(c <= '9' ? c - '0' : c - 'A' + 10);
  }
  return v;
}

/* AT+INQ 搜索, 逐个查设备名, 只绑定名字为 HC-06 / JDY 的车端模块
   (周围手机电脑太多, "绑第一个搜到的"会随机绑错 —— 实测踩坑) */
static uint8_t at_inq_bind(void) {
  char cap[400];
  uint16_t nc = at_capture("AT+INQ", cap, sizeof cap, 5000);
  for (uint16_t i = 0; i < nc && i < 255; i++) dbg_cap[i] = cap[i];
  dbg_cap[(nc < 255) ? nc : 255] = 0;
  at_send("AT+INQC");                        /* 停止搜索 */
  uint32_t t0 = millis();
  while (millis() - t0 < 300) {}             /* 等模块退出搜索态再发下一条 */

  char *p = cap;
  while ((p = strstr(p, "+INQ:")) != 0) {
    for (uint8_t i = 0; i < 14; i++) {
      char c = p[5 + i];
      if (!c) break;                         /* 该行被截断就跳过这个设备 */
      if (c >= 'a' && c <= 'f') c = (char)(c - 'a' + 'A');
      if (c == ':') c = ',';
      bt_addr[i] = c;
    }
    bt_addr[14] = 0;
    dbg_bt_nap = (uint16_t)hex_n(bt_addr, 4);
    dbg_bt_uap = (uint16_t)hex_n(bt_addr + 5, 2);
    dbg_bt_lap = hex_n(bt_addr + 8, 6);
    OLED_Printf(0, 32, "%04X %02X %06lX",
                dbg_bt_nap, dbg_bt_uap, (unsigned long)dbg_bt_lap);
    OLED_Refresh();
    p += 5;

    /* COD 过滤: 串口蓝牙模块 (HC-06/JDY-31) 的设备类是 0x1F00,
       手机/电脑的完全不同 —— 只绑串口模块, 防止绑错设备。
       (RNAME 带参数查询在部分 HC-05 固件上回 ERROR, 不能靠名字) */
    char *cs = strchr(p, ',');
    if (cs) {
      cs++;
      uint32_t cod = 0;
      while (*cs && *cs != ',') {
        char c = *cs++;
        uint8_t v = (uint8_t)(c <= '9' ? c - '0' : ((c | 0x20) - 'a' + 10));
        cod = cod * 16u + v;
      }
      if (cod != 0x1F00u) continue;          /* 不是串口模块, 看下一个 */
    }

    char bind[24];
    strcpy(bind, "AT+BIND=");
    strcat(bind, bt_addr);
    return (uint8_t)(at_exchange(bind, 2000) == 1);
  }
  return 0;
}

static void bt_provision(void) {
  uint32_t t0;
  dbg_prov = 1;
  OLED_Clear();
  OLED_DrawString(0, 0, "BT PAIRING");
  OLED_Printf(0, 32, "CAR ON? WAIT...");
  OLED_Refresh();
  KEY_H();                                   /* 固件拉 KEY 进 AT 模式 (免跳线) */
  t0 = millis();
  while (millis() - t0 < 600) {}
  usart1_set_baud(38400);                    /* HC-05 AT 模式固定波特率 */

  uint8_t r = at_exchange("AT", 1000);
  if (r != 1) {                              /* 38400 无响应: 部分兼容模块 AT 在 9600 */
    usart1_set_baud(9600);
    r = at_exchange("AT", 1000);
  }
  OLED_Printf(0, 16, "AT   %s", r == 1 ? "OK " : "ERR");
  OLED_Refresh();
  if (r != 1) { dbg_prov = 3; dbg_prov_err = 1; goto fail; }

  r = at_exchange("AT+ROLE=1", 1000);        /* 主角色: 没有它模块永远是从机, 不会连小车 */
  OLED_Printf(0, 16, "ROLE %s", r == 1 ? "OK " : "ERR");
  OLED_Refresh();
  if (r != 1) { dbg_prov = 3; dbg_prov_err = 7; goto fail; }

  r = at_exchange("AT+PSWD=\"1234\"", 1000);
  OLED_Printf(0, 16, "PSWD %s", r == 1 ? "OK " : "ERR");
  OLED_Refresh();
  if (r != 1) { dbg_prov = 3; dbg_prov_err = 2; goto fail; }

  r = at_exchange("AT+UART=9600,0,0", 1000);
  OLED_Printf(0, 16, "UART %s", r == 1 ? "OK " : "ERR");
  OLED_Refresh();
  if (r != 1) { dbg_prov = 3; dbg_prov_err = 3; goto fail; }

  r = at_exchange("AT+INIT", 2000);
  OLED_Printf(0, 16, "INIT %s", r != 0 ? "OK " : "ERR");   /* 个别固件 INIT 回 ERROR, 容忍 */
  OLED_Refresh();
  if (r == 0) { dbg_prov = 3; dbg_prov_err = 4; goto fail; }

  OLED_Printf(0, 16, "INQ 4s...");
  OLED_Refresh();
  if (!at_inq_bind()) { dbg_prov = 3; dbg_prov_err = 5; goto fail; }
  OLED_Printf(0, 16, "BIND OK");
  OLED_Printf(0, 32, "%04X %02X %06lX", dbg_bt_nap, dbg_bt_uap, (unsigned long)dbg_bt_lap);
  OLED_Refresh();

  r = at_exchange("AT+CMODE=0", 1000);       /* 只连绑定地址, 上电自动回连 */
  OLED_Printf(0, 16, "CMODE %s", r == 1 ? "OK " : "ERR");
  OLED_Refresh();
  if (r != 1) { dbg_prov = 3; dbg_prov_err = 6; goto fail; }

  at_send("AT+RESET");                       /* 模块重启, 回数据模式 9600 */
  OLED_Printf(0, 16, "RESET  DONE");
  OLED_Refresh();
  dbg_prov = 2;
  t0 = millis();
  while (millis() - t0 < 1500) {}
  usart1_set_baud(9600);
  KEY_L();                                   /* 回数据模式 */
  return;
fail:
  OLED_Printf(0, 48, "FAIL err=%u", dbg_prov_err);
  OLED_Refresh();
  dbg_prov = 3;
  t0 = millis();
  while (millis() - t0 < 2500) {}
  usart1_set_baud(9600);
  KEY_L();
}

/* 蓝牙自动重连: 车端每 200ms 心跳, 超过 1.5s 收不到判定断链。
   HC-05 断链后自身的重试不可靠 (车端断电再上电经常挂死), 由固件踢一脚:
   KEY 拉高进 AT 模式 -> AT+RESET -> 模块重启后按存好的绑定地址自动回连。
   全程阻塞约 2 秒, 期间不发帧 (两个轮的指令断 2s, 车端 500ms 超时先刹车, 安全)。*/
static void bt_recover(void) {
  KEY_H();
  uint32_t t0 = millis();
  while (millis() - t0 < 300) {}
  usart1_set_baud(38400);
  if (at_exchange("AT", 1000) == 1) {
    at_send("AT+RESET");                     /* 模块重启 -> 自动回连 */
  }
  t0 = millis();
  while (millis() - t0 < 300) {}
  usart1_set_baud(9600);
  KEY_L();
  while (USART1->SR & USART_SR_RXNE) (void)USART1->DR;
  rx_ms = millis();                          /* 重置观察窗, 给回连留时间 */
  rx_count = 0;
  dbg_recov++;
}

/* ---------- 按键去抖 ---------- */
typedef struct { uint8_t db, raw, init; uint32_t last; } btn_db_t;
#define BTN_DOWN 1
#define BTN_UP   2

/* 原始电平稳定 25ms 才认作新状态; 返回本圈新产生的边沿 (BTN_DOWN/BTN_UP/0) */
static uint8_t btn_edges(btn_db_t *b, uint8_t level, uint32_t now) {
  uint8_t e = 0;
  if (!b->init) {                      /* 首圈只采样: 上电时按着的不算数 */
    b->init = 1; b->db = level; b->raw = level; b->last = now;
    return 0;
  }
  if (level != b->raw) { b->raw = level; b->last = now; }
  if ((now - b->last) >= 25 && b->db != level) {
    b->db = level;
    e = b->db ? BTN_DOWN : BTN_UP;
  }
  return e;
}

int main(void) {
  SystemInit();            /* 由启动文件调用前已跑, 这里保险再配一次时钟 */
  clock_init();
  /* 不能用 SystemCoreClock 算 tick: cube 框架这变量实测是 8M
     (烧进 RVR 只有 7999), 直接按 72MHz 写死。 */
  SysTick_Config(72000000 / 1000);
  gpio_init();
  usart1_init();
  adc1_init();

  if (FW_ID[0] != 'R') for (;;);   /* 永不成立: 只是防止链接器把 FW_ID 丢弃 */
  OLED_Init();             /* 探测不到屏也照常发车 */

  /* 按住 MODE 上电 = 蓝牙自配对 (KEY 跳线需接高, 模块灯慢闪) */
  if (!(GPIOA->IDR & (1 << 5))) {
    bt_provision();
  }

  OLED_DrawString(44, 24, "Hello");        /* 居中: 5 字 x8px = 40 宽 */
  OLED_DrawString(24, 44, "RemoteCtrl");   /* 10 字 x8px = 80 宽 */
  OLED_Refresh();

  uint32_t t0 = millis();
  while (millis() - t0 < 300) {}           /* 等电源和摇杆稳 */
  dbg_cx  = adc_avg(0, 64);                /* 双摇杆四轴开机自动标定 */
  dbg_cy  = adc_avg(1, 64);
  dbg_crx = adc_avg(2, 64);
  dbg_cry = adc_avg(3, 64);
  dbg_xok  = (dbg_cx  > 400 && dbg_cx  < 3700);   /* 贴边/为0 = 轴没接好 */
  dbg_yok  = (dbg_cy  > 400 && dbg_cy  < 3700);
  dbg_rxok = (dbg_crx > 400 && dbg_crx < 3700);
  dbg_ryok = (dbg_cry > 400 && dbg_cry < 3700);
  while (millis() - t0 < 800) {}           /* Hello 总共停留 0.8s */

  dbg_mode_sel = 3;                        /* 默认单摇杆 (原行为, 左杆全控) */
  OLED_Clear();                            /* 清开机画面, 进主界面 */
  ui_draw();
  OLED_Refresh();

  uint32_t last = 0, ui_t = 0;
  uint8_t  ui_force = 0;
  rx_ms = millis();                        /* 心跳观察窗从此刻起算 */
  while (1) {
    uint32_t now = g_ms;

    /* 按键电平 */
    dbg_stop = (GPIOA->IDR & (1 << 4)) ? 0 : 1;   /* 按下=1 */
    dbg_mode = (GPIOA->IDR & (1 << 5)) ? 0 : 1;
    dbg_sw   = (GPIOB->IDR & (1 << 10)) ? 0 : 1;
    dbg_sw2  = (GPIOB->IDR & (1 << 11)) ? 0 : 1;

    /* MODE: 短按切模式, 长按 1 秒进出测试页, 按住时左/右摇杆按键调速。
       三个按键都过 25ms 去抖 (机械抖动会让边沿计数连跳, 实测踩坑) */
    {
      static btn_db_t bmode, bsw1, bsw2;
      static uint8_t  m_long = 0;
      static uint32_t mt0 = 0;
      uint8_t em = btn_edges(&bmode, dbg_mode, now);
      uint8_t e1 = btn_edges(&bsw1, dbg_sw, now);
      uint8_t e2 = btn_edges(&bsw2, dbg_sw2, now);

      if (em & BTN_DOWN) { mt0 = now; m_long = 0; }        /* 按下 MODE */
      if ((em & BTN_UP) && !m_long && !dbg_test) {         /* 短按: 切模式 */
        dbg_mode_sel = (uint8_t)(dbg_mode_sel % 3 + 1);
        ui_force = 1;
      }
      if (bmode.db && !m_long && (now - mt0) >= MODE_HOLD_MS) {  /* 长按: 测试页 */
        m_long = 1;
        dbg_test ^= 1;
        ui_force = 1;
      }

      /* 力度键 (独立于 MODE): 左摇杆按键单击 -10%, 右摇杆按键单击 +10% */
      if (e1 & BTN_DOWN) { if (g_spd > 10)  g_spd = (uint8_t)(g_spd - 10); g_spd_ms = now; }
      if (e2 & BTN_DOWN) { if (g_spd < 100) g_spd = (uint8_t)(g_spd + 10); g_spd_ms = now; }
    }

    /* 蓝牙心跳计数: 车端每 200ms 一个字节, 收到即链路活 */
    while (USART1->SR & USART_SR_RXNE) { (void)USART1->DR; rx_count++; rx_ms = now; }

    /* 链路守护: 静默 >5s 判定断链, 每 8s 最多踢一次重连 */
    {
      static uint32_t lost = 0, recov = 0;
      if ((now - rx_ms) < 1500) lost = 0;
      else if (lost == 0)      lost = now;
      if (lost && (now - lost) > 5000 && (now - recov) > 8000) {
        recov = lost = now;
        bt_recover();
      }
    }

    if (now - last >= SEND_MS) {                  /* 20Hz 发送 */
      last = now;
      dbg_rawx  = adc_avg(0, 8);
      dbg_rawy  = adc_avg(1, 8);
      dbg_rawrx = adc_avg(2, 8);
      dbg_rawry = adc_avg(3, 8);

      int dlx = (int)dbg_rawx  - (int)dbg_cx;
      int dly = (int)dbg_rawy  - (int)dbg_cy;
      int drx = (int)dbg_rawrx - (int)dbg_crx;
      int dry = (int)dbg_rawry - (int)dbg_cry;
#if FLIP_LX
      dlx = -dlx;
#endif
#if FLIP_LY
      dly = -dly;
#endif
#if FLIP_RX
      drx = -drx;
#endif
#if FLIP_RY
      dry = -dry;
#endif
      int LX = to_units(dlx);              /* 归一值: 右推/上推为正 */
      int LY = to_units(-dly);
      int RX = to_units(drx);
      int RY = to_units(-dry);
      dbg_lxu = (int8_t)LX; dbg_lyu = (int8_t)LY;
      dbg_rxu = (int8_t)RX; dbg_ryu = (int8_t)RY;

      int lin = 0, rot = 0, wl = 0, wr = 0;
      if (dbg_mode_sel == 1) {
        /* 坦克: 左杆Y=左轮, 右杆Y=右轮, 直发坦克帧; 一侧杆居中=该轮静止 */
        if (dbg_yok && dbg_ryok) { wl = LY; wr = RY; }
      } else if (dbg_mode_sel == 2) {
        /* 手柄: 左杆前后 + 右杆左右 */
        if (dbg_yok && dbg_rxok) { lin = LY; rot = RX; }
      } else {
        /* 单摇杆: 左杆全控 (原行为) */
        if (dbg_xok && dbg_yok) { lin = LY; rot = LX; }
      }

      /* 力度档位: 整体缩放 (坦克模式在混控后缩 = 两轮同比例, 保持精确) */
      lin = lin * (int)g_spd / 100;
      rot = rot * (int)g_spd / 100;
      wl  = wl  * (int)g_spd / 100;
      wr  = wr  * (int)g_spd / 100;

      /* 急停: STOP 键 (摇杆按键已改为力度键) */
      if (dbg_stop) {
        lin = 0; rot = 0; wl = 0; wr = 0;
      }
      dbg_wl = wl; dbg_wr = wr;
      dbg_lin = (dbg_mode_sel == 1) ? wl : lin;   /* SWD 观察用 */
      dbg_rot = (dbg_mode_sel == 1) ? wr : rot;

      if (dbg_mode_sel == 1) {                    /* 坦克: 专用帧 */
        if (wl == 0 && wr == 0) send_frame(0, 0); /* 都居中/急停: 混控停车帧 */
        else                    send_tank_frame((uint8_t)wl, (uint8_t)wr);
      } else {
        send_frame((uint8_t)lin, (uint8_t)rot);
      }
    }

    if (now - ui_t >= UI_MS) {              /* 10Hz 界面: 只重建显存 */
      ui_t = now;
      if (ui_force) { OLED_Clear(); ui_force = 0; }   /* 换页/换模式: 整屏重画 */
      ui_draw();
    }
    OLED_RefreshChunk();                    /* 每圈最多发一页, 不卡 30Hz 发送 */
    OLED_Poll();                            /* 屏掉线自愈心跳 */
  }
}
