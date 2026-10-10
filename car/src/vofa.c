#include "vofa.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

static uint8_t txbuf[72];

/* 发送计数器: 每成功启动一次串口发送就 +1。
   调试时用调试器读这个变量, 就能知道模式四的数据到底发出去了没有。
   正常模式四下大约每秒增加 20。 */
volatile uint32_t vofa_tx_count = 0;

/* 把浮点写成 "12.3" 这种定点文本 (避免引入浮点 printf 那一大坨代码) */
static int append_fixed(char *dst, float v) {
  int neg = 0;
  if (v < 0.0f) { neg = 1; v = -v; }
  if (v > 99999.0f) v = 99999.0f;
  long x = (long)(v * 10.0f + 0.5f);
  if (neg) {
    dst[0] = '-';
    snprintf(dst + 1, 15, "%ld.%ld", x / 10, x % 10);
  } else {
    snprintf(dst, 16, "%ld.%ld", x / 10, x % 10);
  }
  return (int)strlen(dst);
}

void vofa_init(void) {
  /* 串口本身在 main.c 里初始化, 这里只确保接口可用 */
}

static void send_line(char *s, int len) {
  if (huart1.gState != HAL_UART_STATE_READY) return;   /* 上一次还没发完就跳过 */
  if (len > (int)sizeof(txbuf)) len = (int)sizeof(txbuf);
  memcpy(txbuf, s, (size_t)len);
  if (HAL_UART_Transmit_IT(&huart1, txbuf, (uint16_t)len) == HAL_OK) {
    vofa_tx_count++;
  }
}

void vofa_send2(float a, float b) {
  char line[40];
  int n = 0;
  n += append_fixed(line + n, a);
  line[n++] = ',';
  n += append_fixed(line + n, b);
  line[n++] = '\n';
  send_line(line, n);
}

void vofa_send3(float a, float b, float c) {
  char line[48];
  int n = 0;
  n += append_fixed(line + n, a);
  line[n++] = ',';
  n += append_fixed(line + n, b);
  line[n++] = ',';
  n += append_fixed(line + n, c);
  line[n++] = '\n';
  send_line(line, n);
}

/* 四通道: 目标速度, 实际速度, 航向, 角速度 —— VOFA+ 里会画成四条曲线 */
void vofa_send4(float a, float b, float c, float d) {
  char line[64];
  int n = 0;
  n += append_fixed(line + n, a);
  line[n++] = ',';
  n += append_fixed(line + n, b);
  line[n++] = ',';
  n += append_fixed(line + n, c);
  line[n++] = ',';
  n += append_fixed(line + n, d);
  line[n++] = 10;   /* 换行符 */
  send_line(line, n);
}

void vofa_send_str(const char *s) {
  send_line((char *)s, (int)strlen(s));
}
