/**
 * imu.c —— MPU6050 姿态: 陀螺仪积分求航向
 *
 * 重要: 模块的安装方向不一定水平!
 *   如果模块平着插(芯片朝上), 竖直方向是 Z 轴 -> 用陀螺 Z 轴算偏航;
 *   如果模块立着插, 竖直方向可能是 X 或 Y 轴 -> 转车身会让那个轴的读数变化,
 *   而 Z 轴几乎不变。所以开机时用**加速度计**判断哪个轴指向竖直方向
 *   (静止时那个轴会读到 ±1g), 就用那个陀螺轴作为偏航轴, 并据此决定符号。
 */
#include "imu.h"
#include "board.h"
#include "soft_i2c.h"
#include "util.h"
#include <string.h>

/* ================= 可调参数 ================= */
#define GYRO_LSB_PER_DPS   32.8f    /* ±1000°/s 量程 */
#define CALIB_SAMPLES      256
#define IMU_STALE_MS       200      /* 超过这么久没读到数据 = 数据过期(掉线) */
#define IMU_FAIL_LIMIT     20       /* 连续读失败这么多次 = 掉线 (100Hz 下 0.2 秒) */
#define CALIB_MIN_N        64       /* 校准至少采到这么多点才敢更新零偏 */
/* =========================================== */

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

uint8_t mpu_who = 0;           /* WHO_AM_I: 0x68=MPU6050, 0x70~0x73=MPU6500 兼容 */
uint8_t mpu_addr = MPU_ADDR;   /* 实际应答的地址 */
uint8_t mpu_pwr = 0;           /* 回读 PWR_MGMT_1: 0x00 才说明真的唤醒了 */
uint8_t mpu_gcfg = 0;          /* 回读 GYRO_CONFIG */
uint8_t yaw_axis = 2;          /* 偏航轴: 0=X 1=Y 2=Z, 开机自动判断 */

int16_t gyro_raw[3] = {0, 0, 0};    /* 最近的原始陀螺读数 (调试页显示) */
int16_t accel_raw[3] = {0, 0, 0};   /* 原始加速度读数 */

static float   yaw_sign = -1.0f;
static float   bias = 0.0f;
static float   yaw_deg = 0.0f;
static float   gyro_dps = 0.0f;
static bool    present = false;

/* 运行中掉线检测用 */
static uint32_t last_ok_ms = 0;    /* 最近一次读成功的时间 */
static uint16_t fail_cnt = 0;      /* 连续读失败次数 */
uint16_t imu_calib_n = 0;          /* 上次零偏校准采到的样本数 */

static int read_raw6(uint8_t reg, int16_t *out) {
  uint8_t b[6];
  if (soft_i2c_read_regs(mpu_addr, reg, b, 6) != 0) return -1;
  out[0] = (int16_t)((b[0] << 8) | b[1]);
  out[1] = (int16_t)((b[2] << 8) | b[3]);
  out[2] = (int16_t)((b[4] << 8) | b[5]);
  return 0;
}

/* 用加速度计判断模块朝向: 竖直轴静止时读到约 ±1g, 它就是偏航轴 */
static void detect_yaw_axis(void) {
  int16_t a[3];
  if (read_raw6(REG_ACCEL_XOUT_H, a) != 0) return;
  memcpy(accel_raw, a, sizeof(accel_raw));

  int best = 2;
  int32_t bestv = -1;
  for (int i = 0; i < 3; i++) {
    int32_t v = (a[i] < 0) ? -a[i] : a[i];
    if (v > bestv) { bestv = v; best = i; }
  }
  if (bestv < 4000) return;                 /* ±2g 量程下 1g≈16384, 太小说明车没放平 */
  for (int i = 0; i < 3; i++) {             /* 其它轴不能也很大, 否则不判断 */
    if (i == best) continue;
    int32_t v = (a[i] < 0) ? -a[i] : a[i];
    if (v > bestv / 2) return;
  }
  yaw_axis = (uint8_t)best;
  /* 轴向朝上时, 右转(顺时针)在这个轴上读数为负, 所以符号取 -1; 模块倒装则反过来 */
  yaw_sign = (a[best] > 0) ? -1.0f : 1.0f;
}

bool imu_init(void) {
  present = false;
  mpu_who = 0;

  if (soft_i2c_probe(MPU_ADDR) == 0) {
    mpu_addr = MPU_ADDR;
  } else if (soft_i2c_probe(MPU_ADDR_ALT) == 0) {
    mpu_addr = MPU_ADDR_ALT;
  } else {
    return false;
  }

  if (soft_i2c_read_regs(mpu_addr, REG_WHO_AM_I, &mpu_who, 1) != 0) return false;

  /* 很多标着 MPU6050 的模块其实是 MPU6500/9250, WHO_AM_I 不是 0x68,
     但用到的寄存器一样, 所以不卡这个值, 只记录下来。 */

  /* 每一次写入都要检查返回值: 软件 I2C 写失败时寄存器根本没改,
     后面所有读数就都是按旧配置来的 (比如量程不对, 角速度直接差一倍)。
     宁可开机就报"没找到", 也别带着错的配置跑。 */
  if (soft_i2c_write_reg(mpu_addr, REG_PWR_MGMT_1, 0x00) != 0) return false;  /* 唤醒 */
  HAL_Delay(50);
  if (soft_i2c_write_reg(mpu_addr, REG_SMPLRT_DIV, 0x04) != 0) return false;   /* 200Hz 采样 */
  if (soft_i2c_write_reg(mpu_addr, REG_CONFIG, 0x03) != 0) return false;       /* DLPF 44Hz */
  if (soft_i2c_write_reg(mpu_addr, REG_GYRO_CONFIG, 0x10) != 0) return false;  /* ±1000°/s */
  if (soft_i2c_write_reg(mpu_addr, REG_ACCEL_CONFIG, 0x00) != 0) return false; /* ±2g */
  HAL_Delay(20);

  /* 回读两个关键寄存器, 确认真的写进去了 (调试时读这两个变量就知道配置成没成) */
  if (soft_i2c_read_regs(mpu_addr, REG_PWR_MGMT_1, &mpu_pwr, 1) != 0) return false;
  if (soft_i2c_read_regs(mpu_addr, REG_GYRO_CONFIG, &mpu_gcfg, 1) != 0) return false;
  if ((mpu_pwr & 0x40) != 0) {          /* SLEEP 位还亮着 = 没唤醒成功, 再写一次 */
    soft_i2c_write_reg(mpu_addr, REG_PWR_MGMT_1, 0x00);
    HAL_Delay(50);
    soft_i2c_read_regs(mpu_addr, REG_PWR_MGMT_1, &mpu_pwr, 1);
    if ((mpu_pwr & 0x40) != 0) return false;
  }

  present = true;
  fail_cnt = 0;
  last_ok_ms = HAL_GetTick();
  detect_yaw_axis();      /* 先判断朝向, 再校准零偏 */
  imu_calibrate();
  return true;
}

void imu_calibrate(void) {
  int16_t g[3];
  float sum = 0.0f;
  int n = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    if (read_raw6(REG_GYRO_XOUT_H, g) == 0) {
      sum += (float)g[yaw_axis];
      n++;
    }
    delay_us(800);
  }
  imu_calib_n = (uint16_t)n;
  /* 采样太少说明 I2C 一直失败, 这时候算出来的零偏是垃圾, 宁可保留旧值 */
  if (n >= CALIB_MIN_N) {
    bias = sum / (float)n / GYRO_LSB_PER_DPS;
    fail_cnt = 0;
    last_ok_ms = HAL_GetTick();
  } else {
    fail_cnt = IMU_FAIL_LIMIT;      /* 直接判掉线, 上层会拒绝启动/停车 */
  }
  yaw_deg = 0.0f;
}

void imu_update(float dt) {
  int16_t g[3];
  if (!present) return;
  /* 陀螺 4 倍过采样: 电机振动频率会被 100Hz 采样混叠成直流假偏置
     (实测转 360° 误差 ±5~10° 且方向随机), 连读 4 次取平均把它平均掉 */
  float sum = 0.0f;
  int ok = 0;
  uint32_t t0 = HAL_GetTick();
  for (int i = 0; i < 4; i++) {
    if (read_raw6(REG_GYRO_XOUT_H, g) == 0) {
      sum += (float)g[yaw_axis];
      ok++;
    }
    last_ok_ms = HAL_GetTick();
    /* 自适应上限: 软 I2C 慢的时候少采几次, 别挤爆 10ms 控制环 */
    if (i < 3 && (HAL_GetTick() - t0) >= 3u) break;
  }
  if (ok == 0) {
    if (fail_cnt < 60000) fail_cnt++;      /* 读失败: 航向保持不动, 由 imu_alive() 判掉线 */
    return;
  }
  fail_cnt = 0;
  memcpy(gyro_raw, g, sizeof(gyro_raw));
  float dps = sum / (float)ok / GYRO_LSB_PER_DPS - bias;
  gyro_dps = dps;
  yaw_deg += yaw_sign * dps * dt * GYRO_SCALE;   /* GYRO_SCALE: 比例校准, 见 board.h */
}

void imu_reset_yaw(float deg) { yaw_deg = deg; }
/* 交叉检查判定驼螺不可信时调用: 假转速会被阻尼项放大成真转向,
   必须置零才不会把车拧歪 */
void imu_zero_rate(void) { gyro_dps = 0.0f; }
float imu_yaw(void)    { return yaw_deg; }
float imu_gyro_z(void) { return gyro_dps; }
/* 与航向同号的角速度 = d(yaw)/dt: 航向是 yaw_sign*dps 积分出来的,
   而 imu_gyro_z() 返回的是没乘 yaw_sign 的原始值, 两者符号相反 (实测 yaw_sign=-1)。
   阻尼/转速反馈必须用这个, 用错就变成正反馈(越转越偏) */
float imu_yaw_rate(void) { return yaw_sign * gyro_dps; }
bool  imu_ok(void)     { return present; }

bool imu_alive(void) {
  if (!present) return false;
  if (fail_cnt >= IMU_FAIL_LIMIT) return false;
  /* 数据过期也算掉线: 即使没有连续失败, 超过 IMU_STALE_MS 没有新数据也不能用 */
  return (uint32_t)(HAL_GetTick() - last_ok_ms) <= IMU_STALE_MS;
}
