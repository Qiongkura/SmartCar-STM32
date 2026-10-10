/**
 * imu.h —— MPU6050 姿态: 只取陀螺 Z 轴积分得到航向角 yaw
 *
 * 小车跑几分钟的场合, 零偏校准后陀螺积分足够用 (MPU6050 没有磁力计,
 * 没法做绝对航向, 但做"保持原方向"和"转指定角度"完全够)。
 * 上电和长按 K5 都会重新校准零偏, 校准时车必须静止。
 */
#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include <stdbool.h>

bool  imu_init(void);            /* 返回 false = 没找到 MPU6050 或初始化写寄存器失败 */
void  imu_calibrate(void);       /* 重校零偏, 车必须静止 (约 1 秒) */
void  imu_update(float dt);      /* 100Hz 调用 */
void  imu_reset_yaw(float deg);  /* 把当前航向置为 deg */

float imu_yaw(void);             /* 航向角(度), 顺时针为正 */
float imu_gyro_z(void);          /* 角速度(度/秒) */
float imu_yaw_rate(void);        /* 航向角速度 d(yaw)/dt (与航向同号) */
void  imu_zero_rate(void);       /* 置零当前角速度 (驼螺不可信时用) */
bool  imu_ok(void);              /* 开机时认到过陀螺仪 */

/* 运行中是否还"活着": 连续读失败或数据过期(掉线)都返回 false。
   上层每个控制周期都要查 —— 陀螺掉线后航向就停止更新了,
   车会拿着一个越来越不准的旧航向继续跑, 必须停车报故障。 */
bool  imu_alive(void);

/* 调试用: WHO_AM_I 读到的值 (0x68=MPU6050, 0x70~0x73=MPU6500/9250 兼容芯片),
   以及实际应答的 I2C 地址 (0x68 或 0x69) */
extern uint8_t mpu_who;
extern uint8_t mpu_addr;
extern uint8_t mpu_pwr;          /* 回读的电源管理寄存器: 0x00 = 已唤醒 */
extern uint8_t mpu_gcfg;         /* 回读的陀螺量程寄存器: 0x10 = ±1000°/s */
extern uint8_t yaw_axis;         /* 自动判断出的偏航轴: 0=X 1=Y 2=Z */
extern int16_t gyro_raw[3];      /* 三轴陀螺原始读数 (调试页显示) */
extern int16_t accel_raw[3];     /* 三轴加速度原始读数 */
extern uint16_t imu_calib_n;     /* 上次零偏校准成功采到的样本数 (远小于 256 = 读数不稳) */

#endif
