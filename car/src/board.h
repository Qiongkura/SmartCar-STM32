/**
 * board.h —— 全部硬件引脚定义和可调参数
 *
 * 引脚不是随便定的, 是从"单协二面小车"PCB 工程的网表里读出来的, 别改错。
 * 需要现场调试的参数都集中在本文件 + control.h, 改这里就够了。
 */
#ifndef __BOARD_H
#define __BOARD_H

#include "main.h"

/* ===================== OLED (I2C1 重映射到 PB8/PB9) ===================== */
#define OLED_I2C_ADDR    0x78     /* 0x3C<<1, 开机自动探测 */
#define OLED_I2C_ADDR_ALT 0x7A    /* 0x3D<<1, 探测不到 0x3C 时用它 */
#define OLED_USE_SH1106  0        /* 1.3 寸屏改成 1 */
#define OLED_I2C_TIMEOUT 50

/* ===================== MPU6050 (软件 I2C: PA15/PB3) =====================
 * 这两个脚是 JTAG 的 JTDI/JTDO, 初始化里必须先关 JTAG (保留 SWD) 才能当 GPIO 用。
 * 板子上没有硬件 I2C 能走到这两个脚 (I2C1 给 OLED, I2C2 的 PB10/PB11 给了按键和超声波),
 * 所以只能用软件 I2C。
 */
#define MPU_SCL_PORT   GPIOA
#define MPU_SCL_PIN    GPIO_PIN_15
#define MPU_SDA_PORT   GPIOB
#define MPU_SDA_PIN    GPIO_PIN_3
#define MPU_ADDR       0x68       /* ADO 接地, 开机自动探测 */
#define MPU_ADDR_ALT   0x69       /* ADO 接高时的地址 */
#define MPU_INT_PORT   GPIOB      /* 中断脚, 本固件用轮询, 没用到 */
#define MPU_INT_PIN    GPIO_PIN_5

/* ===================== 电机 TB6612 ===================== */
/* 电机A: PWMA=PA11(TIM1_CH4), AIN1=PB13, AIN2=PB12, 编码器=PB6/PB7(TIM4) */
/* 电机B: PWMB=PA8 (TIM1_CH1), BIN1=PB14, BIN2=PB15, 编码器=PA0/PB1(TIM2) */
#define MOTA_IN1_PORT  GPIOB
#define MOTA_IN1_PIN   GPIO_PIN_13
#define MOTA_IN2_PORT  GPIOB
#define MOTA_IN2_PIN   GPIO_PIN_12
#define MOTB_IN1_PORT  GPIOB
#define MOTB_IN1_PIN   GPIO_PIN_14
#define MOTB_IN2_PORT  GPIOB
#define MOTB_IN2_PIN   GPIO_PIN_15

#define PWM_MAX        3600       /* TIM1 自动重装值, 20kHz */

/* ---- 装车调试时如果电机方向反了, 改这 4 行 (改 0/1 即可, 不用重新理解代码) ---- */
#define MOTOR_A_IS_LEFT      1    /* 电机A 装在左边=1, 装在右边=0 */
#define MOTOR_A_OUT_REVERSE  1    /* A 电机转向相反就改 1  (实测: 两个电机都反, 已置 1) */
#define MOTOR_B_OUT_REVERSE  1    /* B 电机转向相反就改 1 */
#define ENCODER_A_REVERSE    1    /* A 编码器计数方向相反就改 1  (实测: 往前转是负的, 已置 1) */
#define ENCODER_B_REVERSE    1    /* B 编码器计数方向相反就改 1  (同上) */

/* ===================== 超声波 HC-SR04 =====================
 * 注意: 模块电源在板上接的是 3.3V(VCC), 而 HC-SR04 标称要 5V。
 * 如果现场测距不稳/测不到, 把模块 VCC 改到 +5V (Echo 输出 5V, PB4 是 FT 引脚能承受)。
 */
/* 声速换算常数 (往返): 距离(cm) = 脉宽(us) / SONAR_US_PER_CM
   这个值取决于室温, 声速 v = 331.3 + 0.606*T(°C), 对应:
        0°C -> 60.4    10°C -> 59.3   20°C -> 58.2   21°C -> 58.0
       25°C -> 57.7    30°C -> 57.2   35°C -> 56.7   40°C -> 56.3
   室温偏高时用 58 会测偏少(按比例, 2m 处约差 2~3cm), 按实际室温改这个值就行。 */
#define SONAR_US_PER_CM  58.0f

/* 校准后距离 = 未补偿距离 * SCALE + OFFSET。
   两点标定: SCALE=(实际2-实际1)/(原始2-原始1), OFFSET=实际1-SCALE*原始1。
   默认比例不变, 先用调试第二屏的原始小数读数标定, 不要靠整数显示调整。 */
#define SONAR_SCALE      1.034f

/* 固定补偿: 实测值稳定地比真实值少(或多)多少 cm, 就填多少。
   注意区分两种情况:
     - 误差和距离成正比 (1m 差 1.3cm、2m 差 2.7cm) -> 是声速/温度问题, 改上面的 SONAR_US_PER_CM
     - 各距离都差同一个值 (20cm 和 2m 都差 2cm)   -> 是固定偏移, 改这里
   校准: 硬纸板放在已知距离处(尺子顶住两个圆柱探头的前端面量), 看调试页第二屏的
   "原始"那一行, 读数比真实值**多**就填负数, **少**就填正数。
   例: 真值 20cm, 读数 21cm -> 填 -1.0 */
#define SONAR_OFFSET_CM  1.60f  /* 标定(两轮共 8 点: 10/16.5/20/23.5/30 -> 8.5/13.8/17.7/21/27,
                                   又 24.8/30.5/37.5 -> 25.6/31.9/38.1):
                                   拟合 原始 = 0.9675×真值 - 1.55 -> 真值 = 原始×1.034 + 1.6
                                   残余 ±0.5cm 属于十欧本身噪声, 再拟合也压不下去了 */

/* 超声波测量周期 (ms)。HC-SR04 数据手册要求 >= 60ms, 否则上一次的余振会干扰下一次。
   调小 = 刷新更快更跟手, 但余振干扰和跳变会变多; 调大 = 更稳但更迟钝。
   滑动中位数在每次完成测量后更新; 三点窗口对持续变化约滞后一笔。 */
#define SONAR_PERIOD_MS  100
#define SONAR_STALE_MS   400u   /* 超过此时间没有完成新测量, 旧距离作废 */
#define SONAR_DISPLAY_ALPHA 0.35f /* 仅屏幕平滑, 物体计数不使用这个值 */
#define SONAR_DISPLAY_JUMP_CM 3.0f /* 明显换目标时直接跟随, 不缓慢拖尾 */

/* 中位数滤波窗口: 3 或 5。
   越大显示越稳, 但响应越迟钝 (五点窗口对持续变化约滞后两笔)。
   模式六建议保持 3 —— 车 30cm/s 前进, 物体掠过探头只有零点几秒,
   滤波太狠会把"经过时最近的距离"抹平, 反而测不准。 */
#define SONAR_MEDIAN_N   3

#define TRIG_PORT      GPIOB
#define TRIG_PIN       GPIO_PIN_11
#define ECHO_PORT      GPIOB
#define ECHO_PIN       GPIO_PIN_4    /* EXTI4, 5V 输入耐受 */

/* ===================== 蓝牙串口 (模式四 VOFA+) ===================== */
#define BT_BAUD        9600     /* HC-06 出厂默认就是 9600 (模块手册写明的);
                                   改成其它值的话模块和单片机必须一致, 否则一个字都传不过去。
                                   模式四里长按 L/R 可以在 9600/115200 之间临时切换。 */
/* 蓝牙上传: 0 = 只在模式四发送 (符合考核要求); 1 = 所有模式都发 (调 PID 时方便) */
#define VOFA_ALWAYS    0     /* HC-06 出厂默认 9600; 改过波特率就改这里 */

/* ===================== 按键 (低电平 = 按下) =====================
 * 5 个按键的物理位置对应关系 PCB 上看不出来, 装好后按一下看 OLED 反应,
 * 如果对不上, 把这 5 行的手指到别的脚即可 (都是从网表读出来的 5 个按键脚)。
 */
/* 板子上 5 个按键是一排的, 从左到右依次接 A12 B10 C13 C14 C15,
   所以按物理顺序分配功能: 最左边 = K1。 */
#define KEY_MODE_PORT  GPIOA      /* K1 (最左): 切换模式 */
#define KEY_MODE_PIN   GPIO_PIN_12
#define KEY_UP_PORT    GPIOB      /* K2: + */
#define KEY_UP_PIN     GPIO_PIN_10
#define KEY_DOWN_PORT  GPIOC      /* K3: - */
#define KEY_DOWN_PIN   GPIO_PIN_13
#define KEY_SET_PORT   GPIOC      /* K4: 方向 / 设置 */
#define KEY_SET_PIN    GPIO_PIN_14
#define KEY_RUN_PORT   GPIOC      /* K5 (最右): 启动/停止 (长按=陀螺重新校准) */
#define KEY_RUN_PIN    GPIO_PIN_15

/* ===================== 8 路循迹 (10P 座, 1~8 脚) =====================
 * 硬件顺序: PA2 PA3 PA4 PA5 PA6 PA7 PB0 PB1
 * 如果模块装反了(左右颠倒), 把 LINE_REVERSE_ORDER 改成 1。
 * 如果传感器"压线输出高"而代码判断相反, 改 LINE_ACTIVE_LEVEL。
 */
#define LINE_ACTIVE_LEVEL  0      /* 0 = 检测到黑线时输出低电平(常见); 1 = 相反 */
/* ★ 实测: 传感器左右顺序和 PCB 走线相反 —— 车头偏左时是左边的位为 1,
   会导致循迹往反方向打舵 (左右甩 → 丢线)。置 1 之后: 线在右边时右边位为 1。 */
#define LINE_REVERSE_ORDER 1

/* ===================== 陀螺比例校准 =====================
 * 使用地面标记确定实际转角, 不把闭环设定角度当成实际角度。
 * 新系数 = 旧系数 * 实际转角 / 同一时刻的显示角度增量。
 * 2026-10-05: 用户实测实际360度时显示356~357度, 取356.5度:
 * 1.007 * 360 / 356.5 = 1.0168878, 暂取1.017, 待左右多次复测。
 * 此系数用于所有模式的航向角积分, 不改变原始角速度显示。
 */
/* 过采样修好后重新实测: 右360多7° 左360多5° -> 陀螺少读 1.67%,
   刻度 1.000 x (360/366) -> 1.017 (和最早的原始标定值一致) */
#define GYRO_SCALE       1.017f

/* ===================== 机械参数 (必须实测校准) ===================== */
#define WHEEL_DIAMETER_CM   6.5f  /* 轮子直径 */
#define WHEEL_BASE_CM       12.0f /* 两轮中心距 */

/* 8 路探头的机械参数 (照实物量) */
/* Measured by the owner: probe row is 4.8cm ahead of the drive axle. */
#define LINE_SENSOR_FORWARD_CM 4.8f
/* Diagnostic buffers stay in RAM; no runtime erase can overlap firmware. */
#define DIAG_FLASH_LOGS 0
#define LINE_SENSOR_PITCH_CM  1.1f   /* 相邻两路探头的横向间距 cm */
#define LINE_SENSOR_OFFSET_CM 0.0f   /* 循线探头(排)中心相对车心的横向偏移 cm (0=不补).
                                        注意: 这个不是模式6的测距补偿,
                                        测距补偿在 app.c 的 OBJ_TRACK_OFFSET_CM */
/* 每厘米走多少个编码器计数 —— ★ 左右轮分开标!
 * 两个电机的减速比/编码器线数不一定完全一样 (常见: 一个 1:30 一个 1:48),
 * 用同一个系数会让速度环把其中一个轮子一直调得更快 ——
 * 表现就是"起步时一边快、车往另一边偏", 而且偏航 PID 只能边偏边纠, 治不了根。
 * 标定: 进调试页记下 编码L/R, 用手把车往前推 1 米, 再记一次:
 *         左轮系数 = (推完的编码L - 推前的编码L) / 100
 *         右轮系数 = (推完的编码R - 推前的编码R) / 100 */
/* ★ 实测: 左轮 5 圈 7775 计数, 右轮 5 圈 6609 计数 —— 左边多 17.6%,
   两个电机规格不一样 (不是同一批次/同一减速比)。
   用同一个系数会让速度环把右轮加速 17.6% 去"追"左轮, 车就一直往左偏。
   标定值: 1555/(π×6.5)=76.15, 1322/(π×6.5)=64.73, 平均值 70.6 (和距离标定一致) */
#define ENCODER_CM_L  77.4f      /* 左轮: 每厘米计数 (原 76.3, +1.4%:
                                   实测 50cm 目标固定少走 ~0.7cm = 里程多报 1.4%) */
#define ENCODER_CM_R  64.90f      /* 65.8 → 73.9: 实测直线右轮多走 12.1% */      /* 右轮: 每厘米计数 (同步 +1.4%) */
/* ↑ 每厘米走多少个编码器计数(4 倍频)。
 *   ★ 实测标定值: 命令 50/70/80/90/100cm 时实际只走了 46/64/73/82/92cm
 *     (五次都差 8%), 按公式 64.6 × 100/92 = 70.2~70.9, 取 70.6。
 *     以后重新标定还是这个公式: 新值 = 旧值 × 程序显示 ÷ 尺子实测
 *   默认按 11 线编码器 + 30:1 减速 + 65mm 轮子: 11*30*4/(π*6.5)=64.6
 *
 *   校准方法: 跑模式三的 100cm, 再用尺子量实际走了多少。
 *   程序里的距离 = 计数 / 这个系数, 所以这个系数填大了, 显示的距离就偏小。
 *       新值 = 旧值 × 程序显示的路程 ÷ 尺子实测的路程
 *   例: 显示 100cm、实际走了 120cm -> 新值 = 64.6 × 100/120 = 53.8 (要减小!)
 *   反过来: 显示 100cm、实际只走 80cm -> 新值 = 64.6 × 100/80 = 80.8 (要增大)
 *   这个系数同时决定里程显示、速度显示和速度闭环, 校准前先别调 PID。 */

#endif /* __BOARD_H */
