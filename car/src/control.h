/**
 * control.h —— 运动控制层: PID + 轮速闭环
 *
 * 上层(各模式)只需要给出两个量:
 *     lin : 前进速度 cm/s (负 = 后退)
 *     rot : 旋转角速度 度/秒 (正 = 右转/顺时针)
 * 本层负责换算成左右轮目标速度, 用 PI 闭环输出 PWM。
 */
#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdbool.h>

typedef struct {
  float kp, ki, kd;
  float i;          /* 积分累加 */
  float ilim;       /* 积分限幅 */
  float prev;       /* 上次误差(微分用) */
  float out_lim;    /* 输出限幅 */
} pid_ctrl_t;

float pid_step(pid_ctrl_t *p, float err, float dt);
void  pid_reset(pid_ctrl_t *p);

void control_init(void);

/* 设定本轮目标, 内部会做加速度限幅 */
void control_set_cmd(float lin_cm_s, float rot_deg_s);
/* 模式二专用: 左右轮行程平衡 + 有界的逐轮起步补偿。 */
void control_set_spin(float rot_deg_s);

/* 原地转(带左轮增益): gain_l = 左轮目标速度倍率 (1.0 对称; 0.8 = 左轮降 20%) */
void control_set_spin_ex(float rot_deg_s, float gain_l);

/* 内轮枢轴转: 绕内侧轮子转 (内轮停, 外轮动), 转轴半径 = 半轮距。
   转完探头落在线附近, 出弯不偏。 */
void control_set_inner_pivot(float rot_deg_s);

/* 直角弯的差速转 (循线用): 外轮 outer_cm_s 正转, 内轮 inner_cm_s 带符号
   (负 = 反转), dir: -1 左转 / +1 右转。 */
void control_set_arc(float outer_cm_s, float inner_cm_s, int dir);
/* 坦克/直控轮: 直接给左右轮目标速度 (cm/s, 带符号), 不经混控;
   目标 0 的轮子速度环主动钉在 0 (原地静止), 另一轮照常转 */
void control_set_wheels(float l_cm_s, float r_cm_s);
/* 循迹弯道: 以进入本次原地转向时的行程为平衡基准。 */
void control_set_pivot(float rot_deg_s);

/* 100Hz 调用: 读编码器 -> 算速度 -> PI -> PWM */
void control_update(float dt);

void  control_stop(void);     /* 断电机, 滑行停车 */
void  control_brake(void);    /* 刹车 */
/* Immediate remote stop without Flash writes. */
void  control_brake_fast(void);
void  control_reset(void);    /* 清目标/积分/滤波 (启动新一轮之前调用) */
float control_speed_l(void);  /* 实测左轮速度 cm/s */
float control_speed_r(void);
float control_lin(void);      /* 实测前进速度 cm/s */
float control_dist_cm(void);  /* 从上电到现在的累计里程 cm */
extern float dist_l_cm, dist_r_cm;  /* 左右轮各自累计里程 (标定/诊断用) */

/* 左轮加速斜坡系数: 按模式分开
   - 循迹(模式5/6): 压左轮(实测加速右偏)
   - 遥控: 两轮目标斜坡相同，电机差异由各自速度环补偿
   - 其它(里程计/定速): 保留原有补偿
   app.c 在起跑时按当前模式设定 acc_slew_l */
#define ACC_SLEW_L       1.00f   /* Geometric tracking: equal straight acceleration targets. */
#define ACC_SLEW_L_OTHER 0.90f   /* 同上: 0.78 时加速左偏, 放回 0.90 */   /* 其它模式左轮斜坡 (0.90 左轮偏快, 0.80 偏慢) */
#define ACC_SLEW_L_RC    1.00f   /* MIX/TANK 遥控: 等加速度目标，避免加速主动制造偏航 */
extern float acc_slew_l;         /* 当前生效值 (由 app.c 每次起跑设定) */

/* 轮速闭环同步 (直线时): 慢积分消稳态不对称, 不依赖电池/地面状态 */
#define KI_SYNC     0.8f    /* 积分增益 (0=关): ~1.2s 时间常数, 振荡就减半 */
#define SYNC_MAX    3.0f    /* 同步修正限幅 cm/s */
extern float sync_trim;         /* 当前同步修正量 (诊断可读) */
void  control_reset_dist(void);

/* ---- 现场调试主要调这几个 ---- */
#define SPD_KP       10.0f    /* 速度环比例 */
#define SPD_KI       45.0f    /* 速度环积分 */
/* 前馈: 每 cm/s 需要多少 PWM, 按轮分开 —— 手转实测左编码器 1556.5 计数/圈、
   右 1322.3 计数/圈, 两个电机齿轮箱减速比差 17.7%, 同样占空比左轮天然慢;
   数值取自 24cm/s 巡航时的实际占空比 (左24.7% 右21.3%), PI 只管修正瞬态 */
#define SPD_KFF_L    32.3f   /* 实测左轮约快0.5%, 32.5 -> 32.3 */
#define SPD_KFF_R    32.5f
#define SPD_ILIM     600.0f
#define MIN_PWM      240.0f   /* 克服静摩擦的最低 PWM, 太小会不动 */
#define SPD_LPF      0.45f    /* 速度滤波系数 0~1, 越小越平滑但越迟钝 */
/* 加速度限幅 cm/s²。
   原来 220 是"弹射起步"(0→20cm/s 只要 0.09 秒), 轮子会空转一小段 ——
   编码器把空转也记成里程, 造成"走 50cm 只走 45.5cm"的固定误差;
   两个轮子打滑程度不同, 还会让车起步跑歪。
   降到 80 之后 0→20cm/s 用 0.25 秒, 起步柔和, 这两个毛病一起好。 */
#define ACC_LIMIT    100.0f   /* 150 时提速打滑引起偏航, 放缓到 100 */   /* 出弯回速 0.2s; 起步打滑有摩擦补偿兜底 */
#define SPD_MAX_CM_S 130.0f   /* 目标速度上限 */
#define ROT_MAX_DPS  220.0f   /* 目标角速度上限 */

#endif
