#include "rc_protocol.h"
/**
 * app.c —— 应用层: 7 个模式的调度 + 按键交互 + OLED 界面
 *
 * 按键约定 (5 个键, 物理位置对应关系可在 board.h 里对调):
 *   K1 短按 : 切换模式 1→2→…→7→1       K1 长按 : 进入/退出调试页
 *   K2 短按 : 参数 + (长按连加)          K3 短按 : 参数 - (长按连减)
 *   K4 短按 : 模式二的转向方向           K4 长按 : 模式四切蓝牙波特率
 *   K5 短按 : 启动 / 停止                K5 长按 : 重新校准陀螺零偏 (车要静止)
 *
 * 界面文字宽度限制: 一行 16 格, 一个汉字占 2 格, 一个 ASCII 字符占 1 格。
 * 改文字时注意别超宽, 超了会被截掉。
 *
 * 超声波数据分四路, 用途不同不要混 (见 sonar_poll):
 *   原始值 -> 标定用 / 校准值 -> 单次结果 / 中位数 -> 物体计数 / 平滑值 -> 屏幕
 */
#include <stdio.h>
#include <string.h>

#include "app.h"
#include "board.h"
#include "control.h"
#include "hcsr04.h"
#include "imu.h"
#include "keys.h"
#include "line.h"
#include "line_map.h"
#include "motor.h"
#include "oled.h"
#include "pwrlog.h"
#include "soft_i2c.h"
#include "util.h"
#include "vdd.h"
#include "vofa.h"

/* ================= 现场调试主要调这些 ================= */
#define CRUISE_CM_S       35.0f   /* 模式一 默认巡航速度 */
/* 方向保持 PID: 角度误差 -> 修正角速度。
   KP 从 2.5 提到 4.0 —— 起步时两个轮子打滑程度不同会让车偏,
   KP 太小纠正不过来, 表现就是"起步一直往一边偏"。 */
#define HEAD_KP           3.0f   /* 2.5 回中偏慢; 5.0 曾振荡, 3.0 取中间 */   /* 5.0 欠阻尼: 空白段航向 ±9° 振荡(实测) */
#define HEAD_KD           0.8f   /* 加阻尼, 压住航向环的来回甩 */
#define HEAD_KI           0.8f   /* 航向环积分: 纯比例压不住常值扰动(两轮巡航时的
                                    固定不对称), 空白段会以 ~1.5°/s 一路漂、修不回来。
                                    积分把这个常值消掉, 车才能真正对着空白区另一端 */
#define HEAD_ILIM         10.0f  /* 积分限幅 (ki*ilim = 最大积分输出 °/s) */
/* 模式二: 速度规划 -> 锁定刹车 -> 停稳检查 -> 有限次数低速微调。
   所有角度使用连续累计航向, 360 度不会被归一化为 0。 */
/* Mode 2 only: each direction indicated 360 deg at a measured 357 deg.
   Keep direction-specific calibration out of the global IMU/line control. */
#define TURN_RIGHT_RAW_RATIO (360.0f / 357.0f)
#define TURN_LEFT_RAW_RATIO  (360.0f / 357.0f)
#define TURN_TOL_DEG       1.0f
#define TURN_BRAKE_DEG     0.5f
#define TURN_MAX_DPS       90.0f
#define TURN_TRIM_DPS      12.0f
#define TURN_MIN_DPS       6.0f
#define TURN_ACCEL_DPS2    60.0f
#define TURN_BRAKE_DPS2    480.0f /* 预测刹车距离: 旧值300预测滑过量偏大, 360度实测固定少约3度 */
#define TURN_STOP_DPS      5.0f
#define TURN_SETTLE_MS     250u
#define TURN_BRAKE_MAX_MS  1800u
#define TURN_MAX_TRIMS     6u
#define TURN_STALL_MS      1500u
/* Mode 3 only: measured 50.3 cm for a 50 cm request. */
#define DIST_STOP_ADVANCE_CM 0.30f
#define DIST_TOL_CM       0.25f    /* 剩这么少就刹车。原 0.4 是低速下调的;
                                     现在减速曲线下轮速滔后 60~100ms, 最后 1cm
                                     实际比曲线要求的快, 实测 50 会走成 51 ——
                                     把这段滞后预支掉。停短了就往小调 */
#define DIST_DECEL        60.0f   /* 停车减速的减速度 cm/s² */
#define LINE_KP           100.0f   /* 循迹: 位置误差 -> 角速度 */
#define LINE_KGYRO   1.5f   /* 循线陀螺阻尼 (°/s 反向): 压突发扰动(打滑/被推), 主阻尼靠 KPSI */
#define LINE_KGYRO_ACC 3.5f /* 速度快速变化时额外加的陀螺阻尼 (压"加速甩尾") */
#define LINE_KPSI    10.0f  /* 8 回中偏慢, 加强航向收敛; 摆就退回 8 */   /* 循线航向误差项 (°/°): 车头相对线方向的偏角反向。
                               原来只用 e 的微分当阻尼, 而 e 是量化的(0.14 台阶)
                               -> 微分很脏; 航向误差是连续的, 是环路正经的阻尼项:
                               2ζω = KPSI/(1+KGYRO), ω=√(0.01745·v·KP/(L·(1+KGYRO))) */
#define LINE_KEST    0.25f  /* 连续偏差估计的修正权重 (每拍) */
#define LINE_HALF_SPAN_CM 3.85f  /* e=±1 对应的横向距离 (3.5×11mm) */
#define LINE_DEADZONE 0.05f /* 死区: 原 0.15 是为了压 0.14 的台阶噪声,
                               现在偏差是连续的了, 可以收小 */
#define LINE_KD           3.2f
/* Mode 5 only: gentler centering; mode 6 retains its existing gains. */
#define MODE5_LINE_KP       70.0f
#define MODE5_LINE_KD        1.2f
#define MODE5_LINE_KPSI      8.0f
#define MODE5_ROT_LIMIT     90.0f
#define MODE5_EDGE_LIMIT    60.0f
#define MODE5_ROT_SLEW     240.0f /* deg/s per second, ordinary line tracking only */
#define MODE5_ACC_FILTER_S   0.15f
#define LINE_CRUISE_CM_S  30.0f
#define LINE_CORNER_DEG   32.0f   /* 航向变化超过它算过了一个弯 */
#define LINE_MIN_GAP_CM   22.0f   /* 两个弯之间至少走这么远 */
#define LOST_TIMEOUT_MS   2500    /* 丢线后找线的最长时间, 超过就算跑飞了 */
/* 丢线之后分两种情况处理:
     - 线是从**侧面**跑掉的(丢线瞬间偏差还很大) -> 多半是直角弯没跟上,
       车冲出去了, 要朝线消失的方向转着找回来
     - 线是在**正前方**断掉的(丢线瞬间偏差很小) -> 地图上的 60cm 空白段,
       按原航向直行过去就行 */
#define LOST_ERR_MIN      0.35f   /* 最近偏差最大值大于它 -> 按"冲出了弯道"处理 */
#define LINE_CROSSING_MAX_MS 500u /* 直角弯穿越捕捉超时 */
#define LINE_PIVOT_DPS    150.0f  /* 原地转最高转速 */
/* 探头在轮轴前方 d 处: 探头刚看到拐角时轮轴还没到。不补这一段直接原地转,
   转完车会与下一条线横向错开 d (≈7cm), 永远接不上。前进 d-滑行 ≈ 5.5cm
   让轮轴也到拐角再转。偏了调这个值: 出弯线偏车左 = 补多了, 偏右 = 补少了 */
/* 左右弯各自的补偿前进量: 机械不对称(两轮/转向), 分开调 */
#define CORNER_ADVANCE_CM     (LINE_SENSOR_FORWARD_CM - 0.8f)    /* Measured offset minus braking travel */
#define CORNER_ADVANCE_CM_S 20.0f  /* 补偿段速度 cm/s */
#define SPIN_GAIN_L       1.0f    /* 1.15 → 1.0: 编码器标定修正后恢复对称 */
#define LINE_PIVOT_MIN_DEG 25.0f
#define LINE_PIVOT_MAX_DEG 115.0f
/* 按弯微调 (度): 某个弯固定少转/多转时用 (实测第3弯少 5°)
   正值 = 多转, 负值 = 少转; 常用 0.5° 一步 */
#define LINE_TURN_TRIM_C1 0.0f
#define LINE_TURN_TRIM_C2 0.0f
#define LINE_TURN_TRIM_C3 0.0f
#define LINE_TURN_TRIM_C4 0.0f
#define LINE_TURN_DEG     90.0f   /* 直角弯: 相对弯前平均航向固定转这么多 */
#define LINE_ARC_OUTER_CM_S  40.0f  /* 直角弯: 外轮(前进那侧)速度 cm/s (随圈速放大) */
#define LINE_ARC_INNER_CM_S (-30.0f) /* 内轮: 反转 15cm/s (左弯=左轮 -15, 右弯镜像) */
#define LINE_PIVOT_LEAD_TRIM (-1.4f) /* 刹车提前量补正: 实测转完残余 +1.4°(正=少转),
                                        要晚刹一点, 所以是负值。改 1.6 时残余变 +3.4,
                                        一比一对应 -> 这个值直接加在 stop_deg 上 */
#define LINE_TURN_TOL      1.0f   /* 收到 1.0: 把剩余的 2° 交给补转 */   /* 航向到位容差 (收紧: 3° 在 60cm 空白段会偏 3cm) */
#define LINE_STRAIGHT_MAX_RATE 8.0f /* 学线方向时的最大车头转速 (°/s):
                                       加速甩尾/出弯修正那几拍车在转, 航向不代表
                                       线方向, 揉进均值就把基准带歪 */
#define LINE_STRAIGHT_EMA  0.1f  /* 0.03 滤波太慢: 弯前纠偏没算进均值, 目标偏 2~3° */  /* 直线航向求平均的滤波系数 (弯前那段路取均值) */
/* 出弯收尾: 航向由陀螺保证(弯前平均航向±90°), 这里只处理"探头下面没有线"。
   注意不能原地拧着"把线对到中间": 传感器量的是线的横向偏移, 不是航向 ——
   原地转时线扫过探头中心, 在任何歪的航向上都可能"看起来居中",
   拿它当停转判据就会停在歪航向上(之前出弯歪就是这么来的)。 */
#define LINE_SETTLE_CREEP    22.0f  /* 看不到线时往前找 cm/s (原 10 太慢: 每个弯拖 2s) */
#define LINE_SETTLE_MAX_CM   12.0f  /* 往前找最多走这么远 (找不到就交给循线空白段接管) */
#define LINE_SETTLE_SEARCH_DPS 30.0f /* 找线时朝外侧的转向速率 */
#define LINE_SETTLE_SEARCH_MAX_DEG 20.0f /* 找线最多偏这么多度, 超过就拉直 */
#define LINE_SETTLE_TIMEOUT  3000u  /* 这个阶段最长 3 秒, 超时放行 */
/* 出弯直行段: 弯道期间线偏差是残留的量化旧值, 一交给循线就会画龙
   (先按旧偏差打一边, 线跳到另一侧再猛打回来)。先按陀螺 90° 航向
   直行一小段, 让线偏差变成新鲜读数后再交给循线。 */
#define LINE_PULL_CM       7.0f   /* 4 -> 7: 出弯纠偏窗口太短 */   /* 出弯直行距离 (8 太长拖时间) */
#define LINE_PULL_MAX_CM_S 18.0f
#define LINE_PULL_KP       100.0f /* 60 → 100: 出弯更积极居中 */  /* 直行段速度上限 (随圈速放大) */
#define LINE_PULL_E_GIVEUP 0.45f  /* 线偏得离谱就提前交给循线 */
#define LINE_GAP_MAX_CM    80.0f /* 题目空白段60cm, 留出探头到轴心等余量 */
/* 分段速度调度 (用户方案): 首段 20 起步; 出弯后加速到巡航;
   距下个弯 25cm(段内 80cm) 减回 20 准备进弯。 */
#define LINE_SEGMENT_CM    100.0f /* 边长 100cm: 每段弯到弯距离 */
#define LINE_APPROACH_CM   10.0f  /* 段内走到 90cm(距下个弯 10cm)开始减速 */
#define LINE_APPROACH_CM_S 20.0f  /* 进弯准备速度 */
#define LINE_LAUNCH_CM_S   30.0f  /* 第一段直线(起步段)速度 */
/* 演示区单弯兜底: 进弯偏移会让垂直线只盖住 2~3 颗传感器(不到触发阈值),
   弯就被冲过去。丢线走 3cm 还没见线 -> 直接执行转弯。
   注意: 正式赛道有直线空白段(60cm), 那里丢线是正常的, 这个兜底必须
   改条件或关掉, 否则空白段起点就会误转弯! */
#define LINE_END_TURN_ENABLE 1   /* 开: 线断了就转弯的兜底 (下面有距离门, 不会在弯后空白段误转) */
#define LINE_END_TURN_CM     3.0f   /* 丢线走这么远还没见线就当弯 */
#define LINE_END_TURN_MIN_SEG_CM 50.0f /* 只有离上个弯超过这么远才算“下一个弯”:
                                          弯后的空白段紧跟着弯, 距离很短 */
#define LINE_END_TURN_DIR    (-1)   /* 演示弯是左弯; 换方向改这里 */

/* ---- 计圈: 数弯 + 回到起点 ----
   只数弯是不够的: 从直道中间起跑时, 第 4 个弯转完还没回到起点,
   车还要再走一段。这里用"编码器里程 + 航向"做航位推算, 判断车
   是否回到了起跑点附近、且航向和起跑时一致。 */
#define LAP_GOAL_R_CM     22.0f   /* 推算位置离起跑点这么近 = 回到起点了 */
#define LAP_GOAL_YAW_DEG  35.0f   /* 同时航向要和起跑时差不多(度) */
#define LAP_TOTAL_CM      400.0f  /* 一圈总里程 cm: 跑满就停 (实测微调) */
#define LAP_RETURN_MAX_CM 200.0f  /* 第 4 个弯之后最多再走这么远(兜底,
                                     防止推算误差太大时永远停不下来) */
#define LAP_MAX           5       /* 最多 5 圈 (和 p_laps 上限一致) */
#define OBJ_DETECT_CM     32.0f   /* 小于它认为看到物体 (题目: 物体离轨道 ≤30cm) */
/* 物体计数用"进入-跟踪-离开"状态机, 不再用"两个物体必须间隔 8cm"这种硬门槛
   (硬门槛会把挨得近的两个物体漏掉)。离开要同时看时间和里程:
   短暂丢一两次回波不能算离开, 否则同一个物体会被切成两个。 */
/* Mode 6 only, refit from the latest readings after the previous correction:
   14.4->13.1, 20.6->18.6, 31.6->30.4 cm.
   Installation offset is included in this fit; do not add it again.
   Raw sonar calibration and other modes are unchanged. */
#define OBJ_TRACK_CAL_SCALE  0.98686767f
#define OBJ_TRACK_CAL_OFFSET_CM (-0.54275670f)
#define OBJ_LOST_MS       250     /* 连续这么久没目标才算离开 */
#define OBJ_LOST_CM       12.0f   /* 或者又走了这么远还没见到目标, 也算离开 */
#define OBJ_MAX_NUM       12
#define VOFA_PERIOD_MS    50

/* ---- 模式七: 蓝牙遥控 ----
   帧格式 (5 字节定长):  AA 55 CMD ARG XOR      XOR = AA^55^CMD^ARG
   命令: 0x01 前进  0x02 后退  0x03 左转  0x04 右转  0x05 停止
   ARG : 0~100, 对应速度/转速档位 (停止时填 0)
   校验不过 / 未知命令一律丢弃, 绝不拿去驱动电机。 */
#define RC_FRAME_LEN      5
#define RC_BUF_LEN        6      /* 混控帧最长 6 字节 */
#define RC_TIMEOUT_MS     800     /* 无有效指令这么久 -> 断联刹车 (800ms: 短暂丢包不误触发, 又远小于人手反应) */
#define RC_CMD_FWD        0x01
#define RC_CMD_BACK       0x02
#define RC_CMD_LEFT       0x03
#define RC_CMD_RIGHT      0x04
#define RC_CMD_STOP       0x05
#define RC_CMD_MIX        0x06    /* 混控帧: AA 55 06 LIN ROT XOR, 边走边转 */
#define RC_CMD_TANK       0x07    /* 坦克帧: AA 55 07 WL WR XOR, 左右轮直控 (负值有符号) */
#define RC_LIN_CM_S       60.0f   /* ARG=100 时的前进速度 */
#define RC_ROT_DPS        120.0f  /* ARG=100 时的转速 */
/* ===================================================== */

/* 提前声明: sonar_poll 里要用到 (实现在下面的"小工具"一节) */
static float f_abs(float v);
static float f_clamp(float v, float lo, float hi);
static float ang_diff(float a, float b);
/* 诊断操作的统一出入口 (实现在 t_key 那一段后面) */
static void diag_begin(void);
static void diag_end(void);

/* ---- 模式七: 蓝牙遥控 状态 ---- */
static rc_parser_t rc_parser;
static uint8_t rc_neutral_count = 0;
static uint32_t rc_neutral_ms = 0;
volatile uint32_t rc_guard_count = 0;
volatile uint32_t rc_stale_count = 0;
#define RC_FRESH_MS 150u
#define RC_NEUTRAL_PAIR_MS 400u
static uint32_t rc_last_ms = 0;
static float    rc_lin = 0.0f, rc_rot = 0.0f;
static float    rc_wl = 0.0f, rc_wr = 0.0f;   /* 坦克帧: 左右轮目标 cm/s */
static bool     rc_tank = false;              /* 最近一帧是不是坦克帧 */
static bool     rc_wait = false;              /* 断联刹车后等待遥控恢复 (免按 K5) */
uint16_t rc_ok_cnt = 0;      /* 收到并执行的有效帧数 (调试用) */
uint16_t rc_bad_cnt = 0;     /* 校验错/未知命令被丢弃的帧数 (调试用) */

#define L0 0
#define L1 16
#define L2 32
#define L3 48

static car_mode_t mode = MODE_HEADING;
static bool running = false;
static bool finished = false;
static bool aborted = false;
static bool imu_fault = false;         /* 陀螺仪运行中掉线, 界面要报故障 */
static bool enc_fault = false;        /* 里程模式: 编码器没动, 界面报故障 */
static bool dbg_lock = false;          /* 做完阻塞操作(蓝牙自检/陀螺校准)之后,
                                          调试页要等测试键松开才恢复驱动电机 */

/* 模式参数 */
static int p_turn_dir = 1;             /* +1 右转, -1 左转 */
static int p_turn_angle = 90;          /* 0~360, 30 的倍数 */
static int p_dist_cm = 80;             /* 50~100 */
static int p_laps = 1;                 /* 1~5 */
static int p_speed = 40;               /* 模式四目标速度 cm/s */
static int p_cruise = 47;              /* 巡航速度 cm/s (模式一/三/五/六共用, 模式一里 K2/K3 调)
   420cm 轨道 17s/圈: 平均24.7, 扣掉弯道/出弯固定开销和摆线减速 -> 巡航42 */
static uint8_t obj_page = 0;
static uint32_t bt_baud = BT_BAUD;   /* 模式四用, K4 长按现场切换 */

static pid_ctrl_t pid_head;

/* 运行期 */
static float yaw_ref = 0.0f;
static uint32_t run_start_ms = 0;
static uint32_t run_end_ms = 0;        /* 完成时刻: 让计时在停车后冻住 */
enum { TURN_DRIVE, TURN_BRAKING };
enum { TURN_ERR_NONE, TURN_ERR_TIMEOUT, TURN_ERR_LEFT, TURN_ERR_RIGHT,
       TURN_ERR_SETTLE, TURN_ERR_IMU, TURN_ERR_DIRECTION };
static uint8_t turn_phase, turn_fault, turn_trims;
static uint32_t turn_brake_ms, turn_stable_ms;
static float turn_last_err, turn_rate_cmd, turn_stall_l, turn_stall_r;
static float lost_ms = 0.0f;
/* 循迹状态: 每次开始运行重置, 不沿用上一次的误差或转弯方向。 */
static bool line_corner_active = false, line_gap_active = false;
static int line_corner_dir = 1;
static float line_corner_yaw = 0.0f, line_center_ms = 0.0f;
static float line_prev_error = 0.0f, line_derivative = 0.0f;
static bool line_prev_valid = false;
static float mode5_rot_cmd = 0.0f, mode5_acc_damping = 0.0f;
static float line_e_lpf = 0.0f;        /* 平滑后的线偏差: 台阶值滤成斜坡, 治画龙 */
static bool line_track_seen = false;
static bool line_end_pending = false;    /* 断线位置看起来像下一个弯 */
static float line_gap_lost_e = 0.0f;     /* 丢线那一刻的线偏差 */
/* ---- 加速段记录器: 丢线瞬间开始录 0.64s 的航向/角速度/轮速差/车速,
   跑完落 Flash (0x0800E900)。专门用来看"加速右偏"那股旋转的形状 ---- */
#define ACC_REC_N   64
#define ACC_REC_ADDR 0x0800E900u
static volatile int16_t acc_yaw[ACC_REC_N], acc_rate[ACC_REC_N];
static volatile int16_t acc_dv[ACC_REC_N], acc_lin[ACC_REC_N];
static volatile uint8_t acc_rec_n = 0, acc_rec_done = 0;
static void acc_rec_tick(void) {
  if (acc_rec_done || acc_rec_n >= ACC_REC_N) { if (acc_rec_n >= ACC_REC_N) acc_rec_done = 1; return; }
  acc_yaw[acc_rec_n]  = (int16_t)(imu_yaw() * 10.0f);
  acc_rate[acc_rec_n] = (int16_t)(imu_yaw_rate() * 10.0f);
  acc_dv[acc_rec_n]   = (int16_t)((control_speed_l() - control_speed_r()) * 10.0f);
  acc_lin[acc_rec_n]  = (int16_t)(control_lin() * 10.0f);
  acc_rec_n++;
  if (acc_rec_n >= ACC_REC_N) acc_rec_done = 1;
}
static void acc_rec_save(void) {
#if DIAG_FLASH_LOGS
  if (!acc_rec_done) return;
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef er; uint32_t err = 0;
  er.TypeErase = FLASH_TYPEERASE_PAGES; er.PageAddress = ACC_REC_ADDR; er.NbPages = 1;
  if (HAL_FLASHEx_Erase(&er, &err) == HAL_OK) {
    uint32_t addr = ACC_REC_ADDR;
    uint16_t h[4] = {0xACDCu, (uint16_t)acc_rec_n, 0u, 0u};
    for (int i = 0; i < 4; i++) { HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, h[i]); addr += 2; }
    for (int k = 0; k < acc_rec_n; k++) {
      uint16_t f[4] = {(uint16_t)acc_yaw[k], (uint16_t)acc_rate[k], (uint16_t)acc_dv[k], (uint16_t)acc_lin[k]};
      for (int i = 0; i < 4; i++) { HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, f[i]); addr += 2; }
    }
  }
  HAL_FLASH_Lock();
#endif
}
static int8_t line_last_corner_dir = -1; /* 上个弯的方向 (同向赛道兜底用) */
static int8_t line_wide_side = 0;        /* 最近一次宽黑哪侧黑多 (±1, 0=未知) */
static float  line_wide_cm = -999.0f;    /* 那一次的里程 */
static uint16_t line_seen_cnt = 0;   /* 连续压到正常细线的拍数 */
static float line_straight_yaw = 0.0f;   /* 弯前直线段的平均航向 */
static bool  line_straight_valid = false;
static bool  line_settle_active = false; /* 出弯收尾中: 探头无线时低速往前找 */
static bool  line_pull = false;          /* 出弯直行段: 不拿旧线偏差打方向 */
static bool  line_end_turned = false;    /* 兜底转弯每轮只用一次 */
static float line_seg_start_cm = 0.0f;   /* 当前段起点(弯完成时刻的里程) */
static int   line_corners_done = 0;      /* 已完成的弯数(0 = 还在首段) */
static float line_settle_dist0 = 0.0f;
static uint32_t line_settle_t0 = 0;
static bool  line_corner_advanced = false;/* 直角弯第0步: 已补探头-轮轴距离 */
/* ---- 事件日志: 每个阶段切换记一条, SWD 直接读 (查"哪一步开始歪的") ----
   字段: 事件号 / 里程 / 航向x10 / 左右轮速差x10 / 相对出弯目标的偏差x10 / 时刻0.1s */
#define EV_MAX 96
static float line_corner_target;   /* 前置声明: ev_log 要用 (后面才定义) */
static volatile int16_t ev_id[EV_MAX];
static volatile int16_t ev_dist[EV_MAX];
static volatile int16_t ev_yaw[EV_MAX];
static volatile int16_t ev_dv[EV_MAX];
static volatile int16_t ev_dtgt[EV_MAX];
static volatile int16_t ev_dl[EV_MAX], ev_dr[EV_MAX];  /* 左右轮各自累计里程 cm */
static volatile uint8_t ev_cnt[EV_MAX], ev_lft[EV_MAX], ev_rgt[EV_MAX], ev_st[EV_MAX];
static volatile uint16_t ev_t[EV_MAX];
static volatile uint8_t ev_n = 0;
static volatile uint8_t ev_dirty = 0;

/* 事件日志落 Flash (4KB 页 @0x0800E000, 掉电不丢): 跑完/异常停车时存一次。
   字段顺序: id, dist, yaw, dv, dtgt, t, dist_l, dist_r */
#define EV_FLASH_ADDR 0x0800E000u
static void ev_save_flash(void) {
#if DIAG_FLASH_LOGS
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef er; uint32_t err = 0;
  er.TypeErase = FLASH_TYPEERASE_PAGES;
  er.PageAddress = EV_FLASH_ADDR;
  er.NbPages = 1;
  if (HAL_FLASHEx_Erase(&er, &err) == HAL_OK) {
    uint32_t addr = EV_FLASH_ADDR;
    uint16_t h[4] = {0xE7E7u, (uint16_t)ev_n, 0u, 0u};
    for (int i = 0; i < 4; i++) { HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, h[i]); addr += 2; }
    for (int k = 0; k < ev_n && k < EV_MAX; k++) {
      uint16_t f[12] = {(uint16_t)ev_id[k], (uint16_t)ev_dist[k], (uint16_t)ev_yaw[k],
                        (uint16_t)ev_dv[k], (uint16_t)ev_dtgt[k], ev_t[k],
                        (uint16_t)ev_dl[k], (uint16_t)ev_dr[k],
                        ev_cnt[k], ev_lft[k], ev_rgt[k], ev_st[k]};
      for (int i = 0; i < 12; i++) { HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, f[i]); addr += 2; }
    }
  }
  HAL_FLASH_Lock();
  ev_dirty = 0;
#else
  ev_dirty = 0;
#endif
}

static void ev_log(int id) {
  if (ev_n >= EV_MAX) return;
  ev_id[ev_n] = (int16_t)id;
  ev_dist[ev_n] = (int16_t)(control_dist_cm() + 0.5f);
  ev_yaw[ev_n] = (int16_t)(imu_yaw() * 10.0f);
  ev_dv[ev_n] = (int16_t)((control_speed_l() - control_speed_r()) * 10.0f);
  ev_dtgt[ev_n] = (int16_t)(ang_diff(imu_yaw(), line_corner_target) * 10.0f);
  ev_t[ev_n] = (uint16_t)(HAL_GetTick() / 100u);
  ev_dl[ev_n] = (int16_t)(dist_l_cm + 0.5f);
  ev_dr[ev_n] = (int16_t)(dist_r_cm + 0.5f);
  {
    uint8_t b = line_bits();
    int lf = 0, rg = 0;
    for (int i = 0; i < 8; i++) if (b & (1u << i)) { if (i < 4) lf++; else rg++; }
    ev_cnt[ev_n] = (uint8_t)line_count();
    ev_lft[ev_n] = (uint8_t)lf;
    ev_rgt[ev_n] = (uint8_t)rg;
    ev_st[ev_n] = line_corner_active ? 1 : (line_settle_active ? 2 : (line_pull ? 3 : (line_lost() ? 4 : 0)));
  }
  ev_n++;
  ev_dirty = 1;
}

static float line_corner_adv0 = 0.0f;
static bool  line_corner_stopped = false; /* 直角弯第1步: 已刹停 */
static uint8_t line_corner_phase = 0;    /* 0=旋转 1=刹停校验 */
static uint8_t line_corner_trims = 0;    /* 补转次数 */
static uint32_t line_corner_t0 = 0;
static uint8_t line_junc_hist = 0;       /* 最近 4 拍的单侧压黑历史 (bit0=最新) */
static int8_t  line_junc_side = 0;       /* 这个窗口里黑多的那侧 (±1, 0=未定) */
static float line_corner_target = 0.0f;  /* 本次转弯的目标航向(弯前均值±90°) */
static bool  line_crossing = false;     /* 直角弯穿越捕捉中 */
static float line_cross_last_e = 0.0f;
static int   line_cross_lft = 0, line_cross_rgt = 0, line_cross_dir = 0;
static float line_gap_start_cm = 0.0f;
static float line_hold_yaw = 0.0f;
static line_map_t track_map;
static float line_align_stable_ms = 0.0f;
static uint8_t line_exit_seen = 0;
/* SWD diagnostics: 1=orthogonal heading only, 2=straight fit, 3=position observed. */
volatile uint8_t line_map_confidence = 0;
volatile uint32_t line_map_version = 0;
volatile float line_map_heading_deg = 0.0f, line_map_cross_cm = 0.0f;
volatile uint8_t line_map_align_limited = 0;

static float lost_err = 0.0f;          /* 最近偏差最大的值 (决定丢线后"直行"还是"转着找") */
static uint32_t lost_err_ms = 0;      /* 上面那个值是什么时候记的 */
static float line_prev_e = 0.0f;       /* 平滑偏差的上一拍值 (求导用) */
static float line_hist[3] = {0.0f, 0.0f, 0.0f};   /* 弯前正常循线时的偏差短历史 */
static int   line_hist_n = 0;

/* 计圈 */
static float corner_yaw_ref = 0.0f;
static float dist_at_corner = 0.0f;
static int corner_count = 0, lap_done = 0;
static bool corner_armed = true;
static uint32_t straight_ms = 0;

/* 航位推算: 估计车相对起跑点的位置 (cm), 用来判断"回到起点了"
   pos_x 沿起跑时的航向, pos_y 是它的左/右侧 */
static float pos_x = 0.0f, pos_y = 0.0f;
static float pos_last_dist = 0.0f;     /* 上次推算用到的里程 */
static float lap_yaw_ref = 0.0f;       /* 本轮起跑时的航向 */
static float dist_at_corner4 = 0.0f;   /* 第 4 个弯时的里程 (兜底判据用) */

/* 计时: 每圈耗时 (毫秒), 用来验证"单圈<20秒" */
static uint32_t lap_ms[LAP_MAX];
/* 圈内分段时间 (诊断: 时间花在哪了) */
static volatile uint32_t t_corner_ms = 0;   /* 弯道三步走(补偿/刹停/原地转) */
static volatile uint32_t t_settle_ms = 0;   /* 出弯找线(低速爬) */
static volatile uint32_t t_pull_ms = 0;     /* 出弯直行段 */
static volatile uint32_t t_lost_ms = 0;     /* 空白段航向保持 */
static volatile uint32_t t_track_ms = 0;    /* 正常循线 */
static uint32_t lap_start_ms = 0;
static float lap_dist0 = 0.0f;         /* 本圈起点里程 (多圈判完成用) */

/* 超声波: 常驻测量, 每 SONAR_PERIOD_MS 一次。
   四路数据分开, 用途不同:
     sonar_raw  : 原始值(未补偿) —— 调试页标定用, 别拿它当结果
     sonar_cal  : 单次原始值经"比例+零点"校准后的结果
     sonar_cm   : 三次中位数(已校准) —— 物体计数用, 不叠加显示平滑
     sonar_disp : 屏幕显示用, 在中位数上再叠一层轻平滑(换目标时直接跟随) */
static float    sonar_raw = -1.0f;
static float    sonar_cal = -1.0f;
static float    sonar_cm  = -1.0f;
static bool     sonar_new = false;
static uint32_t t_sonar = 0;
float    sonar_disp = -1.0f;   /* 调试页显示用: 平滑后的距离, <0 表示没回波 */
uint16_t sonar_ok_cnt = 0;      /* 本次采样有效的次数 (调试用) */
uint16_t sonar_to_cnt = 0;      /* 本次采样无回波的次数 (调试用) */

/* 屏幕平滑: 小波动慢慢跟; 换目标(跳变超过阈值)时直接跟上, 不拖尾。
   只用于显示 —— 物体计数用的是未平滑的中位数, 免得把掠过时的最近距离抹平。 */
static float sonar_smooth(float v) {
  static float s = -1.0f;
  if (v < 0.0f) { s = -1.0f; return -1.0f; }             /* 没回波: 直接清掉 */
  if (s < 0.0f || f_abs(v - s) > SONAR_DISPLAY_JUMP_CM) { s = v; return s; }
  s += SONAR_DISPLAY_ALPHA * (v - s);
  return s;
}

static void sonar_poll(void) {
  uint32_t now = HAL_GetTick();
  if (!hcsr04_ready()) {
    if ((now - t_sonar) > SONAR_PERIOD_MS) {
      hcsr04_start();
      t_sonar = now;
    }
    return;
  }

  /* 本次采样: 原始值(判断这次有没有回波) + 中位数(计物用) */
  float raw = hcsr04_distance_raw_cm();      /* 未补偿; <0 = 本次无效或已过期 */
  float med = hcsr04_distance_median_cm();   /* 三次中位数, 已校准; <0 = 无效 */
  hcsr04_reset();

  sonar_raw = raw;
  sonar_cal = (raw > 0.0f) ? (raw * SONAR_SCALE + SONAR_OFFSET_CM) : -1.0f;

  /* 回波统计按"本次采样是否有效"统计, 不能按中位数结果统计 */
  if (raw > 0.0f) { if (sonar_ok_cnt < 65535) sonar_ok_cnt++; }
  else            { if (sonar_to_cnt < 65535) sonar_to_cnt++; }

  sonar_cm = med;
  sonar_disp = sonar_smooth(med);
  sonar_new = true;
}

/* 模式六: 物体计数 (进入-跟踪-离开 状态机) */
static float obj_dist[OBJ_MAX_NUM];
static int   obj_num = 0;
#define OBJ_IDLE     0
#define OBJ_TRACKING 1
static int      obj_state = OBJ_IDLE;
static uint32_t obj_last_seen_ms = 0;
static float    obj_last_seen_dist = 0.0f;
static int      obj_cur = 0;            /* 正在跟踪的是第几个物体 */

static float obj_track_distance_cm(float sensor_cm) {
  if (sensor_cm <= 0.0f) return -1.0f; /* No echo stays invalid. */
  float track_cm = sensor_cm * OBJ_TRACK_CAL_SCALE
                   + OBJ_TRACK_CAL_OFFSET_CM;
  return track_cm > 0.0f ? track_cm : 0.0f;
}

static void obj_update(void) {
  if (sonar_new) {
    sonar_new = false;
    float d = sonar_cm;                 /* 中位数 + 校准, 不含显示平滑 */
    if (d > 0.0f && d < OBJ_DETECT_CM) {
      float track_cm = obj_track_distance_cm(d);
      obj_last_seen_ms = HAL_GetTick();
      obj_last_seen_dist = control_dist_cm();
      if (obj_state == OBJ_IDLE) {      /* 确认进入: 计数一次 */
        if (obj_num < OBJ_MAX_NUM) {
          obj_dist[obj_num] = track_cm;
          obj_cur = obj_num;
          obj_num++;
        }
        obj_state = OBJ_TRACKING;
      } else if (obj_num > 0 && track_cm < obj_dist[obj_cur]) {
        obj_dist[obj_cur] = track_cm;   /* 记下最接近时的距离 */
      }
    }
  }
  /* 确认离开: 时间或里程任一超限 —— 短暂丢回波不会解除状态,
     所以同一个物体不会被切成两个; 挨得近的两个物体也不会被漏掉。 */
  if (obj_state == OBJ_TRACKING) {
    if ((HAL_GetTick() - obj_last_seen_ms) >= OBJ_LOST_MS ||
        (control_dist_cm() - obj_last_seen_dist) > OBJ_LOST_CM) {
      obj_state = OBJ_IDLE;
    }
  }
}

/* 调试页: 两屏, K5 短按切换 (第 0 屏 电机/陀螺/循迹, 第 1 屏 超声波标定) */
static float dbg_enc_l = 0, dbg_enc_r = 0;
static uint8_t dbg_page = 0;

/* ---- 蓝牙模块 AT 自检 ----
   板上串口直接连着蓝牙模块的 TXD/RXD, 所以能让固件直接问模块。
   调试页里: 短按 L/R = 自检(发 AT), 长按 L/R = 让它复位重新广播。
   下面两个变量是给调试器用的: 用 SWD 写 1 可以远程触发, 方便电脑这边排查。 */
volatile uint8_t bt_test_req = 0;
volatile uint8_t bt_reset_req = 0;
char     bt_resp[24] = "";
uint8_t  bt_rx_level = 0;   /* AT 无应答时 PA10(接模块 TXD) 的电平: 1=模块在驱动 0=没人驱动 */
uint8_t  bt_line_drive = 0; /* 线检测结果: 1 = 把 PA10 拉低再放开能自己回高
                               (= 模块在驱动这条线, 有电且 TXD 接上了);
                               0 = 回不去 (= 没人驱动: 模块没供电 / 线没接) */
uint32_t bt_show_until = 0;
/* 波特率扫描结果 (调试器可读) */
#define BT_SCAN_N 6
uint8_t  bt_scan_hit = 0;                  /* 有应答(任意字符)的波特率个数 */
uint32_t bt_scan_hit_baud[BT_SCAN_N];      /* 是哪些波特率 */
uint8_t  bt_scan_ok_cnt = 0;               /* 回 "OK" 的个数 */
char     bt_scan_raw[24] = "";             /* 收到的第一段原文 */

/* 蓝牙串口线检测。
   光读 PA10 的电平说明不了问题: 单片机自己内部上拉也会把它拉高。
   所以先把 PA10 拉低、再放开(高阻), 然后看电平能不能自己回到高 ——
   UART 空闲时 TXD 应该是高, 只有"模块真的在驱动这条线"才拉得起来。 */
static uint8_t bt_line_check(void) {
  GPIO_InitTypeDef g = {0};

  /* 先把旁边的 PA9(串口 TX) 拉低。它空闲是高电平、又是推挽输出,
     和 PA10 就挨在一起, 电容耦合可能把 PA10 也"带"成高电平 —— 那样检测会误判。
     拉低之后 PA10 还能自己回到高, 才一定是外面真有器件在驱动它。 */
  g.Pin = GPIO_PIN_9;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &g);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  g.Pin = GPIO_PIN_10;
  g.Mode = GPIO_MODE_OUTPUT_OD;        /* 开漏: 只能拉低, 放开即高阻 */
  g.Pull = GPIO_NOPULL;                /* 关键: 关掉内部上拉, 只看外部 */
  HAL_GPIO_Init(GPIOA, &g);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  delay_us(300);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);   /* 放开 */
  delay_us(300);
  uint8_t hi = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET);

  /* 两个脚都恢复成串口该有的样子 */
  g.Pin = GPIO_PIN_9;
  g.Mode = GPIO_MODE_AF_PP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_10;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);
  return hi;
}

static void bt_selftest(void) {
  /* 自检要发 AT 并等应答, 是阻塞的 (最长 1 秒) —— 统一走 diag_begin/end:
     先停车清状态, 做完恢复串口和计时基准, 保持停车 */
  diag_begin();
  bt_rx_stop();                 /* 先停中断接收, 免得改引脚/发 AT 时收到垃圾 */

  /* 1. 线检测: 模块到底有没有在驱动 PA10 */
  bt_line_drive = bt_line_check();

  /* 2. 扫一遍常见波特率发 AT。
        只认真正的 "OK" 才算找到 —— 否则一个杂字符(比如自己发出去的信号串回来)
        就会被误判成"模块在", 后面的波特率也就跟着错了。
        同时记录哪些波特率收到了"任意字符"(说明离正确值不远)和收到的原文,
        这样"完全没反应"和"波特率不对"就能区分开。 */
  static const uint32_t bauds[BT_SCAN_N] = {9600, 115200, 38400, 19200, 57600, 4800};
  uint32_t found = 0;
  bt_scan_hit = 0;
  bt_scan_ok_cnt = 0;
  bt_scan_raw[0] = 0;
  /* 扫两遍: 线检测刚改过 PA9/PA10 的配置, 第一遍的头一次尝试偶尔会被吃掉
     (实测 9600 那一次就丢过)。跑两遍保证不会漏掉任何一个波特率 ——
     9600 正是 HC-06 最常用的出厂值, 漏了它就会把好模块误判成坏的。 */
  for (int pass = 0; pass < 2 && !found; pass++) {
    for (int i = 0; i < BT_SCAN_N; i++) {
      if (!bt_at_cmd(bauds[i], "AT", bt_resp, sizeof(bt_resp), 250)) continue;
      if (bt_scan_raw[0] == 0) strncpy(bt_scan_raw, bt_resp, sizeof(bt_scan_raw) - 1);
      if (bt_scan_hit < BT_SCAN_N) bt_scan_hit_baud[bt_scan_hit] = bauds[i];
      if (bt_scan_hit < 255) bt_scan_hit++;
      if (strstr(bt_resp, "OK")) {
        bt_scan_ok_cnt++;
        if (!found) {
          found = bauds[i];
          strncpy(bt_scan_raw, bt_resp, sizeof(bt_scan_raw) - 1);
        }
      }
    }
  }

  if (found) {
    /* 模块实际用的波特率就是它, 记下来: 模式四(VOFA+)必须用同一个值,
       否则发出去的数据全是乱码。界面上的"K4波特"也跟着变。 */
    bt_baud = found;
    snprintf(bt_resp, sizeof(bt_resp), "OK %u 线%d", (unsigned)found, bt_line_drive);
  } else {
    bt_rx_level = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) ? 1 : 0;
    if (bt_scan_hit) snprintf(bt_resp, sizeof(bt_resp), "乱码 线%d", bt_line_drive);
    else             snprintf(bt_resp, sizeof(bt_resp), "无应答 线%d", bt_line_drive);
  }

  bt_show_until = HAL_GetTick() + 6000;
  diag_end();      /* 恢复串口/计时基准, 并保持停车 */
}

static uint32_t t_key = 0, t_ctrl = 0, t_ui = 0, t_vofa = 0;
static float    vdd_now = 0.0f;      /* 供电电压, 每 0.5 秒测一次 */
static uint32_t t_vdd = 0;

/* ================= 诊断操作前后的统一处理 =================
   蓝牙自检 / 模块复位 / 供电波形记录 都是阻塞操作, 期间主循环停摆。
   统一走这两个函数: 先停车并清掉运行状态 -> 做诊断 -> 恢复串口和计时基准 ->
   **保持停车**, 必须重新按 K5 才能再动。这样不会出现"诊断刚结束, 车突然继续走"。 */
static void diag_begin(void) {
  control_brake();
  running = false;
  finished = false;
  aborted = false;
  dbg_lock = true;                     /* 调试页要等测试键松开才恢复驱动 */
}

static void diag_end(void) {
  bt_set_baud(bt_baud);                /* 恢复用户设定的波特率 */
  bt_rx_start();                       /* 恢复串口中断接收 */
  /* 诊断期间主循环没跑, 这些时间戳要重新对齐,
     否则下一次 tick 的 dt / 超时会算成一大段 */
  uint32_t now = HAL_GetTick();
  t_key = now; t_ctrl = now; t_ui = now; t_vdd = now; t_vofa = now;
  t_sonar = now;
  rc_last_ms = now;                    /* 遥控重新计时, 免得一进模式七就判断联 */
  imu_update(0.01f);                   /* 刷新陀螺数据的新鲜度 */
}

/* ---------------- 小工具 ---------------- */
static float f_abs(float v) { return v < 0.0f ? -v : v; }

/* 显示用: 把连续累加的航向折回 0~360 度。
   内部必须保持连续 —— 计圈(4 个弯 + 回到起点)、转向(目标 = 起点 + 角度)
   都要用连续值, 折了反而算错; 只有"给人看的数字"才折回 0~360。 */
static int yaw_show(float y) {
  int n = (int)(y / 360.0f);              /* 截断取整, 负数也对 */
  float v = y - 360.0f * (float)n;
  if (v < 0.0f) v += 360.0f;
  if (v >= 360.0f) v -= 360.0f;
  return (int)(v + 0.5f);
}
static float f_clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* 三角函数: 用查表 + 线性插值, 不引入 math 库 (sinf/cosf 会多占 Flash,
   而航位推算只需要 0.1° 级的精度)。表是 0~90° 每 10° 一个点。 */
static float sin_deg(float deg) {
  static const float tab[10] = {
      0.0000f, 0.1736f, 0.3420f, 0.5000f, 0.6428f,
      0.7660f, 0.8660f, 0.9397f, 0.9848f, 1.0000f};
  float s = 1.0f;
  while (deg >  180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  if (deg < 0.0f) { deg = -deg; s = -1.0f; }
  if (deg > 90.0f) deg = 180.0f - deg;      /* sin(180-x) = sin(x) */
  int i = (int)(deg * 0.1f);
  if (i > 8) i = 8;
  float f = deg * 0.1f - (float)i;
  return s * (tab[i] + (tab[i + 1] - tab[i]) * f);
}
static float cos_deg(float deg) { return sin_deg(deg + 90.0f); }

/* 上次真正画上屏幕的四行内容。内容没变的行不重画 ——
   否则每帧都要通过 I2C 重发整屏(1024 字节, 400kHz 下约 24ms),
   那段时间控制循环完全停摆, 会明显影响 10ms 周期。 */
static char ui_last[4][40];
static bool ui_last_valid = false;
static bool ui_force = false;     /* 调 OLED_Clear/直接画过之后要强制重画一次 */

static void ui_show(const char *a, const char *b, const char *c, const char *d) {
  const char *lines[4] = {a, b, c, d};
  for (int i = 0; i < 4; i++) {
    if (ui_last_valid && !ui_force && strcmp(ui_last[i], lines[i]) == 0) continue;
    strncpy(ui_last[i], lines[i], sizeof(ui_last[0]) - 1);
    ui_last[i][sizeof(ui_last[0]) - 1] = 0;
    OLED_ClearLine((uint8_t)(i * 16));
    OLED_DrawString(0, (uint8_t)(i * 16), lines[i]);
  }
  ui_force = false;
  ui_last_valid = true;
}

/* 直接操作过屏幕(比如 OLED_Clear)之后调用, 让下一帧全量重画 */
static void ui_invalidate(void) { ui_force = true; }

/* 一位小数的定点文本, 不用浮点 printf */
static void f1(char *dst, float v) {
  int neg = 0;
  if (v < 0) { neg = 1; v = -v; }
  int x = (int)(v * 10.0f + 0.5f);
  if (neg) snprintf(dst, 12, "-%d.%d", x / 10, x % 10);
  else     snprintf(dst, 12, "%d.%d", x / 10, x % 10);
}

/* ================= 各模式控制 ================= */

/* 模式一: 方向保持与自动纠偏 */
static void ctrl_heading(float dt) {
  if (!running) return;   /* 已经停了: 不要再动电机, 保持上次的刹车/滑行状态 */
  float rot = f_clamp(pid_step(&pid_head, yaw_ref - imu_yaw(), dt), -90.0f, 90.0f);
  control_set_cmd((float)p_cruise, rot);
}

/* 模式二: 转到指定角度 */
static float turn_command_angle(void) {
  float angle = (float)p_turn_angle;
  return p_turn_dir > 0 ? angle * TURN_RIGHT_RAW_RATIO : angle * TURN_LEFT_RAW_RATIO;
}

static float turn_display_angle(void) {
  float angle = f_abs(imu_yaw() - yaw_ref);
  return p_turn_dir > 0 ? angle / TURN_RIGHT_RAW_RATIO : angle / TURN_LEFT_RAW_RATIO;
}

static void turn_finish(uint8_t fault) {
  control_brake();
  turn_fault = fault;
  running = false;
  finished = true;
  aborted = (fault != TURN_ERR_NONE);
  run_end_ms = HAL_GetTick();
}

static void ctrl_turn(float dt) {
  if (!running) return;
  uint32_t now = HAL_GetTick();
  uint32_t limit_ms = (uint32_t)p_turn_angle * 1000u / 30u + 8000u;
  float progress = (imu_yaw() - yaw_ref) * (float)p_turn_dir;
  float err = yaw_ref + (float)p_turn_dir * turn_command_angle() - imu_yaw();
  float ae = f_abs(err), rate = f_abs(imu_gyro_z());
  if (now - run_start_ms > limit_ms) { turn_finish(TURN_ERR_TIMEOUT); return; }
  if (progress < -15.0f) { turn_finish(TURN_ERR_DIRECTION); return; }

  if (turn_phase == TURN_BRAKING) {
    /* 始终保持硬刹车, 不能用零目标速度代替, 也不能被速度 PI 再次覆盖。 */
    control_brake();
    if (rate < TURN_STOP_DPS) {
      if (turn_stable_ms == 0) turn_stable_ms = now;
      if (now - turn_stable_ms >= TURN_SETTLE_MS) {
        if (ae <= TURN_TOL_DEG) { turn_finish(TURN_ERR_NONE); return; }
        if (turn_trims >= TURN_MAX_TRIMS) { turn_finish(TURN_ERR_SETTLE); return; }
        ++turn_trims;
        turn_phase = TURN_DRIVE;
        turn_rate_cmd = 0.0f;
        turn_last_err = err;
        turn_stall_l = turn_stall_r = 0.0f;
        return; /* 下一周期从零积分、低速开始微调 */
      }
    } else turn_stable_ms = 0;
    if (now - turn_brake_ms > TURN_BRAKE_MAX_MS) turn_finish(TURN_ERR_SETTLE);
    return;
  }

  /* 进入容差、跨过目标或到达预测刹车点, 都锁定到刹车阶段。
     预测距离被限制在 8 度内, 不会在远处反复停车。 */
  float stop_deg = f_clamp(rate * rate / (2.0f * TURN_BRAKE_DPS2)
                          + rate * 0.03f, TURN_BRAKE_DEG, 8.0f);
  if (ae <= TURN_BRAKE_DEG || err * turn_last_err <= 0.0f ||
      (rate > TURN_STOP_DPS && ae <= stop_deg)) {
    turn_phase = TURN_BRAKING;
    turn_brake_ms = now;
    turn_stable_ms = 0;
    turn_rate_cmd = 0.0f;
    control_brake();
    return;
  }
  turn_last_err = err;
  float vmax = turn_trims ? TURN_TRIM_DPS : TURN_MAX_DPS;
  float wanted = f_clamp(ae * 2.0f, TURN_MIN_DPS, vmax);
  /* 起步渐加速; 接近目标时允许立即降低目标速度。 */
  turn_rate_cmd += TURN_ACCEL_DPS2 * dt;
  if (turn_rate_cmd > wanted) turn_rate_cmd = wanted;

  /* 任一轮持续无反馈就停止, 防止一轮不转仍绕它继续转圈。 */
  if (turn_rate_cmd >= 15.0f) {
    turn_stall_l = f_abs(control_speed_l()) < 0.7f ? turn_stall_l + dt : 0.0f;
    turn_stall_r = f_abs(control_speed_r()) < 0.7f ? turn_stall_r + dt : 0.0f;
    if (turn_stall_l * 1000.0f >= TURN_STALL_MS) { turn_finish(TURN_ERR_LEFT); return; }
    if (turn_stall_r * 1000.0f >= TURN_STALL_MS) { turn_finish(TURN_ERR_RIGHT); return; }
  } else turn_stall_l = turn_stall_r = 0.0f;
  control_set_spin(err > 0.0f ? turn_rate_cmd : -turn_rate_cmd);
}

/* 模式三: 走完指定里程停车 */
static void ctrl_dist(float dt) {
  if (!running) return;   /* 已经停了: 不要再动电机, 保持上次的刹车/滑行状态 */
  /* 编码器保护: 跑了 1.5 秒里程还不到 1cm, 说明编码器没接/没数 ——
     再跑下去永远到不了终点 (以前会一路开到撞墙) */
  if ((HAL_GetTick() - run_start_ms) > 1500 && control_dist_cm() < 1.0f) {
    control_brake();
    running = false; finished = true; aborted = true;
    enc_fault = true;
    return;
  }
  float remain = (float)p_dist_cm - control_dist_cm();
  if (remain <= DIST_TOL_CM + DIST_STOP_ADVANCE_CM) {
    control_brake();
    running = false; finished = true; aborted = false;
    return;
  }
  float v = (float)p_cruise;
  float v2 = 2.0f * DIST_DECEL * remain;
  float x = v2;
  for (int i = 0; i < 8; i++) x = 0.5f * (x + v2 / (x + 1e-6f));   /* 开方, 免 math 库 */
  if (x < v) v = x;
  if (v < 10.0f) v = 10.0f;
  float rot = f_clamp(pid_step(&pid_head, yaw_ref - imu_yaw(), dt), -80.0f, 80.0f);
  control_set_cmd(v, rot);
}

/* 模式四: 定速行驶, 速度曲线由主循环发给 VOFA+ */
static void ctrl_vofa(float dt) {
  if (!running) return;   /* 已经停了: 不要再动电机, 保持上次的刹车/滑行状态 */
  float rot = f_clamp(pid_step(&pid_head, yaw_ref - imu_yaw(), dt), -80.0f, 80.0f);
  control_set_cmd((float)p_speed, rot);
}

/* ================= 直角弯"进入角"测量 =================
   车歪着进直角弯时, 横线不是 8 路一起压黑, 而是从一侧依次扫过来。
   每路探头"白->黑"那一刻记下里程, 探头横向位置 x 与触发里程 s 满足
       s = a + x * tan(φ)
   斜率 tan(φ) 就是车头相对"横线法线"的偏角 (正 = 车头偏右)。
   弯前基准航向 = 扫线时的车头航向 - φ, 出弯目标再 ±90°。
   测量不合格(点太少/残差大/扫线时车在转)就退回弯前直线段的航向平均。

   两个关键点:
   1) 用里程不用速度×时间 —— 扫线期间车正在刹车, 速度不是常数。
   2) 边沿扫描放 1kHz 中断里 —— 主循环 10ms 一拍就是 2mm (20cm/s),
      而 8 路扫过同一条横线的里程差只有几 mm, 主循环根本分辨不出来。
   成对边沿(白->黑 + 黑->白)取中点用: 中点落在线的中心线上,
   各探头触发阈值不一致的系统偏差被抵消。 */
#define LINE_EDGE_WIN_BACK_CM 20.0f   /* 只认弯前这么远以内的边沿记录 */
#define LINE_EDGE_MIN_PTS     3       /* 至少几路有效边沿 */
#define LINE_EDGE_MIN_SPAN_CM 1.8f   /* 有效点横向跨度下限 */
#define LINE_EDGE_RMS_CM      0.20f    /* 拟合残差 RMS 上限 */
#define LINE_EDGE_RMAX_CM     0.30f    /* 单点残差上限: 混进假边沿就整次作废 */
#define LINE_EDGE_MIN_WIDTH_CM 1.0f   /* 成对边沿最小间隔: 真穿越≈线宽(~2cm),
                                         车停稳时探头在线边抖出的假边沿只有 0.x mm */
#define LINE_EDGE_MAX_WIDTH_CM 4.5f   /* 成对边沿最大间隔 (线宽+余量) */
#define LINE_EDGE_YAW_SPAN    8.0f    /* 扫线期间车头转动上限 */
#define LINE_EDGE_BASE_GAP    15.0f   /* 实测基准与直线均值差超过它就不可信 */

static volatile float edge_dist_now = 0.0f;   /* ISR 自己积的里程 */
static volatile float edge_dist_off = 0.0f;   /* 对齐 control_dist_cm() 的偏移 */
static uint16_t edge_cnt_l = 0, edge_cnt_r = 0;
static volatile float edge_lead[8], edge_trail[8];        /* 当前这对边沿(可能还在抖) */
static volatile float edge_lead_yaw[8], edge_trail_yaw[8];
static volatile float edge_vlead[8], edge_vtrail[8];      /* 最近一次"可信"穿越 */
static volatile float edge_vlead_yaw[8], edge_vtrail_yaw[8];
static volatile uint8_t edge_have_valid = 0;              /* 哪几路有可信穿越 */
static uint8_t edge_open[8];                              /* 该路正记着一对边沿 */
static uint8_t edge_prev_bits = 0;
static uint8_t edge_white_run[8];

/* Spatial observations establish square axes. Two outer edge pairs
   constrain corner POSITION, without an unconstrained angle fit. */
static float est_e = 0.0f;         /* 连续线偏差估计 (单位同 e) */
static volatile uint16_t gyro_xfix_cnt = 0;  /* 驼螺被带偏而修正过的次数 (SWD 可读) */

/* ---- 驼螺可信性交叉检查 ----
   两轮里程几乎相等(没有差速)时, 车不可能在转。若驼螺却报出明显转角,
   说明电机振动/干扰把它带偏了 —— 这时照它去纠偏反而会把车拧歪
   (实测: 28cm 空白段两轮里程差 0cm, 航向却转了 14.6°。而我加的阻尼项
   吃的就是这个假转速 -> 直接变成真转向。)
   做法: 以轮子为准把假转角扣掉, 并把假转速置零。 */
static volatile float line_corner_meas_deg = 0.0f;  /* 最近一次测到的进入角 (SWD 可读) */
static volatile uint8_t line_corner_meas_ok = 0;
static volatile uint16_t line_corner_meas_try = 0;   /* 统计: 试了几次 */
static volatile uint16_t line_corner_meas_good = 0;  /* 统计: 成了几次 */
static float line_corner_base_yaw = 0.0f;            /* 扣掉进入角后的弯前基准航向 */
/* 诊断快照: 拟合用的点和判废原因 (跑完用 SWD 读, 记录被后续驾驶覆盖也不怕) */
static volatile float dbg_fit_x[8], dbg_fit_s[8];
static volatile uint8_t dbg_fit_n = 0, dbg_fit_reason = 0;
static volatile float dbg_lead[8], dbg_trail[8];
static volatile uint8_t dbg_have_lead = 0, dbg_have_trail = 0;

/* 1kHz 边沿扫描: 里程用编码器原始计数自己积分 ——
   硬件计数不会因为中断延迟丢步, 比主循环 10ms 采样细 10 倍 */
static void line_edge_isr(void) {
  uint16_t cl = motor_raw_cnt(MOT_LEFT);
  uint16_t cr = motor_raw_cnt(MOT_RIGHT);
  /* 编码器方向和轮速计算保持一致 (ENCODER_A/B_REVERSE, 见 board.h) */
  int16_t dl_raw = (int16_t)(cl - edge_cnt_l);
  int16_t dr_raw = (int16_t)(cr - edge_cnt_r);
#if ENCODER_A_REVERSE
  dl_raw = -dl_raw;
#endif
#if ENCODER_B_REVERSE
  dr_raw = -dr_raw;
#endif
  float d = 0.5f * ((float)dl_raw / ENCODER_CM_L + (float)dr_raw / ENCODER_CM_R);
  edge_cnt_l = cl; edge_cnt_r = cr;
  edge_dist_now += d;
  float dist = edge_dist_now + edge_dist_off;

  uint8_t b = line_read_raw();
  float y = imu_yaw();
  for (int i = 0; i < 8; i++) {
    uint8_t m = (uint8_t)(1u << i);
    if (b & m) {
      if (!(edge_prev_bits & m) && edge_white_run[i] >= 3u && !edge_open[i]) {
        edge_lead[i] = dist;                     /* 白->黑: 开始记一对 */
        edge_lead_yaw[i] = y;
        edge_open[i] = 1;
      }
      edge_white_run[i] = 0;
    } else {
      if ((edge_prev_bits & m) && edge_open[i]) {
        float wcm = dist - edge_lead[i];
        edge_trail[i] = dist;                    /* 黑->白: 收尾 */
        edge_trail_yaw[i] = y;
        edge_open[i] = 0;
        /* 只有宽度像"压过一条线"的才算数 —— 车快停稳时探头停在线的
           边缘会抖出一串 0.x mm 的假边沿, 那种绝不能覆盖真穿越 */
        if (wcm >= LINE_EDGE_MIN_WIDTH_CM && wcm <= LINE_EDGE_MAX_WIDTH_CM) {
          edge_vlead[i] = edge_lead[i];
          edge_vtrail[i] = edge_trail[i];
          edge_vlead_yaw[i] = edge_lead_yaw[i];
          edge_vtrail_yaw[i] = edge_trail_yaw[i];
          edge_have_valid |= m;
        }
      }
      if (edge_white_run[i] < 250u) edge_white_run[i]++;
    }
  }
  edge_prev_bits = b;
}

void TIM3_IRQHandler(void) {
  if (TIM3->SR & TIM_SR_UIF) {
    TIM3->SR = (uint16_t)~TIM_SR_UIF;
    line_edge_isr();
  }
}

static void line_edge_timer_init(void) {
  __HAL_RCC_TIM3_CLK_ENABLE();
  TIM3->PSC = 72 - 1;          /* 72MHz -> 1MHz */
  TIM3->ARR = 1000 - 1;        /* 1kHz */
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = 0;
  TIM3->DIER = TIM_DIER_UIE;
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  TIM3->CR1 = TIM_CR1_CEN;
}

/* 主循环每拍对齐一次里程基准 (两边同源, 只是起点要对齐) */
static void line_edge_sync(float dist_cm) { edge_dist_off = dist_cm - edge_dist_now; }


/* 弯道停稳后算一次: 用窗口内的边沿拟合进入角, 通过检查才采信 */
static void line_corner_fit(void) {
  float centers[8], headings[8];
  uint8_t valid;
  line_corner_meas_try++;
  TIM3->DIER &= (uint16_t)~TIM_DIER_UIE;
  valid = edge_have_valid;
  for (int i = 0; i < 8; ++i) {
    centers[i] = .5f * (edge_vlead[i] + edge_vtrail[i]);
    headings[i] = edge_vlead_yaw[i] + .5f * ang_diff(edge_vtrail_yaw[i], edge_vlead_yaw[i]);
    dbg_lead[i] = edge_vlead[i]; dbg_trail[i] = edge_vtrail[i];
  }
  TIM3->DIER |= TIM_DIER_UIE;
  line_corner_meas_ok = line_map_corner_edges(&track_map, line_corner_dir, valid, centers, headings);
  if (line_corner_meas_ok) ++line_corner_meas_good;
  dbg_fit_reason = line_corner_meas_ok ? 0 : 1;
  line_corner_base_yaw = track_map.incoming_heading;
  line_corner_meas_deg = ang_diff(line_corner_yaw, track_map.incoming_heading);
}

static void line_edge_reset(void) {
  TIM3->DIER &= (uint16_t)~TIM_DIER_UIE;
  edge_cnt_l = motor_raw_cnt(MOT_LEFT); edge_cnt_r = motor_raw_cnt(MOT_RIGHT);
  edge_dist_now = control_dist_cm(); edge_dist_off = 0;
  edge_have_valid = 0; edge_prev_bits = line_read_raw();
  memset(edge_open, 0, sizeof(edge_open));
  memset(edge_white_run, 0, sizeof(edge_white_run));
  TIM3->DIER |= TIM_DIER_UIE;
}

static void line_map_diagnostics(void) {
  line_map_version = 0x20261009u;
  line_map_confidence = track_map.confidence;
  line_map_heading_deg = track_map.heading;
  line_map_cross_cm = track_map.position_valid ? line_map_cross_error(&track_map) : 0;
}

static void line_alignment_begin(uint32_t now) {
  line_settle_active = true;
  line_settle_dist0 = control_dist_cm(); line_settle_t0 = now;
  line_align_stable_ms = 0; line_exit_seen = 0;
  track_map.integral = track_map.rot_cmd = 0;
  line_hold_yaw = line_straight_yaw = track_map.heading;
  line_straight_valid = true;
  line_corners_done++;
  line_seg_start_cm = control_dist_cm();
  pid_reset(&pid_head); line_prev_valid = false; line_derivative = 0;
}


static void line_begin_corner(int dir) {
  line_corner_active = true;
  line_last_corner_dir = (int8_t)dir;
  line_end_pending = false;
  ev_log(2);                                 /* 事件2: 判定进弯 */
  line_corner_advanced = false;
  line_corner_adv0 = control_dist_cm();
  line_corner_stopped = false;
  line_corner_phase = 0;
  line_corner_trims = 0;
  line_corner_t0 = HAL_GetTick();
  line_gap_active = false;
  line_corner_dir = dir;
  line_corner_yaw = imu_yaw();
  line_map_begin_corner(&track_map, dir, line_straight_valid ? line_straight_yaw : imu_yaw());
  line_corner_target = track_map.heading;
  line_map_diagnostics();
  line_center_ms = lost_ms = 0.0f;
  line_prev_valid = false;
  line_derivative = 0.0f;
  line_hist[0] = line_hist[1] = line_hist[2] = 0.0f;
  line_hist_n = 0;
  line_settle_active = false;
  if (!line_straight_valid) {           /* 没积累过直线均值: 用入弯航向兜底 */
    line_straight_yaw = imu_yaw();
    line_straight_valid = true;
  }
  control_brake();
}

/* 加速瞬态航向保持: 两轮加减速不对称, 提速那一下会把车甩出一个航向偏差。
   空白段里那次加速(出弯直行 18 -> 巡航)根本没有线可参照, 甩出去就一路带下去,
   所以循线/出弯直行/空白段三个分支都要吃这个阻尼。
   返回额外陀螺阻尼增益(只在速度快速变化时非零, 稳态不拖慢环路)。 */
static float acc_yaw_hold(float dt) {
  static float lin_prev = 0.0f;
  float a = (control_lin() - lin_prev) / (dt > 0.001f ? dt : 0.001f);
  lin_prev = control_lin();
  return (a > 60.0f || a < -60.0f) ? LINE_KGYRO_ACC : 0.0f;
}

/* 仅完整单侧四路压黑且另一侧全白持续40ms, 才触发直角找线。 */
static void line_follow(float dt, float cruise) {
  line_update();
  uint8_t bits = line_bits();
  int count = line_count();
  float e = line_error();
  line_map_update(&track_map, control_dist_cm(), imu_yaw());
  line_map_diagnostics();
  uint32_t now = HAL_GetTick();

  /* 弯口现场: 出现大面积压黑(>=4路)就记一条, 15cm 限流一次。
     没认出来的弯, 读日志就能看到那一刻探头到底看到了什么图案 */
  {
    if (count >= 4 && (control_dist_cm() - line_wide_cm) > 15.0f) {
      int lf2 = 0, rg2 = 0;
      for (int i = 0; i < 8; i++) if (bits & (1u << i)) { if (i < 4) lf2++; else rg2++; }
      line_wide_side = (lf2 > rg2) ? -1 : ((rg2 > lf2) ? 1 : 0);  /* 黑多的那侧=弯的方向 */
      line_wide_cm = control_dist_cm();
      ev_log(13);                            /* 事件13: 大面积压黑 */
    }
  }

  if (line_settle_active) {
    lost_ms = 0;
    /* Validate a short run of contiguous thin-line samples, then update only
       the outgoing line's POSITION. Its orthogonal heading remains locked. */
    uint8_t shifted = bits;
    while (shifted && !(shifted & 1u)) shifted >>= 1;
    bool thin = count > 0 && count <= 3 && !(shifted & (shifted + 1u));
    if (thin) { if (line_exit_seen < 20) ++line_exit_seen; }
    else line_exit_seen = 0;
    if (line_exit_seen >= 3) line_map_observe_exit(&track_map, bits);
    float heading_error = f_abs(ang_diff(track_map.heading, imu_yaw()));
    float rot = line_map_steer(&track_map, imu_yaw(), imu_yaw_rate(), dt, true);
    float v = heading_error > 5 ? 8.0f : 18.0f;
    if (v > cruise) v = cruise;
    control_set_cmd(v, rot);
    bool ready = line_map_aligned(&track_map, imu_yaw(), imu_yaw_rate());
    if (ready) line_align_stable_ms += dt * 1000;
    else line_align_stable_ms = 0;
    float travelled = control_dist_cm() - line_settle_dist0;
    bool limit = travelled > 14.0f || now - line_settle_t0 > 1600u;
    if ((line_align_stable_ms >= 100 && travelled >= 1.5f) || limit) {
      /* Large angular/position disagreement is not permission to accelerate
         blindly through 60cm of unobservable track. */
      if (limit && (heading_error > 5 ||
          (track_map.position_valid && f_abs(line_map_cross_error(&track_map)) > 2.5f))) {
        line_map_align_limited = 2;
        control_brake_fast(); running = false; finished = aborted = true;
        run_end_ms = now; ev_log(16); return;
      }
      line_map_align_limited = limit ? 1 : 0;
      ev_log(limit ? 6 : 5);
      line_settle_active = false; line_pull = false;
      line_map_finish_corner(&track_map);
      line_straight_yaw = line_hold_yaw = track_map.heading;
      pid_reset(&pid_head); line_prev_valid = false; line_derivative = 0;
      /* Keep a continuous gap length from the turn exit, including alignment. */
      if (line_lost()) {
        line_gap_active = true; line_gap_start_cm = line_settle_dist0;
        line_end_pending = false; line_map_lock_gap(&track_map);
      }
    }
    line_map_diagnostics();
    return;
  }

  /* Corner: measured advance, brake, symmetric rotation to the map
     heading, settle verification, then low-speed position alignment. */
  if (line_corner_active) {
    /* 弯道有自己的超时保护(swept 上限 + 刹停 700ms 上限), 不能吃
       lost_ms 计时器 —— 它累加超过 LOST_TIMEOUT_MS 会被"找线失败"判据
       当成跑飞, 把整轮直接停掉 (转完弯不动就是这个) */
    lost_ms = 0.0f;
    float swept = (imu_yaw() - line_corner_yaw) * (float)line_corner_dir;

    /* ---- 第 0 步: 前进补偿"探头到轮轴"的距离 ---- */
    if (!line_corner_advanced) {
      float adv_cm = CORNER_ADVANCE_CM;
      float travelled = control_dist_cm() - line_corner_adv0;
      if (travelled > .5f && edge_have_valid) line_corner_fit();
      float remaining = track_map.position_valid ? line_map_advance_remaining(&track_map) - .8f
                                                : adv_cm - travelled;
      if (now - line_corner_t0 > 1200u) {
        control_brake_fast(); running = false; finished = aborted = true;
        run_end_ms = now; return;
      }
      if (remaining > 0) {
        float rot = f_clamp(4.5f * ang_diff(track_map.incoming_heading, imu_yaw())
                            - 1.2f * imu_yaw_rate(), -25, 25);
        float v = f_clamp(remaining * 8, 8, CORNER_ADVANCE_CM_S);
        control_set_cmd(v, rot);
        return;
      }
      line_corner_advanced = true;
      line_corner_t0 = now;
      control_brake();
      return;
    }

    /* ---- 第 1 步: 刹停 (等实测速度真的降下来) ---- */
    if (!line_corner_stopped) {
      control_brake();
      /* 左右轮分别低速且持续 120ms 才算停稳 (不能只看平均值) */
      if (now - line_corner_t0 > 700u ||
          (now - line_corner_t0 > 120u &&
           f_abs(control_speed_l()) < 6.0f && f_abs(control_speed_r()) < 6.0f)) {
        line_corner_stopped = true;
        ev_log(3);                           /* 事件3: 停稳, 开始原地转 */
        line_corner_fit();        /* 横线已扫完: 估"歪着进弯"的角度 */
        line_corner_phase = 0;
        line_corner_t0 = now;
      }
      return;
    }

    /* The target comes from fitted square axes, frozen when this corner
       was confirmed. It never follows a transient car heading. */
    float yaw_target = line_corner_target;
    float yaw_need = ang_diff(yaw_target, imu_yaw()) * (float)line_corner_dir;
    if (line_corner_phase == 1) {
      control_brake_fast();
      if (now - line_corner_t0 < 120u) return;
      if (f_abs(imu_yaw_rate()) > 8 || f_abs(control_speed_l()) > 3 || f_abs(control_speed_r()) > 3) {
        if (now - line_corner_t0 < 700u) return;
        running = false; finished = aborted = true; run_end_ms = now; return;
      }
      if (f_abs(yaw_need) > 1.5f) {
        if (line_corner_trims >= 2 || f_abs(yaw_need) > 12) {
          running = false; finished = aborted = true; run_end_ms = now; return;
        }
        ++line_corner_trims; line_corner_phase = 0; line_corner_t0 = now;
        return;
      }
      line_corner_active = false; ev_log(4);
      line_alignment_begin(now);
      return;
    }

    /* ---- 第 2 步: 原地对称转 (比例调速 + 预测刹车; 过冲会反向补) ---- */
    float rate = f_abs(imu_gyro_z());
    /* 预测刹车滑过角: 原 /900 估得太大, 110°/s 时提前 17.8° 就刹, 车总差
       3~5° 停住, 然后补转循环"转-刹-转"地抽。改保守(/2500): 宁可略过冲,
       再用慢速微调平滑修回 */
    float stop_deg = f_clamp(rate * rate / 1500.0f + rate * 0.03f + LINE_PIVOT_LEAD_TRIM, 0, 15);
    if (line_corner_trims) stop_deg = f_clamp(rate * rate / 1500.0f + rate * .02f, 0, 1.2f);
    if (f_abs(yaw_need) <= stop_deg + 0.4f) {   /* 取绝对值: 过冲(负)也能进校验 */
      control_brake();
      line_corner_phase = 1;
      line_corner_t0 = now;
      return;
    }
    if (swept < -25.0f || swept > 130.0f) {
      control_brake_fast();
      running = false; finished = aborted = true; run_end_ms = now;
      return;
    }
    if (now - line_corner_t0 > 2000u) {    /* 保护: 旋转卡住(轮子推不动) */
      ev_log(15);                          /* 事件15: 原地转超时 */
      control_brake();
      if (f_abs(yaw_need) > 20.0f) {       /* 还差 20° 以上 = 真卡死: 停车,
                                              不能没转到位就冲进空白区 */
        running = false; finished = true; aborted = true;
        run_end_ms = HAL_GetTick();
        return;
      }
      line_corner_phase = 1;               /* 只差一点: 放行交给循线收尾 */
      line_corner_t0 = now;
      return;
    }
    /* 带符号: yaw_need 为正朝目标转, 为负(过冲)反向转回来。
       比例调速增益 3.5 (原 1.6, 90° 要转 2.5s): 顶速 LINE_PIVOT_DPS,
       末段保底 10°/s。原来小误差段是 1.5 倍/上限 15/保底 6 ——
       光"最后 10°"就要 1s, 是转弯慢的主因。收尾仍是比例律(越近越慢)
       +预测刹车, 不会像固定 25°/s 微调那样过冲。 */
    float peak = line_corner_trims ? 18.0f : LINE_PIVOT_DPS;
    float dps = f_clamp(yaw_need * 3.5f, -peak, peak);
    float minimum = line_corner_trims ? 5.0f : 10.0f;
    if (f_abs(dps) < minimum) dps = dps >= 0 ? minimum : -minimum;
    control_set_spin_ex((float)line_corner_dir * dps, SPIN_GAIN_L);
    return;
  }

  if (!line_lost()) {
    if (line_gap_active) {
      ev_log(9); track_map.frozen = false;
      track_map.count = track_map.wr = 0;
      line_prev_valid = false; line_derivative = 0;
    }
    line_gap_active = false;
    line_end_pending = false;
    lost_ms = 0.0f;
    /* 武装弯道检测: 探头下是"正常细线"(路数<=3)连续 4 拍就武装, 不管偏在哪侧。
       原来要求"线必须在正中间", 空白段回来时线偏在一侧(只占左边两路)就
       永远武装不上 -> 那个弯被直接开过去(实测第4个弯就是这么丢的) */
    if (count <= 3) {
      if (line_seen_cnt < 250) ++line_seen_cnt;
    } else {
      line_seen_cnt = 0;
    }
    if (line_seen_cnt >= 4) line_track_seen = true;
    if (count <= 3) {
      if (line_map_observe(&track_map, bits) || track_map.axis_valid) {
        line_straight_yaw = track_map.heading; line_straight_valid = true;
      } else if ((bits & 0x18u) && f_abs(e) <= .35f && f_abs(imu_yaw_rate()) < LINE_STRAIGHT_MAX_RATE) {
        if (!line_straight_valid) line_straight_yaw = imu_yaw();
        else line_straight_yaw += .03f * ang_diff(imu_yaw(), line_straight_yaw);
        line_straight_valid = true;
      }
    }
    if (f_abs(e) > f_abs(lost_err) || now - lost_err_ms > 500u) {
      lost_err = e; lost_err_ms = now;
    }
    /* 直角弯穿越捕捉: 垂直线扫过探头时黑块从入口边扫到出口边,
       出口边在哪侧车就往哪转 —— 这个信号不受进弯横向偏移影响
       (入口偏侧会把方向判反, 实测 +0.57 被判成右弯)。 */
    /* 直角弯检测: 连续 3 拍 (>=30ms) >=4 路压黑, 且黑集中在单侧
       (弯臂沿探头扫过), 且上一拍有线 (排除空白段重新上线)。
       单拍抖动 / 斜穿线不触发, 防止直线上误转 90°。 */
    {
      int lft = 0, rgt = 0;
      for (int i = 0; i < 8; i++) {
        if (bits & (1u << i)) { if (i < 4) lft++; else rgt++; }
      }
      /* 单边全黑(另一侧最多 2 颗杂散)才算弯; 双边黑 = 十字/横线, 不转 */
      bool one_side = (lft >= 4 && rgt <= 2) || (rgt >= 4 && lft <= 2);
      bool ok = line_track_seen && line_prev_count >= 1 && count >= 4 && one_side;
      line_junc_hist = (uint8_t)((line_junc_hist << 1) | (ok ? 1u : 0u));
      if (line_junc_hist == 0) line_junc_side = 0;   /* 窗口空了: 释放方向, 不锁死 */
      if (ok) {
        int8_t sd = (lft >= rgt) ? -1 : 1;
        if (line_junc_side == 0) line_junc_side = sd;
        else if (line_junc_side != sd) line_junc_hist &= (uint8_t)~1u;  /* 方向不一致: 这拍不算 */
      }
      {
        int cnt = 0;
        for (int i = 0; i < 4; i++) if (line_junc_hist & (1u << i)) cnt++;
        /* 最近 4 拍里 3 拍成立就认弯。原来要求"连续 3 拍", 但弯口的线
           常常断续(丢一拍又回来): 一断计数就清零, 永远凑不满, 弯被开过去
           (实测第 4 个弯)。方向必须一致, 左/右交替不算, 防抖动误判。 */
        if (cnt >= 3 && line_junc_side != 0) {
          int dir = (int)line_junc_side;
          line_junc_hist = 0; line_junc_side = 0;
          line_cross_lft = lft;         /* 记进黑匣子, 出错可查 */
          line_cross_rgt = rgt;
          line_cross_dir = dir;
          line_begin_corner(dir);
          return;
        }
      }
    }

    /* ---- 连续偏差估计: 治"输入是离散的" ----
       8 路探头给出的是量化位置(台阶 = 11mm 间距 -> e 台阶 0.143), 直接用它
       做闭环, 环路增益一高就抖。这里把两次压线之间的横移推算出来:
       车头相对线方向偏 psi, 车往前走, 线在探头下就以 v·sin(psi) 横移 ——
       这个量是连续的(陀螺+航向给的), 于是台阶中间也有估计值。
       量化测量只用来把它拉回来(KEST), 所以不会漂。 */
    /* 线方向还没建立(line_straight_valid=0)时, line_straight_yaw 是 0 或旧值:
       开机时人把车搬上赛道会把它转过几十度, 那时 psi 是个大角度 ->
       航向项顶到打满, 车一启动就朝一边猛转。所以没建立方向之前两项都不用。 */
    float psi_deg = line_straight_valid ? ang_diff(imu_yaw(), line_straight_yaw) : 0.0f;
    if (psi_deg >  30.0f) psi_deg =  30.0f;
    if (psi_deg < -30.0f) psi_deg = -30.0f;
    {
      float psi_rad = psi_deg * 0.01745f;
      est_e += -(control_lin() * psi_rad / LINE_HALF_SPAN_CM) * dt;
      est_e += LINE_KEST * (e - est_e);
      if (est_e >  1.0f) est_e =  1.0f;
      if (est_e < -1.0f) est_e = -1.0f;
    }
    if (!line_prev_valid) {
      line_e_lpf = e; est_e = e;
      if (mode == MODE_LINE) mode5_rot_cmd = mode5_acc_damping = 0.0f;
    }
    line_e_lpf += 0.35f * (e - line_e_lpf);
    float e_steer = est_e;
    if (e_steer < LINE_DEADZONE && e_steer > -LINE_DEADZONE) e_steer = 0.0f;
    float d = line_prev_valid && dt > 0.0001f ? (e_steer - line_prev_e) / dt : 0.0f;
    line_prev_e = e_steer; line_prev_valid = true;
    line_derivative += 0.25f * (d - line_derivative);
    /* 转向 = 位置比例 + 微分 + 航向误差 + 陀螺阻尼。
       航向误差项 psi 是主阻尼: 它不反"正在转"这个动作, 只反"车头歪了"这个状态,
       所以压摆的同时不牺牲环路带宽(陀螺项会, 2ζω = KPSI/(1+KGYRO))。
       陀螺项留一点, 压突发扰动(轮子打滑/被推)。 */
    /* 加速瞬态航向保持: 两轮加减速不对称会让车在提速那一下甩出去,
       而这段常常正好在"进空白段之前"——甩出去就没人纠正了。
       检测到速度在快速变化时额外加陀螺阻尼, 把这一甩压掉;
       速度平稳时不生效(不拖慢循线环)。 */
    float kacc = acc_yaw_hold(dt);
    bool soft_center = (mode == MODE_LINE);
    if (soft_center) {
      float step_dt = f_clamp(dt, 0.0f, 0.05f);
      float alpha = step_dt / (MODE5_ACC_FILTER_S + step_dt);
      mode5_acc_damping += alpha * (kacc - mode5_acc_damping);
      kacc = mode5_acc_damping;
    }
    float kp = soft_center ? MODE5_LINE_KP : LINE_KP;
    float kd = soft_center ? MODE5_LINE_KD : LINE_KD;
    float kpsi = soft_center ? MODE5_LINE_KPSI : LINE_KPSI;
    float psi_term = -kpsi * psi_deg;
    /* 航向项自己最多给 60°/s, 不许它把转向打满。
       (上一版这个注释把负方向的 if 吞进注释里了 -> 正方向限 60、负方向不限,
        左右纠偏强度不对称, 是"总是偏一边"的一个来源) */
    if (psi_term >  60.0f) psi_term =  60.0f;
    if (psi_term < -60.0f) psi_term = -60.0f;
    float rot = f_clamp(kp * e_steer + kd * line_derivative
                        + psi_term
                        - (LINE_KGYRO + kacc) * imu_yaw_rate(), -150.0f, 150.0f);
    /* 线在探头边缘时(大偏差+路数少)打满方向会把线甩出探头 -> 丢线 ->
       进入找线大摇摆。收着打 + 保持前进, 让线留在探头里慢慢收回来。 */
    if (count <= 2 && f_abs(e) > 0.4f) {
      /* 原 50 太住: 线在边缘时回中几乎不动 (用户实测);
         现在有驼螺阻尼项, 放到 90 也不会像当年那样甩出探头 */
      float edge_limit = soft_center ? MODE5_EDGE_LIMIT : 90.0f;
      if (rot >  edge_limit) rot =  edge_limit;
      if (rot < -edge_limit) rot = -edge_limit;
    }
    if (soft_center) {
      rot = f_clamp(rot, -MODE5_ROT_LIMIT, MODE5_ROT_LIMIT);
      float step = MODE5_ROT_SLEW * f_clamp(dt, 0.0f, 0.05f);
      mode5_rot_cmd += f_clamp(rot - mode5_rot_cmd, -step, step);
      rot = mode5_rot_cmd;
    }
    /* 降速也用死区后的偏差: 用原始量化值的话, 正中(±0.14)都会掉 11% 速度 */
    float v = cruise * (1.0f - 0.5f * f_abs(e_steer));   /* 原 0.8: 边缘只剩两成速度, 入弯爬行 */
    if (v < 4.0f) v = 4.0f;
    if (count >= 6 && v > 4.0f) v = 4.0f;
    if (count <= 2 && f_abs(e) > 0.4f && v < 15.0f) v = 15.0f;   /* 大偏差下限 8→15: 边缘回中不爬行 */
    line_hold_yaw = imu_yaw();
    pid_reset(&pid_head);
    control_set_cmd(v, rot);
    return;
  }

  if (line_crossing) {
    /* 穿越中瞬间丢线(L 形本该连通): 按最后线位置定方向收尾 */
    line_crossing = false;
  line_junc_hist = 0; line_junc_side = 0;
    int dir = (line_cross_last_e < 0.0f) ? -1 : 1;
    line_begin_corner(dir);
    return;
  }
  if (!line_gap_active) {
    line_gap_active = true;
    if (!track_map.axis_valid && !track_map.frozen) track_map.heading = line_straight_yaw;
    line_map_lock_gap(&track_map);
    line_seen_cnt = 0;                       /* 丢线: 武装计数清零 */
    line_junc_hist = 0; line_junc_side = 0;  /* 丢线: 候选方向一并释放 */
    /* 断线位置离上个弯超过 50cm -> 这是下一个弯, 而不是弯后空白段。
       再加一道: 断线那一刻线必须基本居中(|e|小) —— 弯口的线是"走到头消失"的;
       若是直线中途跑偏甩出去的, 丢线前线早滑到探头边缘(e 大), 那不是弯 */
    line_gap_lost_e = e;
    acc_rec_tick();                      /* 加速段记录: 从丢线这一拍开始 */
    line_end_pending = ((control_dist_cm() - line_seg_start_cm) > LINE_END_TURN_MIN_SEG_CM) &&
                       (f_abs(e) < 0.35f) &&
                       ((LAP_TOTAL_CM - (control_dist_cm() - lap_dist0)) > 20.0f);
    ev_log(8);                               /* 事件8: 丢线, 进空白段 */
    line_gap_start_cm = control_dist_cm();
    line_prev_valid = false; line_derivative = 0.0f;
    pid_reset(&pid_head);
  }
  /* 单弯兜底: 丢线 3cm 没见线 = 到弯了, 直接进转弯流程 */
  if (LINE_END_TURN_ENABLE && line_end_pending &&
      (control_dist_cm() - line_gap_start_cm) > LINE_END_TURN_CM) {
    /* 兜底: 断线走了 3cm 还没见线, 且这段本来就够长
       -> 当成下一个弯转。方向优先用最近看到的宽黑分布,
       否则沿用上个弯的方向(同向赛道) */
    int dir = (line_wide_side != 0 && (control_dist_cm() - line_wide_cm) < 25.0f)
                  ? (int)line_wide_side : (int)line_last_corner_dir;
    line_end_pending = false;
    line_begin_corner(dir);
    return;
  }
  lost_ms += dt * 1000.0f;
  /* 空白段保持巡航速度, 不在中间减速再加速(实测那样反而顿挫+拖时间);
     进弯减速由弯道自己的补偿段+刹停负责 */
  float cruise_lost = cruise;
  /* 空白段航向: 用"稳定居中时"的航向均值(line_straight_yaw), 不用最后一拍
     的瞬时值 —— 最后一拍可能正在纠偏, 那个歪方向会被空白段一路保持下去。
     限幅 ±45: 42 巡航下 ±70 会饱和振荡(实测 +38/-70 来回打) */
  float rot = line_map_steer(&track_map, imu_yaw(), imu_yaw_rate(), dt, false);
  /* Heading-only fallback traverses the gap gently; trustworthy geometry can
     use the requested cruise. Rate feedback and bounded correction stay active
     throughout acceleration, without altering the reference heading. */
  if (track_map.confidence < 2 && cruise_lost > 20) cruise_lost = 20;
  if (f_abs(ang_diff(track_map.heading, imu_yaw())) > 8 && cruise_lost > 12) cruise_lost = 12;
  control_set_cmd(cruise_lost, rot);
}

/* 分段速度: 首段固定起步速度; 之后出弯加速到巡航, 距下个弯 10cm 减回准备速度。
   最后一弯之后没有下一个弯了: 直道全速冲线, 不再减到进弯准备速度 */
static float line_cruise_eff(void) {
  if (line_corners_done == 0) return LINE_LAUNCH_CM_S;
  if (line_corners_done >= 4) return (float)p_cruise;   /* 最后一弯后: 全速直道 */
  float into = control_dist_cm() - line_seg_start_cm;
  /* 兜底: 本段走了 1.5 倍段长还没进弯 -> 这个弯没认出来(或弯被开过去了)。
     不处理的话 into 会一直大于减速点, 车会挂着"进弯准备速度"20cm/s
     跑完剩下的整圈(用户实测: 倒数第二段直线突然变慢就这个)。
     重新起一段, 让它恢复巡航 */
  if (into > LINE_SEGMENT_CM * 1.5f) {
    ev_log(14);                          /* 事件14: 段长异常, 重启本段 */
    line_seg_start_cm = control_dist_cm();
    into = 0.0f;
  }
  if (into > LINE_SEGMENT_CM - LINE_APPROACH_CM) return LINE_APPROACH_CM_S;
  return (float)p_cruise;
}

static bool line_search_failed(float cruise) {
  if (!line_gap_active) return lost_ms > LOST_TIMEOUT_MS;
  float speed = cruise < 5.0f ? 5.0f : cruise;
  if (track_map.confidence < 2 && speed > 20) speed = 20;
  if (f_abs(ang_diff(track_map.heading, imu_yaw())) > 8 && speed > 12) speed = 12;
  float timeout = LINE_GAP_MAX_CM / speed * 1000.0f + 1500.0f;
  return control_dist_cm() - line_gap_start_cm > LINE_GAP_MAX_CM || lost_ms > timeout;
}

/* 用航向变化识别直角弯 (只数弯, 不在这里判圈) */
static void corner_update(void) {
  float dyaw = f_abs(imu_yaw() - corner_yaw_ref);
  float dist = control_dist_cm();
  if (corner_armed) {
    if (dyaw > LINE_CORNER_DEG) {            /* 航向开始大角度变化 = 进入弯道 */
      corner_armed = false;
      straight_ms = 0;
    }
  } else if (f_abs(imu_gyro_z()) < 25.0f) {  /* 车重新走直 = 这个弯转完了 */
    if (++straight_ms > 20) {
      /* 弯道"转完"才计数, 而不是刚转 32° 就计数 ——
         否则从直道中间起跑时, 会在第四个弯还没转完就提前停车 */
      if ((dist - dist_at_corner) > LINE_MIN_GAP_CM) {
        corner_count++;
        dist_at_corner = dist;
        if (corner_count == 4) dist_at_corner4 = dist;   /* 第 4 个弯的位置, 兜底判据用 */
      }
      corner_yaw_ref = imu_yaw();
      corner_armed = true;
    }
  } else {
    straight_ms = 0;
  }
}

/* 航位推算: 用编码器里程 + 相对起跑航向, 估计车离起跑点多远 (cm) */
static void pos_update(void) {
  float d = control_dist_cm();
  float dd = d - pos_last_dist;
  pos_last_dist = d;
  if (dd <= 0.0f || dd > 50.0f) return;    /* 倒车/异常跳变不计入 */
  float rel = imu_yaw() - lap_yaw_ref;
  pos_x += dd * cos_deg(rel);
  pos_y += dd * sin_deg(rel);
}

/* 角度差归一化到 -180~180。
   陀螺航向是一直累加的: 跑完一整圈之后 imu_yaw() 比起点大了 360°,
   直接相减得到 360°, 会被当成"方向不一致"。归一化之后 360° 变成 0°,
   顺/逆时针跑满一圈都能正确判成"回到起跑方向"。 */
static float ang_diff(float a, float b) {
  float d = a - b;
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

/* 一圈的判定结果 */
#define LAP_NONE 0
#define LAP_DONE 1
#define LAP_FAIL 2      /* 兜底超限: 异常停车, 不增加圈数 */

/* 回到起点了没有? 三个条件: 4 个弯都转完 + 位置回到起跑点附近 + 航向一致 */
static int lap_check(void) {
  if (corner_count < 4) return LAP_NONE;   /* 弯都没转完, 不可能回到起点 */
  /* 圈完成: 本圈里程跑满一圈(LAP_TOTAL_CM)就停 —— 最直接可靠。
     注意用 (总里程 - 本圈起点里程): 直接拿总里程比 400 的话,
     第二圈一开头里程就已经 >400, 数到第 4 个弯就误判成跑完 */
  if ((control_dist_cm() - lap_dist0) >= LAP_TOTAL_CM) return LAP_DONE;
  float d2 = pos_x * pos_x + pos_y * pos_y;
  if (d2 > LAP_GOAL_R_CM * LAP_GOAL_R_CM) {
    /* 兜底: 推算误差可能让车永远进不了这个圈。以前这里直接算"跑完",
       等于把多跑的一大段当成成功; 现在改成报异常 -> 上层停车, 圈数不加。 */
    if ((control_dist_cm() - dist_at_corner4) > LAP_RETURN_MAX_CM) return LAP_FAIL;
    return LAP_NONE;
  }
  if (f_abs(ang_diff(imu_yaw(), lap_yaw_ref)) > LAP_GOAL_YAW_DEG) return LAP_NONE;
  return LAP_DONE;
}

/* 每周期调用: 推算位置 -> 数弯 -> 判圈。跑完一圈时记耗时、重置本轮基准 */
static int lap_update(void) {
  pos_update();
  corner_update();
  int r = lap_check();
  if (r != LAP_DONE) return r;

  if (lap_done < LAP_MAX) lap_ms[lap_done] = HAL_GetTick() - lap_start_ms;
  lap_start_ms = HAL_GetTick();
  lap_done++;
  corner_count = 0;
  dist_at_corner4 = control_dist_cm();
  lap_dist0 = control_dist_cm();           /* 本圈起点: 下一圈按新起点算里程 */
  line_corners_done = 0;                   /* 循线速度调度也按新的一圈重来 */
  line_seg_start_cm = control_dist_cm();
  pos_x = pos_y = 0.0f;                    /* 以这里为新起点, 免得误差越攒越大 */
  lap_yaw_ref = imu_yaw();
  return LAP_DONE;
}

/* 模式五: 循线 N 圈 */
static void ctrl_line(float dt) {
  if (!running) return;   /* 已经停了: 不要再动电机, 保持上次的刹车/滑行状态 */
  /* 总时限安全网: 正常一圈 ~20s。跑飞后若反复偶遇杂线, 丢线计时会被不断
     重置, 车会一直兜圈(实测兜了 100+ 秒) —— 这里给整次运行一个上限 */
  if (HAL_GetTick() - run_start_ms > (uint32_t)(p_laps * 40000u + 15000u)) {
    ev_log(12);                            /* 事件12: 超时异常停车 */
    control_brake();
    running = false; finished = true; aborted = true;
    run_end_ms = HAL_GetTick();
    return;
  }
  line_follow(dt, line_cruise_eff());
  if (!running) return;
  int r = lap_update();
  if (r == LAP_FAIL) {                   /* 第4个弯后走太远还没回到起点: 异常停车 */
    control_brake();
    running = false; finished = true; aborted = true;
    run_end_ms = HAL_GetTick();
    return;
  }
  if (lap_done >= p_laps) {
    ev_log(10);                              /* 事件10: 跑完一圈 */
    control_brake();
    running = false; finished = true; aborted = false;
    run_end_ms = HAL_GetTick();            /* 冻住计时 */
  } else if (line_search_failed((float)p_cruise)) {
    ev_log(11);                              /* 事件11: 找线失败停车 */
    control_brake();
    running = false; finished = true; aborted = true;
    run_end_ms = HAL_GetTick();
  }
}

/* 模式六: 循线一圈 + 超声波数物体测距离 */
static void ctrl_object(float dt) {
  if (!running) return;   /* 已经停了: 不要再动电机, 保持上次的刹车/滑行状态 */
  line_follow(dt, line_cruise_eff());
  if (!running) return;

  obj_update();                            /* 进入-跟踪-离开 状态机 */
  int r = lap_update();
  if (r == LAP_FAIL) {                     /* 兜底超限: 异常停车, 不算跑完 */
    control_brake();
    running = false; finished = true; aborted = true;
    run_end_ms = HAL_GetTick();
    return;
  }
  if (lap_done >= 1) {
    control_brake();
    running = false; finished = true; aborted = false;
    run_end_ms = HAL_GetTick();
    obj_page = 0;
  } else if (line_search_failed((float)p_cruise)) {
    control_brake();
    running = false; finished = true; aborted = true;
    run_end_ms = HAL_GetTick();
  }
}

/* ================= 模式七: 蓝牙遥控 =================
   帧格式: AA 55 CMD ARG XOR    (XOR = AA^55^CMD^ARG)
   0x06 混控帧 (6字节): AA 55 06 LIN ROT XOR  边走边转
   0x07 坦克帧 (6字节): AA 55 07 WL WR XOR    左右轮直控, 带符号
   校验失败、UART 错误或超时立即刹停；两帧新鲜回中指令后才能恢复。
   旧 XOR 协议不能识别所有错位；payload 内的 AA 55 保留为新帧头。
   超过 RC_TIMEOUT_MS 没有有效指令 -> 刹停并等待; 遥控恢复(且杆回中)后
   自动恢复驾驶, 不用按 K5 —— 蓝牙模块断链会自动回连, 车端也自动接上。 */
static void rc_apply(uint8_t cmd, uint8_t arg) {
  float a = (float)arg;
  rc_tank = false;
  if (a > 100.0f) a = 100.0f;
  switch (cmd) {
  case RC_CMD_FWD:   rc_lin =  a * (RC_LIN_CM_S / 100.0f); rc_rot = 0.0f; break;
  case RC_CMD_BACK:  rc_lin = -a * (RC_LIN_CM_S / 100.0f); rc_rot = 0.0f; break;
  case RC_CMD_LEFT:  rc_lin = 0.0f; rc_rot = -a * (RC_ROT_DPS / 100.0f); break;
  case RC_CMD_RIGHT: rc_lin = 0.0f; rc_rot =  a * (RC_ROT_DPS / 100.0f); break;
  case RC_CMD_STOP:  rc_lin = 0.0f; rc_rot = 0.0f; break;
  default:                       /* 未知命令: 只统计, 不执行 */
    if (rc_bad_cnt < 65535) rc_bad_cnt++;
    return;
  }
  rc_last_ms = HAL_GetTick();
  if (rc_ok_cnt < 65535) rc_ok_cnt++;
}

/* 混控帧: 前进和转向同时给 (真·边走边转, 和普通遥控车的油门+转向混控一样) */
static void rc_apply_mix(int8_t lin8, int8_t rot8) {
  float l = lin8, r = rot8;
  rc_tank = false;
  if (l > 100.0f) l = 100.0f;
  if (l < -100.0f) l = -100.0f;
  if (r > 100.0f) r = 100.0f;
  if (r < -100.0f) r = -100.0f;
  /* 后退时转向取反, 符合真车习惯: 倒车按左 = 车尾甩向左 (车头向右摆)。
     不取反的话倒车+转向会按"车头指向"转, 和直觉相反。 */
  if (l < 0.0f) r = -r;
  rc_lin = l * (RC_LIN_CM_S / 100.0f);
  rc_rot = r * (RC_ROT_DPS / 100.0f);
  rc_last_ms = HAL_GetTick();
  if (rc_ok_cnt < 65535) rc_ok_cnt++;
}

/* 坦克帧: 直接左右轮控制。一侧给 0 另一侧照转, 大差速不再受混控通道量程限制 */
static void rc_apply_tank(int8_t wl8, int8_t wr8) {
  rc_wl = wl8 * (RC_LIN_CM_S / 100.0f);
  rc_wr = wr8 * (RC_LIN_CM_S / 100.0f);
  rc_tank = true;
  rc_last_ms = HAL_GetTick();
  if (rc_ok_cnt < 65535) rc_ok_cnt++;
}

/* 每周期调用: 取字节 -> 组帧 -> 校验 -> 执行。
   帧长由命令字决定: 0x06/0x07 为 6 字节, 其余 5 字节。 */
/* A transport discontinuity invalidates the previous motion command.
 * Keep the run selected, but require two new neutral frames to unlock. */
static void rc_guard(void) {
  /* 同一批坏数据只触发一次保护，避免反复 control_brake_fast() 把主循环拖慢。 */
  bool first = !rc_wait;
  if (first && rc_guard_count < 65535) ++rc_guard_count;
  rc_wait = true;
  rc_neutral_count = 0;
  rc_lin = rc_rot = rc_wl = rc_wr = 0.0f;
  rc_tank = false;
  if (first && mode == MODE_RC && running) control_brake_fast();
}

static void rc_apply_packet(const rc_packet_t *packet) {
  if (mode != MODE_RC || !running) return;
  bool idle = packet->a == 0 && packet->b == 0;
  if (rc_wait) {
    if (!idle) { rc_neutral_count = 0; return; }
    if (!rc_neutral_count || (uint32_t)(packet->received_ms - rc_neutral_ms) > RC_NEUTRAL_PAIR_MS)
      rc_neutral_count = 1;
    else if (packet->received_ms != rc_neutral_ms)
      ++rc_neutral_count;
    rc_neutral_ms = packet->received_ms;
    if (rc_neutral_count < 2) return;
    rc_wait = false;
    rc_neutral_count = 0;
    control_reset();
  }
  if (packet->cmd == RC_CMD_MIX) rc_apply_mix((int8_t)packet->a, (int8_t)packet->b);
  else if (packet->cmd == RC_CMD_TANK) rc_apply_tank((int8_t)packet->a, (int8_t)packet->b);
  else rc_apply(packet->cmd, (uint8_t)packet->a);
  rc_last_ms = packet->received_ms;  /* age is measured from IRQ reception */
}

static void rc_poll(void) {
  uint8_t b;
  uint32_t ms;
  rc_packet_t packet, latest;
  bool have_latest = false;

  if (bt_rx_take_fault()) { rc_parser_reset(&rc_parser); rc_guard(); }
  while (bt_rx_get_timed(&b, &ms)) {
    if (bt_rx_take_fault()) { rc_parser_reset(&rc_parser); rc_guard(); return; }
    /* 队列里积压的旧字节只丢当前候选帧，不刹车锁死；真正断联由 RC_TIMEOUT_MS 处理。 */
    if ((uint32_t)(HAL_GetTick() - ms) > RC_FRESH_MS) {
      if (rc_stale_count < 65535) ++rc_stale_count;
      rc_parser_reset(&rc_parser);
      continue;
    }
    int event = rc_parser_feed(&rc_parser, b, ms, &packet);
    if (event == RC_PARSE_BAD) {
      if (rc_bad_cnt < 65535) ++rc_bad_cnt;
      continue;                         /* 坏帧丢弃，保留当前有效控制 */
    }
    if (event == RC_PARSE_PACKET) {
      if (bt_rx_take_fault()) { rc_parser_reset(&rc_parser); rc_guard(); return; }
      if (mode == MODE_RC && running) {
        latest = packet;                 /* 只保留最新完整帧，丢弃旧积压 */
        have_latest = true;
      }
    }
  }
  if (bt_rx_take_fault()) { rc_parser_reset(&rc_parser); rc_guard(); return; }
  if (rc_parser_expire(&rc_parser, HAL_GetTick()) == RC_PARSE_BAD) {
    if (rc_bad_cnt < 65535) ++rc_bad_cnt;
    /* 半帧超时只重同步，不把一次串口抖动升级成断联锁定。 */
  }
  if (have_latest) rc_apply_packet(&latest);
}

static void ctrl_rc(float dt) {
  (void)dt;
  if (!running) return;
  /* Catch an IRQ fault occurring after rc_poll, before issuing motor targets. */
  if (bt_rx_take_fault()) { rc_parser_reset(&rc_parser); rc_guard(); return; }
  if ((HAL_GetTick() - rc_last_ms) > RC_TIMEOUT_MS) {
    /* 断联: 刹停并进入"等待遥控"状态 (不再退出运行)。模块会自动回连,
       遥控恢复且先发来居中停车帧后自动恢复驾驶, 不用按 K5。
       断联期间摇杆还推着的话, 恢复后也要求先回中, 防止窜车。 */
    if (!rc_wait) {
      control_brake_fast();
      rc_wait = true;
      rc_neutral_count = 0;
      rc_lin = rc_rot = rc_wl = rc_wr = 0.0f;
    }
    return;
  }
  if (rc_wait) {
    control_brake_fast();
    return;  /* only rc_poll's two neutral frames may release the latch */
  }
  /* 松开所有键 = 停止帧: 直接动力刹车, 不走速度环。
     走 PI 刹停的话积分会把轮子刹过头倒退一小段 (松开转向键右轮回退就是这个) */
  bool rc_idle = rc_tank ? (rc_wl == 0.0f && rc_wr == 0.0f)
                         : (rc_lin == 0.0f && rc_rot == 0.0f);
  if (rc_idle) {
    control_brake_fast();
    return;
  }
  /* 坦克帧: 直控左右轮。进出坦克模式都清一次控制状态,
     避免混控模式留下的积分/目标把切换瞬间甩一下 */
  {
    static bool prev_tank = false;
    if (rc_tank != prev_tank) {
      control_reset();
      prev_tank = rc_tank;
    }
  }
  if (rc_tank) {
    control_set_wheels(rc_wl, rc_wr);
    return;
  }
  /* 进出"纯转向"时清一次控制状态:
     转向中两轮一正一反, 积分/滤波里存的是转向的运动状态; 直接切前进的话,
     这些旧状态会让追赶距离更大的那个轮子(刚在反转的)猛窜一下,
     车头甩向刚才转向的反方向。清掉后从干净状态起步。 */
  {
    static bool prev_turn = false;
    bool now_turn = (rc_lin == 0.0f && rc_rot != 0.0f);
    if (now_turn != prev_turn) control_reset();
    prev_turn = now_turn;
  }
  /* 纯转向(不按前后) = 原地差速对转, 走 spin 通道:
     轮子目标只有几cm/s, 是静摩擦最大的速度区, 不走这条左轮经常顶不开 */
  if (rc_lin == 0.0f && rc_rot != 0.0f) control_set_spin(rc_rot);
  else                                  control_set_cmd(rc_lin, rc_rot);
}

/* 调试页: 按住 K2/K3 分别转左右轮, 核对电机转向和编码器方向 */
static void ctrl_debug(void) {
  /* 蓝牙自检 / 陀螺校准这类阻塞操作刚做完时, 必须先松手再按才恢复驱动:
     否则用户按着测试键做完校准, 手没松车就窜出去了 */
  bool u = keys_pressed(KEY_UP), n = keys_pressed(KEY_DOWN);
  if (dbg_lock) {
    if (!u && !n) dbg_lock = false;
    u = n = false;
  }
  motor_set_duty(MOT_LEFT,  u ? 0.35f : 0.0f);
  motor_set_duty(MOT_RIGHT, n ? 0.35f : 0.0f);
  dbg_enc_l += (float)motor_read_delta(MOT_LEFT);
  dbg_enc_r += (float)motor_read_delta(MOT_RIGHT);
  line_update();
}

/* ================= 界面 ================= */
static void ui_draw(void) {
  char a[40], b[40], c[40], d[40];
  char n1[12];
  a[0] = b[0] = c[0] = d[0] = 0;

  /* 陀螺仪掉线是最要命的一种故障: 航向已经不再更新了, 所有依赖航向的
     功能(方向保持/转向/循迹/计圈)都会失效, 所以优先霸屏报故障 */
  if (imu_fault) {
    ui_show("陀螺掉线 已停车", "查 PA15/PB3 接线", "静止后长按K5重校", "K1换模式 K5重试");
    return;
  }

  switch (mode) {
  case MODE_HEADING:
    snprintf(a, sizeof(a), "1 方向保持");
    snprintf(b, sizeof(b), "航向 %d 度", yaw_show(imu_yaw()));
    if (running) {
      snprintf(c, sizeof(c), "运行中 目标%d", p_cruise);
      f1(n1, control_lin());
      snprintf(d, sizeof(d), "实测 %s cm/s", n1);
    } else {
      snprintf(c, sizeof(c), "K2/3调速 K5开始");
      snprintf(d, sizeof(d), "巡航 %d cm/s", p_cruise);   /* 模式三/五/六共用这个值 */
    }
    break;

  case MODE_TURN:
    snprintf(a, sizeof(a), "2 指定角度转向");
    snprintf(b, sizeof(b), "方向%s %d度", p_turn_dir > 0 ? "右" : "左", p_turn_angle);
    if (finished) {
      snprintf(c, sizeof(c), aborted ? "异常停止" : "转向完成");
      if (aborted) {
        const char *reason = "IMU";
        switch (turn_fault) {
        case TURN_ERR_TIMEOUT: reason = "TIMEOUT"; break;
        case TURN_ERR_LEFT: reason = "LEFT STALL"; break;
        case TURN_ERR_RIGHT: reason = "RIGHT STALL"; break;
        case TURN_ERR_SETTLE: reason = "SETTLE"; break;
        case TURN_ERR_DIRECTION: reason = "DIRECTION"; break;
        default: break;
        }
        snprintf(d, sizeof(d), "%s", reason);
      } else {
        snprintf(d, sizeof(d), "实转 %d 度",
                 (int)(turn_display_angle() + 0.5f));
      }
    } else if (running) {
      snprintf(c, sizeof(c), turn_phase == TURN_BRAKING ? "刹车确认" :
               (turn_trims ? "低速微调" : "正在转向"));
      snprintf(d, sizeof(d), "误差 %d 度",
               (int)((float)p_turn_dir * (yaw_ref + (float)p_turn_dir * turn_command_angle() - imu_yaw())
                     / (p_turn_dir > 0 ? TURN_RIGHT_RAW_RATIO : TURN_LEFT_RAW_RATIO)));
    } else {
      snprintf(c, sizeof(c), "K2/3角度 K4方向");
      snprintf(d, sizeof(d), "K5开始");
    }
    break;

  case MODE_DIST:
    snprintf(a, sizeof(a), "3 里程计");
    snprintf(b, sizeof(b), "目标%d 速度%d", p_dist_cm, p_cruise);
    if (finished) {
      if (enc_fault) snprintf(c, sizeof(c), "编码器没动 查线");
      else           snprintf(c, sizeof(c), aborted ? "异常停止" : "已到达");
    } else if (running) {
      snprintf(c, sizeof(c), "行驶中");
    } else {
      snprintf(c, sizeof(c), "K2/3里程 K5开始");
    }
    f1(n1, control_dist_cm());
    snprintf(d, sizeof(d), "已走 %s cm", n1);
    break;

  case MODE_VOFA:
    snprintf(a, sizeof(a), "4 速度曲线");
    snprintf(b, sizeof(b), "目标速度 %d", p_speed);
    snprintf(c, sizeof(c), running ? "输出中" : "K2/3调速 K5开始");
    if (running) {
      f1(n1, control_lin());
      snprintf(d, sizeof(d), "实测 %s cm/s", n1);
    } else {
      snprintf(d, sizeof(d), "K4波特%u", (unsigned)bt_baud);
    }
    break;

  case MODE_LINE:
    snprintf(a, sizeof(a), "5 循线行驶");
    if (finished) {
      snprintf(b, sizeof(b), "共%d圈", p_laps);
      /* 用 run_end_ms 而不是 HAL_GetTick(): 停车之后秒数不能再往上涨 */
      uint32_t end = run_end_ms ? run_end_ms : HAL_GetTick();
      if (aborted) snprintf(c, sizeof(c), "丢线停在第%d圈", lap_done + 1);
      else         snprintf(c, sizeof(c), "完成%d圈 %d秒", lap_done,
                            (int)((end - run_start_ms) / 1000));
      /* 显示最慢的一圈: 它过了"单圈<20秒"这条要求才算过 */
      uint32_t worst = 0;
      for (int i = 0; i < lap_done && i < LAP_MAX; i++) {
        if (lap_ms[i] > worst) worst = lap_ms[i];
      }
      if (worst > 0) snprintf(d, sizeof(d), "最慢%d秒 K5重跑", (int)((worst + 500) / 1000));
      else           snprintf(d, sizeof(d), "K5重跑 K1换模式");
    } else if (running) {
      snprintf(b, sizeof(b), "第%d/%d圈", lap_done + 1, p_laps);
      uint8_t lb = line_bits();
      snprintf(c, sizeof(c), "线%d%d%d%d%d%d%d%d",
               !!(lb & 1), !!(lb & 2), !!(lb & 4), !!(lb & 8),
               !!(lb & 16), !!(lb & 32), !!(lb & 64), !!(lb & 128));
      if (line_corner_active) snprintf(d, sizeof(d), "%s转找线",
                                      line_corner_dir > 0 ? "右" : "左");
      else if (line_gap_active) snprintf(d, sizeof(d), "%d秒 空白段直行",
                                      (int)((HAL_GetTick() - lap_start_ms) / 1000));
      else snprintf(d, sizeof(d), "%d秒 循迹中",
                    (int)((HAL_GetTick() - lap_start_ms) / 1000));
    } else {
      snprintf(b, sizeof(b), "圈数 %d", p_laps);
      snprintf(c, sizeof(c), "K2/3圈数 K5开始");
      snprintf(d, sizeof(d), "巡航%d 先放线上", p_cruise);
    }
    break;

  case MODE_OBJECT:
    if (finished) {
      snprintf(a, sizeof(a), "物体数: %d", obj_num);
      for (int row = 0; row < 3; row++) {
        char *dst = (row == 0) ? b : (row == 1) ? c : d;
        dst[0] = 0;
        for (int k = 0; k < 2; k++) {
          int idx = (int)obj_page * 6 + row * 2 + k;
          if (idx >= obj_num) break;
          f1(n1, obj_dist[idx]);
          char tmp[16];
          snprintf(tmp, sizeof(tmp), "%d:%s ", idx + 1, n1);
          size_t len = strlen(dst);
          strncat(dst, tmp, 39 - len);
        }
        if (!dst[0]) snprintf(dst, 40, " ");
      }
      if (obj_num > 6) {
        size_t len = strlen(d);
        if (len < 36) strncat(d, "K2/3", 39 - len);
      }
    } else if (running) {
      snprintf(a, sizeof(a), "6 物体计数测距");
      snprintf(b, sizeof(b), "已走 %d cm", (int)control_dist_cm());
      snprintf(c, sizeof(c), "已发现 %d 个", obj_num);
      /* 实时与结束记录都用同一轨道距离标定；实时值另带显示平滑。 */
      if (sonar_disp < 0.0f) snprintf(d, sizeof(d), "到轨 -- cm");
      else { f1(n1, obj_track_distance_cm(sonar_disp)); snprintf(d, sizeof(d), "到轨 %s cm", n1); }
    } else {
      /* 待机时也显示实时距离: 摆物体/标定探头时不用先按 K5 让车跑起来 */
      snprintf(a, sizeof(a), "6 物体计数测距");
      snprintf(b, sizeof(b), "K5开始 跑一圈");
      if (sonar_disp < 0.0f) snprintf(c, sizeof(c), "到轨 无回波");
      else { f1(n1, obj_track_distance_cm(sonar_disp)); snprintf(c, sizeof(c), "到轨 %s cm", n1); }
      snprintf(d, sizeof(d), "物体放轨道外");
    }
    break;

  case MODE_RC:
    snprintf(a, sizeof(a), "7 蓝牙遥控");
    if (running && rc_wait) {
      snprintf(b, sizeof(b), "遥控断联 已刹车");
      snprintf(c, sizeof(c), "等遥控器回连");
      snprintf(d, sizeof(d), "松杆自动恢复");
    } else if (running) {
      snprintf(b, sizeof(b), "已收%d 丢%d", (int)rc_ok_cnt, (int)rc_bad_cnt);
      if (rc_lin > 0.5f)       snprintf(c, sizeof(c), "前进");
      else if (rc_lin < -0.5f) snprintf(c, sizeof(c), "后退");
      else if (rc_rot > 0.5f)  snprintf(c, sizeof(c), "右转");
      else if (rc_rot < -0.5f) snprintf(c, sizeof(c), "左转");
      else                     snprintf(c, sizeof(c), "停止");
      uint32_t age = HAL_GetTick() - rc_last_ms;
      snprintf(d, sizeof(d), "%d ms前指令", (int)(age > 9999 ? 9999 : age));
    } else if (finished && aborted) {
      snprintf(b, sizeof(b), "已收%d 丢%d", (int)rc_ok_cnt, (int)rc_bad_cnt);
      snprintf(c, sizeof(c), "启动失败 已停车");
      snprintf(d, sizeof(d), "K5重试");
    } else {
      snprintf(b, sizeof(b), "连HC-06后发指令");   /* 15 格 */
      snprintf(c, sizeof(c), "帧头AA55 带校验");   /* 15 格 */
      snprintf(d, sizeof(d), "K5开始 K1换模式");
    }
    break;

  case MODE_DEBUG: {
    /* 注意: newlib-nano 默认不支持 printf 的 %f(会打印成空白),
       所以电压用整数拆成 "3.27V" 这样显示 */
    int v100 = (int)(vdd_now * 100.0f + 0.5f);
    if (dbg_page == 0) {
      /* 第一屏: 电机/编码器/陀螺/循迹 */
      if (HAL_GetTick() < bt_show_until) {
        /* 不要按字节截断! 一个汉字在 UTF-8 里占 3 字节, %.11s 会把
           "无应答 线1" 从中间切开, 屏幕上的 UTF-8 解码器认不出半个字就不画了
           (之前"只显示无应答、没有线1"就是这个原因)。
           这里直接给完整字符串, 让 OLED_DrawString 按像素在右边截断。 */
        snprintf(a, sizeof(a), "蓝牙:%s", bt_resp);
      } else {
        /* 原始硬件计数器 (16位环形): 手转轮子核对左右编码器标定 ——
           每个轮子转同样圈数, 两边计数的增量应该一致 */
        snprintf(a, sizeof(a), "L计数%u R%u",
                 motor_raw_cnt(MOT_LEFT), motor_raw_cnt(MOT_RIGHT));
      }
      /* 超声波实时距离 + 三轴陀螺原始读数(÷100)。
         测距标定(拿纸板放在已知距离处看读数)就看这一行;
         转动车身时哪个陀螺数字变, 说明那个轴就是实际的偏航轴 —— 用来确认安装方向。 */
      char sd1[12];
      if (sonar_disp < 0.0f) {
        snprintf(b, sizeof(b), "距-- 陀%d/%d/%d",
                 gyro_raw[0] / 100, gyro_raw[1] / 100, gyro_raw[2] / 100);
      } else {
        f1(sd1, sonar_disp);   /* 一位小数 */
        snprintf(b, sizeof(b), "距%s 陀%d/%d/%d", sd1,
                 gyro_raw[0] / 100, gyro_raw[1] / 100, gyro_raw[2] / 100);
      }
      /* 航向 / 角速度 / 自动判断出的偏航轴 (0=X 1=Y 2=Z) */
      snprintf(c, sizeof(c), "yaw%d gz%d 轴%d", yaw_show(imu_yaw()), (int)imu_gyro_z(), yaw_axis);
      snprintf(d, sizeof(d), "线%d%d%d%d%d%d%d%d %d.%02dV",
               !!(line_bits() & 1), !!(line_bits() & 2), !!(line_bits() & 4), !!(line_bits() & 8),
               !!(line_bits() & 16), !!(line_bits() & 32), !!(line_bits() & 64), !!(line_bits() & 128),
               v100 / 100, v100 % 100);
    } else {
      /* 测距调试：原始保留未补偿回波；到轨与模式六共用标定。
         补/比仍是底层探头标定，模式六轨道补偿只在显示和记录出口使用。 */
      char s1[12], s2[12];
      if (sonar_raw < 0.0f) { snprintf(a, sizeof(a), "原始 -- cm"); }
      else                  { f1(s1, sonar_raw); snprintf(a, sizeof(a), "原始 %s cm", s1); }
      if (sonar_cm < 0.0f)  { snprintf(b, sizeof(b), "到轨 -- cm"); }
      else                  { f1(s2, obj_track_distance_cm(sonar_cm));
                              snprintf(b, sizeof(b), "到轨 %s cm", s2); }
      f1(n1, SONAR_OFFSET_CM);
      char n2[12]; f1(n2, SONAR_SCALE);
      snprintf(c, sizeof(c), "补%s 比%s", n1, n2);
      snprintf(d, sizeof(d), "回%d 超%d", (int)sonar_ok_cnt, (int)sonar_to_cnt);
    }
    break;
  }
  }
  ui_show(a, b, c, d);
}

/* ================= 启动 / 停止 ================= */
static void start_run(void) {
  /* 所有模式都依赖航向角(方向保持/转角度/循迹丢线直行/计圈),
     陀螺仪没认到就拒绝启动 —— 否则模式二会一直转、永远到不了目标角度 */
  if (!imu_ok() || !imu_alive()) {
    running = false;
    finished = true;
    aborted = true;
    imu_fault = true;
    return;
  }
  running = true;
  finished = false;
  aborted = false;
  rc_wait = false;                     /* 新模式七运行: 从干净状态开始 */
  imu_fault = false;
  enc_fault = false;
  run_start_ms = HAL_GetTick();
  run_end_ms = 0;
  control_reset_dist();                /* 每次起跑里程归零: 模式3的目标/循迹的本圈
                                          里程都从 0 开始, 连续跑第二次才不会乱 */
  lap_dist0 = control_dist_cm();
  /* 遥控使用相同目标加速度，避免直行指令在加速阶段主动制造差速。
     电机差异交给左右轮各自的速度环；其它模式保留已有斜坡参数。 */
  acc_slew_l = mode == MODE_DIST ? 1.0f :
      (mode == MODE_RC ? ACC_SLEW_L_RC :
      ((mode == MODE_LINE || mode == MODE_OBJECT) ? ACC_SLEW_L : ACC_SLEW_L_OTHER));
  t_corner_ms = t_settle_ms = t_pull_ms = t_lost_ms = t_track_ms = 0;
  /* 起跑这一下的航向 = "线方向"的高置信度参考: 车是摆在线上、对着线放的。
     开机后车被人搬动过, 沿用旧基准会让"车头相对线方向"变成假的大角度;
     用这里做初值, 航向误差项从第一拍就是对的, 之后由压线时的均值慢慢修。 */
  line_straight_yaw = imu_yaw();
  line_straight_valid = true;
  est_e = 0.0f;
  acc_rec_n = 0; acc_rec_done = 0;
  ev_n = 0; ev_log(1);                       /* 事件1: 起跑 (记下参考航向) */
  yaw_ref = imu_yaw();
  line_hold_yaw = imu_yaw();
  corner_yaw_ref = imu_yaw();
  lap_yaw_ref = imu_yaw();
  pid_reset(&pid_head);
  turn_phase = TURN_DRIVE;
  turn_fault = TURN_ERR_NONE;
  turn_trims = 0;
  turn_rate_cmd = turn_stall_l = turn_stall_r = 0.0f;
  turn_brake_ms = turn_stable_ms = 0;
  turn_last_err = (float)p_turn_dir * turn_command_angle();
  line_corner_active = line_gap_active = false;
  line_settle_active = false;
  line_pull = false;
  line_end_turned = false;
  line_seg_start_cm = 0.0f;
  line_corners_done = 0;
  line_track_seen = false;
  line_end_pending = false;
  line_wide_side = 0; line_wide_cm = -999.0f;
  line_straight_valid = true;
  line_crossing = false;
  line_map_reset(&track_map, imu_yaw(), control_dist_cm(), LINE_SENSOR_FORWARD_CM,
                 LINE_SENSOR_OFFSET_CM, LINE_SENSOR_PITCH_CM);
  line_map_align_limited = 0; line_exit_seen = 0;
  line_edge_reset();
  line_center_ms = line_prev_error = line_derivative = 0.0f;
  line_prev_valid = false;
  line_gap_start_cm = 0.0f;
  lost_ms = 0.0f;
  lost_err = 0.0f;
  lost_err_ms = HAL_GetTick();
  straight_ms = 0;
  corner_armed = true;
  corner_count = 0;
  lap_done = 0;
  dist_at_corner = 0.0f;
  dist_at_corner4 = 0.0f;
  pos_x = pos_y = 0.0f;
  pos_last_dist = 0.0f;
  lap_start_ms = HAL_GetTick();
  for (int i = 0; i < LAP_MAX; i++) lap_ms[i] = 0;
  obj_num = 0;
  obj_state = OBJ_IDLE;
  obj_last_seen_ms = HAL_GetTick();
  obj_last_seen_dist = 0.0f;
  obj_page = 0;
  /* 遥控: 重置组帧状态, 并把"最后一条指令时间"设为现在 ——
     否则一进模式七就会因为 (now - 0) > 500ms 直接判断联 */
  rc_parser_reset(&rc_parser);
  rc_neutral_count = 0;
  rc_wait = true;
  bt_rx_start();
  rc_lin = rc_rot = 0.0f;
  rc_last_ms = HAL_GetTick();
  control_reset();          /* 清积分/滤波: 车刚被搬动过也不受影响 */
  control_reset_dist();

  pid_head.kp = HEAD_KP; pid_head.ki = HEAD_KI; pid_head.kd = HEAD_KD;
  pid_head.ilim = HEAD_ILIM;  pid_head.out_lim = 90.0f;

  if (mode == MODE_TURN && p_turn_angle == 0) turn_finish(TURN_ERR_NONE);
}

static void do_recal(void) {
  /* 校准要 1 秒且要求车静止 —— 先把电机停死, 并让调试页在期间不再驱动电机,
     否则按着调试页的测试键做校准, 车会一边跑一边校, 零偏直接校歪 */
  control_stop();
  dbg_lock = true;
  ui_show("陀螺校准", "请勿移动小车", "约1秒", "");
  OLED_Refresh();
  imu_calibrate();
  finished = false;
  aborted = false;
  imu_fault = !imu_alive();     /* 校准都读不到数据 -> 直接报故障 */
  OLED_Clear();
  ui_invalidate();
}

/* ================= 按键 ================= */
static void handle_keys(void) {
  key_event_t e_run = keys_take_event(KEY_RUN);
  key_event_t e_mode = keys_take_event(KEY_MODE);

  if (e_run == KEY_EV_LONG) {          /* K5 长按 */
    if (running) {                     /* 运行中: 长按也必须能停车!
                                          否则按住超过 1 秒就不会产生短按事件,
                                          反而变成"按得越久越停不下来" */
      control_brake();
      running = false;
      finished = false;
      aborted = false;
    } else {
      do_recal();                      /* 空闲时: 重新校准陀螺零偏 */
    }
    return;
  }
  if (e_mode == KEY_EV_LONG) {         /* K1 长按: 进出调试页 */
    if (!running) {
      mode = (mode == MODE_DEBUG) ? MODE_HEADING : MODE_DEBUG;
      control_stop();                  /* 进出调试页都先停车, 别让电机带着转 */
      finished = false;
      aborted = false;
      imu_fault = false;               /* 进调试页就是为了看诊断信息 */
      OLED_Clear();
      ui_invalidate();
    }
    return;
  }

  if (running) {                       /* 运行中只认 K5 停止 */
    if (e_run == KEY_EV_SHORT) {
      control_brake();
      running = false;
      finished = false;
      aborted = false;
    }
    return;
  }

  if (e_mode == KEY_EV_SHORT) {        /* K1 短按: 切模式 (按住 K3"−" 时松开 K1 = 倒退) */
    if (mode == MODE_DEBUG) {
      /* 离开调试页必须显式停车: 调试页是"按着键直接给 PWM"的,
         松手前最后一次的占空比会一直留在电机上, 不处理的话切到别的
         模式之后轮子还在转, 而且没有任何地方会去清它 */
      control_stop();
      mode = MODE_HEADING;
    } else if (keys_pressed(KEY_DOWN)) {
      mode = (car_mode_t)((int)mode == 1 ? MODE_COUNT : (int)mode - 1);   /* 倒退, 1 之前回到 7 */
    } else {
      mode = (car_mode_t)((int)mode % MODE_COUNT + 1);   /* 1~7 循环 */
    }
    finished = false;
    aborted = false;
    imu_fault = false;
    OLED_Clear();
    ui_invalidate();
    return;
  }

  key_event_t e_up = keys_take_event(KEY_UP);
  key_event_t e_dn = keys_take_event(KEY_DOWN);
  key_event_t e_set = keys_take_event(KEY_SET);
  bool u = (e_up == KEY_EV_SHORT || e_up == KEY_EV_REPEAT);
  bool n = (e_dn == KEY_EV_SHORT || e_dn == KEY_EV_REPEAT);

  switch (mode) {
  case MODE_TURN:
    /* 0~360 共 13 档 (含 360): 360 度就是要转满一圈, 0 度是不转 */
    if (u) p_turn_angle = (p_turn_angle >= 360) ? 0 : p_turn_angle + 30;
    if (n) p_turn_angle = (p_turn_angle <= 0) ? 360 : p_turn_angle - 30;
    if (e_set == KEY_EV_SHORT) p_turn_dir = -p_turn_dir;
    if (e_run == KEY_EV_SHORT) start_run();
    break;
  case MODE_DIST:
    if (u) p_dist_cm = (p_dist_cm >= 100) ? 50 : p_dist_cm + 10;
    if (n) p_dist_cm = (p_dist_cm <= 50) ? 100 : p_dist_cm - 10;
    if (e_run == KEY_EV_SHORT) start_run();
    break;
  case MODE_VOFA:
    if (u) p_speed = (p_speed >= 120) ? 120 : p_speed + 5;
    if (n) p_speed = (p_speed <= 10) ? 10 : p_speed - 5;
    if (e_set == KEY_EV_LONG) {      /* 现场切换蓝牙波特率, 不用重新烧录 */
      bt_baud = (bt_baud == 115200u) ? 9600u : 115200u;
      bt_set_baud(bt_baud);
    }
    if (e_run == KEY_EV_SHORT) start_run();
    break;
  case MODE_LINE:
    if (u) p_laps = (p_laps >= 5) ? 1 : p_laps + 1;
    if (n) p_laps = (p_laps <= 1) ? 5 : p_laps - 1;
    if (e_run == KEY_EV_SHORT) start_run();
    break;
  case MODE_OBJECT:
    if (e_run == KEY_EV_SHORT) start_run();
    if (finished) {
      if (u) obj_page = (uint8_t)((obj_page + 1) % 3);
      if (n) obj_page = (uint8_t)((obj_page + 2) % 3);
    }
    break;
  case MODE_RC:
    if (e_run == KEY_EV_SHORT) start_run();   /* 启动/停止都靠 K5, 遥控只给方向 */
    break;
  case MODE_DEBUG:
    if (e_run == KEY_EV_SHORT) dbg_page ^= 1;      /* K5 短按: 切调试页的两屏 */
    if (e_set == KEY_EV_SHORT) bt_selftest();
    if (e_set == KEY_EV_LONG) {         /* K4 长按: 让模块复位重新广播 */
      diag_begin();
      bt_at_cmd(bt_baud, "AT+RESET", bt_resp, sizeof(bt_resp), 500);
      snprintf(bt_resp, sizeof(bt_resp), "已发复位");
      bt_show_until = HAL_GetTick() + 6000;
      diag_end();
    }
    break;                              /* 调试页不启动任务 */
  case MODE_HEADING:
    /* 巡航速度在模式一里调, 模式三/五/六共用这个值 */
    if (u) p_cruise = (p_cruise >= 60) ? 60 : p_cruise + 5;
    if (n) p_cruise = (p_cruise <= 10) ? 10 : p_cruise - 5;
    if (e_run == KEY_EV_SHORT) start_run();
    break;
  default:
    if (e_run == KEY_EV_SHORT) start_run();
    break;
  }
}

/* ================= 对外接口 ================= */
void app_init(void) {
  OLED_Init();
  OLED_Clear();
  ui_invalidate();
  ui_show("智能小车 自检中", "", "", "");
  OLED_Refresh();

  keys_init();
  line_init();
  line_edge_timer_init();   /* 1kHz 扫 8 路探头边沿 (直角弯进入角测量用) */
  control_init();
  hcsr04_init();
  vofa_init();
  soft_i2c_init();
  pwrlog_init();          /* 使能芯片自带的欠压检测器 (PVD 2.9V) */
  bt_rx_start();          /* 开串口接收中断 (模式七遥控用) */

  ui_show("智能小车 自检中", "MPU6050 ...", "", "");
  OLED_Refresh();
  bool ok = imu_init();
  bool alive = ok && imu_alive();      /* 认到了不等于读得动: 写配置/校准采样都可能失败 */
  char imumsg[28];   /* 注意: 汉字在 UTF-8 里占 3 字节, 显示只占 2 格 */
  if (!ok)          snprintf(imumsg, sizeof(imumsg), "MPU6050 未找到");
  else if (!alive)  snprintf(imumsg, sizeof(imumsg), "MPU6050 读数异常");
  else              snprintf(imumsg, sizeof(imumsg), "MPU6050 %02X", mpu_who);
  ui_show("智能小车 自检中", imumsg,
          alive ? "陀螺校准中 勿动" : "查PA15/PB3接线", "");
  OLED_Refresh();
  HAL_Delay(alive ? 800 : 1500);

  OLED_Clear();
  ui_invalidate();
  mode = MODE_HEADING;
  running = false;
  finished = false;
  aborted = false;
  /* 自检那几百毫秒里没有调用过 imu_update, 先读一次把"数据新鲜度"刷新,
     否则主循环刚开始的十几毫秒里 imu_alive() 会因为数据过期返回假 */
  imu_update(0.01f);
  control_stop();
}

/* 空闲任务: 跑完/停车后把事件日志落 Flash (掉电不丢, 方便事后用 SWD 读) */
uint8_t app_is_vofa_mode(void) { return (uint8_t)(mode == MODE_VOFA); }

/* 每次刹车都会被调用: 有新数据就落盘 (用户经常跑完直接断电, 只在
   "正常停车"存的话数据全丢)。只在有脏数据时写, 不是每个刹车都擦写。 */
void app_flush_logs(void) {
  if (ev_dirty) ev_save_flash();       /* 存完 ev_dirty 清零, 有新事件才重写 */
  if (acc_rec_done) {
    acc_rec_save();
    acc_rec_done = 0;                  /* 存完允许录下一段(内容总是最新那段) */
  }
}

void app_idle_tasks(void) {
  if (!running) app_flush_logs();
}

void app_loop(void) {
  uint32_t now = HAL_GetTick();

  if (now - t_key >= 10) {                      /* 按键 10ms */
    t_key = now;
    keys_scan();
    handle_keys();
  }

  if (mode == MODE_RC) rc_poll();       /* 遥控接收每次主循环处理，优先于100Hz控制 */

  if (now - t_ctrl >= 10) {                     /* 控制 10ms */
    float dt = (float)(now - t_ctrl) * 0.001f;
    t_ctrl = now;
    if (dt > 0.05f) dt = 0.05f;
    if (dt < 0.005f) dt = 0.005f;

    line_edge_sync(control_dist_cm());   /* 边沿扫描里程对齐主循环里程 */
    if (running) acc_rec_tick();          /* 加速段记录器(丢线触发后连录 0.64s) */
  if (running) {                       /* 分段计时: 看时间到底花在哪 */
    uint32_t ms = (uint32_t)(dt * 1000.0f + 0.5f);
    if (line_corner_active)       t_corner_ms += ms;
    else if (line_settle_active)  t_settle_ms += ms;
    else if (line_pull)           t_pull_ms += ms;
    else if (line_lost())         t_lost_ms += ms;
    else                          t_track_ms += ms;
  }
    imu_update(dt);
    hcsr04_tick();
    sonar_poll();

    /* 陀螺仪运行中掉线: 立刻停车报故障。
       航向已经不再更新了, 再跑下去方向保持/转向/计圈全是错的 ——
       宁可停车, 也不能拿一个越来越不准的旧航向继续跑。 */
    if (running && !imu_alive()) {
      control_brake();
      running = false;
      finished = true;
      aborted = true;
      imu_fault = true;
      run_end_ms = HAL_GetTick();
    }

    /* 调试器触发的供电波形记录: 电池供电、ST-Link 只接 SWD 的情况下,
       用这个把 3.3V 轨的抖动录下来 (详见 pwrlog.h) */
    if (pwrlog_req) {
      pwrlog_req = 0;
      diag_begin();                /* 记录期间车必须是停的, 做完也不自动恢复 */
      pwrlog_run();
      diag_end();
    }

    /* 调试器远程触发蓝牙自检 (方便电脑这边排查模块) */
    if (bt_test_req) { bt_test_req = 0; bt_selftest(); }
    if (bt_reset_req) {
      bt_reset_req = 0;
      control_stop();
      dbg_lock = true;
      bt_at_cmd(bt_baud, "AT+RESET", bt_resp, sizeof(bt_resp), 500);
      bt_set_baud(bt_baud);             /* bt_at_cmd 改过串口波特率, 恢复回去 */
      snprintf(bt_resp, sizeof(bt_resp), "已发复位");
      bt_show_until = HAL_GetTick() + 6000;
    }

    if (mode == MODE_DEBUG) {
      ctrl_debug();                             /* 调试页自己直接控电机 */
    } else {
      switch (mode) {
      case MODE_HEADING: ctrl_heading(dt); break;
      case MODE_TURN:    ctrl_turn(dt);    break;
      case MODE_DIST:    ctrl_dist(dt);    break;
      case MODE_VOFA:    ctrl_vofa(dt);    break;
      case MODE_LINE:    ctrl_line(dt);    break;
      case MODE_OBJECT:  ctrl_object(dt);  break;
      case MODE_RC:      ctrl_rc(dt);      break;
      default: break;
      }
      control_update(dt);                       /* 读编码器 + PI + 输出 PWM */
    }
  }

  /* 蓝牙上传 (VOFA+ FireWater 协议): 目标速度, 实际速度, 航向, 角速度
     模式四一直在发 (包括没启动时, 这样能先确认链路通不通)。
     VOFA_ALWAYS=1 时所有模式都发, 调循迹/方向保持的 PID 时很好用。 */
  if ((mode == MODE_VOFA || VOFA_ALWAYS) && (now - t_vofa >= VOFA_PERIOD_MS)) {
    t_vofa = now;
    vofa_send4(running ? (float)p_speed : 0.0f, control_lin(),
               imu_yaw(), imu_gyro_z());
  }

  if (now - t_vdd >= 500) {                     /* 供电电压 0.5 秒测一次 */
    t_vdd = now;
    vdd_now = vdd_read();
  }

  uint32_t ui_period = (mode == MODE_RC && running) ? 200u : 60u;
  if (now - t_ui >= ui_period) {              /* 遥控运行时降低 OLED 阻塞频率 */
    t_ui = now;
    ui_draw();
    OLED_Refresh();
  }
}
