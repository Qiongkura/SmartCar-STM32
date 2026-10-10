#include "control.h"
#include "motor.h"
#include "board.h"
#include "line.h"
#include "stm32f1xx_hal.h"
#include <math.h>

/* 每 (度/秒) 的旋转对应多少 cm/s 的左右轮速差 */
#define ROT_TO_DIFF  (3.14159265f * WHEEL_BASE_CM / 360.0f)

static pid_ctrl_t pi_l, pi_r;
static bool  stopped = true;   /* true = 已经停车/刹车, control_update 不再输出 PWM */
static float cmd_lin = 0.0f, cmd_rot = 0.0f;
/* 前进目标斜坡按轮分开。加速时左轮实际加速更快 -> 车头往右甩,
   所以把左轮斜坡压成 ACC_SLEW_L 倍。实测 0.85 还压不住(加速仍偏右),
   收到 0.75。压过头(加速时车头往左甩)就往回调。 */
float acc_slew_l = ACC_SLEW_L;   /* 起跑时由 app.c 按模式设定 */
float sync_trim = 0.0f;          /* 轮速同步修正量 (慢积分) */
static float sync_i = 0.0f;      /* 同步积分器 */
static float sync_lin_prev = 0.0f; /* 上拍目标均值 (算加速度用) */
static float ramp_l = 0.0f, ramp_r = 0.0f;   /* 限幅后的前进目标(左右各一份) */
static float meas_l = 0.0f, meas_r = 0.0f;
static float dist_cm = 0.0f;
static bool spin_mode = false;
static bool inner_pivot_mode = false;   /* 内轮枢轴: 绕内侧轮子转 */
static float spin_origin_cm = 0.0f;
static bool  wheels_mode = false;                 /* 坦克帧: 直控左右轮 */
static float cmd_wl = 0.0f, cmd_wr = 0.0f;        /* 左右轮目标 cm/s */
/* ---------------- 起步黑匣子 ----------------
   start_run 后, 从第一条运动指令开始记 200 拍(约2秒), 每拍 8 个 int16:
   [左实测, 右实测, 左目标, 右目标, 左输出, 右输出, 线偏差x100, 压线路数]。
   实测/目标单位 0.01cm/s, 输出为 PWM 计数(0~3600)。
   用来离线分析起步同步、过弯出线的线位置等, SWD 读。 */
#define BB_N    200
#define BB_FLD  8
int16_t bb_buf[BB_N * BB_FLD];
uint16_t bb_len = 0;        /* 已记录拍数, 封顶 BB_N (环形, 永远是最近2秒) */
static uint16_t bb_wr = 0;  /* 环形写指针 */
static bool bb_on = false;
static bool bb_saved = false;
/* 黑匣子落到 flash 最后一页区 (0x0800F000, 4KB): 停车瞬间自动保存,
   掉电不丢。SWD 直接读这段 flash 即可, 格式: [0]=0xA55A [1]=拍数 [2]=写指针, 之后是数据 */
#define BB_FLASH_ADDR 0x0800F000u

static void bb_save_flash(void);     /* 前向声明 */
static void bb_save_if_any(void) {
  /* 必须在 control_reset 清 bb_len 之前调用 (brake/stop 开头), 否则永远存不上 */
  if (!bb_saved && bb_len > 0) {
    bb_saved = true;
    bb_save_flash();
  }
}

static void bb_save_flash(void) {
#if DIAG_FLASH_LOGS
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef er;
  uint32_t err = 0;
  er.TypeErase = FLASH_TYPEERASE_PAGES;
  er.PageAddress = BB_FLASH_ADDR;
  er.NbPages = 4;
  if (HAL_FLASHEx_Erase(&er, &err) == HAL_OK) {
    uint16_t *src = (uint16_t *)bb_buf;
    uint32_t halfwords = (BB_N * BB_FLD) + 3;   /* 数据 + 3 个头 */
    for (uint32_t i = 0; i < halfwords; i++) {
      uint16_t v;
      if (i == 0)        v = 0xA55A;
      else if (i == 1)   v = bb_len;
      else if (i == 2)   v = bb_wr;
      else               v = src[i - 3];
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, BB_FLASH_ADDR + i * 2, v);
    }
  }
  HAL_FLASH_Lock();
#endif
}

/* 起步助推只在原地转生效: 目标速度不为0但轮子实测没动起来时,
   立即把输出垫到上限(左60%/右45%), 动起来立即交回PI。
   直行起步扭矩由下面的衰减式前馈负责, 不做"没动就猛推"。 */
#define START_ASSIST_MAX_L  0.3f
#define START_ASSIST_MAX_R  0.3f
#define SPIN_DUTY_MAX      0.65f
/* 低速静摩擦前馈(所有模式持续生效): 目标速度不为0时, 按"实测速度越低补得越多"
   给一笔扭矩, 在 STICT_FADE_CM_S 处线性衰减到 0。
   起步(meas≈0)给满额 -> 两轮同时有力突破静摩擦;
   低速原地转/转弯(轮子目标只有几cm/s)持续有力 -> 不会顶不开;
   高速自动退出 -> 不影响正常行驶。
   左轮静摩擦实测比右轮大, 满额给得更高。 */
/* 12% 对称小额: 原来 30%/25% 的瞬间大额前馈, 每次起步猛踢一脚、
   速度上来后前馈消失, 占空比从 45% 塌回 20% —— 就是"出弯抽搐",
   且左右不对称会踢歪。静摩擦交给速度环积分平滑地做。 */
#define STICT_FF_L       (0.12f * PWM_MAX)
#define STICT_FF_R       (0.12f * PWM_MAX)
/* 衰减速度从 12 收到 6: 原来在弯道里内轮(命令-2)被补偿顶着实际转-6.6,
   PI 要一秒才扳得回来, 整个弯的旋转中心都是错的, 出弯必偏右。
   收到 6 之后补偿只管 0~6cm/s 的起步破摩, 弯道里轮速真实受控。 */
#define STICT_FADE_CM_S   6.0f
/* 左右轮各自累计的里程 (cm)。除了标定, 还能用来独立验证陀螺:
   原地转 θ 度时, 两个轮子的行程差 = π×轮距×θ/360。
   转 360° 时差值应该是 75.4cm —— 如果实际多转了 27°, 差值会多 5.7cm,
   用这个就能判断"是陀螺测少了"还是"车真的转多了"。 */
float dist_l_cm = 0.0f, dist_r_cm = 0.0f;

/* ---------------- PID ---------------- */
float pid_step(pid_ctrl_t *p, float err, float dt) {
  float d = (dt > 0.0001f) ? (err - p->prev) / dt : 0.0f;
  p->prev = err;
  /* 反饱和: 输出已经顶到限幅时不再累加积分。
     否则饱和期间积分一直涨, 退出饱和后要慢慢往回吐, 表现为
     "被碰一下/丢线一段之后车朝一个方向持续偏"(航向环实测如此)。 */
  float p_term = p->kp * err + p->kd * d;
  bool sat = (p->out_lim > 0.0f) &&
             ((p->ki * p->i + p_term) > p->out_lim || (p->ki * p->i + p_term) < -p->out_lim);
  if (!sat) {
    p->i += err * dt;
    if (p->i >  p->ilim) p->i =  p->ilim;
    if (p->i < -p->ilim) p->i = -p->ilim;
  }
  float out = p_term + p->ki * p->i;
  if (p->out_lim > 0.0f) {
    if (out >  p->out_lim) out =  p->out_lim;
    if (out < -p->out_lim) out = -p->out_lim;
  }
  return out;
}

void pid_reset(pid_ctrl_t *p) {
  p->i = 0.0f;
  p->prev = 0.0f;
}

void control_init(void) {
  pi_l.kp = SPD_KP;  pi_l.ki = SPD_KI;  pi_l.kd = 0.0f;
  pi_l.ilim = SPD_ILIM;  pi_l.out_lim = (float)PWM_MAX;
  pi_r = pi_l;
  pid_reset(&pi_l);
  pid_reset(&pi_r);
  motor_init();
}

void control_set_cmd(float lin_cm_s, float rot_deg_s) {
  spin_mode = false;
  inner_pivot_mode = false;
  /* 停止->运动 的瞬间把实测速度清零: 刹车期间 meas 还留着上一段(枢轴转)
     的残值, 不清的话 PI 和同步项会拿残值算出巨大的修正 -> 起步抽搐 */
  if (stopped && (lin_cm_s != 0.0f || rot_deg_s != 0.0f)) {
    meas_l = 0.0f;
    meas_r = 0.0f;
  }
  wheels_mode = false;
  if (lin_cm_s != 0.0f || rot_deg_s != 0.0f) stopped = false;   /* 有新的运动指令才解锁 */
  if (lin_cm_s >  SPD_MAX_CM_S) lin_cm_s =  SPD_MAX_CM_S;
  if (lin_cm_s < -SPD_MAX_CM_S) lin_cm_s = -SPD_MAX_CM_S;
  if (rot_deg_s >  ROT_MAX_DPS) rot_deg_s =  ROT_MAX_DPS;
  if (rot_deg_s < -ROT_MAX_DPS) rot_deg_s = -ROT_MAX_DPS;
  cmd_lin = lin_cm_s;
  cmd_rot = rot_deg_s;
}

/* 坦克帧: 左右轮目标直接给到速度环, 不经过 lin/rot 混控。
   一侧给 0 时该轮被速度环钉在原地, 另一侧照常转 -- 混控协议做不到的事 */
void control_set_wheels(float l_cm_s, float r_cm_s) {
  spin_mode = false;
  wheels_mode = true;
  if (l_cm_s != 0.0f || r_cm_s != 0.0f) stopped = false;
  if (l_cm_s >  SPD_MAX_CM_S) l_cm_s =  SPD_MAX_CM_S;
  if (l_cm_s < -SPD_MAX_CM_S) l_cm_s = -SPD_MAX_CM_S;
  if (r_cm_s >  SPD_MAX_CM_S) r_cm_s =  SPD_MAX_CM_S;
  if (r_cm_s < -SPD_MAX_CM_S) r_cm_s = -SPD_MAX_CM_S;
  cmd_wl = l_cm_s;
  cmd_wr = r_cm_s;
}

static float spin_gain_l = 1.0f;   /* 本次原地转的左轮增益 (1.0 = 对称) */

void control_set_spin_ex(float rot_deg_s, float gain_l) {
  bool entering = !spin_mode;
  control_set_cmd(0.0f, rot_deg_s);
  if (entering) spin_origin_cm = dist_cm;  /* 位移修正以本次转弯起点为基准 */
  spin_mode = true;
  spin_gain_l = gain_l;
}

void control_set_spin(float rot_deg_s) { control_set_spin_ex(rot_deg_s, 1.0f); }

/* 差速转: 把"外轮 v_out / 内轮 v_in(带符号)"折算成 (lin, rot) 走普通速度环。
   不走 spin 通道, 所以不受原地转的平移修正影响, 两轮速度严格按给定值。 */
void control_set_arc(float outer_cm_s, float inner_cm_s, int dir) {
  float lin  = (outer_cm_s + inner_cm_s) * 0.5f;
  float diff = (outer_cm_s - inner_cm_s) * 0.5f;
  float rot  = (dir < 0) ? -(diff / ROT_TO_DIFF) : (diff / ROT_TO_DIFF);
  control_set_cmd(lin, rot);
  /* 进弯必须"钉住"前进目标: 巡航 42 入弯时按 50cm/s² 的限幅要 0.7s 才减
     到弯速, 期间滑出去 ~17cm, 弯直接冲出去。立即到位, 靠轮速环急刹。 */
  ramp_l = ramp_r = lin;
}

/* 内轮枢轴转: 绕内侧轮子转 (内轮目标 0, 外轮 = ω×轮距)。
   转轴在轮轴上、距车心 半轮距(6cm); 探头在车头前方 d, 转完后探头
   离线的横向偏移 = |d - 6| ≈ 1cm, 正好落在线上。
   对称对转是绕车心, 探头会偏出线 d≈7~10cm —— 那才是"出弯偏右/偏左"的根。 */
void control_set_inner_pivot(float rot_deg_s) {
  control_set_cmd(0.0f, rot_deg_s);
  inner_pivot_mode = true;
}

void control_set_pivot(float rot_deg_s) {
  bool entering = !spin_mode;
  control_set_cmd(0.0f, rot_deg_s);
  if (entering) spin_origin_cm = dist_cm;
  spin_mode = true;
}

static float start_boost(float out, float target, float measured, bool with_cap, float max_duty) {
  if ((target > 0.5f || target < -0.5f) && measured > -0.7f && measured < 0.7f) {
    float floor = max_duty * PWM_MAX;
    if (target > 0 && out >= 0 && out < floor) out = floor;
    if (target < 0 && out <= 0 && out > -floor) out = -floor;
  }
  if (with_cap) {
    /* 只有原地转需要封顶(不需要全力); 直行时 PI 输出该多大就多大 */
    float cap = SPIN_DUTY_MAX * PWM_MAX;
    if (out > cap) out = cap;
    if (out < -cap) out = -cap;
  }
  return out;
}

static float slew(float now, float target, float step) {
  float d = target - now;
  if (d >  step) d =  step;
  if (d < -step) d = -step;
  return now + d;
}

void control_update(float dt) {
  /* 1. 实测速度 + 累计里程 (停车后也照常做, 界面显示和循线里程都要用) */
  float dl = (float)motor_read_delta(MOT_LEFT)  / ENCODER_CM_L;
  float dr = (float)motor_read_delta(MOT_RIGHT) / ENCODER_CM_R;
  float vl = (dt > 0.0001f) ? dl / dt : 0.0f;
  float vr = (dt > 0.0001f) ? dr / dt : 0.0f;
  meas_l += SPD_LPF * (vl - meas_l);
  meas_r += SPD_LPF * (vr - meas_r);

  dist_cm += (dl + dr) * 0.5f;
  dist_l_cm += dl;
  dist_r_cm += dr;

  /* 2. 停车/刹车之后: 连 PI 都不要跑。
     否则车停着被人搬动/推动时, 编码器脉冲会一直往积分项里灌,
     下一次启动的瞬间输出会突然冲一下 (而且停车时输出 PWM 还会覆盖
     control_brake()/control_stop() 的效果)。 */
  if (stopped) return;

  /* 3. 目标速度做加速度限幅 (旋转不平滑, 转向要快), 左右轮各自斜坡 */
  float tgt_l, tgt_r;
  float diff = 0.0f;                 /* 坦克模式下 spin_mode 恒 false, 用不到 */
  if (wheels_mode) {
    /* 坦克帧: 两轮各追各的目标, 不做混控 */
    ramp_l = slew(ramp_l, cmd_wl, ACC_LIMIT * acc_slew_l * dt);
    ramp_r = slew(ramp_r, cmd_wr, ACC_LIMIT * dt);
    tgt_l = ramp_l;
    tgt_r = ramp_r;
  } else {
    /* 原地转(spin_mode)时左右轮对称起步, 都用 1.0 斜坡; 混控模式才压左轮 */
    float slew_l = spin_mode ? 1.0f : acc_slew_l;
    ramp_l = slew(ramp_l, cmd_lin, ACC_LIMIT * slew_l * dt);
    ramp_r = slew(ramp_r, cmd_lin, ACC_LIMIT * dt);
    diff = cmd_rot * ROT_TO_DIFF;
    tgt_l = ramp_l + diff;
    tgt_r = ramp_r - diff;
  }

  /* 加速同步项已删除: 它是"实测速度差->比例修正目标"的闭环, 有相位滞后,
     会自激振荡(起步/转弯后的抽搐), 增益怎么调都不干净。
     两轮的电机差异改由前馈开环处理: SPD_KFF_L/R 分开 + ACC_SLEW_L 斜坡分开。 */

  if (inner_pivot_mode) {
    /* 内轮枢轴: 绕内侧轮子转。cmd_rot > 0 = 右转 -> 绕右轮, 左轮前进;
       cmd_rot < 0 = 左转 -> 绕左轮, 右轮后退。 */
    float w = fabsf(cmd_rot) * 0.0174533f;      /* °/s -> rad/s */
    float v = w * WHEEL_BASE_CM;                /* 外轮线速度 cm/s */
    /* 左转: 绕左轮, 右轮【前进】(东侧向前推, 车头才摆向西 = 左);
       右转: 绕右轮, 左轮前进。差速运动学: ω ∝ (v_r - v_l), 左转要 v_r > v_l */
    if (cmd_rot > 0.0f) { tgt_l = v;    tgt_r = 0.0f; }
    else                { tgt_l = 0.0f; tgt_r = v;    }
    ramp_l = tgt_l; ramp_r = tgt_r;
  } else if (spin_mode) {
    /* 原地旋转(模式二/循线过弯)要求两轮速度对称: tgt_l ≈ +diff, tgt_r ≈ -diff。
       这里只保留一点点平移修正压漂移, 上限收到 ±0.1|diff| ——
       修正量直接叠加在两轮目标上, 给大了就会把两轮转速拉歪
       (上限 0.5|diff| 时会出现一个轮子只有另一个一半速度)。 */
    float correction = 0.6f * (dist_cm - spin_origin_cm) + 0.25f * (meas_l + meas_r);
    float limit = (diff < 0 ? -diff : diff) * 0.1f;
    if (correction > limit) correction = limit;
    if (correction < -limit) correction = -limit;
    tgt_l = diff - correction;
    tgt_r = -diff - correction;
    tgt_l *= spin_gain_l;              /* 左轮增益 (弯道实测左轮偏快) */
  }

  /* 3.5 轮速闭环同步 (只在直线): 两轮实测速度差做慢积分, 消掉电机/摩擦的
     稳态不对称 —— 开环斜坡补偿(ACC_SLEW_L)随电池/地面漂移, 这个不漂。
     只用慢积分不用比例(历史教训: 高增益同步环和轮速环打架会振荡),
     转向/原地转/坦克模式时冻结不积分, 修正量限幅。 */
  if (!spin_mode && !inner_pivot_mode && !wheels_mode && fabsf(cmd_rot) < 3.0f) {   /* 3°/s: 航向保持的小修正不被同步抢 */
    /* 加速/减速瞬态不积分: 那段两轮差速是斜坡补偿(ACC_SLEW)的正常产物,
       积进去的话每次提速都会存下一笔"左轮偏快"的常值修正 (实测如此),
       加速结束后两轮等速、误差消失, 但存量还在 -> 左轮持续偏快 */
    float lin_avg = 0.5f * (ramp_l + ramp_r);
    float lin_acc = fabsf(lin_avg - sync_lin_prev) / (dt > 0.001f ? dt : 0.001f);
    sync_lin_prev = lin_avg;
    if (lin_acc < 30.0f) {               /* 只有稳速时才积分 */
      sync_i += (meas_l - meas_r) * dt * KI_SYNC;
      if (sync_i >  SYNC_MAX) sync_i =  SYNC_MAX;
      if (sync_i < -SYNC_MAX) sync_i = -SYNC_MAX;
    } else {
      sync_i *= 0.90f;                   /* 瞬态期间把存量慢慢吐掉 */
    }
    if (KI_SYNC < 0.01f) sync_i = 0.0f;  /* KI=0 即彻底关闭 */
    sync_trim = sync_i;
  } else {
    /* 转向/转弯时必须完全撤掉修正量: 只冻结的话, 它会和航向控制抢差速
       (实测: 循线纠偏被拽住, 线回不到中间, 只降速不回中) */
    sync_lin_prev = 0.5f * (ramp_l + ramp_r);
    sync_trim = 0.0f;
  }
  tgt_l -= sync_trim;
  tgt_r += sync_trim;

  /* 4. 轮速 PI + 前馈 */
  float old_i_l = pi_l.i, old_i_r = pi_r.i;
  float out_l = SPD_KFF_L * tgt_l + pid_step(&pi_l, tgt_l - meas_l, dt);
  float out_r = SPD_KFF_R * tgt_r + pid_step(&pi_r, tgt_r - meas_r, dt);

  /* 低速静摩擦前馈: 目标不为0才给, 按 sign(目标) 方向, 随实测速度线性退场 */
  if (tgt_l > 0.05f || tgt_l < -0.05f) {   /* 0.05: 极小的微调目标也要给破摩助力,
                                              否则 0.8cm/s 的微调轮子根本推不动 */
    float f = 1.0f - fabsf(meas_l) / STICT_FADE_CM_S;
    if (f < 0.0f) f = 0.0f;
    out_l += (tgt_l > 0.0f ? 1.0f : -1.0f) * f * STICT_FF_L;
  }
  if (tgt_r > 0.05f || tgt_r < -0.05f) {
    float f = 1.0f - fabsf(meas_r) / STICT_FADE_CM_S;
    if (f < 0.0f) f = 0.0f;
    out_r += (tgt_r > 0.0f ? 1.0f : -1.0f) * f * STICT_FF_R;
  }

  /* 5. 静摩擦补偿: 想动但 PWM 太小的话垫到 MIN_PWM */
  if (tgt_l > 0.5f && out_l > 0.0f && out_l < MIN_PWM) out_l = MIN_PWM;
  if (tgt_l < -0.5f && out_l < 0.0f && out_l > -MIN_PWM) out_l = -MIN_PWM;
  if (tgt_r > 0.5f && out_r > 0.0f && out_r < MIN_PWM) out_r = MIN_PWM;
  if (tgt_r < -0.5f && out_r < 0.0f && out_r > -MIN_PWM) out_r = -MIN_PWM;

  if (spin_mode) {
    /* 饱和时停止同方向积分增长, 避免堵转后积累很大的驱动力。 */
    float cap = SPIN_DUTY_MAX * PWM_MAX;
    if ((out_l >= cap && tgt_l > meas_l) || (out_l <= -cap && tgt_l < meas_l)) pi_l.i = old_i_l;
    if ((out_r >= cap && tgt_r > meas_r) || (out_r <= -cap && tgt_r < meas_r)) pi_r.i = old_i_r;
    out_l = start_boost(out_l, tgt_l, meas_l, true,  START_ASSIST_MAX_L);
    out_r = start_boost(out_r, tgt_r, meas_r, true,  START_ASSIST_MAX_R);
  }
  /* 直行/循迹: 起步扭矩由 START_I_L/R 积分预载负责, 不做"没动就猛推" */
  if (bb_on) {
    int16_t *r = &bb_buf[bb_wr * BB_FLD];
    r[0] = (int16_t)(meas_l * 100.0f);
    r[1] = (int16_t)(meas_r * 100.0f);
    r[2] = (int16_t)(tgt_l * 100.0f);
    r[3] = (int16_t)(tgt_r * 100.0f);
    r[4] = (int16_t)out_l;
    r[5] = (int16_t)out_r;
    r[6] = (int16_t)(line_error() * 100.0f);
    r[7] = (int16_t)line_count();
    bb_wr = (bb_wr + 1) % BB_N;
    if (bb_len < BB_N) bb_len++;
  }
  motor_set_duty(MOT_LEFT,  out_l / (float)PWM_MAX);
  motor_set_duty(MOT_RIGHT, out_r / (float)PWM_MAX);
}

void control_reset(void) {
  spin_mode = false;
  wheels_mode = false;
  cmd_wl = 0.0f;
  cmd_wr = 0.0f;
  /* 清掉控制状态, 但不动电机 —— 每次 start_run() 之前调一次,
     保证新的一轮是从"零目标、零积分、零滤波"开始的 */
  cmd_lin = 0.0f;
  cmd_rot = 0.0f;
  ramp_l = 0.0f;
  ramp_r = 0.0f;
  /* 注意: 不清 meas_l/meas_r! 清了的话"刹停等速度降下来"的判断
     会在第一拍就通过 (读到的永远是 0), 车会带着惯性直接进枢轴转 */
  pid_reset(&pi_l);
  pid_reset(&pi_r);
  sync_i = 0.0f;
  inner_pivot_mode = false;
  spin_gain_l = 1.0f;
  bb_len = 0;
  bb_wr = 0;
  /* bb_saved 不再复位: 每次上电只落一次盘(第一个弯道刹停时)。
     原来每个弯的刹停都阻塞写 Flash ~150ms, 一圈 4 次;
     弯道过程数据现在由事件日志(独立 Flash 页)承担 */
  bb_on = true;                        /* 重新武装黑匣子: 环形记录最近2秒 */
}

extern void app_flush_logs(void);

void control_stop(void) {
  /* 先停再写 Flash (同 control_brake): 写 Flash 阻塞一百多毫秒,
     放在前面的话这段时间电机还在按旧 PWM 转, 车会多滑出去一段 */
  stopped = true;
  motor_coast(MOT_LEFT);
  motor_coast(MOT_RIGHT);
  bb_save_if_any();              /* 落盘(会阻塞), control_reset 会清 RAM 记录 */
  control_reset();
  app_flush_logs();              /* 事件日志跟着落盘: 断电也不丢 */
}

void control_brake_fast(void) {
  stopped = true;
  motor_brake(MOT_LEFT);
  motor_brake(MOT_RIGHT);
  control_reset();
}

void control_brake(void) {
  /* 先立即制动, 再写 Flash: 写 Flash 要阻塞一百多毫秒, 放在前面的话
     这段时间里 PWM 还在按旧值输出, 车会带着惯性冲出去 */
  stopped = true;
  motor_brake(MOT_LEFT);
  motor_brake(MOT_RIGHT);
  bb_save_if_any();              /* 落盘(会阻塞), 此时电机已经刹住 */
  control_reset();
  app_flush_logs();              /* 事件日志跟着落盘 */
}

float control_speed_l(void) { return meas_l; }
float control_speed_r(void) { return meas_r; }
float control_lin(void)     { return (meas_l + meas_r) * 0.5f; }
float control_dist_cm(void) { return dist_cm; }
void  control_reset_dist(void) { dist_cm = 0.0f; dist_l_cm = 0.0f; dist_r_cm = 0.0f; }
