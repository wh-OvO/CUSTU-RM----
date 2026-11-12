/**
  ******************************************************************************
  * @file    arm_joint.c
  * @brief   关节执行层：把"底座闭环"和"其余三路直通"藏在同一个接口后面
  ******************************************************************************
  * @note    **本文件只干两件事**
  *
  *          1) 位置型关节（J2/J3/J4）：转手交给 arm_pwm.c，一行逻辑都不加。
  *          2) 速度型关节（J1 底座）：跑位置闭环，输出的是**转速**而不是角度。
  *
  *          上层（插补器 / 协议层）永远只说"我要到多少度"，不关心底下怎么实现。
  *
  * @note    **底座闭环的控制律**
  *
  *              脉宽 = us_stop + 前馈 + Kp × 误差
  *
  *            误差 = wrap180(目标角 − 实测角)     ← 必须绕到 ±180，否则 359° 和 0°
  *                                                  之间会被算成"差 359 度"
  *
  *          为什么必须有前馈：插补器每 20ms 把目标往前推一点，底座在追一个
  *          **斜坡**。纯比例控制追斜坡必然有稳态误差 e = ω / Kp ——
  *          底座 180°/s、Kp = 3 时，运动中会稳定落后 60°，肉眼可见。
  *          前馈把"跟上这个速度该出多大力"直接算好，Kp 只负责收尾。
  *
  * @note    **为什么读不到角度就停转，而不是"保持上一次的脉宽"**
  *
  *          上一次的脉宽是个**非零转速**的指令。失去了角度反馈还继续发这个指令，
  *          底座就会一直转，转到线被拧断或者撞上什么东西为止。
  *          停下来是唯一安全的选择，同时点亮故障位让上位机看到。
  ******************************************************************************
  */

#include "arm_cfg.h"
#include "arm_pwm.h"
#include "arm_joint.h"
#include "as5600.h"

/* ---- 状态 ---- */
static float   s_tgt_deg[ARM_JOINT_NUM];   /* 每个关节的当前目标角（度） */
static float   s_ff_dps[ARM_JOINT_NUM];    /* 插补器给的目标速度（度/秒），前馈用 */
static uint8_t s_manual[ARM_JOINT_NUM];    /* 1 = 被 0x31 手动占用，闭环别抢 */

static uint8_t s_base_fault;               /* 1 = AS5600 读不到 */
static float   s_base_err;                 /* 最近一次闭环误差的绝对值（度），给状态位用 */

/* ------------------------------------------------------------------------- */
/* 角度归一化：把一个角度绕进 (-180, +180]                                    */
/* ------------------------------------------------------------------------- */
static float wrap180(float a)
{
  while (a > 180.0f)
  {
    a -= 360.0f;
  }
  while (a <= -180.0f)
  {
    a += 360.0f;
  }
  return a;
}

/* ------------------------------------------------------------------------- */
/* 底座实测角：把 AS5600 的原始读数换算到"机械坐标系"                          */
/*                                                                            */
/* 这是**全工程唯一**该读 AS5600 的地方。别在别的文件里再直接调                */
/* as5600_read_deg()，否则同一个角度会出现两种坐标系，调起来没完没了。         */
/*                                                                            */
/* 返回值和 g_arm_joint[ARM_JOINT_BASE].deg_min..deg_max 是**同一个坐标**：    */
/*     home(90°) → 报 ≈90     行程下端点 → 报 ≈0     上端点 → 报 ≈180          */
/* 所以 vel_tick() 里可以直接算 `s_tgt_deg[joint] - meas`，不需要再加减偏置。  */
/* ------------------------------------------------------------------------- */
static uint8_t base_read_deg(float *deg)
{
  const arm_joint_cfg_t *c = &g_arm_joint[ARM_JOINT_BASE];
  float raw    = 0.0f;
  float center;

  if (as5600_read_deg(&raw) == 0u)
  {
    return 0u;                       /* 读失败：*deg 保持不动，调用方自己判断 */
  }

  /* 行程中点。J1 = (0 + 180) / 2 = 90，正好就是 deg_home。 */
  center = 0.5f * ((float)c->deg_min + (float)c->deg_max);

  /* 机械角 = wrap180( SIGN × (原始角 − ZERO) ) + CENTER
   *
   *   减 ZERO    磁铁装在哪，AS5600 的零点就在哪，跟机械角度无关
   *   乘 SIGN    磁铁装反 / 舵机反向时两者方向相反，不乘就变正反馈
   *   绕 ±180    减完可能跑到 0~360 之外，绕回来才好算误差
   *   + CENTER   ★ 把 0 从"行程中点"平移回"机械角 0"
   *
   * ★ 少了最后这个 + CENTER 会怎样（2026-11-12 修掉的坑）：
   *   返回值变成"以行程中点为 0"，home 处报 0 而 s_tgt_deg[0] = 90，
   *   误差恒定 90° → u 限到 300µs → 底座全速转出 90° 才停 → 顶死。
   *
   * ★ 反过来，如果绕的是 0 而不是 CENTER，机械行程 0~180 就骑在 ±180
   *   接缝上：物理 180.4° 绕成 -179.6°，限位判据和 hold_position() 都会错。
   *   这两件事一起解决，就靠下面这一行。 */
  *deg = wrap180(ARM_BASE_ENC_SIGN * (raw - ARM_BASE_ENC_ZERO_DEG)) + center;

  return 1u;
}

/* ------------------------------------------------------------------------- */
/* 速度型关节的闭环：一拍                                                      */
/* ------------------------------------------------------------------------- */
static void vel_tick(uint8_t joint)
{
  const arm_joint_cfg_t *c = &g_arm_joint[joint];
  float    meas = 0.0f;
  float    err;
  float    u;
  float    u_ff = 0.0f;
  uint16_t us_stop;

  if (s_manual[joint] != 0u)
  {
    return;                          /* 手动模式：脉宽由 0x31 说了算，闭环让开 */
  }

  /* ---- ① 读角度（已换算到机械坐标系）。读不到就停转，别猜 ---- */
  if (base_read_deg(&meas) == 0u)
  {
    s_base_fault = 1u;
    us_stop = (c->us_stop != 0u) ? c->us_stop : SERVO_US_MID;
    (void)arm_set_us(joint, us_stop);
    return;
  }
  s_base_fault = 0u;

  /* ---- ② 误差，绕到 ±180 ---- */
  err        = wrap180(s_tgt_deg[joint] - meas);
  s_base_err = (err >= 0.0f) ? err : -err;

  /* ---- ③ 软件限位：已经在端点上、还想继续往外走 → 停 ----
   * 这是"走线保护"：360° 回转时线缆会扭，deg_min/deg_max 就是给线留的余量。 */
  if (((meas <= (float)c->deg_min) && (err < 0.0f)) ||
      ((meas >= (float)c->deg_max) && (err > 0.0f)))
  {
    us_stop = (c->us_stop != 0u) ? c->us_stop : SERVO_US_MID;
    (void)arm_set_us(joint, us_stop);
    return;
  }

  /* ---- ④ 前馈 + 比例 ---- */
#if ARM_BASE_FF_ENABLE
  u_ff = s_ff_dps[joint] * c->us_per_dps;   /* 度/秒 × µs每度每秒 = µs */
#else
  u_ff = 0.0f;                              /* 还没标定 us_per_dps，先只用 P */
#endif

  u = u_ff + (ARM_BASE_KP_US_PER_DEG * err);

  if (u >  ARM_BASE_U_MAX_US) { u =  ARM_BASE_U_MAX_US; }
  if (u < -ARM_BASE_U_MAX_US) { u = -ARM_BASE_U_MAX_US; }

  /* 死区：误差很小就彻底停转。连续舵机在零速点附近会嗡嗡响、发热 */
  if (s_base_err < ARM_BASE_DEADBAND_DEG)
  {
    u = 0.0f;
  }

  /* ---- ⑤ 写脉宽：us_stop ± u ---- */
  us_stop = (c->us_stop != 0u) ? c->us_stop : SERVO_US_MID;
  (void)arm_set_us(joint, (uint16_t)((float)us_stop + u));
}

/* ------------------------------------------------------------------------- */
/* 对外                                                                      */
/* ------------------------------------------------------------------------- */
void arm_joint_init(void)
{
  uint8_t j;

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    s_tgt_deg[j] = (float)g_arm_joint[j].deg_home;
    s_ff_dps[j]  = 0.0f;
    s_manual[j]  = 0u;
  }

  s_base_fault = 0u;
  s_base_err   = 0.0f;

  if (as5600_init() == 0u)
  {
    /* 探测不到编码器，只有"当前配置里真有速度型关节"时才算故障。
     * 现在 J1 还是 POS，所以这里不该把 bit4 点亮，其余三路照常工作。 */
    for (j = 0u; j < ARM_JOINT_NUM; j++)
    {
      if (g_arm_joint[j].kind == ARM_JOINT_VEL)
      {
        s_base_fault = 1u;
        break;
      }
    }
  }
}

void arm_joint_set_traj(uint8_t joint, float deg, float v_dps)
{
  if (joint >= (uint8_t)ARM_JOINT_NUM)
  {
    return;
  }

  s_tgt_deg[joint] = deg;
  s_ff_dps[joint]  = v_dps;

  if (g_arm_joint[joint].kind == ARM_JOINT_POS)
  {
    /* 位置型：立即生效，函数返回时舵机已经在动了 */
    (void)arm_set_deg(joint, deg);
  }
  /* 速度型：只记下目标，等 arm_joint_tick() 每一拍去追 */
}

void arm_joint_set_deg(uint8_t joint, float deg)
{
  arm_joint_set_traj(joint, deg, 0.0f);
}

float arm_joint_get_deg(uint8_t joint)
{
  float d = 0.0f;

  if (joint >= (uint8_t)ARM_JOINT_NUM)
  {
    return 0.0f;
  }

  if (g_arm_joint[joint].kind == ARM_JOINT_VEL)
  {
    if (base_read_deg(&d) != 0u)
    {
      return d;                      /* 真实测量值（已是机械坐标系） */
    }
    return s_tgt_deg[joint];         /* 读不到：退回目标值，至少不返回垃圾 */
  }

  return arm_get_deg(joint);         /* 位置型：CCR 反算 */
}

uint16_t arm_joint_get_us(uint8_t joint)
{
  return arm_get_us(joint);
}

void arm_joint_tick(uint32_t now)
{
  uint8_t j;

  (void)now;                         /* 目前不用时间戳；将来要做速度估算时会用到 */

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    if (g_arm_joint[j].kind == ARM_JOINT_VEL)
    {
      vel_tick(j);
    }
  }
}

void arm_joint_set_manual(uint8_t joint, uint8_t on)
{
  if (joint >= (uint8_t)ARM_JOINT_NUM)
  {
    return;
  }

  s_manual[joint] = (on != 0u) ? 1u : 0u;

  if (s_manual[joint] != 0u)
  {
    s_ff_dps[joint] = 0.0f;          /* 手动时不留前馈，免得交还闭环时突然一冲 */
  }
}

void arm_joint_hold_position(void)
{
  uint8_t j;

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    if (g_arm_joint[j].kind == ARM_JOINT_VEL)
    {
      float d = 0.0f;

      /* 把目标改成"我现在实际在哪"，闭环于是原地不动 */
      if (base_read_deg(&d) != 0u)
      {
        s_tgt_deg[j] = d;
      }
      s_ff_dps[j] = 0.0f;
    }
    /* 位置型不需要处理：CCR 不被改写，舵机自己就保持住了 */
  }
}

uint8_t arm_joint_base_fault(void)
{
  return s_base_fault;
}

uint8_t arm_joint_moving(void)
{
  /* 本工程只有底座一个是速度型关节，所以只需要看它。
   * 将来如果加了第二个闭环关节，这里要改成遍历，别只判底座。 */
  if (g_arm_joint[ARM_JOINT_BASE].kind != ARM_JOINT_VEL)
  {
    return 0u;                       /* 不是闭环关节，没有"没到位"这回事 */
  }
  if (s_base_fault != 0u)
  {
    return 0u;                       /* 故障了就不再报"运动中"，让 bit2/bit4 去说 */
  }

  return (uint8_t)((s_base_err > ARM_MOVING_STILL_DEG) ? 1u : 0u);
}
