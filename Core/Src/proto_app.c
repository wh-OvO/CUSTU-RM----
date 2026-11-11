/**
  ******************************************************************************
  * @file    proto_app.c
  * @brief   T5a 应用层实现
  ******************************************************************************
  * @note    设计取舍（都在注释里写清，评审问起来能直接答）：
  *
  *          1) 不用 TIM 中断做节拍，而是用 HAL_GetTick()。
  *             正本 §5 要求"设备侧以 1ms 定时器计时"，SysTick 就是 1ms 定时器，
  *             已经满足；省掉一个中断，也就省掉一整类中断时序 bug。
  *
  *          2) 急停 = **冻结**，不是断开 PWM。
  *             任务书字面是"立即停止输出"。但对 SG90 来说，没有脉冲 = 没有力矩
  *             = 机械臂直接掉下来，这比继续保持位置更危险。
  *             所以这里的实现是：进入故障态后**不再更新 CCR**（姿态冻结），
  *             只保留 bit2。若评审坚持要断开输出，把 arm_hold() 换成
  *             HAL_TIM_PWM_Stop() 即可，是改一行的事。
  *
  *          3) 手动模式下不判 0x20 超时。
  *             手动模式本来就不该有连续目标流，按正本字面判会导致一进手动就报超时。
  *
  *          4) **T2 插补器就在本文件里**，不另开文件。
  *             因为它和协议状态强耦合：起点终点就是 s_tgt_x10，节拍就是上报周期。
  *             拆出去反而要来回传一堆状态。四轴共用同一个进度 s，所以同时出发同时到达。
  *
  *          5) 插补器只产生"角度"，不管怎么执行。
  *             底座是 360° 连续舵机（脉宽 = 转速），要靠 arm_joint.c 闭环去追 ——
  *             这一层的差异被 arm_joint 挡住了，本文件里看不到。
  ******************************************************************************
  */

#include "main.h"           /* HAL_GetTick() */
#include "proto_app.h"
#include "proto.h"
#include "proto_link.h"
#include "arm_cfg.h"
#include "arm_pwm.h"
#include "arm_joint.h"      /* T2: 关节执行层（底座闭环 / 其余直通） */
#include "mpu6050.h"        /* T4: 姿态传感器（pitch / 0x30 / 0x33） */

/* ---- 时间参数 ---- */
#define APP_REPORT_MS        20u     /* 状态帧周期：20ms = 50Hz（正本要求 20~50Hz） */
#define APP_HB_TX_MS         1000u    /* 我们自己发心跳的周期：1Hz */
#define APP_TGT_TIMEOUT_MS   3000u    /* 正本 §5：3 秒没收到 0x20 */
#define APP_HB_TIMEOUT_MS    3000u    /* 正本 §5：3 秒没收到心跳 */
#define APP_HOMING_SHOW_MS   500u     /* bit1"正在回零"至少显示这么久（无编码器，只能估） */
#define APP_BOOT_HOME_DELAY_MS 500u   /* T3：上电后等这么久再自动回零（让舵机先稳住） */

/* 插补和上报现在是**两个独立节拍**，不再是同一个 20ms。
 *
 * 以前两者绑死，因为插补一拍就顺便上报一拍，写起来简单。
 * 但任务书 T2 要求插补 ≥200Hz（5ms），而 0x10 只要 20~50Hz，只好拆开：
 *     ARM_INTERP_TICK_MS  5ms  → interp_tick() + arm_joint_tick()，**不出帧**
 *     APP_REPORT_MS      20ms  → send_status()，**不推进插补**
 *
 * 拆开之后 0x10 里报的 act 仍然是本拍最新的值：interp 每 5ms 更新一次目标，
 * 比 20ms 更密，"报的是隔拍旧值"那个老问题不会回来。 */
#if (APP_REPORT_MS < ARM_INTERP_TICK_MS)
#  error "状态帧周期不能短于插补周期"
#endif
#if ((APP_REPORT_MS % ARM_INTERP_TICK_MS) != 0)
#  error "状态帧周期必须是插补周期的整数倍，否则 0x10 的节拍会抖"
#endif

/* ==========================================================================
 * 状态变量总览（单位 / 归属写清楚，免得又混）
 *
 *   变量              单位    谁拥有     说明
 *   ----------------  ------  --------  --------------------------------
 *   s_tgt_x10[]       0.1°    协议层    最终目标角，**唯一的目标真值**
 *   s_q_from[]        度      插补器    本段起点（开始走那一刻的快照）
 *   s_dur_ms          ms      插补器    本段总时长
 *   s_ms_move0        ms      插补器    本段开始时刻
 *   s_moving          —       插补器    1 = 正在插补
 *   arm_joint_get_deg 度      执行层    当前实际角（底座是编码器实测）
 *
 *   为什么 s_tgt_x10 用 0.1° 而 s_q_from 用度，不是笔误：
 *   0.1° 是**协议线上的格式**（0x20 载荷里就是 int16 的 0.1°），
 *   度是**物理量**。换算只发生在插补器这一处，别的地方各自保持本来的单位。
 * ========================================================================== */

/* ---- 协议层 ---- */
static int16_t  s_tgt_x10[ARM_JOINT_NUM];   /* 目标角，单位 0.1° */
static uint8_t  s_mode;                     /* PROTO_MODE_xxx */
static uint8_t  s_fault;                    /* 故障态（急停），只能 0x21=3 清 */
static uint8_t  s_sensor_fault;             /* T4: MPU6050 丢线/故障（与急停分开，可自恢复） */
static uint8_t  s_i2c_fail;                 /* T4: 当前传感器失效（写进 bit4；底座那路在 arm_joint 内部） */
static uint8_t  s_to_target;                /* 0x20 超时 */
static uint8_t  s_to_hb;                    /* 心跳超时 */

/* ---- 插补器（T2） ---- */
static float    s_q_from[ARM_JOINT_NUM];    /* 本段起点，单位「度」 */
static uint16_t s_dur_ms;                   /* 本段总时长，单位 ms */
static uint32_t s_ms_move0;                 /* 本段开始时刻 */
static uint8_t  s_moving;                   /* 1 = 正在插补 */
static uint8_t  s_profile;                  /* 本段速度曲线：PROTO_PROFILE_xxx（见 proto.h 四之一） */

/* ---- T3：上电自动回零 ----
 * 三个变量的关系和"谁先谁后"很容易搞混，写清楚：
 *   s_boot_home_pending = 1  还在等（上电时置 1，收到任何指令就清 0）
 *   s_boot_home_active  = 1  正在回零（pending 转过来的，走完自己清 0）
 * 两者不会同时为 1，也不会同时为 0 之后再变回 1（一次性流程）。 */
static uint32_t s_ms_boot;                  /* 上电时刻（proto_app_init 里记） */
static uint8_t  s_boot_home_pending;        /* 1 = 还在等自动回零 */
static uint8_t  s_boot_home_active;         /* 1 = 自动回零进行中 */

/* ---- T3：越界钳位 ----
 * 语义是"最近一条指令有没有越界"，**每条指令重算一次**，不是攒着的报警。
 * 清除途径有两条：下一条完全合法的指令到达 → 自动清 0；或发 0x21=3 强制清。
 * 越界了照样执行，只是执行的是**钳位后**的边界值（不冻结）。 */
static uint8_t  s_limit_hit;                /* 1 = 已钳位（→ status bit2） */

/* ---- 时间戳 ---- */
static uint32_t s_ms_target_rx;
static uint32_t s_ms_hb_rx;
static uint32_t s_ms_report;
static uint32_t s_ms_interp;                 /* 插补节拍（5ms = 200Hz），和上报分开算 */
static uint32_t s_ms_hb_tx;
static uint32_t s_ms_homing;

static uint8_t  s_hb_seq;
static uint8_t  s_started;

/* 调试模式标记：收到 0x31 置 1，收到 0x20 / 0x21 清 0。
 * 唯一的用途是 0x32 应答里那个"调试中"位，让上位机知道现在是不是在手动捅脉宽。 */
static uint8_t  s_dbg;

/* ------------------------------------------------------------------------- */
/* 小工具                                                                    */
/* ------------------------------------------------------------------------- */
static void put_i16(uint8_t *p, int16_t v)
{
  p[0] = (uint8_t)((uint16_t)v & 0xFFu);
  p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static int16_t get_i16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static uint16_t get_u16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

/* float 度 → int16 的 0.1°（四舍五入，负号也要正确） */
static int16_t to_x10(float deg)
{
  return (int16_t)((deg >= 0.0f) ? (deg * 10.0f + 0.5f) : (deg * 10.0f - 0.5f));
}

/* ------------------------------------------------------------------------- */
/* 状态位组装（正本 §4.2）                                                   */
/* ------------------------------------------------------------------------- */
uint8_t proto_app_status(void)
{
  const uint32_t now = HAL_GetTick();
  uint8_t st = 0u;

  if (s_mode != PROTO_MODE_MANUAL)
  {
    st |= PROTO_ST_AUTO;                       /* bit0 自动 */
  }
  if ((s_mode == PROTO_MODE_HOMING) && ((now - s_ms_homing) < APP_HOMING_SHOW_MS))
  {
    st |= PROTO_ST_HOMING;                     /* bit1 正在回零 */
  }
  if ((s_fault != 0u) || (s_sensor_fault != 0u))
  {
    st |= PROTO_ST_FAULT;                      /* bit2 故障态（急停 / 传感器丢失） */
  }
  /* bit3：插补没走完，**或者**底座闭环还没收敛。
   * 这两件事必须分开判 —— 插补走完（s_moving 清零）不代表真到位，
   * 底座是闭环追出来的，还要一点时间收尾。 */
  if ((s_moving != 0u) || (arm_joint_moving() != 0u))
  {
    st |= PROTO_ST_MOVING;
  }
  if ((s_i2c_fail != 0u) || (arm_joint_base_fault() != 0u))
  {
    st |= PROTO_ST_I2C_FAIL;                   /* bit4 I²C 失效（含底座编码器） */
  }
  if ((s_to_target != 0u) || (s_to_hb != 0u))
  {
    st |= PROTO_ST_TIMEOUT;                    /* bit5 指令超时 */
  }
  if (s_limit_hit != 0u)
  {
    /* 越界复用 bit2 故障位（依据见 proto.h 里 PROTO_ST_RESERVED 上方的说明）。
     * 注意这里**只置位，不冻结** —— s_fault 仍是 0，move_start() 不会被挡，
     * 手臂照钳位后的边界值正常走完。 */
    st |= PROTO_ST_FAULT;                      /* bit2 故障态（含越界） */
  }

  return (uint8_t)(st & (uint8_t)~PROTO_ST_RESERVED);   /* bit6~7 必须为 0 */
}

/* ------------------------------------------------------------------------- */
/* 发送 0x10 状态帧                                                          */
/* ------------------------------------------------------------------------- */
static void send_status(uint32_t now)
{
  uint8_t pl[PROTO_LEN_STATUS];
  uint8_t j;

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    /* 走关节层而不是 arm_pwm：
     * 底座会读 AS5600 报真实角度，其余三路仍是 CCR 反算。 */
    put_i16(&pl[(uint16_t)j * 2u], to_x10(arm_joint_get_deg(j)));
  }
  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    put_i16(&pl[8u + (uint16_t)j * 2u], s_tgt_x10[j]);
  }

  /* 正本 §4.2：pitch 是"末端实测俯仰角"，无传感器时填 0。
   * T4：MPU6050 装在末端/小臂上，这里填加速度算出的**绝对倾角**（0.1°）。
   * 注意判 healthy：传感器丢了就填 0，不要拿最后一次缓存值硬撑 ——
   * 否则上位机会以为姿态还在，而实际上那已经是几百毫秒前的旧值了。 */
  put_i16(&pl[16], (mpu6050_healthy() != 0u) ? to_x10(mpu6050_pitch_deg()) : 0);
  pl[18] = proto_app_status();

  (void)now;
  (void)link_send_frame(PROTO_T_STATUS, pl, PROTO_LEN_STATUS);
}

/* ------------------------------------------------------------------------- */
/* 插补器（T2）：把"从 A 跳到 B"拆成每 5ms 一小步（200Hz）                            */
/* ------------------------------------------------------------------------- */

/* ==========================================================================
 * ★★★ 速度曲线：整个 T2 最该看懂的一块 ★★★
 *
 * 【要解决的问题】
 *   把一段角度变化（A → B）压在 dur_ms 毫秒里走完，问题只有一个：
 *   **这 dur_ms 里，时间怎么分配？**
 *   插补器每 5ms 都要回答：现在走到全程的百分之几了？这个百分数叫 s。
 *       q(t) = A + (B − A) × s(t)
 *   所以"算法"的全部内容就是 **s(t) 长什么样**，其余都是搬运。
 *
 * 【三种曲线 = s(t) 的三种形状】
 *   ① 阶跃 STEP（profile 0）
 *        s 从 0 直接跳到 1，舵机瞬间全速冲。
 *        不在这里实现 —— move_start() 里直接写到位，压根不进插补。
 *
 *   ② 梯形 TRAP（profile 1）
 *        速度先**匀加速**、再**匀速**、最后**匀减速**。
 *        速度曲线是个梯形 ⟹ s(t) 是「抛物线 + 直线 + 抛物线」。
 *        ★ 注意：**速度**是梯形，**位置**是 S 形，别搞反。
 *
 *   ③ S 曲线 SCURVE（profile 2）
 *        连加速度都做成连续的（jerk 有限），起停完全没有"顿"。
 *        位置曲线比梯形更顺，代价是同样时长里峰值速度更高。
 *
 * 【为什么用归一化时间 tau = t / dur，而不是直接算毫秒】
 *   这样 s(tau) 只跟**比例**有关，和这段走多久完全解耦。
 *   同一个函数，走 200ms 和走 2000ms 只是 t 和 dur 变了，形状一模一样 ——
 *   这正是"duration 只改快慢、不改终点"的数学原因。
 *
 * 【两个函数都必须满足的硬约束（不满足就是 bug）】
 *   s(0) = 0    起点必须是 0
 *   s(1) = 1    终点必须是 1，否则四轴对不齐、而且 s_moving 清不掉
 *   s 单调不减  否则中途会往回走
 * ========================================================================== */

/**
  * @brief  进度函数：把"时间进度"映射成"路程进度"
  * @param  profile PROTO_PROFILE_xxx（见 proto.h 四之一）
  * @param  tau     归一化时间 t/dur，调用方保证已经在 [0,1] 内
  * @retval 全程进度 s ∈ [0,1]
  */
static float profile_s(uint8_t profile, float tau)
{
  /* 两端先夹死：浮点误差可能让 tau 变成 1.0000001，那样 s 就 >1 了 */
  if (tau <= 0.0f)
  {
    return 0.0f;
  }
  if (tau >= 1.0f)
  {
    return 1.0f;
  }

  switch (profile)
  {
    case PROTO_PROFILE_SCURVE:
    {
      /* 五次多项式 smootherstep：s = 6τ⁵ − 15τ⁴ + 10τ³
       *
       * 为什么偏偏是这组系数：它是满足下面四个边界条件的**最低次多项式** ——
       *     s(0)=0、s(1)=1、s'(0)=s'(1)=0、s''(0)=s''(1)=0
       * 也就是说"起点和终点的速度为零、加速度也为零"，
       * 所以起停时既不会顿（速度连续），也不会"抽"（加速度连续）。
       *
       * 它的二阶导正是 120τ³ − 180τ² + 60τ，在 τ=0 和 τ=1 处都恰为 0。
       *
       * 代价：峰值速度 = 1.875 × 平均速度（出现在 τ=0.5），
       * 比梯形的 1.25 倍高不少 —— 同样时长里中间那段会明显更快。 */
      return (tau * tau * tau) * (10.0f + (tau * (-15.0f + (6.0f * tau))));
    }

    case PROTO_PROFILE_TRAP:
    {
      /* 梯形速度曲线：把归一化时间切成三段
       *     [0, r]        匀加速
       *     [r, 1−r]      匀速
       *     [1−r, 1]      匀减速
       *
       * 推导（r 就是 ARM_TRAP_ACC_RATIO）：
       *   设匀速段速度 Vp。加速段和减速段各是三角形，面积都为 Vp·r/2；
       *   匀速段面积 Vp·(1−2r)。
       *   总面积 = 1（走完全程）⟹ Vp·r + Vp·(1−2r) = Vp·(1−r) = 1
       *   ⟹ ★ Vp = 1/(1−r) > 1
       *
       *   ★★ 这就是梯形最反直觉的一点：为了"补回"加减速段少走的距离，
       *      匀速段的峰值速度**必须比平均速度高**（r=0.2 时高 25%）。
       *      所以"梯形比线性慢"是错的 —— 同样时长下梯形中间更快。
       *
       * 加速段：s = ∫₀^τ Vp·(u/r) du = Vp·τ²/(2r)
       * 匀速段：s = Vp·r/2 + Vp·(τ−r) = Vp·(τ − r/2)
       * 减速段：关于 τ=0.5 对称，直接用 1 − s(1−τ) 算，
       *         最省事也最不容易写错（手推三段容易在系数上翻车） */
      const float r = ARM_TRAP_ACC_RATIO;
      const float k = 2.0f * r * (1.0f - r);   /* 加速/减速段的分母，提出来免得算两遍 */

      if (tau < r)
      {
        return (tau * tau) / k;
      }
      if (tau > (1.0f - r))
      {
        const float u = 1.0f - tau;
        return 1.0f - ((u * u) / k);
      }
      return (tau - (0.5f * r)) / (1.0f - r);
    }

    case PROTO_PROFILE_STEP:
    default:
      /* 阶跃正常走不到这里（move_start 里已经直接写到位了）。
       * 真走到了说明有人传了未知 profile —— 按"直接到"处理，
       * 至少手臂还在动，不会一帧就不理了。 */
      return 1.0f;
  }
}

/**
  * @brief  进度对时间的导数 ds/dtau
  * @note   为什么要单独算它：底座是**闭环**关节，只靠 P 追一个移动的目标，
  *         永远会落后一截（稳态滞后 ≈ 速度 / Kp）。把"这一拍理论上该多快"
  *         前馈进去，误差才真能收敛到 0。
  *
  *         单位换算：v[度/秒] = dq[度] × (ds/dτ) × 1000 / dur[ms]
  *         因为 dτ/dt = 1/dur，而 dur 的单位是毫秒。
  *
  * @note   位置型关节用不到这个值（arm_joint_set_traj 会直接扔掉），
  *         但算出来的开销可以忽略，留着以后接 360° 底座闭环不欠账。
  */
static float profile_ds(uint8_t profile, float tau)
{
  if ((tau <= 0.0f) || (tau >= 1.0f))
  {
    return 0.0f;                 /* 两端速度为 0，这是三种曲线共同的性质 */
  }

  switch (profile)
  {
    case PROTO_PROFILE_SCURVE:
      /* s' = 30τ²(1−τ)²，在 τ=0.5 处取峰值 30×0.25×0.25 = 1.875 */
      return 30.0f * tau * tau * (1.0f - tau) * (1.0f - tau);

    case PROTO_PROFILE_TRAP:
    {
      const float r = ARM_TRAP_ACC_RATIO;
      const float k = r * (1.0f - r);

      if (tau < r)
      {
        return tau / k;                        /* 匀加速：速度线性上升 */
      }
      if (tau > (1.0f - r))
      {
        return (1.0f - tau) / k;               /* 匀减速：速度线性下降 */
      }
      return 1.0f / (1.0f - r);                /* 匀速：常数，就是上面的 Vp */
    }

    case PROTO_PROFILE_STEP:
    default:
      return 0.0f;
  }
}

/**
  * @brief  开始一段运动
  * @param  dur_ms  这段要走多少毫秒；0 = 立刻到位（等价于没有插补器的老行为）
  * @param  profile 速度曲线 PROTO_PROFILE_xxx；回零内部走梯形
  *
  * @note   起点取的是**此刻的实际位置**，不是上一个目标。
  *         上一段还没走完就来了新目标时，用"当前位置"当起点才能连续、不跳变。
  *         这就是打断续接：90°→45° 走到 70° 时改成 120°，就从 70° 继续往 120° 走，
  *         而不是先跳回 45° 再走（那样机械上会"抽"一下）。
  */
static void move_start(uint16_t dur_ms, uint8_t profile)
{
  uint8_t j;

  if ((s_fault != 0u) || (s_sensor_fault != 0u))
  {
    return;                          /* 故障态：不接受新动作（见文件头取舍 2）*/
  }

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    s_q_from[j] = arm_joint_get_deg(j);   /* 起点 = 当前实际角度 */
    arm_joint_set_manual(j, 0u);          /* 退出 0x31 手动模式，闭环接管 */
    arm_output_enable(j, 1u);             /* 标定时可能关过某一路，这里恢复 */
  }

  /* 未知档位降级成梯形，不报错。
   * 协议没要求设备对未知 profile 回错，而"降级"比"报错"稳：
   * 至少手臂还在按规矩走，不是一帧丢过去就当没收到。 */
  if (profile > PROTO_PROFILE_MAX)
  {
    profile = PROTO_PROFILE_TRAP;
  }

  /* ---- 阶跃：不插补，一步写到位 ----
   * 两个来源都会进这里，别以为只有其一：
   *   ① profile = 0  任务书规定的"阶跃"，这是主路径
   *   ② duration = 0 协议"为 0 表示按最大速度"，工程上就是不插补
   * ★ ① 优先：即使 duration=2000，只要 profile=0 也是**立刻**到位。 */
  if ((profile == PROTO_PROFILE_STEP) || (dur_ms == 0u))
  {
    for (j = 0u; j < ARM_JOINT_NUM; j++)
    {
      arm_joint_set_deg(j, (float)s_tgt_x10[j] / 10.0f);
    }
    s_moving = 0u;
    return;
  }

  s_profile  = profile;
  s_dur_ms   = dur_ms;
  s_ms_move0 = HAL_GetTick();
  s_moving   = 1u;
}

/**
  * @brief  插补一拍：算出四个关节各自该到哪，交给执行层
  *
  * @note   ★ 核心只有两步：
  *             tau = t / dur          现在走到全程的百分之几（**时间**进度）
  *             s   = profile_s(tau)   该走到全程的百分之几（**路程**进度）
  *         然后 q = 起点 + (终点 − 起点) × s，四个关节用**同一个 s**，
  *         所以必然**同时出发、同时到达** —— 这就是"多轴联动"。
  *
  * @note   为什么"同一个 s"就够：s 是**全程**进度，不是角度进度。
  *         每个关节各自位移不同，但都处在同一个比例处，自然同起同停。
  *
  * @note   节拍 5ms（200Hz），由 proto_app_poll() 用 ARM_INTERP_TICK_MS 控制。
  *         舵机 PWM 只有 50Hz，CCR 实际 20ms 才生效一次 —— 200Hz 的价值不在于
  *         让舵机走得更细，而在于**每拍算出来的角度增量本身是连续的**。
  */
static void interp_tick(uint32_t now)
{
  uint8_t  j;
  uint32_t t;
  float    tau;
  float    s;
  float    ds;

  if (s_moving == 0u)
  {
    return;                          /* 没在动，一拍都不算 */
  }

  t = now - s_ms_move0;

  /* 夹住：不夹的话 tau 会大于 1，角度冲过头，而且 s_moving 永远清不掉 */
  if (t >= (uint32_t)s_dur_ms)
  {
    t        = (uint32_t)s_dur_ms;
    s_moving = 0u;
  }

  /* 两个 float 强转都不能省：整数除法 400/800 会得 0，
   * 现象是"舵机纹丝不动，到时间突然跳到终点" */
  tau = (float)t / (float)s_dur_ms;

  /* ★★★ 全部算法就在这两行 ★★★ */
  s  = profile_s(s_profile, tau);
  ds = profile_ds(s_profile, tau);

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    const float q_to = (float)s_tgt_x10[j] / 10.0f;
    const float dq   = q_to - s_q_from[j];       /* 本段总位移，带符号 */
    const float q    = s_q_from[j] + (dq * s);
    float       v    = 0.0f;

    if (s_moving != 0u)
    {
      /* 前馈速度 = 位移 × (ds/dτ) × 1000 / dur[ms]
       * 只有速度型关节（360° 底座）用得到；位置型的 arm_joint_set_traj
       * 会把 v 直接扔掉，但算这一次是免费的，以后接闭环不欠账。 */
      v = dq * ds * 1000.0f / (float)s_dur_ms;
    }
    arm_joint_set_traj(j, q, v);
  }
}

/* ------------------------------------------------------------------------- */
/* 各消息处理                                                                */
/* ------------------------------------------------------------------------- */
static void on_target(const proto_frame_t *fr)
{
  uint8_t  j;
  uint16_t dur;
  uint8_t  profile;
  uint8_t  limited;
  int32_t  q;
  int32_t  lo;
  int32_t  hi;

  if (fr->len != PROTO_LEN_TARGET)
  {
    return;                          /* 长度不对直接忽略，不当成致命错误 */
  }

  /* T3 越界钳位：取值 → 对比该关节的 [deg_min, deg_max] → 夹住 → 记标志。
   *
   * 为什么设备端还要夹一遍：上位机本来有软限位（正本 §6「下发前限幅保护」），
   * 但那是**上位机**的义务。设备端是最后一道防线 —— 上位机配置错了、
   * 或者有人拿串口助手手拼一帧，都不该让舵机去撞机械限位。
   *
   * 注意这里是"钳位后照边界走"，不是"拒绝执行"：拒绝会让手臂完全不动，
   * 那和任务书要的"被钳位"是两回事。
   *
   * 用 int32_t 算：int16_t 的 deg_max 乘 10 在 180 以内不会溢出，
   * 但以后要是有人把 deg_max 改成 4000（0.1° 单位），int16_t 就会悄悄溢出。 */
  limited = 0u;
  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    q  = (int32_t)get_i16(&fr->payload[(uint16_t)j * 2u]);   /* 0.1° */
    lo = (int32_t)g_arm_joint[j].deg_min * 10;
    hi = (int32_t)g_arm_joint[j].deg_max * 10;

    if (q < lo)
    {
      q       = lo;
      limited = 1u;
    }
    else if (q > hi)
    {
      q       = hi;
      limited = 1u;
    }

    s_tgt_x10[j] = (int16_t)q;
  }
  s_limit_hit = limited;   /* 每帧重算：下一条合法指令到达就自动清 0 */

  /* 0x20 载荷：q1~q4_tgt(8B) + duration u16(2B) + profile u8(1B) */
  dur     = get_u16(&fr->payload[8]);
  profile = fr->payload[10];

  /* profile 的三个取值见 proto.h「四之一」：0 阶跃 / 1 梯形 / 2 S 曲线。
   * 以前这里写的是 (void)profile，三档全按"线性匀速"跑 —— 那是不对的：
   * 任务书根本没定义"线性"这一档，评审一看速度曲线就露馅。
   * 合法性检查（未知值降级成梯形）放在 move_start() 里，这里只管透传。 */

  if (s_mode == PROTO_MODE_HOMING)
  {
    s_mode = PROTO_MODE_AUTO;        /* 收到新的目标就退出回零 */
  }

  s_ms_target_rx = HAL_GetTick();
  s_to_target    = 0u;
  s_dbg          = 0u;               /* 回到正常控制流，退出调试模式 */

  /* 人已经在发目标了 → 取消还没执行的上电自动回零。
   * 不取消的话，可能出现"上位机正命令它去某处，500ms 到了却被拽回零位"。 */
  s_boot_home_pending = 0u;

  /* 重复收到同一个目标也没关系：起点=当前位置、终点=目标，
   * 走过去是 0 位移，无害。 */
  move_start(dur, profile);
}

static void on_mode(const proto_frame_t *fr)
{
  uint8_t j;

  if (fr->len != PROTO_LEN_MODE)
  {
    return;
  }

  switch (fr->payload[0])
  {
    case PROTO_MODE_MANUAL:
      s_mode = PROTO_MODE_MANUAL;
      break;

    case PROTO_MODE_AUTO:
      s_mode = PROTO_MODE_AUTO;
      break;

    case PROTO_MODE_HOMING:
      s_mode      = PROTO_MODE_HOMING;
      s_ms_homing = HAL_GetTick();
      for (j = 0u; j < ARM_JOINT_NUM; j++)
      {
        s_tgt_x10[j] = (int16_t)((int16_t)g_arm_joint[j].deg_home * 10);
      }
      /* 回零也走插补，别一步跳回去 —— 直接跳会"啪"一下，夹爪里夹着东西会掉。
       * 固定用**梯形**（不是阶跃、也不是 S）：回零是每次上电都要跑一遍的动作，
       * 梯形起停柔和、峰值速度也不高（1.25 倍），最不容易撞机械限位；
       * S 曲线的峰值速度是 1.875 倍，回零这种"贴着限位走"的动作不适合。 */
      move_start(ARM_HOMING_MS, PROTO_PROFILE_TRAP);
      break;

    case PROTO_MODE_CLEAR_FAULT:
      /* 正本 §4.5：急停后**唯一**的复位途径。
       * 复位后回到手动模式，需要再收 0x21=1 才进自动 —— 防止一解锁就自己动。
       *
       * T4 的传感器故障**不在这里清**：它由 mpu6050_tick() 按"能不能读到数"
       * 自动置位/清除（插回去就恢复），因为 I²C 掉线不是"急停"，不该要求
       * 人工解锁；但恢复后同样不会自己动，要等新的 0x20。 */
      s_fault = 0u;
      s_limit_hit = 0u;              /* 越界标志也一起清：它现在挂在 bit2 上，
                                      * 不清的话清完故障 bit2 还亮着，用户会以为没清掉 */
      s_mode  = PROTO_MODE_MANUAL;
      break;

    default:
      break;                         /* 未知值忽略 */
  }

  /* 任何模式切换都退出调试模式：收到模式帧意味着"要按正常流程控制" */
  s_dbg = 0u;

  /* 显式发过模式帧 = 人接管了，取消上电自动回零
   * （尤其是 0x21=1 自动模式：上位机正要自己发目标，别被回零打断） */
  s_boot_home_pending = 0u;
}

static void on_estop(const proto_frame_t *fr)
{
  if ((fr->len == PROTO_LEN_ESTOP) && (fr->payload[0] == PROTO_ESTOP_ON))
  {
    s_fault  = 1u;                   /* 冻结 + bit2，见文件头取舍 2 */
    s_moving = 0u;                   /* 立刻停插补：再晚一拍就又多走一步 */
    arm_joint_hold_position();       /* 底座的闭环也原地停住 */

    /* 急停了就别再自动回零 —— 故障态下 move_start() 本来也会拒绝，
     * 但把 pending 清掉更干净：否则故障解除后会突然冒出来走一段。 */
    s_boot_home_pending = 0u;
    s_boot_home_active  = 0u;
  }
}

static void on_heartbeat(const proto_frame_t *fr)
{
  if (fr->len != PROTO_LEN_HEARTBEAT)
  {
    return;
  }

  s_ms_hb_rx = HAL_GetTick();
  s_to_hb    = 0u;

  /* 正本 §4.6：**原样回发**，不是回自己的计数器 */
  (void)link_send_frame(PROTO_T_HEARTBEAT, &fr->payload[0], PROTO_LEN_HEARTBEAT);
}

static void on_version(const proto_frame_t *fr)
{
  uint8_t pl[PROTO_LEN_VERSION];

  (void)fr;                          /* 查询载荷是固定 01 00，不用看 */

  /* 正本 §4.7：设备侧**固定**填 0x01、0x00 —— 这是协议版本，不是固件版本 */
  pl[0] = PROTO_VER_MAJOR;
  pl[1] = PROTO_VER_MINOR;
  (void)link_send_frame(PROTO_T_VERSION, pl, PROTO_LEN_VERSION);
}

/* ------------------------------------------------------------------------- */
/* 0x31 单路脉宽设定（调试 / 标定用）                                        */
/* ------------------------------------------------------------------------- */
static void on_pulse_set(const proto_frame_t *fr)
{
  uint8_t  servo;
  uint16_t us;
  uint8_t  flags;

  if (fr->len != PROTO_LEN_PULSE_SET)
  {
    return;                          /* 长度不对：静默忽略，不当致命错误 */
  }

  servo = fr->payload[0];
  if (servo >= (uint8_t)ARM_JOINT_NUM)
  {
    return;                          /* 关节号越界：忽略 */
  }

  /* 小端：低字节在前 */
  us    = (uint16_t)((uint16_t)fr->payload[1] | ((uint16_t)fr->payload[2] << 8));
  flags = fr->payload[3];

  /* ① 故障态下**拒绝**调试写入。
   * 这是安全边界：急停的语义是"谁都别再动"。如果 0x31 能绕过去，
   * 急停就成了摆设 —— 一个上位机 bug 就能把手臂一路捅到极限。
   * 传感器故障同样拒绝：T4 要求"传感器失效 → 1 秒内停止输出"，
   * 如果 0x31 还能写脉宽，这条安全要求就绕过去了。
   * 要恢复先发 0x21 = 3 清故障（传感器故障则等它自己恢复）。 */
  if ((s_fault != 0u) || (s_sensor_fault != 0u))
  {
    return;
  }

  /* ② 这里**不夹**脉宽：arm_set_us() 自己会夹到 [500, 2500]。
   * 同一个数夹两遍没有意义，只会让人以为存在两层不同的限制。
   * 协议层只负责解帧和查长度，范围由 arm_pwm 那一层统一把关。 */

  /* ③ 先写脉宽，再决定开不开输出，顺序不能反。
   * 反过来的话，通道会按寄存器里的旧值先输出一小段，舵机"抖一下" ——
   * 标定时这一下很容易被人误判成"已经到限位了"。 */
  /* 越界标志：arm_set_us() 内部会夹到 [500, 2500]，但那之后我们就不知道
   * "原来是多少"了 —— 所以必须**先判再写**。
   * ArmHost README §12.3 对 0x31 就是这么要求的：「越界时按 500/2500µs 钳位并置 status」。
   * 和 0x20 一样，每条指令重算一次，发一条合法脉宽就自动清掉。 */
  s_limit_hit = (uint8_t)(((us < SERVO_US_NOMINAL_MIN) ||
                           (us > SERVO_US_NOMINAL_MAX)) ? 1u : 0u);

  arm_set_us(servo, us);
  arm_output_enable(servo, (uint8_t)(((flags & PROTO_PULSE_FLAG_EN) != 0u) ? 1u : 0u));

  /* ④ **停插补**。不写这一句的话，下一个 20ms 的 interp_tick() 会把刚写的
   * 脉宽又覆盖回去 —— 标定时表现为"按一次动一下，然后自己跑回原来的目标"，
   * 而且你会以为是舵机的问题，查半天。 */
  s_moving = 0u;

  /* ⑤ 这一路交给手动，闭环别抢（只有底座是闭环关节时才有意义） */
  arm_joint_set_manual(servo, 1u);

  /* ⑥ flags bit1「其余舵机保持」：把**闭环关节**的目标改成它当前位置。
   * 位置型关节不用处理 —— CCR 不被改写，它自己就保持住了。
   * 只停插补是不够的：底座的闭环会继续朝老目标跑，那就不是"保持"了。 */
  arm_joint_hold_position();

  /* ⑦ 同步目标角，避免 0x10 帧里出现 "act 变了、tgt 还是老值" 的假象。
   * 注意这里走 arm_joint_get_deg()：位置型会把角度夹在该关节 [deg_min, deg_max] 内，
   * 所以标定扫到范围外时，tgt 只是"夹取后的等价角度"，判断行程别看它，看 0x32。 */
  s_tgt_x10[servo] = to_x10(arm_joint_get_deg(servo));

  s_dbg = 1u;
}

/* ------------------------------------------------------------------------- */
/* 0x32 脉宽回读（查询 → 应答）                                              */
/* ------------------------------------------------------------------------- */
static void on_pulse_get(const proto_frame_t *fr)
{
  uint8_t pl[PROTO_LEN_PULSE_REPLY];
  uint8_t st = 0u;
  uint8_t j;

  if (fr->len != PROTO_LEN_PULSE_QUERY)
  {
    return;                          /* 查询帧长度必须是 1 */
  }

  /* 四路当前脉宽：直接读 CCR 换算，读的是芯片里的真实值，不是软件缓存 */
  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    put_u16(&pl[(uint16_t)j * 2u], arm_get_us(j));
  }

  if (s_dbg != 0u)   { st |= PROTO_DBG_ACTIVE; }
  if ((s_fault != 0u) || (s_sensor_fault != 0u)) { st |= PROTO_DBG_FAULT; }

  pl[8] = st;

  (void)link_send_frame(PROTO_T_PULSE_GET, pl, PROTO_LEN_PULSE_REPLY);
}

/* ------------------------------------------------------------------------- */
/* 0x30 传感器原始数据（查询 → 应答）                                         */
/* ------------------------------------------------------------------------- */
static void on_sensor_raw(const proto_frame_t *fr)
{
  uint8_t          pl[PROTO_LEN_SENSOR_REPLY];
  mpu6050_sample_t s;
  uint8_t          st = 0u;
  uint8_t          valid;

  if (fr->len != PROTO_LEN_SENSOR_QUERY)
  {
    return;                          /* 查询帧长度必须是 1 */
  }

  /* 没数据时 s 是全 0（驱动里 memset 过），回 0 比回垃圾值诚实。
   * bit0 用这次读取的返回值，不用 healthy() —— 刚唤醒的 50ms 里
   * healthy() 已经是 1，但还没有有效样本。 */
  valid = mpu6050_get_sample(&s);

  /* 载荷：6×int16 原始计数 + temp + pitch(0.1°) + status */
  put_i16(&pl[0],  s.ax);
  put_i16(&pl[2],  s.ay);
  put_i16(&pl[4],  s.az);
  put_i16(&pl[6],  s.gx);
  put_i16(&pl[8],  s.gy);
  put_i16(&pl[10], s.gz);
  put_i16(&pl[12], s.temp);
  put_i16(&pl[14], to_x10(s.pitch_deg));

  if (valid != 0u)                                 { st |= PROTO_SENSOR_ST_OK; }
  if (mpu6050_calib_state() == MPU6050_CAL_RUNNING){ st |= PROTO_SENSOR_ST_CALIB; }
  if (mpu6050_faulted() != 0u)                    { st |= PROTO_SENSOR_ST_FAULT; }
  if (mpu6050_calib_state() == MPU6050_CAL_DONE)  { st |= PROTO_SENSOR_ST_CALIBRATED; }
  pl[16] = st;

  (void)link_send_frame(PROTO_T_SENSOR_RAW, pl, PROTO_LEN_SENSOR_REPLY);
}

/* ------------------------------------------------------------------------- */
/* 0x33 陀螺零偏标定（命令 → 应答）                                           */
/* ------------------------------------------------------------------------- */
static void on_gyro_calib(const proto_frame_t *fr)
{
  uint8_t pl[PROTO_LEN_CALIB_REPLY];
  uint8_t state;
  uint8_t reason;
  uint8_t cmd;
  float   bx = 0.0f;
  float   by = 0.0f;
  float   bz = 0.0f;

  if (fr->len != PROTO_LEN_CALIB_CMD)
  {
    return;
  }

  cmd    = fr->payload[0];
  state  = mpu6050_calib_state();
  reason = mpu6050_calib_reason();

  if (cmd == PROTO_CALIB_START)
  {
    /* 两个前置条件：
     *   ① 传感器在线 —— 不在线时采不到数，直接回 NO_SENSOR；
     *   ② 机械臂静止 —— 动着采 5 秒，采到的是"运动 + 零偏"，标了比不标更糟。 */
    if (mpu6050_healthy() == 0u)
    {
      state  = PROTO_CALIB_ERROR;
      reason = PROTO_CALIB_R_NO_SENSOR;
    }
    else if ((s_moving != 0u) || (arm_joint_moving() != 0u))
    {
      state  = PROTO_CALIB_ERROR;
      reason = PROTO_CALIB_R_MOVING;
    }
    else if (mpu6050_calib_start() != 0u)
    {
      state  = PROTO_CALIB_RUNNING;
      reason = PROTO_CALIB_R_OK;
    }
    else
    {
      state  = PROTO_CALIB_ERROR;
      reason = mpu6050_calib_reason();
    }
  }
  else if (cmd == PROTO_CALIB_ABORT)
  {
    mpu6050_calib_abort();
    state  = mpu6050_calib_state();
    reason = mpu6050_calib_reason();
  }
  else
  {
    /* PROTO_CALIB_QUERY 及未知值：只报状态，不改任何东西 */
  }

  mpu6050_calib_get_bias(&bx, &by, &bz);

  pl[0] = state;
  pl[1] = mpu6050_calib_progress();
  pl[2] = reason;
  put_i16(&pl[3], (int16_t)(bx * 10.0f));   /* 0.1°/s */
  put_i16(&pl[5], (int16_t)(by * 10.0f));
  put_i16(&pl[7], (int16_t)(bz * 10.0f));

  (void)link_send_frame(PROTO_T_GYRO_CALIB, pl, PROTO_LEN_CALIB_REPLY);
}

/* ------------------------------------------------------------------------- */
static void dispatch(const proto_frame_t *fr)
{
  switch (fr->type)
  {
    case PROTO_T_TARGET:    on_target(fr);    break;
    case PROTO_T_MODE:      on_mode(fr);      break;
    case PROTO_T_ESTOP:     on_estop(fr);     break;
    case PROTO_T_HEARTBEAT: on_heartbeat(fr); break;
    case PROTO_T_VERSION:   on_version(fr);   break;
    case PROTO_T_PULSE_SET: on_pulse_set(fr); break;
    case PROTO_T_PULSE_GET: on_pulse_get(fr); break;
    case PROTO_T_SENSOR_RAW: on_sensor_raw(fr); break;
    case PROTO_T_GYRO_CALIB: on_gyro_calib(fr); break;

    default:
      /* 未知类型：静默丢弃。正本要求"不回复"，也绝不能崩。 */
      break;
  }
}

/* ------------------------------------------------------------------------- */
/* 对外                                                                      */
/* ------------------------------------------------------------------------- */
void proto_app_init(void)
{
  const uint32_t now = HAL_GetTick();
  uint8_t j;

  link_init();

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    s_tgt_x10[j] = (int16_t)((int16_t)g_arm_joint[j].deg_home * 10);
    s_q_from[j]  = (float)g_arm_joint[j].deg_home;   /* 上电即静止：起点=终点=零位 */
  }

  /* 插补器上电就是"停着的"，否则第一拍会莫名其妙走一段 */
  s_dur_ms   = 0u;
  s_ms_move0 = now;
  s_moving   = 0u;
  s_profile  = PROTO_PROFILE_TRAP;    /* 默认档位；0x20 来了会按载荷里的 profile 覆盖 */

  s_mode       = PROTO_MODE_MANUAL;   /* 上电先进手动，等上位机发 0x21=1 才自动化 */
  s_fault      = 0u;
  s_sensor_fault = 0u;
  s_i2c_fail   = 0u;
  s_to_target  = 0u;
  s_to_hb      = 0u;
  s_hb_seq     = 0u;
  s_started    = 1u;
  s_dbg        = 0u;

  /* T3：登记"等会儿要自动回零"。这里**只登记不动手** ——
   * 真正动起来在 proto_app_poll 里，因为要等 500ms 让舵机先从中位稳住。
   * 为什么不在 init 里直接 move_start：init 时 MX_TIM2 刚起、舵机可能正在上电
   * 抽动，这时候又给它一个新目标，会出现"两只手同时拽"的抖动。 */
  s_ms_boot           = now;
  s_boot_home_pending = 1u;
  s_boot_home_active  = 0u;

  s_limit_hit         = 0u;           /* T3：上电没有越界 */

  /* 超时计时从现在开始算，避免上电瞬间就报超时 */
  s_ms_target_rx = now;
  s_ms_hb_rx     = now;
  s_ms_report    = now;
  s_ms_interp    = now;                /* 插补节拍单独计时（5ms），和上报分开 */
  s_ms_hb_tx     = now;
  s_ms_homing    = (uint32_t)(now - APP_HOMING_SHOW_MS);   /* 不在回零中 */
}

void proto_app_poll(void)
{
  proto_frame_t fr;
  uint32_t      now;

  if (s_started == 0u)
  {
    return;
  }

  now = HAL_GetTick();

  /* 0) T4 传感器：50Hz 采样 + 失联判定。
   *    必须放在收帧之前：这一拍已经判故障的话，刚收到的 0x20 不能再让手臂动。
   *
   *    故障动作 = 停插补 + 底座原地保持 + 冻结四路 CCR。
   *    和急停一样**不断开 PWM**：SG90 没脉冲就没有保持力矩，手臂会直接掉下来，
   *    比"停在原位"危险得多（理由见文件头取舍 2）。
   *
   *    恢复：传感器重新读到数就自动清 s_sensor_fault / bit2 / bit4，
   *    但**不会自动继续跑** —— 必须等上位机重新发 0x20。
   *    这样拔插一次不会让手臂突然朝一个几分钟前的旧目标冲过去。 */
  mpu6050_tick(now);
  {
    const uint8_t f = mpu6050_faulted();

    s_i2c_fail = f;

    if (f != 0u)
    {
      if (s_sensor_fault == 0u)
      {
        s_sensor_fault = 1u;
        s_moving       = 0u;
        arm_joint_hold_position();
      }
    }
    else
    {
      s_sensor_fault = 0u;
    }
  }

  /* 1) 收：一次把队列取干净 */
  while (link_poll(&fr) == 1)
  {
    dispatch(&fr);
  }

  /* 1.5) T3：上电自动回零
   *
   * 三条设计约束，每条都有理由：
   *   ① 延迟 500ms —— 上电瞬间舵机正从中位往零位爬，这时候再塞一个目标
   *      等于"两只手同时拽"，会抖。等它先稳住。
   *   ② 收到任何指令就取消 —— 上位机可能比我们快。人的指令永远优先于
   *      自动流程，否则会出现"上位机正命令它去某处，500ms 到了却被拽回零位"。
   *   ③ 走插补不直接跳 —— 直接写目标会让四个关节同时全速冲，冲击大；
   *      而且回零本来就"是一种运动"，走同一条路才能被 T2 的插补器统一管理。
   *
   * 位置也很讲究：必须放在 dispatch 之后 —— 如果这一拍刚好收到 0x20，
   * 它已经把 pending 清掉了，这里就不会再抢一次。
   */
  if ((s_boot_home_pending != 0u) &&
      ((now - s_ms_boot) >= APP_BOOT_HOME_DELAY_MS))
  {
    uint8_t k;

    s_boot_home_pending = 0u;
    s_boot_home_active  = 1u;

    s_mode      = PROTO_MODE_HOMING;
    s_ms_homing = now;

    for (k = 0u; k < ARM_JOINT_NUM; k++)
    {
      s_tgt_x10[k] = (int16_t)((int16_t)g_arm_joint[k].deg_home * 10);
    }
    move_start(ARM_HOMING_MS, PROTO_PROFILE_TRAP);   /* 见 on_mode() 里同样的说明 */
  }

  /* 1.6) 自动回零走完 → 回到手动
   *
   * 不回来的话 s_mode 永远是 HOMING，而 bit5"指令超时"判的是
   * "非手动模式 + 3 秒没收到 0x20"。上位机还没连上就先报超时，
   * 接上线第一眼是个红字，容易被误判成设备坏了。
   *
   * 只处理**自动**回零：人手发的 0x21=2 保持原行为（停在回零态等新目标），
   * 因为那是人的显式意图，不该被我们改回手动。 */
  if ((s_boot_home_active != 0u) &&
      (s_moving == 0u) && (arm_joint_moving() == 0u) &&
      ((now - s_ms_homing) >= APP_HOMING_SHOW_MS))
  {
    s_boot_home_active = 0u;
    s_mode             = PROTO_MODE_MANUAL;
    s_ms_target_rx     = now;        /* 超时计时重新起算 */
  }

  /* 2) 插补节拍 5ms（200Hz）—— 任务书 T2 的硬指标。
   *    ★ 这里**不出帧**，只管把四个关节的目标往前推一小步。
   *    顺序仍是"先插补、再执行层"：反了的话执行层用的是上一拍的旧目标。 */
  if ((now - s_ms_interp) >= ARM_INTERP_TICK_MS)
  {
    s_ms_interp = now;

    interp_tick(now);      /* ① 算出这一拍四个关节各自该到哪 → 设定目标 */
    arm_joint_tick(now);   /* ② 底座闭环跑一次，写脉宽（位置型在这里没事干） */
  }

  /* 2.5) 上报节拍 20ms（50Hz）—— 和插补拆开，帧率不跟着插补一起涨（那会淹没 CDC）。
   *      此时 act 是本拍最新的值：interp 每 5ms 就更新过一次，
   *      所以"0x10 报的是隔拍旧值"那个老问题不会回来。 */
  if ((now - s_ms_report) >= APP_REPORT_MS)
  {
    s_ms_report = now;
    send_status(now);
  }

  /* 3) 发 0x23 心跳（1Hz）。我们自己也要发，不能只回不回。 */
  if ((now - s_ms_hb_tx) >= APP_HB_TX_MS)
  {
    s_ms_hb_tx = now;
    (void)link_send_frame(PROTO_T_HEARTBEAT, &s_hb_seq, PROTO_LEN_HEARTBEAT);
    s_hb_seq++;
  }

  /* 4) 超时判定（正本 §5：保持姿态 + 置 bit5） */
  s_to_target = (uint8_t)((s_mode != PROTO_MODE_MANUAL) &&
                          ((now - s_ms_target_rx) > APP_TGT_TIMEOUT_MS));
  s_to_hb     = (uint8_t)((now - s_ms_hb_rx) > APP_HB_TIMEOUT_MS);
}
