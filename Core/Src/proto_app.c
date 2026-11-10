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

/* 插补节拍必须和上报周期一致，否则 0x10 里报出来的 act 是隔拍的旧值。 */
#if (APP_REPORT_MS != ARM_INTERP_TICK_MS)
#  error "APP_REPORT_MS 和 ARM_INTERP_TICK_MS 必须相等"
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

/* ---- 时间戳 ---- */
static uint32_t s_ms_target_rx;
static uint32_t s_ms_hb_rx;
static uint32_t s_ms_report;
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
/* 插补器（T2）：把"从 A 跳到 B"拆成每 20ms 一小步                            */
/* ------------------------------------------------------------------------- */

/**
  * @brief  开始一段运动
  * @param  dur_ms 这段要走多少毫秒；0 = 立刻到位（等价于没有插补器的老行为）
  *
  * @note   起点取的是**此刻的实际位置**，不是上一个目标。
  *         上一段还没走完就来了新目标时，用"当前位置"当起点才能连续、不跳变。
  *         这就是打断续接：90°→45° 走到 70° 时改成 120°，就从 70° 继续往 120° 走，
  *         而不是先跳回 45° 再走（那样机械上会"抽"一下）。
  */
static void move_start(uint16_t dur_ms)
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

  if (dur_ms == 0u)
  {
    /* duration = 0 表示"立刻到"：一步写下去，老行为 */
    for (j = 0u; j < ARM_JOINT_NUM; j++)
    {
      arm_joint_set_deg(j, (float)s_tgt_x10[j] / 10.0f);
    }
    s_moving = 0u;
    return;
  }

  s_dur_ms   = dur_ms;
  s_ms_move0 = HAL_GetTick();
  s_moving   = 1u;
}

/**
  * @brief  插补一拍：算出四个关节各自该到哪，交给执行层
  *
  * @note   核心只有一行：q = 起点 + (终点 − 起点) × s，s 从 0 走到 1。
  *         四个关节共用同一个 s，所以**同时出发、同时到达**。
  */
static void interp_tick(uint32_t now)
{
  uint8_t  j;
  uint32_t t;
  float    s;

  if (s_moving == 0u)
  {
    return;                          /* 没在动，一拍都不算 */
  }

  t = now - s_ms_move0;

  /* 夹住：不夹的话 s 会大于 1，角度冲过头，而且 s_moving 永远清不掉 */
  if (t >= (uint32_t)s_dur_ms)
  {
    t        = (uint32_t)s_dur_ms;
    s_moving = 0u;
  }

  /* 两个 float 强转都不能省：整数除法 400/800 会得 0，
   * 现象是"舵机纹丝不动，到时间突然跳到终点" */
  s = (float)t / (float)s_dur_ms;

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    const float q_to = (float)s_tgt_x10[j] / 10.0f;
    const float q    = s_q_from[j] + ((q_to - s_q_from[j]) * s);
    float       v    = 0.0f;

    if (s_moving != 0u)
    {
      /* 线性插补 → 本段速度恒定，正好可以当底座的前馈量：v = 位移 / 时长 */
      v = (q_to - s_q_from[j]) * 1000.0f / (float)s_dur_ms;
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

  if (fr->len != PROTO_LEN_TARGET)
  {
    return;                          /* 长度不对直接忽略，不当成致命错误 */
  }

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    s_tgt_x10[j] = get_i16(&fr->payload[(uint16_t)j * 2u]);
  }

  /* 0x20 载荷：q1~q4_tgt(8B) + duration u16(2B) + profile u8(1B) */
  dur     = get_u16(&fr->payload[8]);
  profile = fr->payload[10];

  /* profile：0 = 线性插补。梯形 / S 曲线还没做，
   * 其它档位**静默按线性跑** —— 协议没要求设备对未知 profile 回错，
   * 降级比报错稳：至少手臂还在动，不是一帧就不理了。 */
  (void)profile;

  if (s_mode == PROTO_MODE_HOMING)
  {
    s_mode = PROTO_MODE_AUTO;        /* 收到新的目标就退出回零 */
  }

  s_ms_target_rx = HAL_GetTick();
  s_to_target    = 0u;
  s_dbg          = 0u;               /* 回到正常控制流，退出调试模式 */

  /* 重复收到同一个目标也没关系：起点=当前位置、终点=目标，
   * 走过去是 0 位移，无害。 */
  move_start(dur);
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
      /* 回零也走插补，别一步跳回去 —— 直接跳会"啪"一下，夹爪里夹着东西会掉 */
      move_start(ARM_HOMING_MS);
      break;

    case PROTO_MODE_CLEAR_FAULT:
      /* 正本 §4.5：急停后**唯一**的复位途径。
       * 复位后回到手动模式，需要再收 0x21=1 才进自动 —— 防止一解锁就自己动。
       *
       * T4 的传感器故障**不在这里清**：它由 mpu6050_tick() 按"能不能读到数"
       * 自动置位/清除（插回去就恢复），因为 I²C 掉线不是"急停"，不该要求
       * 人工解锁；但恢复后同样不会自己动，要等新的 0x20。 */
      s_fault = 0u;
      s_mode  = PROTO_MODE_MANUAL;
      break;

    default:
      break;                         /* 未知值忽略 */
  }

  /* 任何模式切换都退出调试模式：收到模式帧意味着"要按正常流程控制" */
  s_dbg = 0u;
}

static void on_estop(const proto_frame_t *fr)
{
  if ((fr->len == PROTO_LEN_ESTOP) && (fr->payload[0] == PROTO_ESTOP_ON))
  {
    s_fault  = 1u;                   /* 冻结 + bit2，见文件头取舍 2 */
    s_moving = 0u;                   /* 立刻停插补：再晚一拍就又多走一步 */
    arm_joint_hold_position();       /* 底座的闭环也原地停住 */
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

  s_mode       = PROTO_MODE_MANUAL;   /* 上电先进手动，等上位机发 0x21=1 才自动化 */
  s_fault      = 0u;
  s_sensor_fault = 0u;
  s_i2c_fail   = 0u;
  s_to_target  = 0u;
  s_to_hb      = 0u;
  s_hb_seq     = 0u;
  s_started    = 1u;
  s_dbg        = 0u;

  /* 超时计时从现在开始算，避免上电瞬间就报超时 */
  s_ms_target_rx = now;
  s_ms_hb_rx     = now;
  s_ms_report    = now;
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

  /* 2) 20ms 节拍：**先推进插补，再跑执行层，最后上报**。
   *    顺序反了的话，0x10 里报的 act 比实际慢一拍，3D 视图会明显滞后。 */
  if ((now - s_ms_report) >= APP_REPORT_MS)
  {
    s_ms_report = now;

    interp_tick(now);      /* ① 算出这一拍四个关节各自该到哪 → 设定目标 */
    arm_joint_tick(now);   /* ② 底座闭环跑一次，写脉宽（位置型在这里没事干） */
    send_status(now);      /* ③ 上报，此时 act 是本拍最新的值 */
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
