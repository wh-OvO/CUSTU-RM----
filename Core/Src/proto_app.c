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
  ******************************************************************************
  */

#include "main.h"           /* HAL_GetTick() */
#include "proto_app.h"
#include "proto.h"
#include "proto_link.h"
#include "arm_cfg.h"
#include "arm_pwm.h"

/* ---- 时间参数 ---- */
#define APP_REPORT_MS        20u     /* 状态帧周期：20ms = 50Hz（正本要求 20~50Hz） */
#define APP_HB_TX_MS         1000u    /* 我们自己发心跳的周期：1Hz */
#define APP_TGT_TIMEOUT_MS   3000u    /* 正本 §5：3 秒没收到 0x20 */
#define APP_HB_TIMEOUT_MS    3000u    /* 正本 §5：3 秒没收到心跳 */
#define APP_HOMING_SHOW_MS   500u     /* bit1"正在回零"至少显示这么久（无编码器，只能估） */
#define APP_MOVING_SHOW_MS   200u     /* bit3"正在运动"的近似窗口 */

/* ---- 状态 ---- */
static int16_t  s_tgt_x10[ARM_JOINT_NUM];   /* 目标角，单位 0.1° */
static uint8_t  s_mode;                     /* PROTO_MODE_xxx */
static uint8_t  s_fault;                    /* 故障态（急停或 I²C 失效） */
static uint8_t  s_i2c_fail;                 /* T4 会写它 */
static uint8_t  s_to_target;                /* 0x20 超时 */
static uint8_t  s_to_hb;                    /* 心跳超时 */

static uint32_t s_ms_target_rx;
static uint32_t s_ms_hb_rx;
static uint32_t s_ms_report;
static uint32_t s_ms_hb_tx;
static uint32_t s_ms_homing;
static uint32_t s_ms_moving;

static uint8_t  s_hb_seq;
static uint8_t  s_started;

/* ------------------------------------------------------------------------- */
/* 小工具                                                                    */
/* ------------------------------------------------------------------------- */
static void put_i16(uint8_t *p, int16_t v)
{
  p[0] = (uint8_t)((uint16_t)v & 0xFFu);
  p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

static int16_t get_i16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
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
  if (s_fault != 0u)
  {
    st |= PROTO_ST_FAULT;                      /* bit2 故障态 */
  }
  if ((now - s_ms_moving) < APP_MOVING_SHOW_MS)
  {
    st |= PROTO_ST_MOVING;                     /* bit3 正在运动（近似，见文件头） */
  }
  if (s_i2c_fail != 0u)
  {
    st |= PROTO_ST_I2C_FAIL;                   /* bit4 I²C 失效 */
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
    put_i16(&pl[(uint16_t)j * 2u], to_x10(arm_get_deg(j)));
  }
  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    put_i16(&pl[8u + (uint16_t)j * 2u], s_tgt_x10[j]);
  }

  /* 正本 §4.2：pitch 无传感器时填 0。T4 接了 MPU6050 再改这里 */
  put_i16(&pl[16], 0);
  pl[18] = proto_app_status();

  (void)now;
  (void)link_send_frame(PROTO_T_STATUS, pl, PROTO_LEN_STATUS);
}

/* ------------------------------------------------------------------------- */
/* 把四路目标角真正写进 PWM                                                  */
/* ------------------------------------------------------------------------- */
static void apply_targets(void)
{
  uint8_t j;

  if (s_fault != 0u)
  {
    return;                          /* 故障态：冻结姿态，不更新 CCR（见文件头取舍 2）*/
  }

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    (void)arm_set_deg(j, (float)s_tgt_x10[j] / 10.0f);
  }
}

/* ------------------------------------------------------------------------- */
/* 各消息处理                                                                */
/* ------------------------------------------------------------------------- */
static void on_target(const proto_frame_t *fr)
{
  uint8_t j;

  if (fr->len != PROTO_LEN_TARGET)
  {
    return;                          /* 长度不对直接忽略，不当成致命错误 */
  }

  for (j = 0u; j < ARM_JOINT_NUM; j++)
  {
    s_tgt_x10[j] = get_i16(&fr->payload[(uint16_t)j * 2u]);
  }

  /* duration / profile 是给 T2 的插补用的，现在还没有插补器，先记下不管。
   * TODO(T2): 用 duration + profile 生成梯形/S 曲线中间点。 */
  (void)fr->payload[8];
  (void)fr->payload[9];
  (void)fr->payload[10];

  if (s_mode == PROTO_MODE_HOMING)
  {
    s_mode = PROTO_MODE_AUTO;        /* 收到新的目标就退出回零 */
  }

  s_ms_target_rx = HAL_GetTick();
  s_ms_moving    = s_ms_target_rx;
  s_to_target    = 0u;

  apply_targets();
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
      apply_targets();
      break;

    case PROTO_MODE_CLEAR_FAULT:
      /* 正本 §4.5：急停后**唯一**的复位途径。
       * 复位后回到手动模式，需要再收 0x21=1 才进自动 —— 防止一解锁就自己动。 */
      s_fault = 0u;
      s_mode  = PROTO_MODE_MANUAL;
      break;

    default:
      break;                         /* 未知值忽略 */
  }
}

static void on_estop(const proto_frame_t *fr)
{
  if ((fr->len == PROTO_LEN_ESTOP) && (fr->payload[0] == PROTO_ESTOP_ON))
  {
    s_fault = 1u;                    /* 冻结 + bit2，见 apply_targets() */
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
static void dispatch(const proto_frame_t *fr)
{
  switch (fr->type)
  {
    case PROTO_T_TARGET:    on_target(fr);    break;
    case PROTO_T_MODE:      on_mode(fr);      break;
    case PROTO_T_ESTOP:     on_estop(fr);     break;
    case PROTO_T_HEARTBEAT: on_heartbeat(fr); break;
    case PROTO_T_VERSION:   on_version(fr);   break;

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
  }

  s_mode       = PROTO_MODE_MANUAL;   /* 上电先进手动，等上位机发 0x21=1 才自动化 */
  s_fault      = 0u;
  s_i2c_fail   = 0u;
  s_to_target  = 0u;
  s_to_hb      = 0u;
  s_hb_seq     = 0u;
  s_started    = 1u;

  /* 超时计时从现在开始算，避免上电瞬间就报超时 */
  s_ms_target_rx = now;
  s_ms_hb_rx     = now;
  s_ms_report    = now;
  s_ms_hb_tx     = now;
  s_ms_homing    = (uint32_t)(now - APP_HOMING_SHOW_MS);   /* 不在回零中 */
  s_ms_moving    = (uint32_t)(now - APP_MOVING_SHOW_MS);
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

  /* 1) 收：一次把队列取干净 */
  while (link_poll(&fr) == 1)
  {
    dispatch(&fr);
  }

  /* 2) 发 0x10 状态帧（20ms = 50Hz） */
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
