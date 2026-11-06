/**
  ******************************************************************************
  * @file    arm_pwm.c
  * @brief   角度 → 脉宽 → CCR 的实现（T1 的核心逻辑）
  ******************************************************************************
  * @note    整个换算链就三行公式，别被代码行数吓到：
  *
  *            ① 角度夹进该关节的 [deg_min, deg_max]
  *
  *            ② 线性插值成脉宽：
  *                 us = us_at_deg_min
  *                    + (deg - deg_min) × (us_at_deg_max - us_at_deg_min)
  *                                       ÷ (deg_max - deg_min)
  *
  *            ③ 脉宽夹进安全窗 [SERVO_US_SAFE_MIN, SERVO_US_SAFE_MAX]
  *               再乘 ARM_PWM_TICKS_PER_US 得到 CCR
  *
  *          公式里刻意用 int32 做中间量、最后才转 uint16：
  *          如果直接用 uint16 算减法，us_at_deg_max < us_at_deg_min（装反的关节）
  *          会下溢成一个巨大的正数，舵机瞬间冲到底。
  ******************************************************************************
  */

#include "main.h"        /* HAL、htim2 所在的头文件 */
#include "arm_cfg.h"
#include "arm_pwm.h"

/* main.c 里定义的定时器句柄，这里只声明不定义 */
extern TIM_HandleTypeDef htim2;

/* 关节编号 → TIM2 通道。顺序必须和 g_arm_joint[] / 引脚表一致 */
static const uint32_t s_channel[ARM_JOINT_NUM] =
{
  TIM_CHANNEL_1,   /* J1 → PA0 */
  TIM_CHANNEL_2,   /* J2 → PA1 */
  TIM_CHANNEL_3,   /* J3 → PA2 */
  TIM_CHANNEL_4,   /* J4 → PA3 */
};

/* ------------------------------------------------------------------------- */
/* 内部：把 CCR 直接写进比较寄存器                                            */
/* ------------------------------------------------------------------------- */
static void arm_write_ccr(uint8_t joint, uint16_t ccr)
{
  __HAL_TIM_SET_COMPARE(&htim2, s_channel[joint], ccr);
}

/* ------------------------------------------------------------------------- */
/* 初始化：四路先给中位，再启动 PWM                                           */
/* ------------------------------------------------------------------------- */
void arm_init(void)
{
  uint8_t j;

  /* 第一步：先把比较值写好。必须在 HAL_TIM_PWM_Start() **之前**，
   * 否则通道一启动就会按寄存器里的旧值（可能是 0）输出一小段，
   * 舵机会抖一下甚至冲向极限。 */
  for (j = 0; j < ARM_JOINT_NUM; j++)
  {
    arm_write_ccr(j, ARM_PWM_US_TO_CCR(SERVO_US_HOME));
  }

  /* 第二步：启动四个通道 */
  for (j = 0; j < ARM_JOINT_NUM; j++)
  {
    (void)HAL_TIM_PWM_Start(&htim2, s_channel[j]);
  }

  /* 第三步：按标定表的零位再走一遍（有的关节零位不是 1500µs） */
  for (j = 0; j < ARM_JOINT_NUM; j++)
  {
    arm_set_deg(j, (float)g_arm_joint[j].deg_home);
  }
}

/* ------------------------------------------------------------------------- */
/* 逻辑角度 → 脉宽（µs）                                                      */
/* ------------------------------------------------------------------------- */
uint16_t arm_deg_to_us(uint8_t joint, float deg)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return SERVO_US_HOME;          /* 编号非法：给个中位，别输出奇怪的脉宽 */
  }

  const arm_joint_cfg_t *c = &g_arm_joint[joint];

  /* ① 角度先夹进该关节允许范围 */
  if (deg < (float)c->deg_min) { deg = (float)c->deg_min; }
  if (deg > (float)c->deg_max) { deg = (float)c->deg_max; }

  const int32_t span_deg = (int32_t)c->deg_max - (int32_t)c->deg_min;
  const int32_t span_us  = (int32_t)c->us_at_deg_max - (int32_t)c->us_at_deg_min;

  int32_t us;

  /* ② 线性插值。span_deg == 0 说明标定表写错了，退回下限值，不做除零 */
  if (span_deg == 0)
  {
    us = (int32_t)c->us_at_deg_min;
  }
  else
  {
    us = (int32_t)c->us_at_deg_min
       + (int32_t)(((float)span_us / (float)span_deg) * (deg - (float)c->deg_min));
  }

  /* ③ 夹进舵机安全窗：标定表填错也不会把舵机顶到堵转 */
  if (us < (int32_t)SERVO_US_SAFE_MIN) { us = (int32_t)SERVO_US_SAFE_MIN; }
  if (us > (int32_t)SERVO_US_SAFE_MAX) { us = (int32_t)SERVO_US_SAFE_MAX; }

  return (uint16_t)us;
}

/* ------------------------------------------------------------------------- */
/* 逻辑角度 → CCR                                                             */
/* ------------------------------------------------------------------------- */
uint16_t arm_deg_to_ccr(uint8_t joint, float deg)
{
  return ARM_PWM_US_TO_CCR(arm_deg_to_us(joint, deg));
}

/* ------------------------------------------------------------------------- */
/* 设置关节角度                                                               */
/* ------------------------------------------------------------------------- */
void arm_set_deg(uint8_t joint, float deg)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return;
  }
  arm_write_ccr(joint, arm_deg_to_ccr(joint, deg));
}

/* ------------------------------------------------------------------------- */
/* 直接按脉宽设置（标定时用，跳过角度换算）                                    */
/* ------------------------------------------------------------------------- */
void arm_set_us(uint8_t joint, uint16_t us)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return;
  }
  /* 同样夹安全窗，标定时乱填也不会烧舵机 */
  if (us < SERVO_US_SAFE_MIN) { us = SERVO_US_SAFE_MIN; }
  if (us > SERVO_US_SAFE_MAX) { us = SERVO_US_SAFE_MAX; }

  arm_write_ccr(joint, ARM_PWM_US_TO_CCR(us));
}

/* ------------------------------------------------------------------------- */
/* 读回当前角度（由 CCR 反算，调试/回帧用）                                    */
/* ------------------------------------------------------------------------- */
float arm_get_deg(uint8_t joint)
{
  if (joint >= ARM_JOINT_NUM)
  {
    return 0.0f;
  }

  const arm_joint_cfg_t *c = &g_arm_joint[joint];

  const uint16_t ccr = (uint16_t)__HAL_TIM_GET_COMPARE(&htim2, s_channel[joint]);
  const int32_t  us  = (int32_t)ARM_PWM_CCR_TO_US(ccr);

  const int32_t span_us  = (int32_t)c->us_at_deg_max - (int32_t)c->us_at_deg_min;
  const int32_t span_deg = (int32_t)c->deg_max - (int32_t)c->deg_min;

  if (span_us == 0)
  {
    return (float)c->deg_min;
  }

  float deg = (float)c->deg_min
            + ((float)(us - (int32_t)c->us_at_deg_min) * (float)span_deg / (float)span_us);

  if (deg < (float)c->deg_min) { deg = (float)c->deg_min; }
  if (deg > (float)c->deg_max) { deg = (float)c->deg_max; }

  return deg;
}
