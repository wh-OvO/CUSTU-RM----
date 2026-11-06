/**
  ******************************************************************************
  * @file    arm_cfg.h
  * @brief   机械臂 PWM 映射的**全部可调参数**集中在这里
  *
  *          任务书 T1 原文要求：「映射参数写在配置文件或宏里，不散落在代码中」。
  *          所以本工程约定：**角度/脉宽/时基这类数字，只允许出现在本文件**。
  *          别的 .c 里如果看到 500、2400 这种魔数，就是写错了。
  ******************************************************************************
  * @note    看这份文件之前，先记住这三个数，后面全由它们推出来：
  *
  *            72 MHz   TIM2 的输入时钟（APB1 定时器时钟 = HCLK = 72MHz）
  *            35       PSC，预分频寄存器值 → 实际分频 = PSC + 1 = 36
  *            39999    ARR，自动重装寄存器值 → 实际周期 = ARR + 1 = 40000 个计数
  *
  *          于是：
  *            计数频率 = 72MHz ÷ 36      = 2 MHz   → 1 个计数 = 0.5 µs
  *            PWM 周期 = 40000 × 0.5µs   = 20 ms   → 50 Hz    （舵机要的帧率）
  *            占空比   = CCR ÷ 40000
  *
  *          因为 1 个计数正好 0.5µs，脉宽和 CCR 就是 2 倍关系：
  *            CCR = 脉宽(µs) × 2
  *              1500µs → CCR 3000   （中位）
  *               600µs → CCR 1200   （安全窗一端）
  *              2400µs → CCR 4800   （安全窗另一端）
  *
  *          分辨率 0.5µs/步，比任务书要求的「步进不大于 2µs」细 4 倍。
  *
  * @warning 改了 PSC / ARR 里的任何一个，都必须同步去 CubeMX 的 TIM2 参数里改，
  *          否则这里算出来的数和芯片里真实跑的时基对不上（.ioc 才是真相）。
  ******************************************************************************
  */
#ifndef ARM_CFG_H
#define ARM_CFG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 一、PWM 时基
 *     这几项必须和 CubeMX 里 TIM2 的 Parameter Settings 完全一致
 * ========================================================================== */

#define ARM_PWM_TIM_CLK_HZ   72000000u   /* TIM2 输入时钟：APB1 定时器时钟 */
#define ARM_PWM_PSC          35u         /* CubeMX: Prescaler (PSC - 1...) = 35 */
#define ARM_PWM_ARR          39999u      /* CubeMX: Counter Period = 39999 */

/* 计数频率 = 72MHz / (35+1) = 2MHz */
#define ARM_PWM_TICK_HZ      (ARM_PWM_TIM_CLK_HZ / (ARM_PWM_PSC + 1u))

/* 每 µs 多少个计数。2MHz → 2；如果是 1MHz 就是 1。下面的换算全靠它 */
#define ARM_PWM_TICKS_PER_US (ARM_PWM_TICK_HZ / 1000000u)

/* 防御：如果时基不是整数 µs，下面的整数换算会累积误差，直接编译期报错 */
#if (ARM_PWM_TICK_HZ % 1000000u) != 0u
#  error "ARM_PWM_TICK_HZ 必须是 1MHz 的整数倍，否则 US_TO_CCR 换算会丢精度"
#endif

/* 脉宽(µs) → CCR 比较值；CCR → 脉宽(µs) */
#define ARM_PWM_US_TO_CCR(us)   ((uint16_t)((uint32_t)(us) * ARM_PWM_TICKS_PER_US))
#define ARM_PWM_CCR_TO_US(ccr)  ((uint16_t)((uint32_t)(ccr) / ARM_PWM_TICKS_PER_US))

/* ==========================================================================
 * 二、SG90 的脉宽窗口
 *     标称 500~2500µs，但实测每只都有偏差。**先别用满量程**，
 *     先用下面这个安全窗把动作跑通，再逐只标定。
 * ========================================================================== */

#define SERVO_US_NOMINAL_MIN   500u    /* 标称下限（仅作参考，不要直接拿来限幅） */
#define SERVO_US_NOMINAL_MAX   2500u   /* 标称上限 */
#define SERVO_US_MID           1500u   /* 理论中位 */

#define SERVO_US_SAFE_MIN      600u    /* 安全窗下限：比标称收进 100µs */
#define SERVO_US_SAFE_MAX      2400u    /* 安全窗上限：比标称收进 100µs */

#define SERVO_US_HOME          1500u   /* 上电初始脉宽，四路都先给这个值 */

/* ==========================================================================
 * 三、关节标定表
 *
 *     每个关节 6 个数，含义见下面结构体注释。
 *     真正填数在 arm_cfg.c，那里一格一格写了怎么测。
 * ========================================================================== */

#define ARM_JOINT_NUM   4       /* 不用 4u：让 for 循环的 uint8_t 比较不触发 -Wsign-compare */

/**
  * @brief 一个关节的标定参数
  *
  *  「逻辑角度」是我们在软件里自己定义的角度，可以不是 0~180。
  *  比如夹爪只开合 60°，就写 deg_min = 0、deg_max = 60。
  *  好处是上层协议（角度 0.1° 的 int16）始终是同一套语义，不用管机械。
  *
  *  **方向不需要单独一个字段**：如果某个关节装反了，
  *  把 us_at_deg_min 和 us_at_deg_max 两个值对调即可，
  *  线性插值会自动反向（下面的换算是通用的，`us_at_deg_max < us_at_deg_min`
  *  也能正确算）。
  */
typedef struct
{
  const char *name;          /* 关节名，只用于串口打印，比如 "J1 底座" */

  int16_t  deg_min;          /* 逻辑角度下限（度），比如  0 */
  int16_t  deg_max;          /* 逻辑角度上限（度），比如 180 */
  int16_t  deg_home;         /* 零位 / 上电位（度），一般取行程中点 */

  uint16_t us_at_deg_min;    /* 走到 deg_min 时输出多少 µs（实测值） */
  uint16_t us_at_deg_max;    /* 走到 deg_max 时输出多少 µs（实测值） */

} arm_joint_cfg_t;

/** 四个关节的标定表，定义在 arm_cfg.c */
extern const arm_joint_cfg_t g_arm_joint[ARM_JOINT_NUM];

#ifdef __cplusplus
}
#endif

#endif /* ARM_CFG_H */
