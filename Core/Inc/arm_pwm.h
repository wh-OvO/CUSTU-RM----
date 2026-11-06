/**
  ******************************************************************************
  * @file    arm_pwm.h
  * @brief   四路舵机 PWM 的对外接口：角度 ↔ 脉宽 ↔ CCR
  ******************************************************************************
  * @note    调用顺序（正常开机时 main.c 已经做好了）：
  *            1. CubeMX 生成的 MX_TIM2_Init()   配好 PSC/ARR/通道
  *            2. arm_init()                     四路给中位 + 启动 PWM
  *            3. arm_set_deg(j, 角度)           之后随便调
  *
  *          角度单位是**度（float）**，不是 0.1°。
  *          0.1° 的整数编码是通信层的事（0x10/0x20 帧），别混进来。
  ******************************************************************************
  */
#ifndef ARM_PWM_H
#define ARM_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  初始化四路舵机：全部先输出中位脉宽，然后启动 PWM
  * @note   上电必须做这一步。CubeMX 生成的代码只把 CH1 配成 3000，
  *         CH2/CH3/CH4 是 0，不补的话另外三路占空比为 0，舵机会乱走。
  */
void arm_init(void);

/**
  * @brief  按逻辑角度设置某个关节
  * @param  joint 关节编号 0~3（J1~J4）
  * @param  deg   逻辑角度，单位「度」；超出 deg_min/deg_max 会被自动夹住
  * @note   内部会把角度换算成脉宽，再夹进安全窗，最后写 CCR
  */
void arm_set_deg(uint8_t joint, float deg);

/**
  * @brief  直接按脉宽设置某个关节（标定时用这个，跳过角度换算）
  * @param  joint 关节编号 0~3
  * @param  us    脉宽，单位 µs；超出安全窗会被夹住
  */
void arm_set_us(uint8_t joint, uint16_t us);

/**
  * @brief  读回某个关节当前的逻辑角度（由 CCR 反算）
  * @param  joint 关节编号 0~3
  * @retval 逻辑角度（度）；编号非法返回 0
  */
float arm_get_deg(uint8_t joint);

/**
  * @brief  逻辑角度 → 脉宽（µs），已夹在 deg 范围和脉宽安全窗内
  */
uint16_t arm_deg_to_us(uint8_t joint, float deg);

/**
  * @brief  逻辑角度 → TIM2 CCR 比较值
  */
uint16_t arm_deg_to_ccr(uint8_t joint, float deg);

#ifdef __cplusplus
}
#endif

#endif /* ARM_PWM_H */
