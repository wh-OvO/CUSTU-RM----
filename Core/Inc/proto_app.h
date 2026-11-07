/**
  ******************************************************************************
  * @file    proto_app.h
  * @brief   T5a 应用层：0x10 周期上报 + 0x20/0x21/0x22/0x23/0x24 消息处理
  ******************************************************************************
  * @note    分工：
  *            proto_link.c  只管"字节怎么进出 USB"
  *            proto_app.c   只管"帧里的意思是什么"（模式、目标、急停、超时）
  *
  *          两者都不该直接碰 PWM —— 舵机由 arm_pwm.c 负责。
  *          这样以后改协议不动舵机代码，换舵机也不动协议代码。
  ******************************************************************************
  */
#ifndef PROTO_APP_H
#define PROTO_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 上电初始化：把目标角对齐到各关节零位，状态清零 */
void proto_app_init(void);

/** 主循环里每轮调用一次（内部自己按时间戳决定何时发帧，不需要定时器） */
void proto_app_poll(void);

/** 当前状态位（正本 §4.2 的 bit0~bit5），供调试/断言使用 */
uint8_t proto_app_status(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTO_APP_H */
