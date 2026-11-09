/**
  ******************************************************************************
  * @file    arm_joint.h
  * @brief   关节执行层：**四个关节对上长得一样，底下不一样**
  ******************************************************************************
  * @note    为什么要有这一层
  *
  *          三个关节（J2/J3/J4）是位置舵机：给角度 → 算脉宽 → 写 CCR，一步到位。
  *          底座（J1）是 360° 连续旋转舵机：脉宽控制的是**转速**，它自己不知道
  *          转到哪，只能"读编码器 → 算误差 → 输出转速"这样闭环追。
  *
  *          如果让插补器自己判断"这个关节是哪种"，那插补器里就到处是 if。
  *          所以把差异全塞进这一层：**上层只管给目标角度，不关心怎么执行。**
  *
  *          分层：
  *
  *              proto_app.c  ── 协议 + 插补器（只产生"角度"）
  *                    │
  *                    │  arm_joint_set_deg() / arm_joint_set_traj()
  *                    ▼
  *              arm_joint.c  ── **本层**：按 kind 分叉
  *                    │                    │
  *                    ▼                    ▼
  *              arm_pwm.c            as5600.c
  *              （脉宽↔CCR）          （I²C 读角度）
  *
  *          arm_pwm.c 完全不知道 AS5600 的存在，这就是分层做对了的标志。
  *
  * @warning **两个关节的语义不完全一样，用之前必须知道：**
  *
  *          |                    | J2/J3/J4（位置型）        | J1（速度型闭环）              |
  *          |--------------------|---------------------------|------------------------------|
  *          | set 之后立刻 get    | 返回**目标角**（已生效）   | 返回**当前实测角**（还没动）   |
  *          | 谁来执行            | 舵机内部，软件不参与       | arm_joint_tick() 每 20ms 推一次 |
  *          | "到位"怎么判        | 写完就算到                | |误差| < 死区 才算到            |
  *          | 可能失败吗          | 不会                      | 会（I²C 挂了就停转 + 报故障）  |
  *
  *          所以千万别写"set 完立刻 get 回来校验"这种代码 —— 在 J1 上必错。
  ******************************************************************************
  */
#ifndef ARM_JOINT_H
#define ARM_JOINT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  初始化关节层
  * @note   必须在 arm_init()（PWM 层）之后调用。
  *         内部会把四个关节的目标都设成各自的 deg_home，并尝试探测 AS5600。
  *         AS5600 不在线**不是错误** —— 只有当某个关节的 kind 是 VEL 时才判定故障。
  */
void arm_joint_init(void);

/**
  * @brief  设定某个关节的目标角度（不带速度前馈，一步到位用）
  * @param  joint 关节编号 0~3
  * @param  deg   目标角度，单位「度」
  * @note   位置型：立即写 CCR，函数返回时已经生效。
  *         速度型：只记下目标，等 arm_joint_tick() 去追。
  */
void arm_joint_set_deg(uint8_t joint, float deg);

/**
  * @brief  设定目标角度 + 目标速度（**插补器专用**）
  * @param  joint 关节编号 0~3
  * @param  deg   目标角度（度）—— 这是"这一小步该到的位置"
  * @param  v_dps 目标速度（度/秒）—— 速度型关节用它算前馈
  * @note   位置型忽略 v_dps（用不上，写死 0 也行）。
  *         速度型拿 v_dps 当**前馈**：光靠 P 追斜坡会稳定落后几十度，
  *         前馈把"该出多大力"直接算好，P 只负责收尾。
  */
void arm_joint_set_traj(uint8_t joint, float deg, float v_dps);

/**
  * @brief  读某个关节的**实际**角度
  * @param  joint 关节编号 0~3
  * @retval 角度（度）
  * @note   位置型：由 CCR 反算（没有编码器，这是能做到的最好的）
  *         速度型：读 AS5600 真实角度；读失败时退回目标值，保证不返回垃圾
  */
float arm_joint_get_deg(uint8_t joint);

/**
  * @brief  读某一路当前脉宽（µs），供 0x32 回帧用
  * @note   直接读 CCR，不做任何角度换算 —— 标定时要的就是这个"原始真相"
  */
uint16_t arm_joint_get_us(uint8_t joint);

/**
  * @brief  周期任务：推进所有速度型关节的闭环
  * @param  now 当前毫秒计数（HAL_GetTick()）
  * @note   **每 20ms 调一次**，和插补器同频。位置型关节在这里什么都不做。
  *         顺序要求：先 arm_joint_set_deg()（插补），再调用本函数（执行）。
  */
void arm_joint_tick(uint32_t now);

/**
  * @brief  进入/退出"手动脉宽"模式（0x31 调试帧用）
  * @param  joint 关节编号 0~3
  * @param  on    1 = 手动占用（闭环暂停，别再抢脉宽）；0 = 交还闭环
  * @note   手动模式下滑块给的脉宽不会被下一拍闭环覆盖，
  *         这是标定流程能用的前提。
  */
void arm_joint_set_manual(uint8_t joint, uint8_t on);

/**
  * @brief  让所有速度型关节**原地停住**
  * @note   做法是把它们的闭环目标改成"当前实测角度"。
  *         0x31 调试帧要求"其余舵机保持"（flags bit1），就是靠这个实现 ——
  *         只停插补还不够，底座的闭环会继续朝老目标跑。
  */
void arm_joint_hold_position(void);

/**
  * @brief  底座角度反馈是否故障
  * @retval 1 = AS5600 读不到（此时底座已经停转）；0 = 正常
  * @note   对应状态帧的 bit4（I²C 失效）。和 T4 的 MPU6050 共用这个故障位。
  */
uint8_t arm_joint_base_fault(void);

/**
  * @brief  有没有速度型关节还在收敛（没到位）
  * @retval 1 = 还在动；0 = 到位或不需要判
  * @note   给状态帧 bit3"运动中"用。插补走完不代表真的到位 ——
  *         底座闭环还要一点时间收尾，位图要如实反映。
  */
uint8_t arm_joint_moving(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_JOINT_H */
