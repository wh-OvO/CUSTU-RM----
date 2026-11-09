/**
  ******************************************************************************
  * @file    as5600.c
  * @brief   AS5600 磁编码器驱动 —— **目前是占位实现，硬件还没到**
  ******************************************************************************
  * @note    为什么先写个占位：
  *
  *          1) 360° 连续旋转舵机还没买，AS5600 也没到；
  *          2) 更关键的：CubeMX 里 **I²C1 还没打开**（T4 的 MPU6050 也没做），
  *             所以现在工程里根本没有 hi2c1 这个东西，直接写 HAL 调用编译不过。
  *
  *          占位实现永远返回"不在线"，于是 arm_joint.c 会把底座判定为
  *          "没有角度反馈" —— 但因为 arm_cfg.c 里 J1 现在是 ARM_JOINT_POS，
  *          闭环那一段根本不会跑，所以不影响任何现有功能。
  *
  * @note    **等硬件到了，按下面三步换成真实现：**
  *
  *          1. CubeMX → Connectivity → I2C1 → I2C，Speed Mode 选 Fast Mode，
  *             Clock Speed 填 400000（任务书 1.3 要求支持 400kHz）。
  *             GENERATE CODE 后记得修 .ld 文件（CubeMX 6.12 的老 bug）。
  *          2. 本文件把 AS5600_HW_READY 改成 1。
  *          3. 确认 ARM_I2C_TIMEOUT_MS 不要太长 —— 底座闭环每 20ms 就要读一次，
  *             一次读阻塞 50ms 会把整个控制节拍拖垮。
  ******************************************************************************
  * @note    AS5600 寄存器速查（数据手册里都能查，别背）：
  *
  *            0x0B  STATUS   bit5 = MD（检测到磁铁） bit3 = ML（磁场太弱）
  *            0x0C  RAW ANGLE 高 8 位（12 位原始值，未做任何处理）
  *            0x0D  RAW ANGLE 低 4 位在 bit7:4
  *            0x0E  ANGLE    高 8 位（已应用 ZPOS/MPOS 零点与量程，一般用这个）
  *            0x0F  ANGLE    低 4 位在 bit7:4
  *
  *          12 位 = 0~4095 对应 0~360°：
  *            角度(度) = 原始值 × 360 / 4096 = 原始值 × 0.087890625
  *
  *          I²C 器件地址 0x36，HAL 里要左移一位 → 0x6C（见下面的 AS5600_ADDR）
  ******************************************************************************
  */

#include "as5600.h"

/* 1 = 真正走 I²C 读芯片；0 = 占位，永远返回"不在线"。
 * 硬件到货 + CubeMX 打开 I2C1 之后改成 1。 */
#define AS5600_HW_READY       0

/* 12 位满量程 */
#define AS5600_RAW_MAX        4096u
#define AS5600_DEG_PER_LSB    (360.0f / (float)AS5600_RAW_MAX)

/* 单次 I²C 事务的超时。**必须短**：底座闭环 20ms 一个节拍，
 * 一次读阻塞太久整个控制节拍就崩了。 */
#define AS5600_I2C_TIMEOUT_MS 2u

#define AS5600_REG_STATUS     0x0Bu
#define AS5600_REG_ANGLE_H    0x0Eu
#define AS5600_ADDR           (0x36u << 1)   /* HAL 要求已左移的 8 位地址 */

static uint8_t s_present;

uint8_t as5600_init(void)
{
  s_present = 0u;

#if AS5600_HW_READY
  {
    uint8_t status = 0xFFu;

    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_REG_STATUS,
                         I2C_MEMADD_SIZE_8BIT, &status, 1u,
                         AS5600_I2C_TIMEOUT_MS) != HAL_OK)
    {
      return 0u;                       /* 芯片没应答：没接 / 地址错 / 上拉没接 */
    }
    s_present = 1u;
  }
#endif

  return s_present;
}

uint8_t as5600_read_deg(float *deg)
{
  if (deg == 0)
  {
    return 0u;
  }

#if AS5600_HW_READY
  {
    uint8_t  buf[2];
    uint16_t raw;

    if (s_present == 0u)
    {
      return 0u;
    }

    /* 一次连续读 2 字节：高 8 位在 0x0E，低 4 位在 0x0F 的 bit7:4 */
    if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_REG_ANGLE_H,
                         I2C_MEMADD_SIZE_8BIT, buf, 2u,
                         AS5600_I2C_TIMEOUT_MS) != HAL_OK)
    {
      return 0u;                       /* 读失败：调用者必须保持上一次的脉宽，别乱动 */
    }

    raw = (uint16_t)(((uint16_t)buf[0] << 4) | ((uint16_t)buf[1] >> 4));
    *deg = (float)raw * AS5600_DEG_PER_LSB;
    return 1u;
  }
#else
  (void)deg;                           /* 占位实现：不改 *deg，只报告失败 */
  return 0u;
#endif
}

uint8_t as5600_present(void)
{
  return s_present;
}
