/**
  ******************************************************************************
  * @file    as5600.c
  * @brief   AS5600 磁编码器驱动（硬件 I²C1，400kHz）
  ******************************************************************************
  * @note    历史：2026-11-09 之前 I²C1 没打开时这里是个占位实现。
  *          T4 把 MPU6050 接上同一条总线后，AS5600 也一并换成真实现了 ——
  *          两颗器件地址不同（AS5600 = 0x36，MPU6050 = 0x68），不冲突。
  *
  *          现在即使 AS5600 没接，也只是 as5600_init() 返回 0；
  *          只有 J1 的 kind 改成 ARM_JOINT_VEL 时才算底座故障（见 arm_joint.c）。
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

#include "main.h"           /* HAL / I2C_HandleTypeDef */
#include "as5600.h"

/* hi2c1 定义在 main.c（CubeMX 生成），这里只声明 */
extern I2C_HandleTypeDef hi2c1;

/* 1 = 真正走 I²C 读芯片；0 = 占位，永远返回"不在线"。
 * I²C1 已在 T0 由 CubeMX 生成（PB6/PB7，400kHz），所以现在恒为 1。 */
#define AS5600_HW_READY       1

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

    /* 能读寄存器不等于能测角度：MD=0 说明芯片前面没有磁铁，
     * 这时 ANGLE 寄存器给的是噪声，当成"不在线"处理更安全。 */
    if ((status & 0x20u) == 0u)
    {
      return 0u;
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
