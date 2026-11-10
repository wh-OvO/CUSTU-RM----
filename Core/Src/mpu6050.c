/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   T4 姿态传感器 MPU6050 驱动（硬件 I²C1，400kHz）
  ******************************************************************************
  * @note    **为什么这个文件不碰 PWM / 不碰协议**
  *
  *          分层和 as5600.c 一样：驱动只负责"把芯片里的数变成物理量"。
  *          什么时候读、故障了要不要停、pitch 填进哪一帧，都是 proto_app.c 的事。
  *          这样换传感器（比如换成 ICM20602）只改这一个文件。
  *
  * @note    **寄存器速查**（数据手册 MPU-6000/6050 Register Map）
  *
  *            0x19 SMPLRT_DIV   采样率分频：输出率 = 1kHz / (1 + 值)
  *            0x1A CONFIG       DLPF：0x03 = 加速度 44Hz / 陀螺 42Hz
  *            0x1B GYRO_CONFIG  0x08 = ±500°/s（65.5 LSB/(°/s)）
  *            0x1C ACCEL_CONFIG 0x00 = ±2g（16384 LSB/g）
  *            0x3B ACCEL_XOUT_H 从这里连续读 14 字节：
  *                                ax,ay,az,temp,gx,gy,gz（各 2 字节，大端）
  *            0x6B PWR_MGMT_1   0x80 = 软复位；0x01 = 唤醒 + 选 X 轴陀螺 PLL
  *            0x6C PWR_MGMT_2   0x00 = 六轴都不待机
  *            0x75 WHO_AM_I     固定 0x68
  *
  * @note    **为什么用软复位**
  *
  *          调试器复位 / 热复位不会给传感器断电，它可能还停在上一次配置
  *          （甚至是 sleep）。上电先 0x80 软复位、等 100ms、再按本工程的
  *          量程重新配一遍，才能保证每次跑起来寄存器状态一致。
  *
  * @note    **掉线重连为什么不用 mpu6050_init()**
  *
  *          init() 里有 HAL_Delay(100) —— 在主循环里等 100ms 会把 50Hz 的
  *          控制节拍直接打崩（少发 5 帧、底座闭环少跑 5 拍）。
  *          所以恢复走 init_fast()：只探 WHO_AM_I + 写配置寄存器，
  *          唤醒后靠 s_ms_ready 等 50ms（非阻塞），期间不读数据。
  ******************************************************************************
  */

#include "main.h"           /* HAL / I2C_HandleTypeDef / HAL_Delay */
#include "mpu6050.h"

#include <math.h>           /* sqrtf / fabsf */
#include <string.h>         /* memset */

/* hi2c1 定义在 main.c（CubeMX 生成）。这里只声明，不重复定义。
 * 和 AS5600 共用同一条总线：两个器件地址不同（0x68 / 0x36），
 * 主循环里所有 I²C 调用都是顺序的，不存在并发抢总线。 */
extern I2C_HandleTypeDef hi2c1;

/* ---- 寄存器地址 ---- */
#define MPU6050_REG_SMPLRT_DIV     0x19u
#define MPU6050_REG_CONFIG         0x1Au
#define MPU6050_REG_GYRO_CONFIG    0x1Bu
#define MPU6050_REG_ACCEL_CONFIG   0x1Cu
#define MPU6050_REG_ACCEL_XOUT_H   0x3Bu
#define MPU6050_REG_PWR_MGMT_1     0x6Bu
#define MPU6050_REG_PWR_MGMT_2     0x6Cu
#define MPU6050_REG_WHO_AM_I       0x75u

/* 单次 I²C 事务超时。400kHz 下读 14 字节约 0.4ms，2ms 足够；
 * 太长会拖慢控制节拍，太短会在总线被拉低时误判。 */
#define MPU6050_I2C_TIMEOUT_MS     2u

/* 连续失败几次才判定"这一轮读废了"。3 次 × 20ms = 60ms，
 * 单次偶发 NAK 不至于让整机停。 */
#define MPU6050_FAIL_LIMIT         3u

/* ---- 状态 ---- */
static mpu6050_sample_t s_sample;          /* 最近一次成功采样的副本 */
static uint8_t  s_present;                 /* 1 = 当前能正常通信 */
static uint8_t  s_ever_ok;                 /* 1 = 本次上电曾经通信成功过 */
static uint8_t  s_sample_valid;            /* 1 = s_sample 里有有效数据 */
static uint8_t  s_fail_streak;             /* 连续读失败次数 */
static uint8_t  s_probe_fail;              /* 连续探测失败次数（用于总线恢复） */

static uint32_t s_ms_last_ok;              /* 最近一次成功读数的时刻 */
static uint32_t s_ms_read;                 /* 上次采样的时刻（限速用） */
static uint32_t s_ms_retry;                /* 上次重连探测的时刻 */
static uint32_t s_ms_ready;                /* 唤醒后要等到这个时刻才允许读 */

/* 倾角滤波 */
static float    s_pitch;
static uint8_t  s_pitch_ready;

/* 零偏标定 */
static uint8_t  s_cal_state;
static uint8_t  s_cal_reason;
static uint32_t s_cal_ms0;
static uint32_t s_cal_n;
static int32_t  s_cal_sum[3];
static float    s_bias[3];                 /* °/s */

/* ==========================================================================
 * 一、I²C 小工具
 * ========================================================================== */

static uint8_t i2c_write(uint8_t reg, uint8_t val)
{
  return (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            &val, 1u, MPU6050_I2C_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
}

static uint8_t i2c_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           buf, len, MPU6050_I2C_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
}

/**
  * @brief  总线恢复：把 I2C1 外设复位重配
  * @note   拔 SDA 时从机可能把 SDA 一直拉低，HAL 会卡在 BUSY；
  *         只重试读写是没用的，必须 DeInit/Init 把外设状态清掉。
  *         只在"曾经连上过、现在掉了"的时候做，避免没装传感器时
  *         每 500ms 折腾一次总线（会干扰同总线的 AS5600）。
  */
static void bus_recover(void)
{
  (void)HAL_I2C_DeInit(&hi2c1);
  (void)HAL_I2C_Init(&hi2c1);
}

/* ==========================================================================
 * 二、配置与探测
 * ========================================================================== */

static uint8_t probe(void)
{
  uint8_t who = 0u;

  if (i2c_read(MPU6050_REG_WHO_AM_I, &who, 1u) == 0u)
  {
    return 0u;
  }

  return (who == MPU6050_WHO_AM_I_VAL) ? 1u : 0u;
}

/**
  * @brief  写一整组配置（不含软复位）
  * @note   顺序有讲究：先唤醒（PWR_MGMT_1），再写其它寄存器。
  *         芯片在 sleep 状态下配置寄存器仍可写，但先唤醒更稳妥。
  */
static uint8_t configure(void)
{
  if (i2c_write(MPU6050_REG_PWR_MGMT_1, 0x01u) == 0u)   /* 唤醒，时钟选 X 轴陀螺 PLL */
  {
    return 0u;
  }

  if (i2c_write(MPU6050_REG_PWR_MGMT_2, 0x00u) == 0u)   /* 六轴都不待机 */
  {
    return 0u;
  }

  if (i2c_write(MPU6050_REG_SMPLRT_DIV, 0x04u) == 0u)   /* 1kHz/(1+4) = 200Hz */
  {
    return 0u;
  }

  if (i2c_write(MPU6050_REG_CONFIG, 0x03u) == 0u)       /* DLPF 44/42Hz */
  {
    return 0u;
  }

  if (i2c_write(MPU6050_REG_GYRO_CONFIG, 0x08u) == 0u)  /* ±500°/s */
  {
    return 0u;
  }

  if (i2c_write(MPU6050_REG_ACCEL_CONFIG, 0x00u) == 0u) /* ±2g */
  {
    return 0u;
  }

  return 1u;
}

uint8_t mpu6050_init(void)
{
  const uint32_t now = HAL_GetTick();

  s_present      = 0u;
  s_ever_ok      = 0u;
  s_sample_valid = 0u;
  s_fail_streak  = 0u;
  s_probe_fail   = 0u;
  s_pitch        = 0.0f;
  s_pitch_ready  = 0u;
  s_cal_state    = MPU6050_CAL_IDLE;
  s_cal_reason   = MPU6050_CAL_OK;
  s_bias[0]      = 0.0f;
  s_bias[1]      = 0.0f;
  s_bias[2]      = 0.0f;
  s_ms_last_ok   = now;
  s_ms_read      = now;
  s_ms_retry     = now;
  s_ms_ready     = now;

  memset(&s_sample, 0, sizeof(s_sample));

  if (probe() == 0u)
  {
    return 0u;                       /* 没接 / 地址不对 / 上拉没接，安静地返回 */
  }

  /* 软复位：把芯片拉回确定的初始状态。数据手册要求复位后等 100ms。 */
  (void)i2c_write(MPU6050_REG_PWR_MGMT_1, 0x80u);
  HAL_Delay(100u);

  if (configure() == 0u)
  {
    return 0u;
  }

  s_present    = 1u;
  s_ever_ok    = 1u;
  s_ms_last_ok = HAL_GetTick();
  s_ms_ready   = s_ms_last_ok + 50u; /* 唤醒后等 50ms 再读，前几帧可能是 0 */
  return 1u;
}

/**
  * @brief  非阻塞重连：探测 + 写配置，不软复位、不 HAL_Delay
  * @note   热插拔后芯片是刚上电的状态（默认 sleep、±2g/±250°/s），
  *         所以必须重新写一遍配置，不能只探到 WHO_AM_I 就开始读。
  */
static uint8_t init_fast(uint32_t now)
{
  if (probe() == 0u)
  {
    /* 曾经连上过：可能是拔线把总线拉死了，复位一次 I2C 外设。
     * 从来没连上过：不动总线，避免干扰 AS5600。 */
    if (s_ever_ok != 0u)
    {
      if (s_probe_fail < 0xFFu)
      {
        s_probe_fail++;
      }
      if (s_probe_fail >= 2u)
      {
        bus_recover();
        s_probe_fail = 0u;
      }
    }
    return 0u;
  }

  s_probe_fail = 0u;

  if (configure() == 0u)
  {
    return 0u;
  }

  s_present    = 1u;
  s_ever_ok    = 1u;
  s_fail_streak = 0u;
  s_ms_last_ok = now;
  s_ms_ready   = now + 50u;
  return 1u;
}

/* ==========================================================================
 * 三、采样与倾角
 * ========================================================================== */

/**
  * @brief  三角函数的近似实现（不用 libm 的 atan2f）
  * @note   为什么自己写：atan2f 在 newlib 里是几百字节到 1KB 的通用实现，
  *         FLASH 只剩 ~12KB，而倾角只需要 |x| ≤ 1 的多项式。
  *         下面这组系数在 |x| ≤ 1 内最大误差 < 1e-4 rad（约 0.006°），
  *         远好于验收要求的 3°。
  */
static float atan_approx(float x)
{
  const float half_pi = 1.57079632679f;
  const float ax      = fabsf(x);
  const float x2      = x * x;
  const float poly    = 0.9998660f + x2 * (-0.3302995f + x2 * (0.1801410f + x2 * (-0.0851330f + x2 * 0.0208351f)));

  if (ax <= 1.0f)
  {
    return x * poly;
  }

  /* |x| > 1：atan(x) = sign(x)·π/2 − atan(1/x) */
  {
    const float inv = 1.0f / x;
    const float i2  = inv * inv;
    const float p   = 0.9998660f + i2 * (-0.3302995f + i2 * (0.1801410f + i2 * (-0.0851330f + i2 * 0.0208351f)));

    return (x > 0.0f) ? (half_pi - inv * p) : (-half_pi - inv * p);
  }
}

/**
  * @brief  由加速度算绝对倾角
  * @note   默认约定：X 轴朝前、Z 轴朝上，抬头为正：
  *
  *             pitch = atan2(−ax, sqrt(ay² + az²))
  *
  *         分子取 −ax 是因为静止时加速度计测到的是"支撑力"方向（+1g 朝上），
  *         不是重力方向。推导见 docs/T4_MPU6050_说明.md。
  */
static float accel_pitch_deg(void)
{
  float fwd;
  float lat;
  float up;
  float horiz;

#if (MPU6050_PITCH_AXIS == MPU6050_PITCH_AXIS_Y)
  fwd = -s_sample.ay_g;
  lat = s_sample.ax_g;
#else
  fwd = -s_sample.ax_g;
  lat = s_sample.ay_g;
#endif
  up = s_sample.az_g;

  horiz = sqrtf((lat * lat) + (up * up));

  /* 分母加一个极小值，防止绝对垂直时除零。这个角度下 atan 已经饱和到 ±90°。 */
  return atan_approx(fwd / (horiz + 1.0e-6f)) * 57.2957795131f;
}

static void update_pitch(void)
{
  float p = (accel_pitch_deg() * MPU6050_PITCH_SIGN) + MPU6050_PITCH_OFFSET_DEG;

  if (s_pitch_ready == 0u)
  {
    s_pitch       = p;               /* 第一帧直接采纳，不从 0 慢慢爬 */
    s_pitch_ready = 1u;
  }
  else
  {
    s_pitch += MPU6050_PITCH_LPF * (p - s_pitch);
  }

  s_sample.pitch_deg = s_pitch;
}

/**
  * @brief  一次读 14 字节并解析
  * @retval 1 = 成功
  * @note   大端：高字节在前（和 STM32 的小端相反，不能直接指针强转）。
  */
static uint8_t read_sample(uint32_t now)
{
  uint8_t b[14];

  if (i2c_read(MPU6050_REG_ACCEL_XOUT_H, b, 14u) == 0u)
  {
    return 0u;
  }

  s_sample.ax = (int16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
  s_sample.ay = (int16_t)(((uint16_t)b[2] << 8) | (uint16_t)b[3]);
  s_sample.az = (int16_t)(((uint16_t)b[4] << 8) | (uint16_t)b[5]);
  s_sample.temp = (int16_t)(((uint16_t)b[6] << 8) | (uint16_t)b[7]);
  s_sample.gx = (int16_t)(((uint16_t)b[8] << 8) | (uint16_t)b[9]);
  s_sample.gy = (int16_t)(((uint16_t)b[10] << 8) | (uint16_t)b[11]);
  s_sample.gz = (int16_t)(((uint16_t)b[12] << 8) | (uint16_t)b[13]);

  s_sample.ax_g = (float)s_sample.ax / MPU6050_ACCEL_LSB_PER_G;
  s_sample.ay_g = (float)s_sample.ay / MPU6050_ACCEL_LSB_PER_G;
  s_sample.az_g = (float)s_sample.az / MPU6050_ACCEL_LSB_PER_G;

  /* 零偏只在标定完成后才有效；没标定时 s_bias = 0，报的是原始角速度 */
  s_sample.gx_dps = ((float)s_sample.gx / MPU6050_GYRO_LSB_PER_DPS) - s_bias[0];
  s_sample.gy_dps = ((float)s_sample.gy / MPU6050_GYRO_LSB_PER_DPS) - s_bias[1];
  s_sample.gz_dps = ((float)s_sample.gz / MPU6050_GYRO_LSB_PER_DPS) - s_bias[2];

  s_sample.ms = now;
  return 1u;
}

/* ==========================================================================
 * 四、零偏标定
 * ========================================================================== */

static void calib_accumulate(void)
{
  int32_t limit;

  if (s_cal_state != MPU6050_CAL_RUNNING)
  {
    return;
  }

  /* 用原始计数判运动，不受当前零偏影响 */
  limit = (int32_t)(MPU6050_CALIB_MAX_DPS * MPU6050_GYRO_LSB_PER_DPS);
  if ((s_sample.gx > limit) || (s_sample.gx < -limit) ||
      (s_sample.gy > limit) || (s_sample.gy < -limit) ||
      (s_sample.gz > limit) || (s_sample.gz < -limit))
  {
    s_cal_state  = MPU6050_CAL_ERROR;
    s_cal_reason = MPU6050_CAL_MOVING;
    return;
  }

  s_cal_sum[0] += s_sample.gx;
  s_cal_sum[1] += s_sample.gy;
  s_cal_sum[2] += s_sample.gz;
  s_cal_n++;

  if ((HAL_GetTick() - s_cal_ms0) >= MPU6050_CALIB_MS)
  {
    if (s_cal_n < 10u)
    {
      s_cal_state  = MPU6050_CAL_ERROR;
      s_cal_reason = MPU6050_CAL_I2C;
      return;
    }

    s_bias[0] = ((float)s_cal_sum[0] / (float)s_cal_n) / MPU6050_GYRO_LSB_PER_DPS;
    s_bias[1] = ((float)s_cal_sum[1] / (float)s_cal_n) / MPU6050_GYRO_LSB_PER_DPS;
    s_bias[2] = ((float)s_cal_sum[2] / (float)s_cal_n) / MPU6050_GYRO_LSB_PER_DPS;

    s_cal_state  = MPU6050_CAL_DONE;
    s_cal_reason = MPU6050_CAL_OK;
  }
}

uint8_t mpu6050_calib_start(void)
{
  /* 必须有"最近真读到过的一帧"才能开始：
   * 刚探测到 WHO_AM_I 但还没读出数据（唤醒等待期）时，s_present 是 1，
   * 可 s_sample 还是空的，这时开始标定会先累加几帧 0。 */
  if ((s_sample_valid == 0u) || (mpu6050_healthy() == 0u))
  {
    s_cal_state  = MPU6050_CAL_ERROR;
    s_cal_reason = MPU6050_CAL_NO_SENSOR;
    return 0u;
  }

  s_cal_state  = MPU6050_CAL_RUNNING;
  s_cal_reason = MPU6050_CAL_OK;
  s_cal_ms0    = HAL_GetTick();
  s_cal_n      = 0u;
  s_cal_sum[0] = 0;
  s_cal_sum[1] = 0;
  s_cal_sum[2] = 0;
  return 1u;
}

void mpu6050_calib_abort(void)
{
  if (s_cal_state == MPU6050_CAL_RUNNING)
  {
    s_cal_state  = MPU6050_CAL_IDLE;
    s_cal_reason = MPU6050_CAL_OK;
  }
}

uint8_t mpu6050_calib_state(void)
{
  return s_cal_state;
}

uint8_t mpu6050_calib_progress(void)
{
  uint32_t elapsed;
  uint32_t pct;

  if (s_cal_state == MPU6050_CAL_DONE)
  {
    return 100u;
  }
  if (s_cal_state != MPU6050_CAL_RUNNING)
  {
    return 0u;
  }

  elapsed = HAL_GetTick() - s_cal_ms0;
  if (elapsed >= MPU6050_CALIB_MS)
  {
    return 99u;                      /* 等下一拍补最后一个样本 */
  }

  pct = (elapsed * 100u) / MPU6050_CALIB_MS;
  return (uint8_t)pct;
}

uint8_t mpu6050_calib_reason(void)
{
  return s_cal_reason;
}

void mpu6050_calib_get_bias(float *bx, float *by, float *bz)
{
  if (bx != 0)
  {
    *bx = s_bias[0];
  }
  if (by != 0)
  {
    *by = s_bias[1];
  }
  if (bz != 0)
  {
    *bz = s_bias[2];
  }
}

/* ==========================================================================
 * 五、周期任务
 * ========================================================================== */

void mpu6050_tick(uint32_t now)
{
  /* 数据"过期"处理：距上次成功超过 LOST_MS 就把缓存判废。
   * 这一步必须独立于"当前是否还在读" —— 掉线后 s_present 会变 0，
   * 但 s_sample 里还留着最后一帧旧值，不判废的话 0x30 会拿旧数据假装还在。 */
  if ((s_sample_valid != 0u) && ((now - s_ms_last_ok) > MPU6050_LOST_MS))
  {
    s_sample_valid = 0u;
    s_pitch        = 0.0f;
    s_pitch_ready  = 0u;
    memset(&s_sample, 0, sizeof(s_sample));
  }

  /* 限速：proto_app_poll() 每个主循环都会调，但 I²C 只按 50Hz 读 */
  if ((now - s_ms_read) < MPU6050_TICK_MS)
  {
    return;
  }
  s_ms_read = now;

  /* ---- 当前不在线：每 500ms 试着重新探测一次（支持热插拔）---- */
  if (s_present == 0u)
  {
    if ((now - s_ms_retry) >= MPU6050_RETRY_MS)
    {
      s_ms_retry = now;
      (void)init_fast(now);
    }
    return;
  }

  /* ---- 刚唤醒，等芯片稳定 ---- */
  if ((int32_t)(now - s_ms_ready) < 0)
  {
    return;
  }

  /* ---- 正常读 ---- */
  if (read_sample(now) != 0u)
  {
    s_fail_streak  = 0u;
    s_ms_last_ok   = now;
    s_sample_valid = 1u;

    update_pitch();
    calib_accumulate();
  }
  else
  {
    if (s_fail_streak < 0xFFu)
    {
      s_fail_streak++;
    }

    if (s_fail_streak >= MPU6050_FAIL_LIMIT)
    {
      /* 判定掉线：停读，转去重连流程。故障位由 mpu6050_faulted() 按
       * "距上次成功超过 LOST_MS" 统一算，这里不用重复置位。 */
      s_present     = 0u;
      s_fail_streak = 0u;
      s_ms_retry    = now;

      if (s_cal_state == MPU6050_CAL_RUNNING)
      {
        s_cal_state  = MPU6050_CAL_ERROR;
        s_cal_reason = MPU6050_CAL_I2C;
      }
    }
  }
}

/* ==========================================================================
 * 六、查询接口
 * ========================================================================== */

uint8_t mpu6050_get_sample(mpu6050_sample_t *out)
{
  if (out == 0)
  {
    return 0u;
  }

  *out = s_sample;
  return s_sample_valid;
}

uint8_t mpu6050_present(void)
{
  return s_present;
}

uint8_t mpu6050_healthy(void)
{
  if (s_present == 0u)
  {
    return 0u;
  }

  return (uint8_t)(((HAL_GetTick() - s_ms_last_ok) <= MPU6050_LOST_MS) ? 1u : 0u);
}

uint8_t mpu6050_faulted(void)
{
  if (s_ever_ok == 0u)
  {
    /* 从来没连上过：按"是否必须有"决定算不算故障 */
    return (uint8_t)((MPU6050_REQUIRED != 0) ? 1u : 0u);
  }

  return (uint8_t)(((HAL_GetTick() - s_ms_last_ok) > MPU6050_LOST_MS) ? 1u : 0u);
}

float mpu6050_pitch_deg(void)
{
  return (s_sample_valid != 0u) ? s_sample.pitch_deg : 0.0f;
}
