/**
  ******************************************************************************
  * @file    proto.h
  * @brief   T5a 通讯协议层 —— 帧格式、类型码、状态位、组帧与解析接口
  ******************************************************************************
  * @note    协议依据：《电控二轮考核_协议与上位机说明 V1.0》第 3~7 节。
  *
  *          帧格式（多字节小端，角度单位 0.1°，有符号 int16）：
  *
  *            偏移  字段      长度   说明
  *            ----  --------  ----   ------------------------------
  *            0     SOF       1      固定 0xA5
  *            1     type      1      见 PROTO_T_xxx
  *            2     N         1      载荷字节数（不含 SOF/type/CRC）
  *            3     payload   N      按消息定义
  *            3+N   CRC8      1      对第 0 字节到第 3+N-1 字节逐字节计算
  *
  *          最小帧 4 字节（N=0，本工程不用），最大帧 36 字节（N=32）。
  *
  * @warning 这一层**故意不包含任何 HAL 头文件**。
  *          目的是让它能被电脑上的 gcc 直接编译并跑单元测试
  *          （见 tests/proto_selftest.c），不需要 ST-Link、不需要板子。
  *          后续 proto_link.c 才负责把它接到 USB CDC 上。
  ******************************************************************************
  */
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 一、帧格式常量
 * ========================================================================== */

#define PROTO_SOF              0xA5u   /* 帧头 */
#define PROTO_MAX_PAYLOAD      32u     /* 正本 §2.4：N 大于 32 即判长度越界 */
#define PROTO_OVERHEAD         4u      /* SOF + type + N + CRC */
#define PROTO_MAX_FRAME        (PROTO_MAX_PAYLOAD + PROTO_OVERHEAD)   /* 36 */

/* ==========================================================================
 * 二、消息类型码（正本 §4.1）
 * ========================================================================== */

#define PROTO_T_STATUS         0x10u   /* 状态上报   设备 → 上位机  20~50Hz */
#define PROTO_T_TARGET         0x20u   /* 目标角度   上位机 → 设备  按需  */
#define PROTO_T_MODE           0x21u   /* 模式切换   上位机 → 设备  事件  */
#define PROTO_T_ESTOP          0x22u   /* 急停       上位机 → 设备  事件  */
#define PROTO_T_HEARTBEAT      0x23u   /* 心跳       双向          1Hz   */
#define PROTO_T_VERSION        0x24u   /* 版本查询   双向          连接时 */

/* 各消息的载荷长度（正本 §4.2~§4.7） */
#define PROTO_LEN_STATUS       19u
#define PROTO_LEN_TARGET       11u
#define PROTO_LEN_MODE          1u
#define PROTO_LEN_ESTOP         1u
#define PROTO_LEN_HEARTBEAT     1u
#define PROTO_LEN_VERSION       2u

/* ==========================================================================
 * 三、0x21 模式（正本 §4.4）
 * ========================================================================== */

#define PROTO_MODE_MANUAL      0u   /* 手动 */
#define PROTO_MODE_AUTO        1u   /* 自动 */
#define PROTO_MODE_HOMING      2u   /* 回零 */
#define PROTO_MODE_CLEAR_FAULT 3u   /* 清除故障（急停后唯一的复位途径） */

/* ==========================================================================
 * 四、0x22 急停载荷（正本 §4.5）
 * ========================================================================== */

#define PROTO_ESTOP_ON         1u   /* 1 = 急停；0 = 保留 */

/* ==========================================================================
 * 五、0x10 状态位（正本 §4.2）
 *         每一位单独用宏，方便 `status |= PROTO_ST_FAULT;` 这种写法，
 *         比记 0x04 这种魔数可读得多。
 * ========================================================================== */

#define PROTO_ST_AUTO          0x01u   /* bit0  0 手动 / 1 自动 */
#define PROTO_ST_HOMING        0x02u   /* bit1  正在回零 */
#define PROTO_ST_FAULT         0x04u   /* bit2  故障态 */
#define PROTO_ST_MOVING        0x08u   /* bit3  正在运动 */
#define PROTO_ST_I2C_FAIL      0x10u   /* bit4  I²C 传感器失效 */
#define PROTO_ST_TIMEOUT       0x20u   /* bit5  指令超时 */
#define PROTO_ST_RESERVED      0xC0u   /* bit6~bit7 保留，必须填 0 */

/* ==========================================================================
 * 六、0x24 版本（正本 §4.7）
 *         设备侧固定填 0x01、0x00 —— 不是"我们自己的版本号"，是**协议版本号**。
 * ========================================================================== */

#define PROTO_VER_MAJOR        0x01u
#define PROTO_VER_MINOR        0x00u

/* ==========================================================================
 * 七、CRC8 参数（正本 §6）
 *         poly 0x07 / init 0x00 / 不反射 / 输出不异或
 *         校验向量：crc8("123456789", 9) == 0xF4（CRC-8/ATM 标准值）
 * ========================================================================== */

uint8_t proto_crc8(const uint8_t *buf, uint32_t len);

/* ==========================================================================
 * 八、一帧的内存表示
 * ========================================================================== */

typedef struct
{
  uint8_t type;                          /* 类型码 */
  uint8_t len;                           /* 载荷实际长度 */
  uint8_t payload[PROTO_MAX_PAYLOAD];    /* 载荷副本 */
} proto_frame_t;

/* ==========================================================================
 * 九、组帧（发送侧）
 * ========================================================================== */

/**
  * @brief  把 type/payload 打包成完整帧（含 SOF 与 CRC）
  * @param  out      输出缓冲
  * @param  cap      out 的容量（字节）
  * @param  type     类型码
  * @param  payload  载荷指针，len 为 0 时可为 NULL
  * @param  len      载荷长度，必须 ≤ PROTO_MAX_PAYLOAD
  * @retval 写入的字节数；参数非法或容量不足返回 0
  */
uint16_t proto_build(uint8_t *out,
                     uint16_t cap,
                     uint8_t  type,
                     const uint8_t *payload,
                     uint8_t  len);

/* ==========================================================================
 * 十、流式解析（接收侧）
 *
 *     为什么要有"流式"：CDC 是字节流，不是包流。上位机发来的 11 字节目标帧
 *     可能被拆成 3 次 USB 传输到达，也可能和上一帧的尾巴粘在一起。
 *     所以解析器必须能"喂任意切片"，自己维持状态。
 *
 *     容错规则（正本 §2.4）：帧头不对 / 长度越界 / CRC 不符，
 *     都**只丢弃 1 个字节**，然后从下一个字节重新找帧头 —— 不是整包丢弃。
 *     这条很重要：如果整包丢，一个错误字节就会连带吃掉后面一整个好帧。
 * ========================================================================== */

typedef struct
{
  uint8_t  buf[PROTO_MAX_FRAME];   /* 待解析字节窗口 */
  uint16_t cnt;                    /* 窗口里现有字节数 */

  /* 统计：出问题时能一眼看出是哪一类错，便于对接上位机的丢帧统计 */
  uint32_t n_sof;                  /* 帧头不是 0xA5 而被丢掉的字节数 */
  uint32_t n_len;                  /* N > 32 而丢掉的字节数 */
  uint32_t n_crc;                  /* CRC 不符而丢掉的字节数 */
  uint32_t n_ok;                   /* 成功解出的帧数 */
} proto_parser_t;

/** 复位解析器（清零窗口与统计） */
void proto_parser_reset(proto_parser_t *p);

/**
  * @brief  喂一个字节
  * @param  out  解出帧时写入这里（可传 NULL 表示只关心"解出没解出"）
  * @retval 1 = 解出一帧；0 = 还没凑齐
  */
int proto_parser_push(proto_parser_t *p, uint8_t byte, proto_frame_t *out);

/**
  * @brief  喂一段字节
  * @param  out      解出的帧按顺序写入这个数组
  * @param  out_cap  数组容量（帧数）
  * @retval 实际写出的帧数（可能因为 out_cap 满而少于实际解出的帧数）
  */
uint16_t proto_parser_feed(proto_parser_t *p,
                           const uint8_t *data,
                           uint16_t len,
                           proto_frame_t *out,
                           uint16_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* PROTO_H */
