/**
  ******************************************************************************
  * @file    proto_link.h
  * @brief   CDC 链路的**唯一出口**：接收环形缓冲 + 单帧发送 + 流式解帧
  ******************************************************************************
  * @note    整个工程里，只有 proto_link.c 允许碰 USB CDC。
  *          任何其它文件想发协议帧，只能调 link_send_frame()。
  *
  *          这个约束不是洁癖，是防错：
  *          CDC_Transmit_FS() **不复制数据**，它只是把指针交给 USB 栈，
  *          传输完成前那块内存必须保持有效。如果每个调用点都自己开一个
  *          （栈上的）缓冲区传进去，一旦上一个包还没发完就调下一次，
  *          发出去的就是被覆盖的垃圾 —— 而且现象极难查（偶发 CRC 错）。
  *
  *          所以：一个 static 发送缓冲 + 一个"忙就拒发"的入口 = 结构上不会错。
  ******************************************************************************
  */
#ifndef PROTO_LINK_H
#define PROTO_LINK_H

#include <stdint.h>
#include "proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* link_send_frame() 的返回值 */
#define LINK_OK         0u   /* 已交给 USB 栈 */
#define LINK_BUSY       1u   /* 上一个包还没发完，本帧未发出（可直接丢弃/下一周期重发）*/
#define LINK_NOT_READY  2u   /* 电脑还没打开串口（CDC 未配置），发了也没人收 */
#define LINK_ERR_PARAM  3u   /* 参数非法（长度超限等）*/

/* 接收环形缓冲容量，必须是 2 的幂 */
#define LINK_RX_RING    256u

/** 初始化（清空缓冲与解析器） */
void link_init(void);

/**
  * @brief  USB 接收回调专用 —— **只能从 CDC_Receive_FS() 里调**
  * @note   运行在 USB 中断上下文，所以这里只做"把字节搬进环形缓冲"，
  *         绝不解析、不控舵机。真正的处理在主循环的 link_poll() 里做。
  *         中断里干重活 = 丢 USB 包 = 上位机看到莫名其妙的断帧。
  */
void link_on_rx(const uint8_t *data, uint16_t len);

/**
  * @brief  主循环里调用：把环形缓冲灌进解析器，解出一帧就返回
  * @param  out  解出的帧
  * @retval 1 = 解出一帧；0 = 暂时没有完整帧
  * @note   调用方应该用 while 循环取干净：while (link_poll(&fr)) { ... }
  */
int link_poll(proto_frame_t *out);

/**
  * @brief  发送一帧（组帧 + 交给 USB）
  * @retval LINK_OK / LINK_BUSY / LINK_NOT_READY / LINK_ERR_PARAM
  */
uint8_t link_send_frame(uint8_t type, const uint8_t *payload, uint8_t len);

/** 统计：接收环形缓冲溢出字节数（正常应长期为 0） */
uint32_t link_rx_overflow(void);

/** 统计：发送因 BUSY 被拒的帧数 */
uint32_t link_tx_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTO_LINK_H */
