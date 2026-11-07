/**
  ******************************************************************************
  * @file    proto_link.c
  * @brief   CDC 链路的唯一出口（见 proto_link.h 开头的说明）
  ******************************************************************************
  */

#include <string.h>

#include "proto_link.h"

#include "usbd_cdc.h"       /* USBD_CDC_HandleTypeDef / TxState */
#include "usbd_cdc_if.h"    /* CDC_Transmit_FS() */

/* CubeMX 把 hUsbDeviceFS 定义在 usb_device.c，但没写进 usb_device.h，
 * 所以在这里自己声明一次（整个工程只有本文件和工作台之外的地方需要它）。 */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ------------------------------------------------------------------------- */
/* 接收侧：单生产者（USB 中断）→ 单消费者（主循环）的环形缓冲                  */
/*   Cortex-M3 上 16 位读写是原子的，所以不需要关中断。                       */
/* ------------------------------------------------------------------------- */
static uint8_t           s_rx[LINK_RX_RING];
static volatile uint16_t s_rx_wr = 0u;      /* 只由中断写 */
static volatile uint16_t s_rx_rd = 0u;      /* 只由主循环写 */
static volatile uint32_t s_rx_ovf = 0u;

static proto_parser_t    s_parser;
static uint32_t          s_tx_busy_cnt = 0u;

/* 发送缓冲：**必须 static**，因为 CDC_Transmit_FS 只保存指针、不复制内容。
 * 复用一份而不是每帧开新的，是为了让"上一次还在发"变成可检测的 LINK_BUSY。 */
static uint8_t           s_tx_buf[PROTO_MAX_FRAME];

/* ------------------------------------------------------------------------- */
void link_init(void)
{
  s_rx_rd = 0u;
  s_rx_wr = 0u;
  s_rx_ovf = 0u;
  s_tx_busy_cnt = 0u;

  proto_parser_reset(&s_parser);
}

/* ------------------------------------------------------------------------- */
/* 中断上下文：只搬字节                                                        */
/* ------------------------------------------------------------------------- */
void link_on_rx(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if (data == NULL)
  {
    return;
  }

  for (i = 0u; i < len; i++)
  {
    const uint16_t next = (uint16_t)((s_rx_wr + 1u) & (LINK_RX_RING - 1u));

    if (next == s_rx_rd)
    {
      /* 满了：丢新字节并计数。宁可丢，也不能覆盖还没被主循环取走的老数据。 */
      s_rx_ovf++;
      return;
    }

    s_rx[s_rx_wr] = data[i];
    s_rx_wr = next;
  }
}

/* ------------------------------------------------------------------------- */
/* 主循环上下文：搬出环形缓冲 → 喂解析器 → 解出帧                              */
/* ------------------------------------------------------------------------- */
int link_poll(proto_frame_t *out)
{
  for (;;)
  {
    /* 1) 缓冲里有字节就先喂进解析器 */
    if (s_rx_rd != s_rx_wr)
    {
      const uint8_t b = s_rx[s_rx_rd];
      s_rx_rd = (uint16_t)((s_rx_rd + 1u) & (LINK_RX_RING - 1u));

      if (proto_parser_push(&s_parser, b, out) == 1)
      {
        return 1;
      }
      continue;
    }

    /* 2) 缓冲空了。但解析器窗口里可能还躺着**下一帧**：
     *    一次 USB 传输可能带来两帧，push() 解出第一帧就返回了，
     *    第二帧完整地留在窗口里，此时没有新字节可喂，必须显式再试一次。 */
    return proto_parser_pop(&s_parser, out);
  }
}

/* ------------------------------------------------------------------------- */
/* 发送：唯一的出口                                                           */
/* ------------------------------------------------------------------------- */
uint8_t link_send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
  uint16_t n;
  USBD_CDC_HandleTypeDef *hcdc;

  /* 电脑没打开串口时不要往 USB 栈里塞东西 */
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return LINK_NOT_READY;
  }

  /* **先查忙再写缓冲**。反过来写就会出现：老包里还指着 s_tx_buf，
   * 我们已经把新帧拷进去了，于是发出去的半新半旧 —— 最难查的一类 bug。 */
  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if ((hcdc == NULL) || (hcdc->TxState != 0u))
  {
    s_tx_busy_cnt++;
    return LINK_BUSY;
  }

  n = proto_build(s_tx_buf, (uint16_t)sizeof s_tx_buf, type, payload, len);
  if (n == 0u)
  {
    return LINK_ERR_PARAM;
  }

  if (CDC_Transmit_FS(s_tx_buf, n) != USBD_OK)
  {
    s_tx_busy_cnt++;
    return LINK_BUSY;
  }

  return LINK_OK;
}

/* ------------------------------------------------------------------------- */
uint32_t link_rx_overflow(void)
{
  return s_rx_ovf;
}

uint32_t link_tx_busy(void)
{
  return s_tx_busy_cnt;
}
