/**
  ******************************************************************************
  * @file    proto_link.c
  * @brief   CDC 链路的唯一出口（见 proto_link.h 开头的说明）
  ******************************************************************************
  * @note    对应任务书 T5a 的三条硬要求：
  *
  *          ① "使用 USB Device 的 CDC 类，作为虚拟串口以 20~50Hz 上报状态帧"
  *             → 周期由 proto_app.c 控制，本文件只负责把帧送出去。
  *
  *          ② "发送缓冲满时丢弃最旧数据，不得阻塞控制节拍"
  *             → 待发槽只有 1 帧：忙的时候新帧直接盖掉槽里的旧帧（丢弃最旧），
  *               函数立刻返回，**从不等待** USB 完成。
  *
  *          ③ "拔插 USB 后 1 秒内自动重连"
  *             → usb_watchdog()：从"已连接"变成"未连接"时，把接收缓冲和
  *               解析器全部清空。不清的话重插后上位机先收到的是一堆旧字节，
  *               表现为"重连后头一秒全是 CRC 错"。
  ******************************************************************************
  */

#include <string.h>

#include "proto_link.h"

#include "usbd_cdc.h"       /* USBD_CDC_HandleTypeDef / TxState */
#include "usbd_cdc_if.h"    /* CDC_Transmit_FS() */

/* CubeMX 把 hUsbDeviceFS 定义在 usb_device.c，但没写进 usb_device.h，
 * 所以在这里自己声明一次（整个工程里只有本文件需要它）。 */
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

/* ------------------------------------------------------------------------- */
/* 发送侧：两个缓冲，分工严格，永不混用                                        */
/*                                                                           */
/*   s_tx_buf   —— 正在被 USB 发送的那一帧。**只在链路空闲时才写**。            */
/*   s_pending  —— 待发槽。只在链路上有卡顿时才用；新帧盖旧帧 = 丢弃最旧。      */
/*                                                                           */
/*   为什么不能只用一个缓冲：USB 忙的时候，如果直接往同一个缓冲里写新帧，        */
/*   正在发的那个包的内容就被改了，发出去半新半旧 —— 偶发 CRC 错，极难复现。   */
/* ------------------------------------------------------------------------- */
static uint8_t  s_tx_buf[PROTO_MAX_FRAME];
static uint8_t  s_pending[PROTO_MAX_FRAME];
static uint16_t s_pending_len  = 0u;
static uint8_t  s_has_pending  = 0u;
static uint32_t s_drop_oldest  = 0u;

/* USB 掉线检测 */
static uint8_t  s_usb_was_ready = 0u;
static uint32_t s_usb_lost      = 0u;

/* ------------------------------------------------------------------------- */
static int usb_ready(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1 : 0;
}

/* 链路此刻能不能立刻发一包 */
static int cdc_tx_idle(void)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  return ((hcdc != NULL) && (hcdc->TxState == 0u)) ? 1 : 0;
}

/* ------------------------------------------------------------------------- */
/* 掉线检测：已连接 → 未连接 的瞬间，把残留状态全清掉                          */
/* ------------------------------------------------------------------------- */
static void usb_watchdog(void)
{
  const int ready = usb_ready();

  if ((s_usb_was_ready != 0u) && (ready == 0))
  {
    s_usb_lost++;

    /* 接收侧：环形缓冲里可能躺着半帧，解析器窗口里也可能有半帧 */
    s_rx_rd = 0u;
    s_rx_wr = 0u;
    proto_parser_reset(&s_parser);

    /* 发送侧：待发的那帧已经没人要了 */
    s_has_pending = 0u;
    s_pending_len = 0u;
  }

  s_usb_was_ready = (uint8_t)ready;
}

/* ------------------------------------------------------------------------- */
/* 待发槽服务：链路空了就把槽里的帧补发出去                                    */
/*   注意这里要先 memcpy 到 s_tx_buf 再发 —— 不能让 s_pending 自己进在途状态， */
/*   否则下一帧想"盖掉它"时就会改到正在发送的内容。                            */
/* ------------------------------------------------------------------------- */
static void tx_service(void)
{
  if (s_has_pending == 0u)
  {
    return;
  }
  if ((usb_ready() == 0) || (cdc_tx_idle() == 0))
  {
    return;
  }

  (void)memcpy(s_tx_buf, s_pending, (size_t)s_pending_len);
  s_has_pending = 0u;

  if (CDC_Transmit_FS(s_tx_buf, s_pending_len) != USBD_OK)
  {
    /* 极小概率：刚判断空闲，真正发的时候又忙了。把帧放回槽里，下一轮再试 */
    (void)memcpy(s_pending, s_tx_buf, (size_t)s_pending_len);
    s_has_pending = 1u;
  }
  else
  {
    s_pending_len = 0u;
  }
}

/* ------------------------------------------------------------------------- */
void link_init(void)
{
  s_rx_rd = 0u;
  s_rx_wr = 0u;
  s_rx_ovf = 0u;

  s_drop_oldest = 0u;
  s_has_pending = 0u;
  s_pending_len = 0u;

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
/* 主循环上下文                                                               */
/* ------------------------------------------------------------------------- */
int link_poll(proto_frame_t *out)
{
  usb_watchdog();
  tx_service();

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

  usb_watchdog();

  /* 电脑没打开串口时不要往 USB 栈里塞东西 */
  if (usb_ready() == 0)
  {
    return LINK_NOT_READY;
  }

  /* 路径 A：链路空闲且槽里没货 → 直接发，零延迟 */
  if ((cdc_tx_idle() != 0) && (s_has_pending == 0u))
  {
    n = proto_build(s_tx_buf, (uint16_t)sizeof s_tx_buf, type, payload, len);
    if (n == 0u)
    {
      return LINK_ERR_PARAM;
    }
    if (CDC_Transmit_FS(s_tx_buf, n) == USBD_OK)
    {
      return LINK_OK;
    }
    /* 掉到下面走路径 B */
  }

  /* 路径 B：忙（或槽里有旧帧）→ 写进待发槽。
   * 槽里原来的帧被直接覆盖，这就是"丢弃最旧"。
   * 本函数立刻返回，**绝不等待** USB 完成 —— 不阻塞控制节拍。 */
  n = proto_build(s_pending, (uint16_t)sizeof s_pending, type, payload, len);
  if (n == 0u)
  {
    return LINK_ERR_PARAM;
  }

  if (s_has_pending != 0u)
  {
    s_drop_oldest++;
  }

  s_pending_len = n;
  s_has_pending = 1u;

  return LINK_QUEUED;
}

/* ------------------------------------------------------------------------- */
uint32_t link_rx_overflow(void)
{
  return s_rx_ovf;
}

uint32_t link_tx_dropped_oldest(void)
{
  return s_drop_oldest;
}

uint32_t link_usb_lost_count(void)
{
  return s_usb_lost;
}
