/**
  ******************************************************************************
  * @file    proto_frame.c
  * @brief   CRC8 / 组帧 / 流式解帧 的实现
  ******************************************************************************
  * @note    本文件**不包含任何 HAL 头文件**，因此可以用电脑上的 gcc 直接编译：
  *
  *            gcc -I Core/Inc tests/proto_selftest.c Core/Src/proto_frame.c -o test.exe
  *
  *          这是 T5a 能"没有硬件也先做完"的关键：协议的对错不依赖板子，
  *          只依赖字节。把纯逻辑和硬件隔开，就能在没有 ST-Link 的时候验证它。
  ******************************************************************************
  */

#include <string.h>
#include "proto.h"

/* ------------------------------------------------------------------------- */
/* CRC8：多项式 0x07，初值 0x00，不反射，输出不异或（正本 §6）                 */
/* ------------------------------------------------------------------------- */
uint8_t proto_crc8(const uint8_t *buf, uint32_t len)
{
  uint8_t  crc = 0x00u;
  uint32_t i;
  int      b;

  if (buf == NULL)
  {
    return 0u;
  }

  for (i = 0u; i < len; i++)
  {
    crc ^= buf[i];
    for (b = 0; b < 8; b++)
    {
      /* 最高位为 1 则左移后异或多项式，否则只左移 */
      if ((crc & 0x80u) != 0u)
      {
        crc = (uint8_t)((uint8_t)(crc << 1) ^ 0x07u);
      }
      else
      {
        crc = (uint8_t)(crc << 1);
      }
    }
  }

  return crc;
}

/* ------------------------------------------------------------------------- */
/* 组帧                                                                       */
/* ------------------------------------------------------------------------- */
uint16_t proto_build(uint8_t *out,
                     uint16_t cap,
                     uint8_t  type,
                     const uint8_t *payload,
                     uint8_t  len)
{
  uint16_t total;

  if ((out == NULL) || (len > PROTO_MAX_PAYLOAD))
  {
    return 0u;
  }
  if ((len > 0u) && (payload == NULL))
  {
    return 0u;
  }

  total = (uint16_t)(3u + (uint16_t)len + 1u);   /* SOF + type + N + payload + CRC */
  if (cap < total)
  {
    return 0u;
  }

  out[0] = PROTO_SOF;
  out[1] = type;
  out[2] = len;
  if (len > 0u)
  {
    (void)memcpy(&out[3], payload, (size_t)len);
  }
  /* CRC 覆盖第 0 字节到第 3+N-1 字节，共 3+N 个字节 */
  out[3u + (uint16_t)len] = proto_crc8(out, (uint32_t)(3u + (uint16_t)len));

  return total;
}

/* ------------------------------------------------------------------------- */
/* 解析器内部：丢掉窗口最前面的 1 个字节                                      */
/* ------------------------------------------------------------------------- */
static void parser_drop1(proto_parser_t *p)
{
  if (p->cnt == 0u)
  {
    return;
  }
  p->cnt--;
  if (p->cnt > 0u)
  {
    (void)memmove(&p->buf[0], &p->buf[1], (size_t)p->cnt);
  }
}

/* ------------------------------------------------------------------------- */
/* 解析器内部：尝试从窗口头部解出一帧                                         */
/*   返回 1 = 成功；0 = 字节还不够                                           */
/*   每轮循环要么返回，要么丢 1 字节，所以必定终止（不会死循环）              */
/* ------------------------------------------------------------------------- */
static int parser_try(proto_parser_t *p, proto_frame_t *out)
{
  for (;;)
  {
    uint8_t  n;
    uint16_t need;

    /* 连 SOF/type/N 三个字节都没凑齐 */
    if (p->cnt < 3u)
    {
      return 0;
    }

    /* ① 帧头不对 → 丢 1 字节重新找 */
    if (p->buf[0] != PROTO_SOF)
    {
      parser_drop1(p);
      p->n_sof++;
      continue;
    }

    /* ② 长度越界 → 丢 1 字节重新找 */
    n = p->buf[2];
    if (n > PROTO_MAX_PAYLOAD)
    {
      parser_drop1(p);
      p->n_len++;
      continue;
    }

    /* ③ 字节还没收全 → 等下一次 push */
    need = (uint16_t)(3u + (uint16_t)n + 1u);
    if (p->cnt < need)
    {
      return 0;
    }

    /* ④ CRC 不符 → 丢 1 字节重新找 */
    if (proto_crc8(&p->buf[0], (uint32_t)(3u + (uint16_t)n)) != p->buf[3u + (uint16_t)n])
    {
      parser_drop1(p);
      p->n_crc++;
      continue;
    }

    /* ⑤ 成功：拷出字段，消费掉 need 个字节 */
    if (out != NULL)
    {
      out->type = p->buf[1];
      out->len  = n;
      if (n > 0u)
      {
        (void)memcpy(&out->payload[0], &p->buf[3], (size_t)n);
      }
    }
    p->n_ok++;

    p->cnt = (uint16_t)(p->cnt - need);
    if (p->cnt > 0u)
    {
      (void)memmove(&p->buf[0], &p->buf[need], (size_t)p->cnt);
    }

    return 1;
  }
}

/* ------------------------------------------------------------------------- */
/* 对外：复位                                                               */
/* ------------------------------------------------------------------------- */
void proto_parser_reset(proto_parser_t *p)
{
  if (p == NULL)
  {
    return;
  }
  p->cnt   = 0u;
  p->n_sof = 0u;
  p->n_len = 0u;
  p->n_crc = 0u;
  p->n_ok  = 0u;
}

/* ------------------------------------------------------------------------- */
/* 对外：喂一个字节                                                         */
/* ------------------------------------------------------------------------- */
int proto_parser_push(proto_parser_t *p, uint8_t byte, proto_frame_t *out)
{
  if (p == NULL)
  {
    return 0;
  }

  /* 窗口满仍未解出：正常不会发生（最长帧正好 36），保命丢最老的字节 */
  if (p->cnt >= PROTO_MAX_FRAME)
  {
    parser_drop1(p);
  }

  p->buf[p->cnt] = byte;
  p->cnt++;

  return parser_try(p, out);
}

/* ------------------------------------------------------------------------- */
/* 对外：不喂字节，只再试一次（取窗口里可能残留的第二帧）                     */
/* ------------------------------------------------------------------------- */
int proto_parser_pop(proto_parser_t *p, proto_frame_t *out)
{
  if (p == NULL)
  {
    return 0;
  }

  return parser_try(p, out);
}

/* ------------------------------------------------------------------------- */
/* 对外：喂一段字节                                                         */
/* ------------------------------------------------------------------------- */
uint16_t proto_parser_feed(proto_parser_t *p,
                           const uint8_t *data,
                           uint16_t len,
                           proto_frame_t *out,
                           uint16_t out_cap)
{
  proto_frame_t tmp;
  uint16_t      stored = 0u;
  uint16_t      i;

  if ((p == NULL) || (data == NULL))
  {
    return 0u;
  }

  for (i = 0u; i < len; i++)
  {
    if (proto_parser_push(p, data[i], &tmp) == 1)
    {
      if ((out != NULL) && (stored < out_cap))
      {
        out[stored] = tmp;
        stored++;
      }
    }
  }

  return stored;
}
