/**
  ******************************************************************************
  * @file    proto_selftest.c
  * @brief   协议层 PC 端单元测试（在电脑上跑，不需要 STM32 / ST-Link）
  ******************************************************************************
  * @note    编译运行：
  *            gcc -std=c11 -Wall -Wextra -Wpedantic -O2 -I Core/Inc \
  *                tests/proto_selftest.c Core/Src/proto_frame.c -o tests/proto_selftest.exe
  *            tests/proto_selftest.exe
  *
  *          退出码 0 = 全部通过，1 = 有失败。
  *
  * @note    测试向量全部取自《协议与上位机说明 V1.0》第 7 节。
  *          这几条是**官方给的**，能全过就说明组帧/解帧与上位机一致。
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include "proto.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *name)
{
  if (cond)
  {
    g_pass++;
    printf("  [PASS] %s\n", name);
  }
  else
  {
    g_fail++;
    printf("  [FAIL] %s\n", name);
  }
}

static void hexdump(const char *tag, const uint8_t *p, int n)
{
  int i;
  printf("         %-18s", tag);
  for (i = 0; i < n; i++)
  {
    printf("%02X ", p[i]);
  }
  printf("\n");
}

/* 小端写入 int16 */
static void put_i16(uint8_t *p, int16_t v)
{
  p[0] = (uint8_t)((uint16_t)v & 0xFFu);
  p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

/* ---- 官方示例帧（协议正本第 7 节，已逐字节核对）---- */
static const uint8_t VEC_10[] = {
  0xA5,0x10,0x13, 0x84,0x03, 0x78,0x05, 0xB0,0x04, 0x2C,0x01,
                  0x84,0x03, 0x78,0x05, 0xB0,0x04, 0x2C,0x01,
                  0x7B,0x00, 0x01, 0xA8 };
static const uint8_t VEC_20[] = {
  0xA5,0x20,0x0B, 0x84,0x03, 0x78,0x05, 0xB0,0x04, 0x2C,0x01,
                  0xF4,0x01, 0x02, 0xAB };
static const uint8_t VEC_21[] = { 0xA5,0x21,0x01,0x02,0x82 };
static const uint8_t VEC_22[] = { 0xA5,0x22,0x01,0x01,0x36 };
static const uint8_t VEC_24[] = { 0xA5,0x24,0x02,0x01,0x00,0x4B };

int main(void)
{
  uint8_t        buf[PROTO_MAX_FRAME + 8];
  uint8_t        pl[PROTO_MAX_PAYLOAD];
  proto_frame_t  fr;
  proto_parser_t ps;
  uint16_t       n;
  uint16_t       i;

  printf("=== protocol layer selftest ===\n\n");

  /* ---------- 1. CRC8 标准校验向量 ---------- */
  printf("[1] CRC8 reference vector\n");
  {
    const uint8_t v[] = { '1','2','3','4','5','6','7','8','9' };
    uint8_t c = proto_crc8(v, 9);
    printf("         crc8(\"123456789\") = 0x%02X (standard CRC-8/ATM = 0xF4)\n", c);
    check(c == 0xF4u, "CRC8 matches CRC-8/ATM standard vector");
  }

  /* ---------- 2. 组帧：5 条官方示例帧逐字节比对 ---------- */
  printf("\n[2] frame build vs official examples\n");
  {
    /* 0x10 状态帧：四轴 900/1400/1200/300（即 90/140/120/30 度），pitch 123，status 0x01 */
    const int16_t q[4] = { 900, 1400, 1200, 300 };
    for (i = 0; i < 4u; i++) { put_i16(&pl[i * 2u], q[i]); }
    for (i = 0; i < 4u; i++) { put_i16(&pl[8u + i * 2u], q[i]); }
    put_i16(&pl[16], 123);
    pl[18] = PROTO_ST_AUTO;

    n = proto_build(buf, sizeof buf, PROTO_T_STATUS, pl, PROTO_LEN_STATUS);
    check(n == sizeof VEC_10 && memcmp(buf, VEC_10, sizeof VEC_10) == 0,
          "0x10 status frame (23 bytes)");
    if (memcmp(buf, VEC_10, sizeof VEC_10) != 0)
    {
      hexdump("got", buf, (int)n);
      hexdump("expect", VEC_10, (int)sizeof VEC_10);
    }
  }
  {
    /* 0x20 目标帧：同四轴，duration 500ms，profile 2 (S 曲线) */
    const int16_t q[4] = { 900, 1400, 1200, 300 };
    for (i = 0; i < 4u; i++) { put_i16(&pl[i * 2u], q[i]); }
    put_i16(&pl[8], 500);
    pl[10] = 2u;

    n = proto_build(buf, sizeof buf, PROTO_T_TARGET, pl, PROTO_LEN_TARGET);
    check(n == sizeof VEC_20 && memcmp(buf, VEC_20, sizeof VEC_20) == 0,
          "0x20 target frame (15 bytes)");
  }
  {
    pl[0] = PROTO_MODE_HOMING;
    n = proto_build(buf, sizeof buf, PROTO_T_MODE, pl, PROTO_LEN_MODE);
    check(n == sizeof VEC_21 && memcmp(buf, VEC_21, sizeof VEC_21) == 0, "0x21 mode frame (5 bytes)");
  }
  {
    pl[0] = PROTO_ESTOP_ON;
    n = proto_build(buf, sizeof buf, PROTO_T_ESTOP, pl, PROTO_LEN_ESTOP);
    check(n == sizeof VEC_22 && memcmp(buf, VEC_22, sizeof VEC_22) == 0, "0x22 estop frame (5 bytes)");
  }
  {
    pl[0] = PROTO_VER_MAJOR;
    pl[1] = PROTO_VER_MINOR;
    n = proto_build(buf, sizeof buf, PROTO_T_VERSION, pl, PROTO_LEN_VERSION);
    check(n == sizeof VEC_24 && memcmp(buf, VEC_24, sizeof VEC_24) == 0, "0x24 version frame (6 bytes)");
  }

  /* ---------- 3. 解帧：官方 0x10 帧字段还原 ---------- */
  printf("\n[3] parse official 0x10 frame\n");
  proto_parser_reset(&ps);
  n = proto_parser_feed(&ps, VEC_10, (uint16_t)sizeof VEC_10, &fr, 1u);
  check(n == 1u, "one frame decoded");
  check(fr.type == PROTO_T_STATUS, "type == 0x10");
  check(fr.len == PROTO_LEN_STATUS, "len == 19");
  {
    /* 小端读回 int16 */
    const int16_t q1  = (int16_t)((uint16_t)fr.payload[0] | ((uint16_t)fr.payload[1] << 8));
    const int16_t q2  = (int16_t)((uint16_t)fr.payload[2] | ((uint16_t)fr.payload[3] << 8));
    const int16_t p16 = (int16_t)((uint16_t)fr.payload[16] | ((uint16_t)fr.payload[17] << 8));
    printf("         q1=%.1f q2=%.1f pitch=%.1f status=0x%02X\n",
           q1 / 10.0, q2 / 10.0, p16 / 10.0, fr.payload[18]);
    check(q1 == 900 && q2 == 1400, "q1/q2 = 90.0 / 140.0 deg");
    check(p16 == 123, "pitch = 12.3 deg");
    check(fr.payload[18] == PROTO_ST_AUTO, "status bit0 = auto");
  }

  /* ---------- 4. 逐字节喂入（模拟串口零散到达）---------- */
  printf("\n[4] byte-by-byte injection\n");
  proto_parser_reset(&ps);
  for (i = 0; i < (uint16_t)sizeof VEC_10; i++)
  {
    n = (uint16_t)proto_parser_push(&ps, VEC_10[i], &fr);
    if (i + 1u < (uint16_t)sizeof VEC_10)
    {
      if (n != 0u) { break; }        /* 没到最后一字节就解出 = 错 */
    }
  }
  check(n == 1u && fr.type == PROTO_T_STATUS, "frame decoded only after last byte");

  /* ---------- 5. 垃圾前缀重同步 ---------- */
  printf("\n[5] resync after garbage prefix\n");
  {
    uint8_t junk[40];
    for (i = 0; i < 40u; i++) { junk[i] = (uint8_t)(0x10u + i); }   /* 里面不含 0xA5 */
    proto_parser_reset(&ps);
    n = proto_parser_feed(&ps, junk, 40u, &fr, 1u);
    check(n == 0u, "no false frame from garbage");
    n = proto_parser_feed(&ps, VEC_20, (uint16_t)sizeof VEC_20, &fr, 1u);
    check(n == 1u && fr.type == PROTO_T_TARGET, "resynced and decoded 0x20");
    printf("         bytes dropped for bad SOF: %u\n", (unsigned)ps.n_sof);
  }

  /* ---------- 6. CRC 错误：丢 1 字节恢复后续好帧 ---------- */
  printf("\n[6] CRC error recovery\n");
  {
    uint8_t bad[sizeof VEC_20];
    memcpy(bad, VEC_20, sizeof VEC_20);
    bad[5] ^= 0xFFu;                       /* 弄坏一个载荷字节，CRC 必不符 */

    proto_parser_reset(&ps);
    n = proto_parser_feed(&ps, bad, (uint16_t)sizeof bad, &fr, 1u);
    check(n == 0u, "corrupted frame rejected");
    check(ps.n_crc > 0u, "CRC error counted");

    n = proto_parser_feed(&ps, VEC_21, (uint16_t)sizeof VEC_21, &fr, 1u);
    check(n == 1u && fr.type == PROTO_T_MODE, "next good frame still decoded");
    printf("         crc errors: %u, good frames: %u\n",
           (unsigned)ps.n_crc, (unsigned)ps.n_ok);
  }

  /* ---------- 7. N > 32 长度越界 ---------- */
  printf("\n[7] length violation (N > 32)\n");
  {
    const uint8_t over[] = { 0xA5, 0x20, 33, 0x00, 0x00, 0x00 };
    proto_parser_reset(&ps);
    n = proto_parser_feed(&ps, over, (uint16_t)sizeof over, &fr, 1u);
    check(n == 0u, "N=33 rejected");
    check(ps.n_len > 0u, "length violation counted");

    n = proto_parser_feed(&ps, VEC_22, (uint16_t)sizeof VEC_22, &fr, 1u);
    check(n == 1u && fr.type == PROTO_T_ESTOP, "recovered after length violation");
  }

  /* ---------- 8. 连续多帧一次喂入 ---------- */
  printf("\n[8] multiple frames in one feed\n");
  {
    uint8_t stream[sizeof VEC_10 + sizeof VEC_20 + sizeof VEC_21];
    uint16_t pos = 0;
    proto_frame_t got[4];

    memcpy(&stream[pos], VEC_10, sizeof VEC_10); pos = (uint16_t)(pos + sizeof VEC_10);
    memcpy(&stream[pos], VEC_20, sizeof VEC_20); pos = (uint16_t)(pos + sizeof VEC_20);
    memcpy(&stream[pos], VEC_21, sizeof VEC_21); pos = (uint16_t)(pos + sizeof VEC_21);

    proto_parser_reset(&ps);
    n = proto_parser_feed(&ps, stream, pos, got, 4u);
    check(n == 3u, "3 frames decoded");
    check(n == 3u && got[0].type == PROTO_T_STATUS && got[1].type == PROTO_T_TARGET
                  && got[2].type == PROTO_T_MODE, "types in order 0x10 / 0x20 / 0x21");
  }

  /* ---------- 9. 随机数据不崩、不越界 ---------- */
  printf("\n[9] random data robustness\n");
  {
    uint32_t seed = 12345u;
    uint32_t k;
    int      window_ok = 1;

    proto_parser_reset(&ps);
    for (k = 0u; k < 20000u; k++)
    {
      seed = seed * 1103515245u + 12345u;
      (void)proto_parser_push(&ps, (uint8_t)(seed >> 16), &fr);
      if (ps.cnt > PROTO_MAX_FRAME) { window_ok = 0; break; }
    }
    check(window_ok, "parser window never overflows");
    printf("         random 20000 bytes -> frames=%u sof=%u len=%u crc=%u\n",
           (unsigned)ps.n_ok, (unsigned)ps.n_sof, (unsigned)ps.n_len, (unsigned)ps.n_crc);

    n = proto_parser_feed(&ps, VEC_24, (uint16_t)sizeof VEC_24, &fr, 1u);
    check(n == 1u && fr.type == PROTO_T_VERSION, "still decodes after random stream");
  }

  /* ---------- 10. proto_build 参数校验 ---------- */
  /* 这一段管的是"**拒绝**"：参数不对必须返回 0，绝不能越界写。
   * 上面 1~9 测的是"正确输入给正确输出"，这里测的是"错误输入给安全结果"。
   * 对嵌入式尤其重要：越界写不会崩溃，只会悄悄改掉旁边的变量。 */
  printf("\n[10] proto_build argument validation\n");
  {
    uint8_t tight[6];                      /* 6 字节，装不下 15 字节的 0x20 帧 */

    n = proto_build(tight, (uint16_t)sizeof tight, PROTO_T_TARGET, pl, PROTO_LEN_TARGET);
    check(n == 0u, "buffer too small -> returns 0 (no overflow)");

    n = proto_build(buf, (uint16_t)sizeof buf, PROTO_T_TARGET, pl,
                    (uint8_t)(PROTO_MAX_PAYLOAD + 1u));
    check(n == 0u, "payload length > 32 -> returns 0");

    n = proto_build(buf, (uint16_t)sizeof buf, PROTO_T_VERSION, NULL, 0u);
    check(n == 4u, "NULL payload with len 0 is allowed (SOF+type+len+crc)");
  }

  /* ---------- 汇总 ---------- */
  printf("\n----------------------------------------\n");
  printf(" RESULT: %d passed, %d failed\n", g_pass, g_fail);
  printf("----------------------------------------\n");

  return (g_fail == 0) ? 0 : 1;
}
