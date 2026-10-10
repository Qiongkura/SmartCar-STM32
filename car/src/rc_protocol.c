#include "rc_protocol.h"

void rc_parser_reset(rc_parser_t *p) { p->used = 0; p->length = 5; p->last_ms = 0; }
int rc_parser_expire(rc_parser_t *p, uint32_t now) {
  if (p->used && (uint32_t)(now - p->last_ms) > RC_PACKET_GAP_MS) {
    rc_parser_reset(p); return RC_PARSE_BAD;
  }
  return RC_PARSE_NONE;
}
static int16_t signed_byte(uint8_t b) { return b < 128u ? b : (int16_t)b - 256; }
static int bad_restart(rc_parser_t *p, uint8_t byte) {
  p->used = byte == 0xAA ? 1 : 0;
  p->data[0] = 0xAA;
  p->length = 5;
  return RC_PARSE_BAD;
}
int rc_parser_feed(rc_parser_t *p, uint8_t byte, uint32_t ms, rc_packet_t *out) {
  int expired = rc_parser_expire(p, ms);
  p->last_ms = ms;
  /* Detect a new header BEFORE the old candidate can pass its XOR check.
   * This specifically rejects AA 55 06 AA 55 06. */
  if (p->used >= 3 && p->used < p->length - 1 &&
      p->data[p->used - 1] == 0xAA && byte == 0x55) {
    p->data[0] = 0xAA; p->data[1] = 0x55; p->used = 2; p->length = 5;
    return RC_PARSE_BAD;
  }
  if (!p->used) {
    if (byte == 0xAA) { p->data[0] = byte; p->used = 1; }
    return expired;
  }
  if (p->used == 1) {
    if (byte == 0x55) { p->data[1] = byte; p->used = 2; }
    else if (byte != 0xAA) p->used = 0;
    return expired;
  }
  if (p->used == 2) {
    if (byte < 1 || byte > 7) return bad_restart(p, byte);
    p->length = byte == 6 || byte == 7 ? 6 : 5;
  }
  p->data[p->used++] = byte;
  if (p->used < p->length) return expired;
  uint8_t x = 0;
  for (uint8_t i = 0; i < p->length - 1; ++i) x ^= p->data[i];
  if (x != byte) {
    if (p->data[p->length - 2] == 0xAA && byte == 0x55) {
      p->data[0] = 0xAA; p->data[1] = 0x55; p->used = 2; p->length = 5;
      return RC_PARSE_BAD;
    }
    return bad_restart(p, byte);
  }
  uint8_t cmd = p->data[2];
  int16_t a = cmd >= 6 ? signed_byte(p->data[3]) : p->data[3];
  int16_t b = cmd >= 6 ? signed_byte(p->data[4]) : 0;
  if (a < -100 || a > 100 || b < -100 || b > 100 || (cmd == 5 && a != 0))
    return bad_restart(p, byte);
  out->cmd = cmd; out->a = a; out->b = b; out->received_ms = ms;
  rc_parser_reset(p);
  return RC_PARSE_PACKET;
}
