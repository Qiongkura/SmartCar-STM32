#ifndef RC_PROTOCOL_H
#define RC_PROTOCOL_H
#include <stdint.h>

/* Legacy AA 55 protocol. AA 55 is reserved inside the payload: senders
 * must not put the exact pair (-86,+85) in a MIX/TANK payload. */
#define RC_PACKET_GAP_MS 80u
typedef struct {
  uint8_t data[6], used, length;
  uint32_t last_ms;
} rc_parser_t;
typedef struct { uint8_t cmd; int16_t a, b; uint32_t received_ms; } rc_packet_t;
enum { RC_PARSE_NONE = 0, RC_PARSE_PACKET = 1, RC_PARSE_BAD = 2 };
void rc_parser_reset(rc_parser_t *p);
int rc_parser_expire(rc_parser_t *p, uint32_t now);
int rc_parser_feed(rc_parser_t *p, uint8_t byte, uint32_t ms, rc_packet_t *out);
#endif
