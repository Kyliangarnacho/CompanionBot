#ifndef COMPANIONBOT_WIRE_H
#define COMPANIONBOT_WIRE_H
#include "control.h"
#ifdef __cplusplus
extern "C" {
#endif
#define CB_WIRE_VERSION 2u
#define CB_WIRE_PAYLOAD 672u
#define CB_STATUS_BYTES 144u
#define CB_WIRE_RAW (16u+CB_WIRE_PAYLOAD+4u)
#define CB_WIRE_ENCODED (CB_WIRE_RAW+CB_WIRE_RAW/254u+2u)
enum { CB_MSG_REFERENCE=1, CB_MSG_HEARTBEAT=2, CB_MSG_YAW=3,
       CB_MSG_SAFETY=4, CB_MSG_RESUME=5, CB_MSG_STATUS=128 };
typedef struct {
    uint8_t type;
    uint16_t length;
    uint32_t session, sequence, tick;
    uint8_t payload[CB_WIRE_PAYLOAD];
} cb_frame;
typedef struct {
    uint8_t bytes[CB_WIRE_ENCODED];
    size_t used;
    uint32_t accepted, rejected, overflows;
    int dropping;
} cb_parser;
uint32_t cb_crc32(const uint8_t *, size_t);
uint32_t cb_core_abi_version(void);
size_t cb_core_abi_size(unsigned type_id);
size_t cb_frame_encode(const cb_frame *, uint8_t *, size_t);
/* Return 1 for a complete frame, 0 while collecting, -1 on rejected delimiter. */
int cb_parser_feed(cb_parser *, uint8_t, cb_frame *);
int cb_wire_validate_command(const cb_frame *);
int cb_mcu_dispatch(cb_mcu *, const cb_frame *, uint32_t session, uint32_t tick);
uint16_t cb_get_u16(const uint8_t *);
uint32_t cb_get_u32(const uint8_t *);
float cb_get_f32(const uint8_t *);
double cb_get_f64(const uint8_t *);
void cb_put_u16(uint8_t *, uint16_t);
void cb_put_u32(uint8_t *, uint32_t);
void cb_put_f32(uint8_t *, float);
#ifdef __cplusplus
}
#endif
#endif
