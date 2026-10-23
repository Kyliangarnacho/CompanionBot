#include "companionbot/wire.h"
#include <math.h>
#include <string.h>

typedef char cb_float32_required[(sizeof(float)==4)?1:-1];
typedef char cb_float64_required[(sizeof(double)==8)?1:-1];
uint32_t cb_core_abi_version(void) { return 0x00010000u; }
size_t cb_core_abi_size(unsigned id) {
    const size_t sizes[]={0,sizeof(cb_reference),sizeof(cb_brake_config),sizeof(cb_brake),
        sizeof(cb_allocation),sizeof(cb_estimator_config),sizeof(cb_sensor),sizeof(cb_estimate),
        sizeof(cb_yaw_estimator),sizeof(cb_mcu_config),sizeof(cb_mcu_output),sizeof(cb_frame),sizeof(cb_parser)};
    return id<sizeof(sizes)/sizeof(sizes[0])?sizes[id]:0;
}
uint16_t cb_get_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
uint32_t cb_get_u32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
void cb_put_u16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
void cb_put_u32(uint8_t *p,uint32_t v) { unsigned i; for(i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i)); }
float cb_get_f32(const uint8_t *p) { uint32_t u=cb_get_u32(p); float f; memcpy(&f,&u,4); return f; }
double cb_get_f64(const uint8_t *p) {
    uint64_t u=0; double d; unsigned i;
    for(i=0;i<8;i++) u|=(uint64_t)p[i]<<(8*i);
    memcpy(&d,&u,8); return d;
}
void cb_put_f32(uint8_t *p,float f) { uint32_t u; memcpy(&u,&f,4); cb_put_u32(p,u); }
uint32_t cb_crc32(const uint8_t *p,size_t n) {
    uint32_t c=0xffffffffu; size_t i; unsigned b;
    for(i=0;i<n;i++) { c^=p[i]; for(b=0;b<8;b++) c=(c>>1)^(0xedb88320u & (0u-(c&1u))); }
    return c^0xffffffffu;
}
static size_t cobs_encode(const uint8_t *in,size_t n,uint8_t *out) {
    size_t read=0,write=1,code_at=0; uint8_t code=1;
    while(read<n) {
        if(in[read]==0) { out[code_at]=code; code=1; code_at=write++; read++; }
        else {
            out[write++]=in[read++]; code++;
            if(code==255) { out[code_at]=code; code=1; code_at=write++; }
        }
    }
    out[code_at]=code; out[write++]=0; return write;
}
static size_t cobs_decode(const uint8_t *in,size_t n,uint8_t *out) {
    size_t r=0,w=0; unsigned i; uint8_t code;
    while(r<n) {
        code=in[r++];
        if(!code || (size_t)(code-1)>n-r || w+(size_t)(code-1)>CB_WIRE_RAW) return 0;
        for(i=1;i<code;i++) out[w++]=in[r++];
        if(code!=255 && r<n) { if(w>=CB_WIRE_RAW) return 0; out[w++]=0; }
    }
    return w;
}
size_t cb_frame_encode(const cb_frame *f,uint8_t *out,size_t cap) {
    uint8_t raw[CB_WIRE_RAW]; size_t n;
    if(!f || !out || f->length>CB_WIRE_PAYLOAD) return 0;
    n=16+f->length;
    if(cap<n+4+(n+4)/254+2) return 0;
    raw[0]=CB_WIRE_VERSION; raw[1]=f->type; cb_put_u16(raw+2,f->length);
    cb_put_u32(raw+4,f->session); cb_put_u32(raw+8,f->sequence); cb_put_u32(raw+12,f->tick);
    memcpy(raw+16,f->payload,f->length); cb_put_u32(raw+n,cb_crc32(raw,n));
    return cobs_encode(raw,n+4,out);
}
int cb_parser_feed(cb_parser *p,uint8_t b,cb_frame *out) {
    uint8_t raw[CB_WIRE_RAW]; size_t n; uint16_t len;
    if(!p || !out) return -1;
    if(b) {
        if(p->dropping) return 0;
        if(p->used>=sizeof(p->bytes)) { p->dropping=1; p->used=0; p->overflows++; return 0; }
        p->bytes[p->used++]=b; return 0;
    }
    if(p->dropping) { p->dropping=0; p->used=0; p->rejected++; return -1; }
    if(!p->used) return 0;
    n=cobs_decode(p->bytes,p->used,raw); p->used=0;
    if(n<20 || raw[0]!=CB_WIRE_VERSION) { p->rejected++; return -1; }
    len=cb_get_u16(raw+2);
    if(len>CB_WIRE_PAYLOAD || n!=(size_t)len+20 || cb_get_u32(raw+n-4)!=cb_crc32(raw,n-4)) {
        p->rejected++; return -1;
    }
    out->type=raw[1]; out->length=len; out->session=cb_get_u32(raw+4);
    out->sequence=cb_get_u32(raw+8); out->tick=cb_get_u32(raw+12);
    memcpy(out->payload,raw+16,len); p->accepted++; return 1;
}
int cb_block_decode(const uint8_t *p,size_t n,cb_block *b) {
    uint32_t count; unsigned i,j;
    if(!p || !b || n<40 || cb_get_u32(p)!=1) return CB_INVALID;
    count=cb_get_u32(p+32);
    if(count<1 || count>CB_BLOCK_SAMPLES || n!=40+24*count) return CB_INVALID;
    b->epoch=cb_get_u32(p+4); b->sequence=cb_get_u32(p+8); b->start_tick=cb_get_u32(p+12);
    b->generated_time=cb_get_f64(p+16); b->dt=cb_get_f64(p+24);
    b->count=count; b->yaw=cb_get_f32(p+36);
    if(!isfinite(b->generated_time) || !isfinite(b->dt) || b->dt<=0 || !isfinite(b->yaw) ||
       b->start_tick>UINT32_MAX-count) return CB_INVALID;
    for(i=0;i<count;i++) for(j=0;j<6;j++) {
        b->samples[i][j]=cb_get_f32(p+40+24*i+4*j);
        if(!isfinite(b->samples[i][j])) return CB_INVALID;
    }
    return CB_OK;
}
int cb_wire_validate_command(const cb_frame *f) {
    cb_block b;
    if(!f || f->length>CB_WIRE_PAYLOAD) return CB_INVALID;
    switch(f->type) {
    case CB_MSG_REFERENCE:
        if(cb_block_decode(f->payload,f->length,&b)!=CB_OK || b.sequence!=f->sequence) return CB_INVALID;
        return CB_OK;
    case CB_MSG_HEARTBEAT: return f->length==0?CB_OK:CB_INVALID;
    case CB_MSG_YAW: return f->length==8 && isfinite(cb_get_f64(f->payload))?CB_OK:CB_INVALID;
    case CB_MSG_SAFETY: return f->length==3 && f->payload[0]<=CB_LONGITUDINAL && cb_get_u16(f->payload+1)>0?CB_OK:CB_INVALID;
    case CB_MSG_RESUME: return f->length==5 && f->payload[4]<=CB_LONGITUDINAL && cb_get_u32(f->payload)>0?CB_OK:CB_INVALID;
    default: return CB_INVALID;
    }
}
int cb_mcu_dispatch(cb_mcu *m,const cb_frame *f,uint32_t session,uint32_t tick) {
    if(!m || !f || f->session!=session || session==0) return CB_EPOCH;
    /* No wrap acceptance: reboot/reconnect before tick/sequence exhaustion. */
    if(f->tick<tick || f->tick-tick>125u) return CB_STALE;
    if(cb_wire_validate_command(f)!=CB_OK) return CB_INVALID;
    switch(f->type) {
    case CB_MSG_REFERENCE: return cb_mcu_reference(m,f->payload,f->length,tick);
    case CB_MSG_HEARTBEAT: return cb_mcu_heartbeat(m,f->sequence,tick);
    case CB_MSG_YAW: return cb_mcu_yaw(m,f->sequence,cb_get_f64(f->payload),tick);
    case CB_MSG_SAFETY: return cb_mcu_safety(m,f->sequence,f->payload[0],cb_get_u16(f->payload+1),tick);
    case CB_MSG_RESUME: return cb_mcu_resume(m,f->sequence,cb_get_u32(f->payload),f->payload[4],tick);
    default: return CB_INVALID;
    }
}
