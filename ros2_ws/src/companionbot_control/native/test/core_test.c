#include "companionbot/control.h"
#include "companionbot/wire.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void) {
    cb_brake b; cb_brake_config bc={CB_DT,0.5,0.8,1.0};
    cb_reference r={1,0.4,0,0.03,0.01,0.02}, o;
    cb_allocation a; cb_frame f={0}, decoded; cb_parser parser={0};
    uint8_t wire[CB_WIRE_ENCODED]; size_t n,i,j; int got;
    cb_estimator e; cb_estimate estimate;
    cb_estimator_config ec={CB_DT,0.5,80,1000,0.04,0.05,0.25,1,-1,1,1};
    cb_sensor sensor={{0,0,9.81},{0,0,0},0,{0,0},1};
    assert(cb_crc32((const uint8_t *)"123456789",9)==0xcbf43926u);
    assert(cb_brake_init(&b,&bc,&r)==CB_OK);
    assert(cb_brake_step(&b,&o)==CB_OK && o.v==r.v && o.u_ff==r.u_ff);
    for(i=0;i<3000;i++) {
        double previous=b.current.a;
        assert(cb_brake_step(&b,&o)==CB_OK);
        assert(fabs(o.a-previous)<=bc.jerk*bc.dt+1e-12);
    }
    assert(b.settled && cb_brake_handoff_done(&b) && fabs(o.theta)<1e-14);
    assert(cb_allocate(0.19,0.4,0.1,&a)==CB_OK);
    assert(fabs(a.u_diff_available-0.01)<1e-12 && fabs(a.left-0.1)<1e-12 && a.differential_clipped);
    assert(cb_allocate(0.21,0,0.1,&a)==CB_INVALID);
    assert(cb_allocate(NAN,0,0.1,&a)==CB_INVALID);
    assert(cb_estimator_init(&e,&ec)==CB_OK);
    assert(cb_estimator_reset(&e,&sensor,&estimate)==CB_OK);
    for(i=0;i<1000;i++) assert(cb_estimator_update(&e,&sensor,&estimate)==CB_OK);
    assert(estimate.position==0 && estimate.velocity==0 && estimate.theta==0);
    sensor.age=-1; assert(cb_estimator_update(&e,&sensor,&estimate)==CB_INVALID);
    f.type=CB_MSG_STATUS; f.session=42; f.sequence=7; f.tick=123;
    for(j=0;j<=CB_WIRE_PAYLOAD;j++) {
        f.length=(uint16_t)j;
        for(i=0;i<j;i++) f.payload[i]=(uint8_t)(i*j);
        n=cb_frame_encode(&f,wire,sizeof(wire)); assert(n>0 && n<=sizeof(wire));
        got=0;
        for(i=0;i<n;i++) if(cb_parser_feed(&parser,wire[i],&decoded)==1) got++;
        assert(got==1 && decoded.length==j && decoded.tick==f.tick);
        assert(memcmp(decoded.payload,f.payload,j)==0);
    }
    wire[n/2]^=1; got=0;
    for(i=0;i<n;i++) if(cb_parser_feed(&parser,wire[i],&decoded)==1) got++;
    assert(got==0 && parser.rejected>0);
    for(i=0;i<2*CB_WIRE_ENCODED;i++) (void)cb_parser_feed(&parser,1,&decoded);
    assert(cb_parser_feed(&parser,0,&decoded)==-1 && parser.overflows==1);
    f.length=0; f.type=CB_MSG_HEARTBEAT;
    assert(cb_wire_validate_command(&f)==CB_OK);
    n=cb_frame_encode(&f,wire,sizeof(wire)); got=0;
    for(i=0;i<n;i++) if(cb_parser_feed(&parser,wire[i],&decoded)==1) got++;
    assert(got==1);
    /* Deterministic malformed stream: boundaries and resynchronization must
     * remain safe under ASan/UBSan; then a clean frame must still be received. */
    {
        uint32_t seed=0x27182818u;
        for(i=0;i<200000;i++) {
            seed=1664525u*seed+1013904223u;
            (void)cb_parser_feed(&parser,(uint8_t)(seed>>24),&decoded);
        }
        (void)cb_parser_feed(&parser,0,&decoded);
        got=0;
        for(i=0;i<n;i++) if(cb_parser_feed(&parser,wire[i],&decoded)==1) got++;
        assert(got==1);
    }
    printf("control, brake, estimator, CRC32, 673 COBS lengths, corruption and overflow passed\n");
    return 0;
}
