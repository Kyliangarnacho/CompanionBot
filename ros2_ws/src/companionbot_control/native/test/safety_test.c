/* MCU-local safety invariants. Host fixtures, not hardware certification. */
#include "companionbot/control.h"
#include "companionbot/wire.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void put_f64(uint8_t *p,double value) {
    uint64_t bits; unsigned i; memcpy(&bits,&value,sizeof(bits));
    for(i=0;i<8;i++) p[i]=(uint8_t)(bits>>(8*i));
}
static int reference(cb_mcu *m,uint32_t epoch,uint32_t seq,uint32_t start,uint32_t tick) {
    uint8_t p[640]={0};
    cb_put_u32(p,1); cb_put_u32(p+4,epoch); cb_put_u32(p+8,seq); cb_put_u32(p+12,start);
    put_f64(p+24,CB_DT); cb_put_u32(p+32,CB_BLOCK_SAMPLES);
    return cb_mcu_reference(m,p,sizeof(p),tick);
}
static void step(cb_mcu *m,uint32_t tick) {
    cb_mcu_output out;
    assert(cb_mcu_step(m,tick,0,0,0,&out)==CB_OK);
    assert(isfinite(out.reference.p) && out.reference.v==0);
}
static void normal(cb_mcu *m) {
    const cb_mcu_config config={{CB_DT,.5,.8,1},.15,.012,0,.2,.8};
    assert(cb_mcu_init(m,&config,0)==CB_OK);
    assert(reference(m,0,0,0,0)==CB_OK);
    assert(reference(m,0,1,25,0)==CB_OK);
    assert(cb_mcu_heartbeat(m,0,0)==CB_OK);
    assert(cb_mcu_yaw(m,0,0,0)==CB_OK);
    step(m,0); assert(m->state==CB_NORMAL);
}
static void hold(cb_mcu *m,int authority) {
    uint32_t tick;
    normal(m);
    assert(cb_mcu_safety(m,0,authority,CB_REASON_PI,1)==CB_OK);
    for(tick=1;tick<=600;tick++) {
        if(tick%20==0) assert(cb_mcu_heartbeat(m,tick,tick)==CB_OK);
        step(m,tick);
    }
    assert(m->state==(authority==CB_HARD?CB_SAFE_HOLD:CB_LONGITUDINAL_STOP));
}
static void arm(cb_mcu *m,int authority,uint32_t start) {
    assert(cb_mcu_yaw(m,601,0,601)==CB_OK);
    assert(reference(m,m->epoch,0,start,601)==CB_OK);
    assert(cb_mcu_resume(m,0,m->epoch,authority,601)==CB_OK);
    step(m,601); assert(m->pending_resume && m->resume_armed);
}
int main(int argc,char **argv) {
    const char *mode=argc>1?argv[1]:"armed_cancel";
    cb_mcu m; uint32_t epoch,brake_step,tick;
    if(!strcmp(mode,"normal_barrier")) {
        normal(&m); assert(cb_mcu_safety(&m,0,CB_HARD,CB_REASON_PI,1)==CB_OK);
        assert(m.epoch==1 && !m.has_active && !m.has_next && m.paused);
        assert(m.state==CB_BRAKING && m.hard_disarmed);
        assert(cb_mcu_safety(&m,1,CB_LONGITUDINAL,CB_REASON_PI,1)==CB_STATE);
        assert(m.authority==CB_HARD && m.safety_sequence==0);
    } else if(!strcmp(mode,"braking_revoke")) {
        normal(&m); assert(cb_mcu_safety(&m,0,CB_HARD,CB_REASON_PI,1)==CB_OK); step(&m,1);
        assert(reference(&m,m.epoch,0,20,2)==CB_OK);
        epoch=m.epoch; brake_step=m.brake.step;
        assert(cb_mcu_safety(&m,1,CB_HARD,CB_REASON_PI,2)==CB_OK);
        assert(m.epoch==epoch+1 && !m.has_next && m.brake.step==brake_step && m.state==CB_BRAKING);
    } else if(!strcmp(mode,"heartbeat_revoke")) {
        hold(&m,CB_LONGITUDINAL); arm(&m,CB_LONGITUDINAL,700); epoch=m.epoch;
        /* Fresh heartbeat before arm expires only after the future block was
         * prepared; move its start beyond the first missing-heartbeat tick. */
        for(tick=602;tick<=700;tick++) step(&m,tick);
        assert(m.pi_lost && m.state==CB_SAFE_HOLD && m.authority==CB_HARD);
        assert(m.epoch==epoch+1 && !m.pending_resume && !m.resume_armed && !m.has_next);
    } else {
        hold(&m,CB_HARD); arm(&m,CB_HARD,610); epoch=m.epoch; brake_step=m.brake.step;
        if(!strcmp(mode,"duplicate_stop")) {
            assert(cb_mcu_safety(&m,0,CB_HARD,CB_REASON_PI,602)==CB_SEQUENCE);
            assert(m.epoch==epoch && m.pending_resume && m.resume_armed && m.has_next);
        } else if(!strcmp(mode,"lower_authority")) {
            assert(cb_mcu_safety(&m,1,CB_LONGITUDINAL,CB_REASON_PI,602)==CB_STATE);
            assert(m.safety_sequence==0 && m.epoch==epoch && m.pending_resume && m.resume_armed);
        } else {
            assert(cb_mcu_safety(&m,1,CB_HARD,CB_REASON_PI,602)==CB_OK);
            assert(m.epoch==epoch+1 && !m.has_active && !m.has_next && !m.pending_resume && !m.resume_armed);
            assert(m.brake.step==brake_step && m.state==CB_SAFE_HOLD && m.hard_rearm_tick==602);
            assert(reference(&m,epoch,1,620,603)==CB_EPOCH);
            assert(cb_mcu_resume(&m,1,epoch,CB_HARD,603)==CB_STATE);
            if(!strcmp(mode,"new_authorization")) {
                assert(cb_mcu_yaw(&m,602,0,602)==CB_OK);
                assert(cb_mcu_resume(&m,2,m.epoch,CB_HARD,602)==CB_STATE);
                assert(cb_mcu_yaw(&m,603,0,603)==CB_OK);
                assert(reference(&m,m.epoch,0,620,603)==CB_OK);
                assert(cb_mcu_resume(&m,3,m.epoch,CB_LONGITUDINAL,603)==CB_STATE);
                assert(cb_mcu_resume(&m,4,m.epoch,CB_HARD,603)==CB_OK);
                for(tick=603;tick<=620;tick++) step(&m,tick);
                assert(m.state==CB_NORMAL && m.epoch==epoch+1);
            } else {
                for(tick=603;tick<=620;tick++) step(&m,tick);
                assert(m.state==CB_SAFE_HOLD && m.epoch==epoch+1);
            }
        }
    }
    printf("{\"case\":\"%s\",\"epoch\":%u,\"state\":%d,\"pending_resume\":%d,\"resume_armed\":%d}\n",
        mode,m.epoch,m.state,m.pending_resume,m.resume_armed);
    return 0;
}
