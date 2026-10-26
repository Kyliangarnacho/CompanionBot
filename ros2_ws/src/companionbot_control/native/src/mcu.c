#include "companionbot/control.h"
#include <math.h>
#include <string.h>

size_t cb_mcu_size(void) { return sizeof(cb_mcu); }
static int reject(cb_mcu *m,int code) { if(m && code>0 && code<8) m->rejected[code]++; return code; }
static uint32_t ticks(double seconds,double dt) { return (uint32_t)ceil(seconds/dt); }
static void sample(const cb_block *b,uint32_t tick,cb_reference *r) {
    const float *s=b->samples[tick-b->start_tick];
    r->p=s[0]; r->v=s[1]; r->a=s[2]; r->theta=s[3]; r->theta_dot=s[4]; r->u_ff=s[5];
}
int cb_mcu_init(cb_mcu *m,const cb_mcu_config *c,uint32_t epoch) {
    cb_brake test; cb_reference zero={0};
    if(!m || !c || cb_brake_init(&test,&c->brake,&zero)!=CB_OK ||
       fabs(c->brake.dt-CB_DT)>1e-12 || !isfinite(c->heartbeat_timeout) || c->heartbeat_timeout<=0 ||
       !isfinite(c->starvation_margin) || c->starvation_margin<0 ||
       !isfinite(c->nominal_pitch) || !isfinite(c->yaw_timeout) || c->yaw_timeout<=0 ||
       !isfinite(c->yaw_decay) || c->yaw_decay<=0) return CB_INVALID;
    memset(m,0,sizeof(*m)); m->config=*c; m->epoch=epoch; return CB_OK;
}
int cb_mcu_reference(cb_mcu *m,const uint8_t *p,size_t len,uint32_t tick) {
    cb_block b;
    if(!m || cb_block_decode(p,len,&b)!=CB_OK) return reject(m,CB_INVALID);
    if(b.epoch!=m->epoch) return reject(m,CB_EPOCH);
    if(b.count!=CB_BLOCK_SAMPLES || fabs(b.dt-m->config.brake.dt)>1e-12) return reject(m,CB_INVALID);
    if(b.start_tick<tick || (m->paused && b.start_tick<=tick)) return reject(m,CB_STALE);
    if(m->paused) {
        if(m->has_next) return reject(m,CB_CAPACITY);
        m->next=b; m->has_next=1;
    } else if(!m->has_active) { m->active=b; m->has_active=1; }
    else if(!m->has_next && b.start_tick==m->active.start_tick+CB_BLOCK_SAMPLES) {
        m->next=b; m->has_next=1;
    } else return reject(m,CB_CAPACITY);
    if(m->has_ref_sequence && b.sequence!=m->last_ref_sequence+1) m->sequence_gaps++;
    m->last_ref_sequence=b.sequence; m->has_ref_sequence=1; return CB_OK;
}
int cb_mcu_heartbeat(cb_mcu *m,uint32_t seq,uint32_t tick) {
    if(!m) return CB_INVALID;
    if(m->has_heartbeat && seq<=m->heartbeat_sequence) return reject(m,CB_SEQUENCE);
    m->has_heartbeat=1; m->heartbeat_sequence=seq; m->heartbeat_tick=tick; m->pi_lost=0; return CB_OK;
}
int cb_mcu_yaw(cb_mcu *m,uint32_t seq,double target,uint32_t tick) {
    if(!m || !isfinite(target)) return reject(m,CB_INVALID);
    if(m->has_yaw && seq<=m->yaw_sequence) return reject(m,CB_SEQUENCE);
    m->has_yaw=1; m->yaw_sequence=seq; m->yaw_target=target; m->yaw_receive_tick=tick; return CB_OK;
}
static void disarm(cb_mcu *m,uint32_t tick) { m->hard_disarmed=1; m->hard_rearm_tick=tick; }
/* A revocation barrier applies at command acceptance, including while already
 * stopped. Advancing the epoch also rejects old references still in flight. */
static void revoke(cb_mcu *m) {
    m->epoch++; m->has_active=0; m->has_next=0; m->has_ref_sequence=0; m->paused=1;
    m->pending_resume=0; m->resume_armed=0;
}
static void takeover(cb_mcu *,uint32_t,int,int);
static int fresh_rearm(const cb_mcu *m,uint32_t tick) {
    return m->has_yaw && m->hard_disarmed && m->yaw_receive_tick>m->hard_rearm_tick &&
           tick>=m->yaw_receive_tick && tick-m->yaw_receive_tick<=ticks(m->config.yaw_timeout,m->config.brake.dt);
}
int cb_mcu_safety(cb_mcu *m,uint32_t seq,int authority,int reason,uint32_t tick) {
    if(!m || (authority!=CB_HARD && authority!=CB_LONGITUDINAL) || reason<=0) return reject(m,CB_INVALID);
    if(m->has_safety_sequence && seq<=m->safety_sequence) return reject(m,CB_SEQUENCE);
    if(authority==CB_LONGITUDINAL) {
        if(m->state!=CB_NORMAL || (m->pending_safety && m->pending_authority==CB_HARD))
            return reject(m,CB_STATE);
        m->pending_safety=1; m->pending_authority=authority; m->pending_reason=reason;
    } else if(m->state==CB_NORMAL) {
        /* Do not defer revocation until the next control tick: ingress may run
         * just after a release. The brake's first sample remains step-driven. */
        takeover(m,tick,authority,reason);
    } else {
        revoke(m); m->pending_safety=0; disarm(m,tick);
        if(m->authority==CB_LONGITUDINAL) {
            m->authority=CB_HARD; m->reason=reason;
            if(m->state==CB_LONGITUDINAL_STOP) m->state=CB_SAFE_HOLD;
        }
        /* Repeated HARD does not restart braking or replace an independent
         * fault's reason. A new sequence still establishes a new barrier. */
    }
    m->has_safety_sequence=1; m->safety_sequence=seq; return CB_OK;
}
int cb_mcu_resume(cb_mcu *m,uint32_t seq,uint32_t epoch,int authority,uint32_t tick) {
    int expected=authority==CB_LONGITUDINAL?CB_LONGITUDINAL_STOP:CB_SAFE_HOLD;
    if(!m || (authority!=CB_HARD && authority!=CB_LONGITUDINAL)) return reject(m,CB_INVALID);
    if(m->has_resume_sequence && seq<=m->resume_sequence) return reject(m,CB_SEQUENCE);
    m->has_resume_sequence=1; m->resume_sequence=seq;
    if(m->state!=expected || epoch!=m->epoch || m->pi_lost ||
       (authority==CB_HARD && !fresh_rearm(m,tick))) return reject(m,CB_STATE);
    m->pending_resume=1; m->pending_resume_epoch=epoch; m->pending_authority=authority; return CB_OK;
}
static void takeover(cb_mcu *m,uint32_t tick,int authority,int reason) {
    cb_reference initial={0};
    if(m->has_last) initial=m->output.reference;
    (void)cb_brake_init(&m->brake,&m->config.brake,&initial);
    revoke(m); m->pending_safety=0; m->state=CB_BRAKING;
    m->authority=authority; m->reason=reason;
    if(authority==CB_HARD) disarm(m,tick);
}
static double yaw_sample(cb_mcu *m,uint32_t tick,int hard_stopped) {
    int fresh=m->has_yaw && tick>=m->yaw_receive_tick &&
        tick-m->yaw_receive_tick<=ticks(m->config.yaw_timeout,m->config.brake.dt);
    if(hard_stopped || !fresh) {
        double step=m->config.yaw_decay*m->config.brake.dt;
        m->yaw_output+=fmax(-step,fmin(-m->yaw_output,step));
    } else m->yaw_output=m->yaw_target;
    return m->yaw_output;
}
int cb_mcu_step(cb_mcu *m,uint32_t tick,double velocity,double pitch,double rate,cb_mcu_output *out) {
    cb_reference safe;
    if(!m || !out || !isfinite(velocity) || !isfinite(pitch) || !isfinite(rate)) return CB_INVALID;
    m->local_velocity=velocity; m->local_pitch=pitch; m->local_pitch_rate=rate;
    if((m->state==CB_SAFE_HOLD || m->state==CB_LONGITUDINAL_STOP) && !m->pi_lost && m->has_heartbeat &&
       (tick-m->heartbeat_tick)*m->config.brake.dt>m->config.heartbeat_timeout+1e-12) {
        m->pi_lost=1; revoke(m); m->state=CB_SAFE_HOLD; m->authority=CB_HARD; disarm(m,tick);
        /* Restored heartbeat clears health, not this independent fault. Keep
         * an earlier local fault as the primary reason; do not leave a Pi STOP
         * reason that would authorize a task owner to resume a lost-link stop. */
        if(m->reason!=CB_REASON_HEARTBEAT && m->reason!=CB_REASON_STARVATION &&
           m->reason!=CB_REASON_UNDERRUN) m->reason=CB_REASON_HEARTBEAT;
    }
    if(m->state==CB_SAFE_HOLD || m->state==CB_LONGITUDINAL_STOP) {
        if(m->pending_resume && m->pending_resume_epoch==m->epoch && !m->resume_armed &&
           m->has_next && m->next.start_tick>tick &&
           (m->pending_authority!=CB_HARD || fresh_rearm(m,tick))) {
            m->resume_armed=1; m->resume_tick=m->next.start_tick;
        }
        if(m->resume_armed && m->resume_tick==tick &&
           (m->pending_authority!=CB_HARD || fresh_rearm(m,tick))) {
            m->active=m->next; m->has_active=1; m->has_next=0; m->paused=0;
            m->state=CB_NORMAL; m->reason=CB_REASON_NONE; m->pending_resume=0; m->resume_armed=0;
        } else if(m->has_next && m->next.start_tick<=tick) { m->has_next=0; m->resume_armed=0; }
    }
    if(m->state==CB_NORMAL) {
        if(m->pending_safety) {
            m->pending_safety=0; takeover(m,tick,m->pending_authority,m->pending_reason);
        } else if(m->has_heartbeat && (tick-m->heartbeat_tick)*m->config.brake.dt>m->config.heartbeat_timeout+1e-12) {
            takeover(m,tick,CB_HARD,CB_REASON_HEARTBEAT);
        } else {
            if(m->has_active && tick==m->active.start_tick+CB_BLOCK_SAMPLES) {
                m->has_active=m->has_next && m->next.start_tick==tick;
                if(m->has_active) m->active=m->next;
                m->has_next=0;
            }
            if(!m->has_active || tick<m->active.start_tick || tick>=m->active.start_tick+CB_BLOCK_SAMPLES) {
                m->underruns++; takeover(m,tick,CB_HARD,CB_REASON_UNDERRUN);
            } else if(m->active.start_tick+CB_BLOCK_SAMPLES-tick<=ticks(m->config.starvation_margin,m->config.brake.dt) &&
                      !(m->has_next && m->next.start_tick==m->active.start_tick+CB_BLOCK_SAMPLES)) {
                takeover(m,tick,CB_HARD,CB_REASON_STARVATION);
            }
        }
    }
    if(m->state==CB_NORMAL) {
        sample(&m->active,tick,&m->output.reference);
        m->output.block_sequence=m->active.sequence; m->output.sample_index=tick-m->active.start_tick;
    } else {
        if(m->state==CB_SAFE_HOLD || m->state==CB_LONGITUDINAL_STOP) {
            memset(&safe,0,sizeof(safe)); safe.p=m->output.reference.p;
        } else (void)cb_brake_step(&m->brake,&safe);
        if(m->state==CB_BRAKING && cb_brake_handoff_done(&m->brake) && m->brake.settled &&
           fabs(velocity)<=0.03 && fabs(pitch-m->config.nominal_pitch)<=0.08 && fabs(rate)<=0.20) {
            m->state=m->authority==CB_LONGITUDINAL?CB_LONGITUDINAL_STOP:CB_SAFE_HOLD;
        }
        if(m->state==CB_SAFE_HOLD || m->state==CB_LONGITUDINAL_STOP) {
            double pos=safe.p; memset(&safe,0,sizeof(safe)); safe.p=pos;
        }
        m->output.reference=safe; m->output.block_sequence=-1; m->output.sample_index=(int64_t)m->brake.step-1;
    }
    m->output.yaw_rate=yaw_sample(m,tick,m->state!=CB_NORMAL && m->authority==CB_HARD);
    m->output.epoch=m->epoch; m->output.safety_state=(uint32_t)m->state;
    m->output.reason=(uint32_t)m->reason; m->output.pi_lost=(uint32_t)m->pi_lost;
    m->has_last=1; *out=m->output; return CB_OK;
}
int cb_mcu_peek(const cb_mcu *m,cb_reference *r) {
    uint32_t tick; const cb_block *b;
    if(!m || !r || !m->has_last) return CB_STATE;
    if(m->state!=CB_NORMAL) { *r=m->output.reference; return CB_OK; }
    tick=m->active.start_tick+(uint32_t)m->output.sample_index+1;
    b=tick<m->active.start_tick+CB_BLOCK_SAMPLES?&m->active:(m->has_next?&m->next:NULL);
    if(!b || tick<b->start_tick || tick>=b->start_tick+b->count) return CB_UNDERRUN;
    sample(b,tick,r); return CB_OK;
}
