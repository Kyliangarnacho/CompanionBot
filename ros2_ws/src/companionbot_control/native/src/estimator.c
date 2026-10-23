#include "companionbot/control.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

size_t cb_estimator_size(void) { return sizeof(cb_estimator); }
int cb_estimator_init(cb_estimator *e, const cb_estimator_config *c) {
    double values[7]; unsigned i;
    if (!e || !c) return CB_INVALID;
    values[0]=c->dt; values[1]=c->pitch_tau; values[2]=c->pll_bandwidth;
    values[3]=c->counts_per_rev; values[4]=c->wheel_radius;
    values[5]=c->acceleration_tau; values[6]=c->acceleration_scale;
    for(i=0;i<7;i++) if(!isfinite(values[i]) || values[i]<=0) return CB_INVALID;
    if(c->dt*2*c->pll_bandwidth>=1 || abs(c->left_sign)!=1 || abs(c->right_sign)!=1) return CB_INVALID;
    memset(e,0,sizeof(*e)); e->config=*c; return CB_OK;
}
static int valid_sensor(const cb_sensor *s) {
    unsigned i;
    if(!s || !isfinite(s->age) || s->age<0) return 0;
    for(i=0;i<3;i++) if(!isfinite(s->accel[i]) || !isfinite(s->gyro[i])) return 0;
    return 1;
}
static void angles(cb_estimator *e) {
    unsigned i; const double tau=6.28318530717958647692;
    for(i=0;i<2;i++) {
        double divisor=(i==0?e->config.left_sign:e->config.right_sign)*e->config.counts_per_rev;
        /* Preserve the frozen NumPy expression (tau * counts) / divisor.
         * Precomputing tau/divisor changes roundoff before contact dynamics. */
        e->output.wheel_angle[i]=(tau*e->output.pll_position[i])/divisor;
        e->output.wheel_velocity[i]=(tau*e->output.pll_velocity[i])/divisor;
    }
}
int cb_estimator_reset(cb_estimator *e, const cb_sensor *s, cb_estimate *out) {
    unsigned i;
    if(!e || !out || !valid_sensor(s)) return CB_INVALID;
    memset(&e->output,0,sizeof(e->output));
    e->output.theta_measurement=atan2(s->accel[1],s->accel[2]);
    e->output.theta_acc=e->output.theta_measurement;
    e->output.raw_theta_acc=e->output.theta_acc;
    e->output.theta_dot=s->gyro[0]; e->output.age=s->age;
    e->output.extrapolation_age=s->extrapolate?s->age:0;
    e->output.theta=cb_wrap(e->output.theta_measurement+s->gyro[0]*e->output.extrapolation_age);
    e->output.correction_gain=1;
    for(i=0;i<3;i++) e->output.compensated_accel[i]=s->accel[i];
    for(i=0;i<2;i++) e->previous_pll_position[i]=e->output.pll_position[i]=(double)s->counts[i];
    angles(e); e->previous_theta=e->output.theta; e->previous_velocity=0;
    e->initialized=1; *out=e->output; return CB_OK;
}
int cb_estimator_update(cb_estimator *e, const cb_sensor *s, cb_estimate *out) {
    double dt,pred,acc,gain=1,innovation,theta,delta_mean=0,kp,ki;
    unsigned i;
    if(!e || !out || !valid_sensor(s)) return CB_INVALID;
    if(!e->initialized) return cb_estimator_reset(e,s,out);
    dt=e->config.dt; kp=2*e->config.pll_bandwidth; ki=0.25*kp*kp;
    pred=e->output.theta_measurement+s->gyro[0]*dt;
    for(i=0;i<3;i++) e->output.compensated_accel[i]=s->accel[i];
    e->output.raw_theta_acc=atan2(s->accel[1],s->accel[2]);
    if(e->config.acceleration_compensation) {
        double ratio=e->output.acceleration_next/e->config.acceleration_scale;
        e->output.compensated_accel[1]+=e->output.acceleration_next*cos(pred);
        e->output.compensated_accel[2]-=e->output.acceleration_next*sin(pred);
        gain=1/(1+ratio*ratio); e->output.acceleration_used=e->output.acceleration_next;
    }
    acc=atan2(e->output.compensated_accel[1],e->output.compensated_accel[2]);
    innovation=cb_wrap(acc-pred);
    e->output.theta_measurement=cb_wrap(pred+(1-e->config.pitch_tau/(e->config.pitch_tau+dt))*gain*innovation);
    e->output.extrapolation_age=s->extrapolate?s->age:0;
    theta=cb_wrap(e->output.theta_measurement+s->gyro[0]*e->output.extrapolation_age);
    for(i=0;i<2;i++) {
        double divisor=(i==0?e->config.left_sign:e->config.right_sign)*e->config.counts_per_rev;
        double err;
        e->output.pll_position[i]+=dt*e->output.pll_velocity[i];
        err=(double)s->counts[i]-floor(e->output.pll_position[i]);
        e->output.pll_position[i]+=dt*kp*err;
        e->output.pll_velocity[i]+=dt*ki*err;
        if(e->config.snap_to_zero && fabs(e->output.pll_velocity[i])<0.5*dt*ki) e->output.pll_velocity[i]=0;
        delta_mean+=(6.28318530717958647692*(e->output.pll_position[i]-e->previous_pll_position[i]))/divisor;
        e->previous_pll_position[i]=e->output.pll_position[i];
    }
    angles(e);
    e->output.position+=e->config.wheel_radius*(0.5*delta_mean+cb_wrap(theta-e->previous_theta));
    e->previous_theta=theta;
    e->output.velocity=e->config.wheel_radius*(0.5*(e->output.wheel_velocity[0]+e->output.wheel_velocity[1])+s->gyro[0]);
    if(e->config.acceleration_compensation) {
        double derivative=(e->output.velocity-e->previous_velocity)/dt;
        e->output.acceleration_next+=dt/(e->config.acceleration_tau+dt)*(derivative-e->output.acceleration_next);
        e->previous_velocity=e->output.velocity;
    }
    e->output.theta=theta; e->output.theta_dot=s->gyro[0]; e->output.theta_acc=acc;
    e->output.age=s->age; e->output.correction_gain=gain;
    *out=e->output; return CB_OK;
}
int cb_yaw_estimate(cb_yaw_estimator *e,double r,double t,double age,int valid,int extrapolate,int reset,double o[3]) {
    if(!e || !o || !isfinite(r) || !isfinite(t) || !isfinite(age) || age<0) return CB_INVALID;
    o[2]=0;
    if(reset) { e->psi=0; e->rate=r; e->sample_time=t; e->initialized=1; }
    else {
        if(!e->initialized) return CB_STATE;
        if(valid) {
            double dt=t-e->sample_time;
            if(dt< -1e-12) return CB_STALE;
            if(dt>1e-12) { e->rate=r; e->psi+=r*dt; e->sample_time=t; o[2]=1; }
        }
    }
    o[0]=e->psi; o[1]=e->psi+e->rate*((extrapolate && (reset || valid))?age:0);
    return CB_OK;
}
