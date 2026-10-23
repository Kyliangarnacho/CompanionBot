#include "companionbot/control.h"
#include <math.h>
#include <string.h>

static double clip(double x, double lo, double hi) { return fmax(lo, fmin(x, hi)); }
double cb_wrap(double a) {
    const double pi=3.14159265358979323846;
    double x=fmod(a+pi, 2.0*pi);
    if (x<0.0) x+=2.0*pi;
    return x-pi;
}
int cb_feedback(const double k[4], const double e[4], double *out) {
    double sum=0.0; unsigned i;
    if (!k || !e || !out) return CB_INVALID;
    for (i=0;i<4;i++) {
        if (!isfinite(k[i]) || !isfinite(e[i])) return CB_INVALID;
        sum+=k[i]*e[i];
    }
    if (!isfinite(sum)) return CB_INVALID;
    *out=-sum; return CB_OK;
}
int cb_yaw_pd(double kp, double kr, double pr, double rr, double p, double r, double o[3]) {
    if (!o || !isfinite(kp) || !isfinite(kr) || kp<0 || kr<0 ||
        !isfinite(pr) || !isfinite(rr) || !isfinite(p) || !isfinite(r)) return CB_INVALID;
    o[0]=cb_wrap(pr-p); o[1]=rr-r; o[2]=kp*o[0]+kr*o[1];
    return isfinite(o[2]) ? CB_OK : CB_INVALID;
}
int cb_allocate(double sum, double diff, double limit, cb_allocation *o) {
    double left,right;
    if (!o || !isfinite(sum) || !isfinite(diff) || !isfinite(limit) || limit<=0 ||
        fabs(sum)>2.0*limit+1e-12) return CB_INVALID;
    memset(o,0,sizeof(*o));
    o->u_sum_requested=sum; o->u_sum_limited=sum; o->u_diff_requested=diff;
    o->u_diff_available=fmax(0.0,2.0*limit-fabs(sum));
    o->u_diff_used=clip(diff,-o->u_diff_available,o->u_diff_available);
    left=0.5*(sum+o->u_diff_used); right=0.5*(sum-o->u_diff_used);
    o->left=clip(left,-limit,limit); o->right=clip(right,-limit,limit);
    o->differential_clipped=fabs(diff-o->u_diff_used)>1e-12;
    o->guard_clipped=fabs(left-o->left)>1e-12 || fabs(right-o->right)>1e-12;
    return CB_OK;
}
int cb_brake_init(cb_brake *b, const cb_brake_config *c, const cb_reference *r) {
    const double *v; unsigned i;
    if (!b || !c || !r || !isfinite(c->dt) || !isfinite(c->deceleration) ||
        !isfinite(c->jerk) || !isfinite(c->handoff) || c->dt<=0 || c->deceleration<=0 ||
        c->jerk<=0 || c->handoff<=0) return CB_INVALID;
    /* Struct fields are checked explicitly; do not treat a struct as an array. */
    { double values[6]={r->p,r->v,r->a,r->theta,r->theta_dot,r->u_ff};
      v=values; for(i=0;i<6;i++) if(!isfinite(v[i])) return CB_INVALID; }
    memset(b,0,sizeof(*b)); b->config=*c; b->initial=*r; b->current=*r; return CB_OK;
}
int cb_brake_step(cb_brake *b, cb_reference *o) {
    double dt,target,da,a,v,p,t,s,s2,s3,d;
    if (!b || !o) return CB_INVALID;
    if (b->step==0) { b->step=1; *o=b->current; return CB_OK; }
    dt=b->config.dt; d=b->config.handoff;
    target=clip(-2.0*b->current.v,-b->config.deceleration,b->config.deceleration);
    da=b->config.jerk*dt;
    a=b->current.a+clip(target-b->current.a,-da,da);
    v=b->current.v+a*dt; p=b->current.p+0.5*(b->current.v+v)*dt;
    t=fmin(b->step*dt,d); s=t/d; s2=s*s; s3=s2*s;
    b->current.p=p; b->current.v=v; b->current.a=a;
    b->current.theta=(2*s3-3*s2+1)*b->initial.theta+(s3-2*s2+s)*d*b->initial.theta_dot;
    b->current.theta_dot=(6*s2-6*s)*b->initial.theta/d+(3*s2-4*s+1)*b->initial.theta_dot;
    b->current.u_ff=(1-3*s2+2*s3)*b->initial.u_ff;
    b->step++; b->settled=fabs(v)<=0.005 && fabs(a)<=0.015;
    *o=b->current; return CB_OK;
}
int cb_brake_handoff_done(const cb_brake *b) {
    return b && b->step>0 && (b->step-1)*b->config.dt>=b->config.handoff-1e-12;
}
const char *cb_result_name(int r) {
    static const char *const names[]={"ok","invalid","epoch","stale","capacity","sequence","underrun","state"};
    return r>=0 && r<8 ? names[r] : "unknown";
}
