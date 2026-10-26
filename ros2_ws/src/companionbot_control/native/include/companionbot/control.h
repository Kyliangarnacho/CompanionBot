#ifndef COMPANIONBOT_CONTROL_H
#define COMPANIONBOT_CONTROL_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* SI units, fixed-size state, no allocator/OS/ROS/HAL dependency. Double is
 * deliberate: first establish parity with the frozen Python baseline. */
#define CB_DT 0.002
#define CB_BLOCK_SAMPLES 25u
typedef enum { CB_OK=0, CB_INVALID=1, CB_EPOCH=2, CB_STALE=3,
    CB_CAPACITY=4, CB_SEQUENCE=5, CB_UNDERRUN=6, CB_STATE=7 } cb_result;
typedef struct { double p, v, a, theta, theta_dot, u_ff; } cb_reference;
typedef struct { double dt, deceleration, jerk, handoff; } cb_brake_config;
typedef struct {
    cb_brake_config config;
    cb_reference initial, current;
    uint32_t step;
    int settled;
} cb_brake;
int cb_brake_init(cb_brake *, const cb_brake_config *, const cb_reference *);
int cb_brake_step(cb_brake *, cb_reference *);
int cb_brake_handoff_done(const cb_brake *);

typedef struct {
    double u_sum_requested, u_sum_limited, u_diff_requested, u_diff_available;
    double u_diff_used, left, right;
    int sum_clipped, differential_clipped, guard_clipped;
} cb_allocation;
int cb_allocate(double u_sum, double u_diff, double limit, cb_allocation *);
int cb_feedback(const double gain[4], const double error[4], double *);
int cb_yaw_pd(double kp, double kr, double psi_ref, double r_ref,
              double psi, double rate, double out[3]);
double cb_wrap(double angle);

typedef struct {
    double dt, pitch_tau, pll_bandwidth, counts_per_rev, wheel_radius;
    double acceleration_tau, acceleration_scale;
    int left_sign, right_sign, snap_to_zero, acceleration_compensation;
} cb_estimator_config;
typedef struct {
    double accel[3], gyro[3], age;
    int64_t counts[2];
    int extrapolate;
} cb_sensor;
typedef struct {
    double theta, theta_dot, theta_acc, theta_measurement, age, extrapolation_age;
    double position, velocity, pll_position[2], pll_velocity[2];
    double wheel_angle[2], wheel_velocity[2];
    double acceleration_used, acceleration_next, raw_theta_acc, correction_gain;
    double compensated_accel[3];
} cb_estimate;
typedef struct {
    cb_estimator_config config;
    cb_estimate output;
    double previous_pll_position[2], previous_theta, previous_velocity;
    int initialized;
} cb_estimator;
int cb_estimator_init(cb_estimator *, const cb_estimator_config *);
int cb_estimator_reset(cb_estimator *, const cb_sensor *, cb_estimate *);
int cb_estimator_update(cb_estimator *, const cb_sensor *, cb_estimate *);
size_t cb_estimator_size(void);
typedef struct { double psi, rate, sample_time; int initialized; } cb_yaw_estimator;
int cb_yaw_estimate(cb_yaw_estimator *, double rate, double sample_time,
                    double age, int valid_new, int extrapolate, int reset,
                    double out[3]);

typedef struct {
    uint32_t epoch, sequence, start_tick, count;
    double generated_time, dt;
    float yaw;
    float samples[CB_BLOCK_SAMPLES][6];
} cb_block;
int cb_block_decode(const uint8_t *, size_t, cb_block *);
typedef enum { CB_NORMAL=0, CB_BRAKING=1, CB_LONGITUDINAL_STOP=2,
               CB_SAFE_HOLD=3 } cb_safety_state;
typedef enum { CB_HARD=0, CB_LONGITUDINAL=1 } cb_authority;
typedef enum { CB_REASON_NONE=0, CB_REASON_PI=1, CB_REASON_HEARTBEAT=2,
               CB_REASON_STARVATION=3, CB_REASON_UNDERRUN=4 } cb_reason;
typedef struct {
    cb_brake_config brake;
    double heartbeat_timeout, starvation_margin, nominal_pitch;
    double yaw_timeout, yaw_decay;
} cb_mcu_config;
typedef struct {
    cb_reference reference;
    double yaw_rate;
    int64_t block_sequence, sample_index;
    uint32_t epoch, safety_state, reason, pi_lost;
} cb_mcu_output;
typedef struct {
    cb_mcu_config config;
    cb_block active, next;
    cb_brake brake;
    cb_mcu_output output;
    uint32_t epoch, last_ref_sequence, heartbeat_sequence, safety_sequence;
    uint32_t resume_sequence, yaw_sequence, yaw_receive_tick, hard_rearm_tick;
    uint32_t heartbeat_tick, resume_tick, pending_resume_epoch;
    uint32_t rejected[8], sequence_gaps, underruns;
    int has_active, has_next, paused, has_ref_sequence, has_heartbeat;
    int has_safety_sequence, has_resume_sequence, has_yaw, hard_disarmed;
    int pending_safety, pending_reason, pending_resume, resume_armed;
    int authority, pending_authority, state, reason, pi_lost, has_last;
    double local_velocity, local_pitch, local_pitch_rate, yaw_target, yaw_output;
} cb_mcu;
int cb_mcu_init(cb_mcu *, const cb_mcu_config *, uint32_t epoch);
int cb_mcu_reference(cb_mcu *, const uint8_t *, size_t, uint32_t tick);
int cb_mcu_heartbeat(cb_mcu *, uint32_t sequence, uint32_t tick);
int cb_mcu_yaw(cb_mcu *, uint32_t sequence, double target, uint32_t tick);
/* CB_OK HARD is an immediate reference/resume revocation barrier in every
 * safety state. Duplicate/stale sequences and authority downgrades reject
 * without advancing the accepted safety sequence. Braking is not restarted.
 * A new HARD sequence invalidates the old epoch even if already in SAFE_HOLD. */
int cb_mcu_safety(cb_mcu *, uint32_t sequence, int authority, int reason, uint32_t tick);
int cb_mcu_resume(cb_mcu *, uint32_t sequence, uint32_t epoch, int authority, uint32_t tick);
int cb_mcu_step(cb_mcu *, uint32_t tick, double velocity, double pitch,
                double pitch_rate, cb_mcu_output *);
int cb_mcu_peek(const cb_mcu *, cb_reference *);
size_t cb_mcu_size(void);
const char *cb_result_name(int result);
#ifdef __cplusplus
}
#endif
#endif
