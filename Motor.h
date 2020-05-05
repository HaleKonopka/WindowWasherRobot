#ifndef MOTOR_H
#define MOTOR_H

#include <Encoder.h>
#include "PIDControl.h"

#define ENCODER_TICKS_PER_REV   (64 * 50) // 64 ticks per rev plus reduction
#define DIST_PER_REV            3.14    // 1 inch diameter spool
#define SPOOL_RAD               0.0127  // spool radius in meters
#define MOTOR_SIZE_AVG_VEL      16

// Motor params include 50:1 gearbox
#define MOTOR_KT                0.07419 // 0.07419
#define MOTOR_KREVS             7.6 // 7.2
#define MOTOR_RES               7.742
#define MOTOR_VBIAS             0


typedef struct {
    uint8_t enb;
    uint8_t en;
    uint8_t pwm_a;
    uint8_t pwm_b;
    uint8_t diag;
    uint8_t cs;
    unsigned long lastTimeMillis;
    long lastCount;
    Encoder *enc;
    float vel_avg[MOTOR_SIZE_AVG_VEL];
    size_t vel_index;
    float vel_cmd;
    float cur_vel;
    pid_control_t *pid;
    bool rev_enc;
    bool rev_motor;
    float max_pwr;
} motor_t;

void motor_initialize(motor_t *m, Encoder *enc, pid_control_t *pid, uint8_t en_pin, 
    uint8_t enb_pin, uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t diag_pin, uint8_t cs_pin, bool rev_enc, bool rev_motor);
float motor_get_current(motor_t *m);
bool motor_has_error(motor_t *m);
void motor_stop(motor_t *m, bool brake);
void motor_coast(motor_t *m);
void motor_run(motor_t *m, float pct);
void motor_run_coast(motor_t *m, float pct);
void motor_set_velocity(motor_t *m, float vel);
float motor_get_velocity(motor_t *m);
float motor_get_position(motor_t *m);
void motor_update_pid(motor_t *m);
void motor_set_torque(motor_t *m, float N, float speed);

#endif