#ifndef MOTOR_H
#define MOTOR_H

#include <Encoder.h>
#include "PIDControl.h"

#define ENCODER_TICKS_PER_REV   (64 * 50) // 64 ticks per rev plus reduction
#define DIST_PER_REV            3.14    // 1 inch diameter spool
#define MOTOR_SIZE_AVG_VEL      16

typedef struct {
    uint8_t enb;
    uint8_t en;
    uint8_t pwm_a;
    uint8_t pwm_b;
    uint8_t diag;
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
} motor_t;

void motor_initialize(motor_t *m, Encoder *enc, pid_control_t *pid, uint8_t en_pin, 
    uint8_t enb_pin, uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t diag_pin, bool rev_enc, bool rev_motor);
bool motor_has_error(motor_t *m);
void motor_stop(motor_t *m, bool brake);
void motor_coast(motor_t *m);
void motor_run(motor_t *m, float pct);
void motor_set_velocity(motor_t *m, float vel);
float motor_get_velocity(motor_t *m);
float motor_get_position(motor_t *m);
void motor_update_pid(motor_t *m);

#endif