#include "Motor.h"

/**
 * Initialize a new motor
 */
void motor_initialize(motor_t *m, Encoder *enc, pid_control_t *pid, uint8_t en_pin, uint8_t enb_pin,
    uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t diag_pin){
    m->en = en_pin;
    m->enb = enb_pin;
    m->pwm_a = pwm_a_pin;
    m->pwm_b = pwm_b_pin;
    m->diag = diag_pin;
    m->enc = enc;
    m->lastCount = 0;
    m->lastTimeMillis = millis();
    m->pid = pid;

    pinMode(m->en,  OUTPUT);
    digitalWrite(m->en, HIGH);
    pinMode(m->enb, OUTPUT);
    digitalWrite(m->enb, LOW);
    pinMode(m->diag, INPUT);
    pinMode(m->pwm_a, OUTPUT);
    pinMode(m->pwm_b, OUTPUT);

    motor_stop(m, true);

    m->vel_index = 0;
    for(size_t i = 0; i < MOTOR_SIZE_AVG_VEL; i++){
        m->vel_avg[i] = 0;
    }

    m->enc->write(0);
}

/**
 * Set the velocity of a motor using the defined PID controller
 */
void motor_set_velocity(motor_t *m, float vel){
    m->vel_cmd = vel;
}

/**
 * Updates the pid command and sets the motor to it
 */
void motor_update_pid(motor_t *m){
    float pid_cmd = pid_control_calculate(m->pid, m->vel_cmd, motor_get_velocity(m), millis());

    motor_run(m, pid_cmd);

    char p[100];
    sprintf(p, "Motor cmd: %s", String(pid_cmd, 2).c_str());
    Serial.println(p);
}

/**
 * Get the current linear velocity of the motor spool
 * 
 * @return velocity in inches/sec
 */
float motor_get_velocity(motor_t *m){
    unsigned long timeMillis = millis();

    long thisCount = m->enc->read();
    long deltaTicks = thisCount - m->lastCount;
    long deltaTime = (timeMillis - m->lastTimeMillis);
    if (deltaTime == 0){
        return m->cur_vel;
    }

    float vel =  deltaTicks * 1000.0 / (timeMillis - m->lastTimeMillis);
    m->lastCount = thisCount;

    vel = vel * DIST_PER_REV / ENCODER_TICKS_PER_REV;
    m->vel_avg[m->vel_index] = vel;
    m->vel_index = (m->vel_index + 1) % MOTOR_SIZE_AVG_VEL;

    float ret;
    for (size_t i = 0; i < MOTOR_SIZE_AVG_VEL; i++){
        ret += m->vel_avg[i];
    }
    ret = ret / (float) MOTOR_SIZE_AVG_VEL;

    m->cur_vel = ret;

    m->lastTimeMillis = timeMillis;

    return m->cur_vel;
}

float motor_get_position(motor_t *m){
    return m->enc->read() * DIST_PER_REV / ENCODER_TICKS_PER_REV;
}

/**
 * Determine if the motor driver is in error
 */
bool motor_has_error(motor_t *m){
    return digitalRead(m->diag);
}

/**
 * Stop the motor, optionally braking or coasting
 */
void motor_stop(motor_t *m, bool brake){
    if (brake) motor_run(m, 0);
    else motor_coast(m);
}

/**
 * Set the motor to coast
 */
void motor_coast(motor_t *m){
    digitalWrite(m->en, LOW);
    digitalWrite(m->enb, LOW);
}

/** 
 * Run the motor at a specified percentage power
 * 
 * @param pct Value from -1 to +1, where -1 is full power
 *            reverse and +1 is full power forward
 */
void motor_run(motor_t *m, float pct){
    float value = pct * 255;
    if (pct > 255.0) pct = 255;
    if (pct < -255.0) pct = -255;
    
    digitalWrite(m->en, HIGH);
    digitalWrite(m->enb, LOW);

    if (value > 0) {
        analogWrite(m->pwm_a, (uint8_t) value);
        analogWrite(m->pwm_b, LOW);
        //Serial.println(String((uint8_t) value).c_str());
    } else if (value < 0){
        analogWrite(m->pwm_b, (uint8_t) -value);
        analogWrite(m->pwm_a, LOW);
    } else {
        analogWrite(m->pwm_a, LOW);
        analogWrite(m->pwm_b, LOW);
    }
}