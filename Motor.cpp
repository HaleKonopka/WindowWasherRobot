#include "Motor.h"

/**
 * Initialize a new motor
 */
void motor_initialize(motor_t *m, Encoder *enc, pid_control_t *pid, uint8_t en_pin, uint8_t enb_pin,
    uint8_t pwm_a_pin, uint8_t pwm_b_pin, uint8_t diag_pin, uint8_t cs_pin, bool rev_enc, bool rev_motor){
    m->en = en_pin;
    m->enb = enb_pin;
    m->pwm_a = pwm_a_pin;
    m->pwm_b = pwm_b_pin;
    m->diag = diag_pin;
    m->cs = cs_pin;
    m->enc = enc;
    m->lastCount = 0;
    m->lastTimeMillis = millis();
    m->pid = pid;
    m->rev_enc = rev_enc;
    m->rev_motor = rev_motor;
    m->max_pwr = 1.0;

    pinMode(m->en,  OUTPUT);
    pinMode(m->enb, OUTPUT);
    pinMode(m->diag, INPUT_PULLUP);
    pinMode(m->pwm_a, OUTPUT);
    pinMode(m->pwm_b, OUTPUT);
    pinMode(m->cs, INPUT);

    motor_stop(m, true);

    m->vel_index = 0;
    for(size_t i = 0; i < MOTOR_SIZE_AVG_VEL; i++){
        m->vel_avg[i] = 0;
    }

    if (m->enc) m->enc->write(0);
}

float motor_get_current(motor_t *m){
    int adc_val = analogRead(m->cs);
    float current = ((float) adc_val) * 5.0 / 1024.0 / 0.5;
    return current;
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
}

int32_t motor_read_encoder(motor_t *m){
    return m->enc->read() * (m->rev_enc ? -1 : 1) * (m->rev_motor ? -1 : 1);
}

/**
 * Get the current linear velocity of the motor spool
 * 
 * @return velocity in inches/sec
 */
float motor_get_velocity(motor_t *m){
    unsigned long timeMillis = millis();

    long thisCount = motor_read_encoder(m);
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

    m->cur_vel = 0;
    for (size_t i = 0; i < MOTOR_SIZE_AVG_VEL; i++){
        m->cur_vel += m->vel_avg[i];
    }
    m->cur_vel /= (float) MOTOR_SIZE_AVG_VEL;

    m->lastTimeMillis = timeMillis;

    return m->cur_vel;
}

float motor_get_position(motor_t *m){
    return motor_read_encoder(m) * DIST_PER_REV / ENCODER_TICKS_PER_REV;
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
    pct *= (m->rev_motor ? -1 : 1);

    if (pct > m->max_pwr){
        pct = m->max_pwr;
    } else if (pct < -m->max_pwr){
        pct = -m->max_pwr;
    }

    float value = pct * 255.0;
    if (value > 255.0) value = 255.0;
    if (value < -255.0) value = -255.0;
    
    digitalWrite(m->en, HIGH);
    digitalWrite(m->enb, LOW);

    if (value > 0) {
        int pwm = value;
        pwm = pwm & 0xFF;
        analogWrite(m->pwm_b, pwm);
        analogWrite(m->pwm_a, LOW);
    } else if (value < 0){
        value *= -1;
        int pwm = value;
        pwm = pwm & 0xFF;
        analogWrite(m->pwm_a, pwm);
        analogWrite(m->pwm_b, LOW);
    } else {
        analogWrite(m->pwm_a, LOW);
        analogWrite(m->pwm_b, LOW);
    }
}

/** 
 * Run the motor at a specified percentage power
 * 
 * @param pct Value from -1 to +1, where -1 is full power
 *            reverse and +1 is full power forward
 */
void motor_run_coast(motor_t *m, float pct){
    pct *= (m->rev_motor ? -1 : 1);

    if (pct > m->max_pwr){
        pct = m->max_pwr;
    } else if (pct < -m->max_pwr){
        pct = -m->max_pwr;
    }

    float value = pct * 255.0;
    if (value > 255.0) value = 255.0;
    if (value < -255.0) value = -255.0;
    
    digitalWrite(m->enb, LOW);

    if (value > 0) {
        int pwm = value;
        pwm = pwm & 0xFF;
        analogWrite(m->en, pwm);
        digitalWrite(m->pwm_b, HIGH);
        digitalWrite(m->pwm_a, LOW);
    } else if (value < 0){
        value *= -1;
        int pwm = value;
        pwm = pwm & 0xFF;
        analogWrite(m->en, pwm);
        digitalWrite(m->pwm_b, LOW);
        digitalWrite(m->pwm_a, HIGH);
    } else {
        analogWrite(m->en, LOW);
        analogWrite(m->pwm_a, LOW);
        analogWrite(m->pwm_b, LOW);
    }
}