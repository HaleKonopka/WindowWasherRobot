#include "PIDControl.h"

void pid_control_contruct(pid_control_t *pid, float p, float i, float d, float ff){
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->ff = ff;
    pid->accum = 0;
    pid->lastError = 0;
    pid->lastTimeMillisec = 0;
}

/**
 * Calculate the next PID control value
 * 
 * @param pid PID object
 * @param setpoint PID setpoint
 * @param process PID process variable
 * @param time Absolute time the function has been run at in milliseconds
 */
float pid_control_calculate(pid_control_t *pid, float setpoint, float process, long time){
    long deltaTime = time - pid->lastTimeMillisec;
    if (pid->lastTimeMillisec == 0) deltaTime = 0;
    pid->lastTimeMillisec = time;

    float error = setpoint - process;

    float deltaError = 0;
    if (deltaTime != 0) deltaError = (pid->lastError - error) * 1000.0 / (float) deltaTime;
    pid->lastError = error;

    float accumError = error * deltaTime / 1000.0;
    pid->accum += accumError;

    return pid->p * error + pid->i * pid->accum + pid->d * deltaError + pid->ff * setpoint;
}

void pid_control_reset(pid_control_t *pid){
    pid->accum = 0;
    pid->lastTimeMillisec = 0;
    pid->lastError = 0;
}