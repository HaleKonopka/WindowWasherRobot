#include "PIDControl.h"

void pid_contruct(pid_control_t *pid, float p, float i, float d, float ff){
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->ff = ff;
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
    pid->lastTimeMillisec = time;

    float error = setpoint - process;

    float deltaError = (pid->lastError - error) * 1000.0 / (float) deltaTime;
    pid->lastError = error;

    float accumError = error * deltaTime / 1000.0;
    pid->accum += accumError;

    return pid->p * error + pid->i * pid->accum + pid->d * deltaError + pid->ff;
}