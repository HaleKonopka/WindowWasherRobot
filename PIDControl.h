#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    float accum;
    float lastError;
    float p;
    float i;
    float d;
    float ff;
    long lastTimeMillisec;
} pid_control_t;

void pid_control_contruct(pid_control_t *pid, float p, float i, float d, float ff);
float pid_control_calculate(pid_control_t *pid, float setpoint, float process, long time);
void pid_control_reset(pid_control_t *pid);

#ifdef __cplusplus
}
#endif

#endif