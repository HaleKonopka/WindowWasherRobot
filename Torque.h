#ifndef TORQUE_UTIL
#define TORQUE_UTIL

#include "Kinematics.h"

#define MASS 4 // kg
#define THETA 0.785398 // Angle from center to spool wrt x axis
#define BASE_T 20 // Minimum tension in bottom motors
#define g 9.81

typedef struct {
    float T1;
    float T2;
    float V1;
    float V2;
} robot_torque_t;

void torque_calc(robot_torque_t *t, robot_pose_t *p);

#endif