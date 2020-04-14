#ifndef TORQUE_UTIL
#define TORQUE_UTIL

#include "Kinematics.h"

#define MASS 4.5 // no idea how heavy this thing is
#define THETA 0.785398
#define BASE_T 10 // arbitrary value right now

typedef struct {
    float T1;
    float T2;
    float V1;
    float V2;
} robot_torque_t;

void torque_calc(robot_torque_t *t, robot_pose_t *p);

#endif