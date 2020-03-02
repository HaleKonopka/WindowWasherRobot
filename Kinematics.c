#include "Kinematics.h"
#include <math.h>
#include <stdbool.h>

#define KINE_WINDOW_WIDTH   60
#define KINE_WINDOW_HEIGHT  72
#define KINE_ROBOT_WIDTH    12
#define KINE_ROBOT_HEIGHT   12
#define KINE_MAX_CABLE_LEN  (powf(powf(KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT,2) + powf(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH,2), 0.5))

static const float kinematics_max_cable_len = KINE_MAX_CABLE_LEN;

/*
 * The origin for the window is the upper-left corner, as it also
 * is for the robot. X axis is positive-right and Y axis is positive-down.
 */

/**
 * Perform forward kinematics on supplied cable lengths to determine
 * the pose of the robot.
 */
void kinematics_forward(robot_cables_t *cab, robot_pose_t *p){
    float mounts_offset_x = KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH;
    float robot_offset_y = 0; //TODO: Add support for angled robot

    float lambda = (powf(mounts_offset_x, 2) + powf(robot_offset_y, 2) - powf(cab->top_right, 2) + powf(cab->top_left, 2)) /
        (2 * mounts_offset_x);

    float phi = (robot_offset_y / mounts_offset_x);

    float a = (1 + powf(phi, 2));
    float b = (-2 * lambda * phi);
    float c = (powf(lambda, 2) - powf(cab->top_left, 2));

    float y_pos = -b + powf(powf(b, 2) - 4*a*c, 0.5) / (2*a);
    float x_pos = powf(powf(cab->top_left,2) - powf(y_pos,2),0.5);

    p->x = x_pos;
    p->y = y_pos;
    p->rad = 0; //TODO: Add support for angled robot
}

/**
 * Perform inverse kinematics on the supplied pose of the robot
 * to determine the needed cable lenghts to be at that posiiton.
 */
void kinematics_reverse(robot_pose_t *p, robot_cables_t *c){
    c->top_left = powf(powf(p->x, 2) + powf(p->y, 2), 0.5);
    c->top_right = powf(powf(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - p->x, 2) + powf(p->y, 2), 0.5);
}

bool kinematics_pose_valid(robot_pose_t *p){
    if (p->x < 0 || p->x > (KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH))  return false;
    if (p->y < 0 || p->y > (KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT)) return false;
    return true;
}

bool kinematics_cables_valid(robot_cables_t *c){
    if (c->top_left < 0 || c->top_left > KINE_MAX_CABLE_LEN)  return false;
    if (c->top_right < 0 || c->top_right > KINE_MAX_CABLE_LEN) return false;
    return true;
}