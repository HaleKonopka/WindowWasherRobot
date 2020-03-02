#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "Kinematics.h"

#define PUREPURSUIT_TRAJ_SIZE   4

typedef struct {
    float x;
    float y;
} pure_pursuit_path_point_t;

typedef struct {
    float lookahead;
    float speed;
    unsigned int lastLpSeg;
    float lastLpSegPct;
} pure_pursuit_t;

void pure_pursuit_segment_point(pure_pursuit_path_point_t *point, unsigned int segment, float pct);
float pure_pursuit_distance(robot_pose_t *robot_pos, pure_pursuit_path_point_t *lp);

#endif