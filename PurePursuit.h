#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "Kinematics.h"
#include <Encoder.h>

#define PUREPURSUIT_TRAJ_SIZE   8

typedef struct {
    float x;
    float y;
} pure_pursuit_path_point_t;

typedef struct {
    float lookahead;
    float speed;
    unsigned int lastLpSeg;
    float lastLpSegPct;
    robot_pose_t *startingPos;
} pure_pursuit_t;

void pure_pursuit_segment_point(pure_pursuit_path_point_t *point, robot_pose_t *offset, unsigned int segment, float pct);
float pure_pursuit_distance(robot_pose_t *robot_pos, pure_pursuit_path_point_t *lp);
void pure_pursuit_initialize(pure_pursuit_t *c, robot_pose_t *start, float lookahead, float speed);
void pure_pursuit_init_trajectory();
void pure_pursuit_calculate(pure_pursuit_t *c, robot_cables_t *cables_pos, robot_cables_t *cables_vel);
void pure_pursuit_find_lookahead(pure_pursuit_path_point_t *lp, pure_pursuit_t *c, robot_pose_t *robot_pos);

#endif