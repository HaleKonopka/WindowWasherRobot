#ifndef KINEMATICS_H
#define KINEMATICS_H

typedef struct {
    float x;
    float y;
    float rad;
} robot_pose_t;

typedef struct {
    float top_right;
    float top_left;
} robot_cables_t;

void kinematics_forward_position(robot_cables_t *c, robot_pose_t *p);
void kinematics_reverse_position(robot_pose_t *p, robot_cables_t *c);
void kinematics_reverse_velocity(robot_pose_t *pos, robot_pose_t *vel, robot_cables_t *c);

#endif