#include "PurePursuit.h"

static pure_pursuit_path_point_t pure_pursuit_traj[PUREPURSUIT_TRAJ_SIZE];

void pure_pursuit_initialize(pure_pursuit_t *c, float lookahead, float speed){
    c->lookahead = lookahead;
    c->speed = speed;
    c->lastLpSeg = 0;
    c->lastLpSegPct = 0;
}

void pure_pursuit_init_trajectory(){
    pure_pursuit_traj[0].x = 0;
    pure_pursuit_traj[0].y = 0;
    pure_pursuit_traj[1].x = 0;
    pure_pursuit_traj[1].y = 12;
    pure_pursuit_traj[2].x = 12;
    pure_pursuit_traj[2].y = 12;
    pure_pursuit_traj[3].x = 12;
    pure_pursuit_traj[3].y = 0;
}

void pure_pursuit_calculate(pure_pursuit_t *c, robot_cables_t *cables_pos, robot_cables_t *cables_vel){
    robot_pose_t robot_pos;
    kinematics_forward_position(&cables_pos, &robot_pos);

    pure_pursuit_path_point_t lp;
    pure_pursuit_find_lookahead(lp, c, robot_pos);

    float dx = lp.x - robot_pos.x;
    float dy = lp.y - robot_pos.y;
    float heading = atan2f(dy, dx);

    robot_pose_t robot_vel;
    robot_vel.x = cosf(heading) * c->speed;
    robot_vel.y = sinf(heading) * c->speed;

    kinematics_reverse_velocity(&robot_pos, &robot_vel, cables_vel);
}

void pure_pursuit_find_lookahead(pure_pursuit_path_point_t *lp, pure_pursuit_t *c, robot_pose_t *robot_pos){
    pure_pursuit_segment_point(lp, c->lastLpSeg, c->lastLpSegPct);
    float pctStep = 0.05;

    while (pure_pursuit_distance(robot_pos, lp) < c->lookahead) {
        if (c->lastLpSegPct < (1 - (pctStep))) {
            c->lastLpSegPct += pctStep;
        } else {
            c->lastLpSeg++;
            c->lastLpSegPct = 0;
        }

        pure_pursuit_segment_point(lp, c->lastLpSeg, c->lastLpSegPct);
    }
}

float pure_pursuit_distance(robot_pose_t *robot_pos, pure_pursuit_path_point_t *lp){
    return pow(pow((lp->x - robot_pos->x), 2) + pow((lp->y - robot_pos->y), 2), 0.5);
}

void pure_pursuit_segment_point(pure_pursuit_path_point_t *point, unsigned int segment, float pct){
    unsigned int segi = segment % PUREPURSUIT_TRAJ_SIZE; // Loop trajectory for now

    pure_pursuit_path_point_t start = pure_pursuit_traj[segi];
    pure_pursuit_path_point_t end = pure_pursuit_traj[(segi + 1) % PUREPURSUIT_TRAJ_SIZE];

    float dx = end.x - start.x;
    float dy = end.y - start.y;

    point->x = start.x + dx * pct;
    point->y = start.y + dy * pct;
}