#include "PurePursuit.h"
#include <math.h>

static pure_pursuit_path_point_t pure_pursuit_traj[PUREPURSUIT_TRAJ_SIZE];

void pure_pursuit_initialize(pure_pursuit_t *c, robot_pose_t *start, float lookahead, float speed){
    c->lookahead = lookahead;
    c->speed = speed;
    c->lastLpSeg = 0;
    c->lastLpSegPct = 0;
    c->startingPos = start;

    pure_pursuit_init_trajectory();
}

void pure_pursuit_init_trajectory(){
    pure_pursuit_traj[0].x = 0;
    pure_pursuit_traj[0].y = 0;
    pure_pursuit_traj[1].x = 0;
    pure_pursuit_traj[1].y = -36;
    pure_pursuit_traj[2].x = 10;
    pure_pursuit_traj[2].y = -36;
    pure_pursuit_traj[3].x = 10;
    pure_pursuit_traj[3].y = 0;
    pure_pursuit_traj[4].x = 20;
    pure_pursuit_traj[4].y = 0;
    pure_pursuit_traj[5].x = 20;
    pure_pursuit_traj[5].y = -20;
    pure_pursuit_traj[6].x = 30;
    pure_pursuit_traj[6].y = -20;
    pure_pursuit_traj[7].x = 30;
    pure_pursuit_traj[7].y = 0;
}

void pure_pursuit_calculate(pure_pursuit_t *c, robot_cables_t *cables_pos, robot_cables_t *cables_vel, robot_orientation_sensor_t *sensor){
    robot_pose_t robot_pos;
    char p[100];
    sprintf(p, "Cable pose: (%s, %s)", String(cables_pos->top_left, 2).c_str(), String(cables_pos->top_right, 2).c_str());
    //Serial.println(p);
    kinematics_forward_position(cables_pos, &robot_pos, sensor);

    sprintf(p, "Robot pose: (%s, %s, %s)", String(robot_pos.x, 2).c_str(), String(robot_pos.y, 2).c_str(), String(robot_pos.rad, 3).c_str());
    Serial.println(p);

    pure_pursuit_path_point_t lp;
    pure_pursuit_find_lookahead(&lp, c, &robot_pos);

    float dx = lp.x - robot_pos.x;
    float dy = lp.y - robot_pos.y;
    float heading = atan2f(dy, dx);

    robot_pose_t robot_vel;
    robot_vel.x = cosf(heading) * c->speed;
    robot_vel.y = sinf(heading) * c->speed;

    kinematics_reverse_velocity(&robot_pos, &robot_vel, cables_vel);
}

void pure_pursuit_find_lookahead(pure_pursuit_path_point_t *lp, pure_pursuit_t *c, robot_pose_t *robot_pos){
    pure_pursuit_segment_point(lp, c->startingPos, c->lastLpSeg, c->lastLpSegPct);
    float pctStep = 0.05;

    while (pure_pursuit_distance(robot_pos, lp) < c->lookahead) {
        if (c->lastLpSegPct < (1 - (pctStep))) {
            c->lastLpSegPct += pctStep;
        } else {
            c->lastLpSeg++;
            c->lastLpSegPct = 0;
        }

        pure_pursuit_segment_point(lp, c->startingPos, c->lastLpSeg, c->lastLpSegPct);
    }
}

float pure_pursuit_distance(robot_pose_t *robot_pos, pure_pursuit_path_point_t *lp){
    return pow(pow((lp->x - robot_pos->x), 2) + pow((lp->y - robot_pos->y), 2), 0.5);
}

void pure_pursuit_segment_point(pure_pursuit_path_point_t *point, robot_pose_t *offset, unsigned int segment, float pct){
    unsigned int segi = segment % PUREPURSUIT_TRAJ_SIZE; // Loop trajectory for now

    pure_pursuit_path_point_t start = pure_pursuit_traj[segi];
    pure_pursuit_path_point_t end = pure_pursuit_traj[(segi + 1) % PUREPURSUIT_TRAJ_SIZE];

    float dx = end.x - start.x;
    float dy = end.y - start.y;

    point->x = start.x + dx * pct;
    point->y = start.y + dy * pct;

    if (offset){
        point->x += offset->x;
        point->y += offset->y;
    }
}
