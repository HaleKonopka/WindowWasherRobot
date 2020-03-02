#include "Kinematics.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

int test(){
    robot_pose_t p;
    robot_cables_t c;

    p.x = 2;
    p.y = 2;
    p.rad = 0;
    kinematics_reverse_position(&p, &c);
    printf("Revese: Pose (%.2f, %.2f) Cables (%.2f, %.2f)\n", p.x, p.y, c.top_left, c.top_right);

    p.x = 24;
    p.y = 10;
    c.top_left = 103.81;
    c.top_right = 103.81;
    kinematics_forward_position(&c, &p);
    printf("Forward: Pose (%.2f, %.2f) Cables (%.2f, %.2f)\n", p.x, p.y, c.top_left, c.top_right);

    float ep = 0.1;
    for (float x = 0; x < 48; x += 0.25){
        for (float y = 0; y < 60; y += 0.25){
            p.x = x;
            p.y = y;
            kinematics_reverse_position(&p, &c);
            kinematics_forward_position(&c, &p);
            if (x + ep < p.x || x - ep > p.x || y + ep < p.y || y - ep > p.y){
                printf("Failed: Pose (%.2f, %.2f) Cables (%.2f, %.2f)\n", p.x, p.y, c.top_left, c.top_right);
            }
            //printf("Original Pose (%.2f, %.2f) Double Pose (%.5f, %.5f) Cables (%.2f, %.2f)\n", x, y, p.x, p.y, c.top_left, c.top_right);
        }
    }

    robot_pose_t pvel;
    robot_cables_t cvel;
    pvel.x = 0;
    pvel.y = 1;
    p.x = 24;
    p.y = 100;
    kinematics_reverse_position(&p, &c);
    kinematics_reverse_velocity(&p, &pvel, &cvel);
    printf("Revese: Pose Pos (%.2f, %.2f) Pose Vel (%.2f, %.2f) Cables Pos (%.2f, %.2f) Cables Vel (%.2f, %.2f)\n",
        p.x, p.y, pvel.x, pvel.y, c.top_left, c.top_right, cvel.top_left, cvel.top_right);
}