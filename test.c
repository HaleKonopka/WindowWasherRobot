#include "Kinematics.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

int main(){
    robot_pose_t p;
    robot_cables_t c;

    p.x = 2;
    p.y = 2;
    p.rad = 0;
    kinematics_reverse(&p, &c);
    printf("Revese: Pose (%.2f, %.2f) Cables (%.2f, %.2f)\n", p.x, p.y, c.top_left, c.top_right);

    c.top_left = 2.83;
    c.top_right = 46.04;
    kinematics_forward(&c, &p);
    printf("Forward: Pose (%.2f, %.2f) Cables (%.2f, %.2f)\n", p.x, p.y, c.top_left, c.top_right);

    float ep = 0.1;
    for (float x = 0; x < 48; x += 0.25){
        for (float y = 0; y < 60; y += 0.25){
            p.x = x;
            p.y = y;
            kinematics_reverse(&p, &c);
            kinematics_forward(&c, &p);
            if (x + ep < p.x || x - ep > p.x || y + ep < p.y || y - ep > p.y){
                printf("Failed: Pose (%.2f, %.2f) Cables (%.2f, %.2f)\n", p.x, p.y, c.top_left, c.top_right);
            }
            //printf("Original Pose (%.2f, %.2f) Double Pose (%.5f, %.5f) Cables (%.2f, %.2f)\n", x, y, p.x, p.y, c.top_left, c.top_right);
        }
    }
}