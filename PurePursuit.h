#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#define PUREPURSUIT_TRAJ_SIZE   4

typedef struct {
    float x;
    float y;
} pure_pursuit_path_point_t;

typedef struct {
    float lookahead;
    float speed;
    size_t lastLpSeg;
    float lastLpSegPct;
} pure_pursuit_t;

#endif