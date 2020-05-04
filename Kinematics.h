#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

typedef struct {
    float x;
    float y;
    float rad;
} robot_pose_t;

typedef struct {
    float top_right;
    float top_left;
    float bot_right;
    float bot_left;
} robot_cables_t;

typedef struct {
    Adafruit_BNO055 bno;
} robot_orientation_sensor_t;

void kinematics_init_orientation_sensor(robot_orientation_sensor_t *sensor);
float kinematics_get_orientation(robot_orientation_sensor_t *sensor);
bool kinematics_get_orientation_valid(robot_orientation_sensor_t *sensor);
void kinematics_forward_position(robot_cables_t *c, robot_pose_t *p, robot_orientation_sensor_t *sensor);
void kinematics_reverse_position(robot_pose_t *p, robot_cables_t *c);
void kinematics_reverse_velocity(robot_pose_t *pos, robot_pose_t *vel, robot_cables_t *c);

#endif