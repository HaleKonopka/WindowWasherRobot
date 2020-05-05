#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define KINE_WINDOW_WIDTH   57
#define KINE_WINDOW_HEIGHT  92
#define KINE_ROBOT_WIDTH    6
#define KINE_ROBOT_HEIGHT   6
#define KINE_MAX_CABLE_LEN  (pow(pow(KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT,2) + pow(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH,2), 0.5))

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

typedef struct {
	// Motor PWM percent
    float V1;
    float V2;
} robot_torque_t;


void kinematics_init_orientation_sensor(robot_orientation_sensor_t *sensor);
float kinematics_get_orientation(robot_orientation_sensor_t *sensor);
bool kinematics_get_orientation_valid(robot_orientation_sensor_t *sensor);
void kinematics_forward_position(robot_cables_t *c, robot_pose_t *p, robot_orientation_sensor_t *sensor);
void kinematics_reverse_position(robot_pose_t *p, robot_cables_t *c);
void kinematics_reverse_velocity(robot_pose_t *pos, robot_pose_t *vel, robot_cables_t *c);
void torque_calc(robot_torque_t *t, robot_pose_t *p);

#endif