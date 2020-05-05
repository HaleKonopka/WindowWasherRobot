#include "Kinematics.h"
#include <math.h>
#include <stdbool.h>

/*
 * The origin for the window is the upper-left corner, as it also
 * is for the robot. X axis is positive-right and Y axis is positive-down.
 */

/**
 * Perform forward kinematics on supplied cable lengths to determine
 * the pose of the robot.
 */
void kinematics_forward_position(robot_cables_t *cab, robot_pose_t *p, robot_orientation_sensor_t *sensor){
    float mounts_offset_x = KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH;
    float robot_offset_y = 0; //TODO: Add support for angled robot

    float lambda = (pow(mounts_offset_x, 2) + pow(robot_offset_y, 2) - pow(cab->top_right, 2) + pow(cab->top_left, 2)) /
        (2 * mounts_offset_x);

    float phi = (robot_offset_y / mounts_offset_x);

    float a = (1 + pow(phi, 2));
    float b = (-2 * lambda * phi);
    float c = (pow(lambda, 2) - pow(cab->top_left, 2));

    //printf("%.2f, %.2f, %.2f, %.2f\n", a, b, c, lambda);

    float y_pos = -b + pow(pow(b, 2) - 4*a*c, 0.5) / (2*a);

    float x_pos = 0;
    if (cab->top_left > y_pos) x_pos = pow(pow(cab->top_left,2) - pow(y_pos,2),0.5);

    p->x = x_pos;
    p->y = y_pos;
    if (sensor && kinematics_get_orientation_valid(sensor)) p->rad = kinematics_get_orientation(sensor);
    else p->rad = 0;
}

/**
 * Perform inverse kinematics on the supplied pose of the robot
 * to determine the needed cable lenghts to be at that posiiton.
 */
void kinematics_reverse_position(robot_pose_t *p, robot_cables_t *c){
    c->top_left = pow(pow(p->x, 2) + pow(p->y, 2), 0.5);
    c->top_right = pow(pow(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - p->x, 2) + pow(p->y, 2), 0.5);
    c->bot_left = pow(pow(KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - p->y, 2) + pow(p->x, 2), 0.5);
    c->bot_right = pow(pow(pow(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - p->x, 2), 2) +
        pow(KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - p->y, 2), 0.5);
}

void kinematics_reverse_velocity(robot_pose_t *pos, robot_pose_t *vel, robot_cables_t *c){
    c->top_left = pow(pow(pos->x, 2) + pow(pos->y, 2), -0.5) * (vel->x*pos->x + vel->y*pos->y);
    c->top_right = pow(pow(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - pos->x, 2) + pow(pos->y, 2), -0.5) * 
        (vel->y*pos->y - (KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - pos->x)*vel->x);

    float yv = vel->y + 0;
    c->bot_left = pow(pow(KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - pos->y, 2) + pow(pos->x, 2), -0.5) *
        (-yv*(KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - pos->y) + vel->x*pos->x);
    c->bot_right = pow(pow(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - pos->x, 2) +
        pow(KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - pos->y, 2), -0.5) *
        (-yv*(KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - pos->y) -
        vel->x*(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - pos->x));
}

bool kinematics_pose_valid(robot_pose_t *p){
    if (p->x < 0 || p->x > (KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH))  return false;
    if (p->y < 0 || p->y > (KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT)) return false;
    return true;
}

bool kinematics_cables_valid(robot_cables_t *c){
    if (c->top_left < 0 || c->top_left > KINE_MAX_CABLE_LEN)  return false;
    if (c->top_right < 0 || c->top_right > KINE_MAX_CABLE_LEN) return false;
    return true;
}

void kinematics_init_orientation_sensor(robot_orientation_sensor_t *sensor){
    sensor->bno = Adafruit_BNO055(55, 0x28);
      if(!sensor->bno.begin_no_reset())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
}

float kinematics_get_orientation(robot_orientation_sensor_t *sensor){
    sensors_event_t event;
    sensor->bno.getEvent(&event);
    float angle = event.orientation.y;
    //Serial.print("ANGLE: "); Serial.print(angle, 4); Serial.println();
    if (angle > 180) angle -= 360; // Shift range to -180 <-> 180
    return angle * 6.283 / 360; // Convert to radians
}

bool kinematics_get_orientation_valid(robot_orientation_sensor_t *sensor){
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    sensor->bno.getCalibration(&system, &gyro, &accel, &mag);

    if (system > 0) return true;
    return false;
}

void torque_calc(robot_torque_t *t, robot_pose_t *p){
	t->V1 = 1.015 + 0.007952*(p->x) - 0.002511*(p->y) - 0.001724*pow(p->x, 2) + 0.0001361*(p->x)*(p->y) + 2.305e-5*pow(p->x, 3) + -7.105e-7*pow(p->x, 2)*(p->y);
	t->V2 = 1-t->V1;
}
