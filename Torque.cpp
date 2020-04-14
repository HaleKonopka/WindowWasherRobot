#include "Torque.h"
#include <math.h>

void torque_calc(robot_torque_t *t, robot_pose_t *p){
	// Angle each cable makes with robot 
    float a = atan(p->y/(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - p->x));
	float b = atan(p->y/p->x);
	float c = atan((KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - p->y)/p->x);
	float d = atan((KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - p->y)/(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - p->x));

    // Rearranged equilibrium equations
	t->T1 = (BASE_T*cos(a)*sin(b)*cos(d - THETA) + BASE_T*cos(a)*sin(d)*cos(b - THETA) + 
		BASE_T*cos(b)*sin(a)*cos(d - THETA) + BASE_T*cos(d)*sin(a)*cos(b - THETA) - 
		BASE_T*sin(a - THETA)*cos(b)*sin(d) + BASE_T*sin(a - THETA)*cos(d)*sin(b) + 
	    9.81*MASS*cos(a)*sin(d)*cos(b - THETA) - 
		9.81*MASS*sin(a - THETA)*cos(b)*sin(d))/(cos(a)*sin(b)*sin(c)*cos(d - THETA) + 
		cos(a)*sin(b)*sin(d)*cos(c - THETA) + cos(b)*sin(a)*sin(c)*cos(d - THETA) + 
		cos(b)*sin(a)*sin(d)*cos(c - THETA) + cos(c)*sin(a)*sin(d)*cos(b - THETA) + 
		cos(d)*sin(a)*sin(c)*cos(b - THETA) + sin(a - THETA)*cos(c)*sin(b)*sin(d) + 
		sin(a - THETA)*cos(d)*sin(b)*sin(c)); 

	t->T2 = (BASE_T*cos(a)*sin(b)*cos(c - THETA) - BASE_T*cos(a)*sin(c)*cos(b - THETA)
	    + BASE_T*cos(b)*sin(a)*cos(c - THETA) + BASE_T*cos(c)*sin(a)*cos(b - THETA) + BASE_T*sin(a - THETA)*cos(b)*sin(c)
	    + BASE_T*sin(a - THETA)*cos(c)*sin(b) - 9.81*MASS*cos(a)*sin(c)*cos(b - THETA) 
	    + 9.81*MASS*sin(a - THETA)*cos(b)*sin(c))/(cos(a)*sin(b)*sin(c)*cos(d - THETA) 
	    + cos(a)*sin(b)*sin(d)*cos(c - THETA) + cos(b)*sin(a)*sin(c)*cos(d - THETA) 
	    + cos(b)*sin(a)*sin(d)*cos(c - THETA) + cos(c)*sin(a)*sin(d)*cos(b - THETA) 
	    + cos(d)*sin(a)*sin(c)*cos(b - THETA) + sin(a - THETA)*cos(c)*sin(b)*sin(d) 
	    + sin(a - THETA)*cos(d)*sin(b)*sin(c));

	t->T1*=0.0254; // convert to torque N.m
	t->T2*=0.0254; 
}