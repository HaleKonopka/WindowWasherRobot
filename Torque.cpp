#include "Torque.h"
#include <math.h>

void torque_calc(robot_torque_t *t, robot_pose_t *p){
	// Angle each cable makes with robot 
    float a = atan(p->y/(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - p->x));
	float b = atan(p->y/p->x);
	float c = atan((KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - p->y)/p->x);
	float d = atan((KINE_WINDOW_HEIGHT - KINE_ROBOT_HEIGHT - p->y)/(KINE_WINDOW_WIDTH - KINE_ROBOT_WIDTH - p->x));
	//std::cout<<c*180/3.1415<<" "<<d*180/3.1415<<std::endl;

    // Rearranged equilibrium equations -> enforcing positive tension

    // Fix tension in line 3
	float T31 = BASE_T;
	float T41 = (BASE_T*cos(a)*sin(b)*sin(THETA - c) - BASE_T*cos(a)*sin(c)*sin(THETA - b) 
		+ BASE_T*cos(b)*sin(a)*sin(THETA - c) + BASE_T*cos(b)*sin(c)*sin(THETA - a) 
		+ BASE_T*cos(c)*sin(a)*sin(THETA - b) + BASE_T*cos(c)*sin(b)*sin(THETA - a) 
		- MASS*g*cos(a)*sin(THETA - b) + MASS*g*cos(b)*sin(THETA - a))/(cos(a)*sin(b)*sin(THETA - d) 
		+ cos(a)*sin(d)*sin(THETA - b) + cos(b)*sin(a)*sin(THETA - d) - cos(b)*sin(d)*sin(THETA - a) 
		+ cos(d)*sin(a)*sin(THETA - b) + cos(d)*sin(b)*sin(THETA - a));

    // Fix tension in line 4
	float T32 = (BASE_T*cos(a)*sin(b)*sin(THETA - d) + BASE_T*cos(a)*sin(d)*sin(THETA - b) 
		+ BASE_T*cos(b)*sin(a)*sin(THETA - d) - BASE_T*cos(b)*sin(d)*sin(THETA - a) 
		+ BASE_T*cos(d)*sin(a)*sin(THETA - b) + BASE_T*cos(d)*sin(b)*sin(THETA - a) 
		+ MASS*g*cos(a)*sin(THETA - b) - MASS*g*cos(b)*sin(THETA - a))/(cos(a)*sin(b)*sin(THETA - c) 
		- cos(a)*sin(c)*sin(THETA - b) + cos(b)*sin(a)*sin(THETA - c) + cos(b)*sin(c)*sin(THETA - a) 
		+ cos(c)*sin(a)*sin(THETA - b) + cos(c)*sin(b)*sin(THETA - a));
	float T42 = BASE_T;

	if(T41>0 && T32<0){
		t->T1 = T31;
		t->T2 = T41;	
	} 
	else if(T32>0 && T41<0)
	{
		t->T1 = T32;	
		t->T2 = T42;
	}
	else{
		t->T1 = T31;
		t->T2 = T41;		
	}

	t->T1*=0.0254; // conversion to torque N.m
	t->T2*=0.0254; 
	t->V1 = t->T1/(t->T1 + t->T2); //normalized
	t->V2 = t->T2/(t->T1 + t->T2);
}