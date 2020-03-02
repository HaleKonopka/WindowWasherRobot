#include "PIDControl.h"
#include "Motor.h"
#include "Kinematics.h"
#include <Encoder.h>

#define KP                    .1
#define KI                    0
#define KD                    0
#define KF                    0.095

// Encoder Pin Definitions
#define ENC_TOP_R_A           20
#define ENC_TOP_R_B           21
#define ENC_TOP_L_A           19
#define ENC_TOP_L_B           18

// Motor Pin Definitions
#define MTR_TOP_R_EN          46
#define MTR_TOP_R_ENB         44 
#define MTR_TOP_R_PWM1        5
#define MTR_TOP_R_PWM2        4
#define MTR_TOP_R_DIAG        52

#define MTR_TOP_L_EN          24        
#define MTR_TOP_L_ENB         22
#define MTR_TOP_L_PWM1        3
#define MTR_TOP_L_PWM2        2
#define MTR_TOP_L_DIAG        50

// Motor Declarations
Encoder rightEncoder(ENC_TOP_R_A, ENC_TOP_R_B);
Encoder leftEncoder(ENC_TOP_L_A, ENC_TOP_L_B);
motor_t top_r_motor;
motor_t top_l_motor;
pid_control_t top_r_control;
pid_control_t top_l_control;

// State Variables
long timeout;
int timer1;
int timer2;
long lastTime;

void setup () 
{
  Serial.begin(115200);  // Initialize the serial port
  Serial.println("Setup start");

  // Setup velocity controllers
  pid_contruct(&top_r_control, KP, KI, KD, KF);
  pid_contruct(&top_l_control, KP, KI, KD, KF);

  // Setup motors
  motor_initialize(&top_r_motor, &rightEncoder, &top_r_control, MTR_TOP_R_EN, MTR_TOP_R_ENB, MTR_TOP_R_PWM1, MTR_TOP_R_PWM2, MTR_TOP_R_DIAG);
  motor_initialize(&top_l_motor, &leftEncoder, &top_l_control, MTR_TOP_L_EN, MTR_TOP_L_ENB, MTR_TOP_L_PWM1, MTR_TOP_L_PWM2, MTR_TOP_L_DIAG);

  timeout = millis();
  timer1 = 0;
  timer2 = 1000;
  lastTime = millis();

  Serial.println("Setup done");
}

void loop () 
{
  if (timer1 <= 0) {
    motor_set_velocity(&top_r_motor, 0.5);
    //motor_set_velocity(top_l_motor, 0.02);

    timer1 = 2000;
  }

  if (timer2 <= 0){
    motor_set_velocity(&top_r_motor, -0.5);
    //motor_set_velocity(top_l_motor, -0.02);

    timer2 = 2000;
  }

  //motor_update_pid(top_l_motor);
  motor_update_pid(&top_r_motor);

  char p[100];
  sprintf(p, "Setpoint Vel: %.2f, Actual Vel: %.2f", top_r_motor.vel_cmd, motor_get_velocity(&top_r_motor));
  Serial.println(p);

  long nowTime = millis();
  timer1 -= nowTime - lastTime;
  timer2 -= nowTime - lastTime;
  lastTime = nowTime;

  delay(10);
}