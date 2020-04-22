#include "PIDControl.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PurePursuit.h"
#include <Encoder.h>
#include <EEPROM.h>

#define KP                    .2
#define KI                    0.7
#define KD                    0
#define KF                    0.095

// Encoder Pin Definitions
#define ENC_TOP_R_A           3
#define ENC_TOP_R_B           2
#define ENC_TOP_L_A           19
#define ENC_TOP_L_B           18

// Motor Pin Definitions
#define MTR_TOP_R_EN          51
#define MTR_TOP_R_ENB         49 
#define MTR_TOP_R_PWM1        5
#define MTR_TOP_R_PWM2        4
#define MTR_TOP_R_DIAG        52
#define MTR_TOP_R_CS          A0

#define MTR_TOP_L_EN          24        
#define MTR_TOP_L_ENB         22
#define MTR_TOP_L_PWM1        7
#define MTR_TOP_L_PWM2        6
#define MTR_TOP_L_DIAG        50
#define MTR_TOP_L_CS          A1

#define MTR_BOT_R_EN          46
#define MTR_BOT_R_ENB         47 
#define MTR_BOT_R_PWM1        11
#define MTR_BOT_R_PWM2        10
#define MTR_BOT_R_DIAG        52  // TODO: Wire
#define MTR_BOT_R_CS          A2  // TODO: Wire

#define MTR_BOT_L_EN          44        
#define MTR_BOT_L_ENB         45
#define MTR_BOT_L_PWM1        9
#define MTR_BOT_L_PWM2        8
#define MTR_BOT_L_DIAG        53 // TODO: Wire
#define MTR_BOT_L_CS          A3 // TODO: Wire

#define ALT_MODE_PIN          31
#define LED_IND_PIN           13

// Motor Declarations
Encoder rightEncoder(ENC_TOP_R_A, ENC_TOP_R_B);
Encoder leftEncoder(ENC_TOP_L_A, ENC_TOP_L_B);
motor_t top_r_motor;
motor_t top_l_motor;
motor_t bot_r_motor;
motor_t bot_l_motor;
pid_control_t top_r_control;
pid_control_t top_l_control;

//Path following
robot_pose_t startingLocation;
pure_pursuit_t follower;
robot_cables_t cableLens;
robot_cables_t cableLenInitial;
robot_cables_t cableVels;

robot_orientation_sensor_t ang_sen;

int iter = 0;

void setup () 
{
  Serial.begin(115200);  // Initialize the serial port
  Serial.println("Setup start");

  // Setup velocity controllers
  pid_control_contruct(&top_r_control, KP, KI, KD, KF);
  pid_control_contruct(&top_l_control, KP, KI, KD, KF);

  // Setup motors
  motor_initialize(&top_r_motor, &rightEncoder, &top_r_control, MTR_TOP_R_EN, MTR_TOP_R_ENB, MTR_TOP_R_PWM1, MTR_TOP_R_PWM2, MTR_TOP_R_DIAG, MTR_TOP_R_CS, false, false);
  motor_initialize(&top_l_motor, &leftEncoder, &top_l_control, MTR_TOP_L_EN, MTR_TOP_L_ENB, MTR_TOP_L_PWM1, MTR_TOP_L_PWM2, MTR_TOP_L_DIAG, MTR_TOP_L_CS, false, false);
  motor_initialize(&bot_r_motor, NULL, NULL, MTR_BOT_R_EN, MTR_BOT_R_ENB, MTR_BOT_R_PWM1, MTR_BOT_R_PWM2, MTR_BOT_R_DIAG, MTR_BOT_R_CS, false, true);
  motor_initialize(&bot_l_motor, NULL, NULL, MTR_BOT_L_EN, MTR_BOT_L_ENB, MTR_BOT_L_PWM1, MTR_BOT_L_PWM2, MTR_BOT_L_DIAG, MTR_BOT_L_CS, false, false);

  // Setup orientation sensor
  kinematics_init_orientation_sensor(&ang_sen);

  // Setup path follower
  startingLocation.x = 7.0;
  startingLocation.y = 71.0;
  pure_pursuit_initialize(&follower, &startingLocation, 1, 2);
  kinematics_reverse_position(&startingLocation, &cableLenInitial);

  // Init Alt-mode pin
  pinMode(ALT_MODE_PIN, INPUT_PULLUP);
  while(!digitalRead(ALT_MODE_PIN)){
    motor_run_coast(&bot_l_motor, -0.2);
    motor_run_coast(&bot_r_motor, 0.2);
  }

  uint8_t cal_stat, a, b, c = 0;
  while (cal_stat < 3 || a < 3 || b< 3 || c < 3){
    ang_sen.bno.getCalibration(&cal_stat, &a, &b, &c);
    Serial.print("Cal status: ");
    Serial.print(cal_stat);
    Serial.print(" ");
    Serial.print(a);
    Serial.print(" ");
    Serial.print(b);
    Serial.print(" ");
    Serial.print(c);
    Serial.print(" ");
  }

  Serial.print("Cal done!\n");
  adafruit_bno055_offsets_t offsets;
  ang_sen.bno.getSensorOffsets(offsets);
  EEPROM.put(0, offsets);
  Serial.println(offsets.accel_offset_x);
  Serial.println(offsets.accel_offset_y);
  Serial.println(offsets.gyro_offset_z);
  Serial.println(offsets.mag_offset_x );
  Serial.print("Written!\n");

  // Wait for good orientation calibration
  pinMode(LED_IND_PIN, OUTPUT);
  digitalWrite(LED_IND_PIN, LOW);
  while(!kinematics_get_orientation_valid(&ang_sen)){
    delay(5);
  }
  digitalWrite(LED_IND_PIN, HIGH);
  delay(5000);

  Serial.println("Setup done");
}

void loop () 
{
  if (kinematics_get_orientation_valid(&ang_sen)){
    digitalWrite(LED_IND_PIN, HIGH);
  } else {
    digitalWrite(LED_IND_PIN, LOW);
  }

  cableLens.top_left = motor_get_position(&top_l_motor);
  cableLens.top_right = motor_get_position(&top_r_motor);
  cableLens.top_left += cableLenInitial.top_left;
  cableLens.top_right += cableLenInitial.top_right;
  
  pure_pursuit_calculate(&follower, &cableLens, &cableVels, &ang_sen);

  motor_set_velocity(&top_l_motor, cableVels.top_left);
  motor_set_velocity(&top_r_motor, cableVels.top_right);

  motor_update_pid(&top_l_motor);
  motor_update_pid(&top_r_motor);

  float pwr_adj = 0;
  if (cableVels.top_left  > 0 && cableVels.top_right > 0){
    pwr_adj += .15;
    Serial.print("Going down!\n");
  }

  float ang = kinematics_get_orientation(&ang_sen);
  if (ang > 0.05){
    motor_run_coast(&bot_l_motor, 0.2 + pwr_adj);
    motor_run_coast(&bot_r_motor, 0.07 + pwr_adj);
  } else if (ang < -0.05){
    motor_run_coast(&bot_l_motor, 0.07 + pwr_adj);
    motor_run_coast(&bot_r_motor, 0.2 + pwr_adj);
  } else {
    motor_run_coast(&bot_l_motor, 0.1 + pwr_adj);
    motor_run_coast(&bot_r_motor, 0.1 + pwr_adj);
  }

  if (iter > 1){
    char p[100];
    sprintf(p, "L Cable Len: %s, Cable Vel: %s, Current: %s", String(cableLens.top_left, 2).c_str(), String(cableVels.top_left, 2).c_str(), String(motor_get_current(&top_l_motor), 2).c_str());
    //Serial.println(p);
    sprintf(p, "R Cable Len: %s, Cable Vel: %s, Current: %s", String(cableLens.top_right, 2).c_str(), String(cableVels.top_right, 2).c_str(), String(motor_get_current(&top_r_motor), 2).c_str());
    //Serial.println(p);

    iter = 0;
  }
  iter++;

  delay(5);
}