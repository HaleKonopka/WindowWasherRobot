#include "PIDControl.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PurePursuit.h"
#include <Encoder.h>
#include "YACL.h"

#define KP                    .2
#define KI                    0.2
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

//Path following
robot_pose_t startingLocation;
pure_pursuit_t follower;
robot_cables_t cableLens;
robot_cables_t cableLenInitial;
robot_cables_t cableVels;

int iter = 0;

boolean sendLog(CBORPair logger)
{
  // Convert from CBOR Pair Object to byte array to be sent over serial
  const uint8_t *cbor_encoded = logger.to_CBOR();
  
  // Send payload length (2 bites for a size_t on Arduino) followed by payload
  // Format: [ payload len LSB | payload len MSB | payload ]
  size_t payload_len = logger.length();     // sizeof(size_t) = 2 bytes
  Serial.write(payload_len % 256);          // Send LSB
  Serial.write(payload_len >> 8);           // Send MSB
  Serial.write(cbor_encoded, payload_len);  // Send Payload
}

void setup () 
{
  Serial.begin(115200);  // Initialize the serial port
  Serial.println("Setup start");

  // Setup velocity controllers
  pid_control_contruct(&top_r_control, KP, KI, KD, KF);
  pid_control_contruct(&top_l_control, KP, KI, KD, KF);

  // Setup motors
  motor_initialize(&top_r_motor, &rightEncoder, &top_r_control, MTR_TOP_R_EN, MTR_TOP_R_ENB, MTR_TOP_R_PWM1, MTR_TOP_R_PWM2, MTR_TOP_R_DIAG, false, false);
  motor_initialize(&top_l_motor, &leftEncoder, &top_l_control, MTR_TOP_L_EN, MTR_TOP_L_ENB, MTR_TOP_L_PWM1, MTR_TOP_L_PWM2, MTR_TOP_L_DIAG, false, false);

  // Setup path follower
  startingLocation.x = 16.0;
  startingLocation.y = 62.0;
  pure_pursuit_initialize(&follower, &startingLocation, 1, 3);
  kinematics_reverse_position(&startingLocation, &cableLenInitial);

  Serial.println("Setup done");
}

void loop () 
{
  // Create CBOR Pair Object. Add items by appending dictionary pairs
  // 500 Bytes for logger arbitrarily chosen, may need to change this depending on usage
  CBORPair logger = CBORPair(500); 
  
  cableLens.top_left = motor_get_position(&top_l_motor);
  cableLens.top_right = motor_get_position(&top_r_motor);
  cableLens.top_left += cableLenInitial.top_left;
  cableLens.top_right += cableLenInitial.top_right;
  
  pure_pursuit_calculate(&follower, &cableLens, &cableVels);

  motor_set_velocity(&top_l_motor, cableVels.top_left);
  motor_set_velocity(&top_r_motor, cableVels.top_right);

  motor_update_pid(&top_l_motor);
  motor_update_pid(&top_r_motor);

  /* Some Notes about CBOR logging
   *  - All samples within a given CBOR-formatted payload will be given the same timestamp. 
   *    If you want to sample different quantities at different frequencies, send them as
   *    individual payloads
   *  - Timestamping samples is achieved by appending a "time(ms)" field 
   *    e.g. CBORPair logger = CBORPair(500); <-- Change buffer size as needed
   *         logger.append("time(ms)", millis())
   *  - On the Raspberry Pi side, all data entries logged with a given timestamp
   *    will be logged on the same row of the output CSV file.
   *    - The flow on Raspberry Pi side is 
   *      1) Serial CBOR-formatted message received, convert this to dictionary form
   *      2) Load into Pandas dataframe (this stores the entire CSV in memory)
   *      3) Periodically (every 30 seconds) replace existing csv file (definitely wasteful 
   *         as opposed to appending, so testing will need to be performed to see if this breaks)
   */
  logger.append("time(ms)", millis()); // When logging, always append timestamp for samples
  logger.append("TL vel set", top_l_motor.vel_cmd);
  logger.append("TL vel meas", motor_get_velocity(&top_l_motor));
  logger.append("TR vel set", top_r_motor.vel_cmd);
  logger.append("TR vel meas", motor_get_velocity(&top_r_motor));
  sendLog(logger);

  if (iter > 20){
    char p[100];
    sprintf(p, "L Cable Len: %s, Cable Vel: %s", String(cableLens.top_left, 2).c_str(), String(cableVels.top_left, 2).c_str());
    Serial.println(p);
    sprintf(p, "R Cable Len: %s, Cable Vel: %s", String(cableLens.top_right, 2).c_str(), String(cableVels.top_right, 2).c_str());
    Serial.println(p);

    iter = 0;
  }
  iter++;

  delay(5);
}
