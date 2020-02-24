// Please download encoder library by Paul Stroffregen
#include <Encoder.h>
#include "HCSR04.h"
#include <AutoPID.h>
#define ENCODER_TICKS_PER_REV 0
#define OUTPUT_MIN            -255
#define OUTPUT_MAX            255
#define KP                    .011
#define KI                    .001
#define KD                    0

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

volatile double inputPosTopRight, setPointTopRight, outputValTopRight;
volatile double inputPosTopLeft, setPointTopLeft, outputValTopLeft;
AutoPID motorPosTopRightPID(&inputPosTopRight, &setPointTopRight, &outputValTopRight, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID motorPosTopLeftPID(&inputPosTopLeft, &setPointTopLeft, &outputValTopLeft, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
long prevPosLeft = 0;
long prevPosRight = 0;
unsigned long prevTime = 0;

// State Variables
long timeout;

void setup () 
{
    Serial.begin(115200);  // Initialize the serial port
    pinMode(MTR_TOP_R_EN,  OUTPUT);
    digitalWrite(MTR_TOP_R_EN, HIGH);
    pinMode(MTR_TOP_R_ENB, OUTPUT);
    digitalWrite(MTR_TOP_R_ENB, LOW);
    pinMode(MTR_TOP_R_DIAG, INPUT);
    pinMode(MTR_TOP_R_PWM1, OUTPUT);
    pinMode(MTR_TOP_R_PWM2, OUTPUT);
    
    pinMode(MTR_TOP_L_EN,  OUTPUT);
    digitalWrite(MTR_TOP_L_EN, HIGH);
    pinMode(MTR_TOP_L_ENB, OUTPUT);
    digitalWrite(MTR_TOP_L_ENB, LOW);
    pinMode(MTR_TOP_L_DIAG, INPUT);
    pinMode(MTR_TOP_L_PWM1, OUTPUT);
    pinMode(MTR_TOP_L_PWM2, OUTPUT);

    // Set up PID
    motorPosTopRightPID.setTimeStep(10);
    setPointTopRight = -5000;
    motorPosTopLeftPID.setTimeStep(10);
    setPointTopLeft = -5000;

    // Set up Encoder
    rightEncoder.write(0);
    leftEncoder.write(0);
    
    timeout = millis();
}

void control_motors()
{
  unsigned long currentTime = millis();
  unsigned long dT = currentTime - prevTime;
  prevTime = currentTime;
  
  // Right Motor Control
  inputPosTopRight = (double)rightEncoder.read();
  motorPosTopRightPID.run();
  /*
  Serial.print("Input Right: ");
  Serial.println(inputPosTopRight);
  Serial.print("Setpoint Right: ");
  Serial.println(setPointTopRight);
  Serial.print("Output Right: ");
  Serial.println(outputValTopRight);
  */
  if(outputValTopRight > 0)
  {
    analogWrite(MTR_TOP_R_PWM2, outputValTopRight);
    digitalWrite(MTR_TOP_R_PWM1, LOW);
  }
  else
  {
    outputValTopRight = -outputValTopRight; // Change sign to be positive
    digitalWrite(MTR_TOP_R_PWM2, LOW);      
    analogWrite(MTR_TOP_R_PWM1, outputValTopRight);
  }

  // Left Motor Control
  inputPosTopLeft = (double)leftEncoder.read();
  motorPosTopLeftPID.run();

  /*
  Serial.print("Input Left: ");
  Serial.println(inputPosTopLeft);
  Serial.print("Setpoint Left: ");
  Serial.println(setPointTopLeft);
  Serial.print("Output Left: ");
  Serial.println(outputValTopLeft);
  */
  if(outputValTopLeft > 0)
  {
    analogWrite(MTR_TOP_L_PWM1, outputValTopLeft);
    digitalWrite(MTR_TOP_L_PWM2, LOW);
  }
  else
  {
    outputValTopLeft = -outputValTopLeft; // Change sign to be positive
    digitalWrite(MTR_TOP_L_PWM1, LOW);
    analogWrite(MTR_TOP_L_PWM2, outputValTopLeft);      
  }

}

void loop () 
{   
    /*
    Serial.print("left encoder ticks: ");
    Serial.print(leftEncoder.read());
    Serial.print(", right encoder ticks: ");
    Serial.print(rightEncoder.read());
    Serial.print("\n");
    */
    if (digitalRead(MTR_TOP_R_DIAG) == LOW || digitalRead(MTR_TOP_L_DIAG) == LOW)
    {
      //Serial.print("Motor Error Occurred! Disabling Motors to prevent damage\n");
      digitalWrite(MTR_TOP_R_EN, LOW);
      digitalWrite(MTR_TOP_L_EN, LOW);
    }
    else
    {
      double right_error = (double)rightEncoder.read() - setPointTopRight;
      double left_error = (double)leftEncoder.read() - setPointTopLeft;

      Serial.print("Right Error: ");
      Serial.print(abs(right_error));
      Serial.print(", left_error: ");
      Serial.println(abs(left_error));
      if(abs(right_error) < 500.0 && abs(left_error) < 500.0 && (timeout - millis() > 10000))
      {
        Serial.println("Setpoint Change!");
        setPointTopRight = -setPointTopRight;
        setPointTopLeft = -setPointTopLeft;
        timeout = millis();
      }
      control_motors(); 
    }
    
    delay(100);
}
