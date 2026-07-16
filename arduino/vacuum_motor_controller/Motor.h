#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Config.h"
#include "EncoderCounter.h"

struct Motor {
  int pwmPin;
  int in1Pin;
  int in2Pin;

  EncoderCounter4x* encoder;

  long lastEncoderCount;

  unsigned long lastEdgeUs;

  float measuredSpeed_in_s;
  float filteredSpeed_in_s;
  float targetSpeed_in_s;

  float error;
  float lastError;
  float integral;

  int pwmOutput;

  // Same idea as motorInvert in your class code.
  // 1 = normal, -1 = invert physical motor direction.
  int motorSign;
};

extern Motor Motor1;  // real right motor
extern Motor Motor2;  // real left motor

extern EncoderCounter4x RightEncoder;
extern EncoderCounter4x LeftEncoder;

void Motor_Init();

void Motor_UpdateSpeed(Motor* motor);
void Motor_SpeedControl(Motor* motor, float targetSpeed_in_s);
void Motor_Go(Motor* motor);

void ResetMotorPID();

#endif