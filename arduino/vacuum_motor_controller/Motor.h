#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Config.h"

struct DC_Motor {
  int PWM;
  int IN1;
  int IN2;
  int encoderA;
  int encoderB;

  volatile long encoderCount;
  long lastEncoderCount;

  int encoderSign;

  float rawSpeed_in_s;
  float filteredSpeed_in_s;

  float error;
  float lastError;
  float integral;

  int u_pwm;
};

extern DC_Motor Motor1;  // Right
extern DC_Motor Motor2;  // Left

void Motor_Init();
void Motor_UpdateSpeed(DC_Motor* motor);
void Motor_SpeedControl(DC_Motor* motor, float targetSpeed);
void Motor_Go(DC_Motor* motor);
void Motor_Stop(DC_Motor* motor);
void ResetMotorPID();

void RightEncoderISR_A();
void LeftEncoderISR_A();

#endif