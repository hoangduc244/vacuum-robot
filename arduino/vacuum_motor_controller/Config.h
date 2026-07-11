#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===================== Pins =====================
// Right Motor
#define RIGHT_PWM 44
#define RIGHT_IN1 30
#define RIGHT_IN2 31
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 2

// Left Motor
#define LEFT_PWM 45
#define LEFT_IN1 32
#define LEFT_IN2 33
#define LEFT_ENCODER_A 19
#define LEFT_ENCODER_B 20

// ===================== Robot Geometry =====================
const float wheelDiameter_in = 3.0;
const float wheelCircumference_in = PI * wheelDiameter_in;

// From your 90 degree turn calibration
const float wheelDistance_in = 5.0;

const float countsPerRev = 1440.0;

// ===================== Timing =====================
const unsigned long controlPeriod_ms = 10;
const unsigned long printPeriod_ms = 100;
const unsigned long startDelay_ms = 3000;

// ===================== PID Settings =====================
const int rightBasePWM = 36;
const int leftBasePWM  = 36;

const int minPWM = 12;
const int maxPWM = 100;

const float Kp = 1.5;
const float Ki = 2.0;
const float Kd = 0.0;

const float alpha = 0.15;

// ===================== Path Settings =====================
const float pathSpeed_in_s = 2.0;
const float waypointTolerance_in = 0.3;
const float headingGain = 0.3;

const float turnSpeed_in_s = 1.0;
const float turnTolerance_rad = 0.05;

#endif