#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===================== Motor Pins =====================
//
// Motor1 = REAL RIGHT motor
// Motor2 = REAL LEFT motor

// Right Motor in REAL LIFE
#define RIGHT_PWM 45
#define RIGHT_IN1 32
#define RIGHT_IN2 33
#define RIGHT_ENCODER_A 19
#define RIGHT_ENCODER_B 20

// Left Motor in REAL LIFE
#define LEFT_PWM 44
#define LEFT_IN1 30
#define LEFT_IN2 31
#define LEFT_ENCODER_A 3
#define LEFT_ENCODER_B 2

// ===================== Robot Geometry =====================

const float wheelDiameter_in = 3.0;
const float wheelCircumference_in = PI * wheelDiameter_in;

// measured wheel center-to-center distance: 25 cm = 9.84252 in
const float wheelDistance_in = 9.84252;

// Full 4x quadrature count
const float countsPerRev = 1440.0;

// ===================== Timing =====================

const unsigned long controlPeriod_ms = 10;
const unsigned long odomPrintPeriod_ms = 100;

// ===================== Speed Control =====================

const int minPWM = 25;
const int maxPWM = 100;

const float Kp = 4.0;
const float Ki = 2;
const float Kd = 0.0;

const float pwmPerInchPerSec = 10.0;

const float alpha = 0.15;

// ===================== Manual Control Limits =====================

const float maxWheelSpeed_in_s = 5.0;

// ===================== Square Path Settings =====================

const float pathSpeed_in_s = 3.0;
const float turnSpeed_in_s = 2.5;

const float waypointTolerance_in = 1.0;
const float turnTolerance_rad = 0.08;

const float headingGain = 4.0;

#endif