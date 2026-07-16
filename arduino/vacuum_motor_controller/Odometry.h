#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"

extern float roverX_in;
extern float roverY_in;
extern float roverHeading_rad;

extern float roverVx_in_s;
extern float roverVy_in_s;
extern float roverOmega_rad_s;

void UpdateRoverOdometry();
void ResetRoverOdometry();
void ReseedRoverOdometry();

float wrapAngle(float angle);

#endif