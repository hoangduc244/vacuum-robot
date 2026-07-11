#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"

extern float roverX_in;
extern float roverY_in;
extern float roverHeading_rad;

void UpdateRoverOdometry();
float wrapAngle(float angle);

#endif