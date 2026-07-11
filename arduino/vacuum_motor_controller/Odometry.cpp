#include "Odometry.h"

float roverX_in = 0.0;
float roverY_in = 0.0;
float roverHeading_rad = 0.0;

long lastRightOdomCount = 0;
long lastLeftOdomCount = 0;

void UpdateRoverOdometry() {
  long rightCurrentCount = Motor1.encoderCount;
  long leftCurrentCount  = Motor2.encoderCount;

  long rightDeltaCount = rightCurrentCount - lastRightOdomCount;
  long leftDeltaCount  = leftCurrentCount - lastLeftOdomCount;

  lastRightOdomCount = rightCurrentCount;
  lastLeftOdomCount  = leftCurrentCount;

  float rightRev = (rightDeltaCount * Motor1.encoderSign) / countsPerRev;
  float leftRev  = (leftDeltaCount * Motor2.encoderSign) / countsPerRev;

  float dRight = rightRev * wheelCircumference_in;
  float dLeft  = leftRev * wheelCircumference_in;

  float dCenter = (dRight + dLeft) / 2.0;
  float dTheta = (dRight - dLeft) / wheelDistance_in;

  roverHeading_rad += dTheta;
  roverHeading_rad = wrapAngle(roverHeading_rad);

  roverX_in += dCenter * cos(roverHeading_rad);
  roverY_in += dCenter * sin(roverHeading_rad);
}


float wrapAngle(float angle) {
  while (angle > PI) {
    angle -= 2.0 * PI;
  }

  while (angle < -PI) {
    angle += 2.0 * PI;
  }

  return angle;
}
