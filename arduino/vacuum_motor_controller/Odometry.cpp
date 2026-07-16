#include "Odometry.h"

float roverX_in = 0.0;
float roverY_in = 0.0;
float roverHeading_rad = 0.0;

float roverVx_in_s = 0.0;
float roverVy_in_s = 0.0;
float roverOmega_rad_s = 0.0;

long lastRightOdomCount = 0;
long lastLeftOdomCount = 0;

// ===================== Odom Direction Signs =====================
//
// This is same idea as ODOM_LEFT_MOTOR_DIR_INVERTED and
// ODOM_RIGHT_MOTOR_DIR_INVERTED in your class code.
//
// Because your motor positive direction physically went backward,
// and encoder counts are corrected to match motor command,
// we use -1 so physical forward becomes positive odom distance.

const int rightOdomSign = -1;
const int leftOdomSign  = 1;

// ===================== Update Odom =====================

void UpdateRoverOdometry() {
  long rightCurrentCount = Motor1.encoder->getCount();
  long leftCurrentCount  = Motor2.encoder->getCount();

  long rightDeltaRaw = rightCurrentCount - lastRightOdomCount;
  long leftDeltaRaw  = leftCurrentCount - lastLeftOdomCount;

  lastRightOdomCount = rightCurrentCount;
  lastLeftOdomCount  = leftCurrentCount;

  long rightDelta = rightDeltaRaw * rightOdomSign;
  long leftDelta  = leftDeltaRaw * leftOdomSign;

  float dt_s = controlPeriod_ms / 1000.0;

  if (rightDelta == 0 && leftDelta == 0) {
    roverVx_in_s = 0.0;
    roverVy_in_s = 0.0;
    roverOmega_rad_s = 0.0;
    return;
  }

  float rightRev = (float)rightDelta / countsPerRev;
  float leftRev  = (float)leftDelta  / countsPerRev;

  float dRight = rightRev * wheelCircumference_in;
  float dLeft  = leftRev  * wheelCircumference_in;

  float dCenter = -(dRight + dLeft) * 0.5;

  // Positive theta = CCW / physical left turn.
  // Same formula as class RobotKinematics:
  // dTheta = (dRight - dLeft) / wheelBase
  float dTheta = (-dRight + dLeft) / wheelDistance_in;

  // Midpoint heading integration from trusted class code.
  float headingMid = roverHeading_rad + dTheta * 0.5;

  roverX_in += dCenter * cos(headingMid);
  roverY_in += dCenter * sin(headingMid);
  roverHeading_rad += dTheta;

  roverVx_in_s = dCenter / dt_s;
  roverVy_in_s = 0.0;
  roverOmega_rad_s = dTheta / dt_s;
}

// ===================== Reset Odom =====================

void ResetRoverOdometry() {
  Motor1.encoder->setCount(0);
  Motor2.encoder->setCount(0);

  Motor1.lastEncoderCount = 0;
  Motor2.lastEncoderCount = 0;

  lastRightOdomCount = 0;
  lastLeftOdomCount = 0;

  roverX_in = 0.0;
  roverY_in = 0.0;
  roverHeading_rad = 0.0;

  roverVx_in_s = 0.0;
  roverVy_in_s = 0.0;
  roverOmega_rad_s = 0.0;
}

// ===================== Reseed Odom =====================
//
// Preserve pose but update tick baseline.
// Useful if encoder count was reset somewhere else.

void ReseedRoverOdometry() {
  lastRightOdomCount = Motor1.encoder->getCount();
  lastLeftOdomCount  = Motor2.encoder->getCount();

  roverVx_in_s = 0.0;
  roverVy_in_s = 0.0;
  roverOmega_rad_s = 0.0;
}

// ===================== Wrap Angle =====================

float wrapAngle(float angle) {
  while (angle > PI) {
    angle -= 2.0 * PI;
  }

  while (angle < -PI) {
    angle += 2.0 * PI;
  }

  return angle;
}