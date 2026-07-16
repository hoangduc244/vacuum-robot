#include "Config.h"
#include "Motor.h"
#include "Odometry.h"

// ===================== Robot Mode =====================

enum RobotMode {
  MODE_STOP,
  MODE_MANUAL,
  MODE_SQUARE_PATH
};

RobotMode currentMode = MODE_MANUAL;

// ===================== Square Path =====================

struct Point {
  float x;
  float y;
};

Point path[] = {
  {0.0, 0.0},
  {12.0, 0.0},
  {12.0, 12.0},
  {0.0, 12.0},
  {0.0, 0.0}
};

const int numPoints = 5;
int segmentIndex = 0;

bool turningMode = false;
bool pathDone = false;

float targetHeading_rad = 0.0;

// ===================== Manual Control =====================

float linear_m_s = 0.0;
float angular_rad_s = 0.0;

String inputString = "";

// ROS/web sends commands repeatedly.
// If command stops coming, robot stops fast.
const unsigned long commandTimeout_ms = 500;
unsigned long lastCommandTime = 0;

// ===================== Motor Targets =====================

float rightTarget = 0.0;
float leftTarget = 0.0;

// ===================== Timing =====================

unsigned long lastControlTime = 0;
unsigned long lastOdomPrintTime = 0;

// ===================== Setup =====================

void setup() {
  Serial.begin(115200);

  Motor_Init();
  ResetRoverOdometry();

  delay(1000);

  Serial.println("READY");

  lastControlTime = millis();
  lastCommandTime = millis();
}

// ===================== Main Loop =====================

void loop() {
  unsigned long currentTime = millis();

  ReadSerialCommand();

  if (currentTime - lastControlTime >= controlPeriod_ms) {
    lastControlTime = currentTime;

    Motor_UpdateSpeed(&Motor1);
    Motor_UpdateSpeed(&Motor2);

    UpdateRoverOdometry();

    if (currentMode == MODE_MANUAL) {
      if (currentTime - lastCommandTime > commandTimeout_ms) {
        linear_m_s = 0.0;
        angular_rad_s = 0.0;
      }
    }

    if (currentMode == MODE_MANUAL) {
      RunManualControl();
    }
    else if (currentMode == MODE_SQUARE_PATH) {
      if (!pathDone) {
        RunSquarePath();
      } else {
        rightTarget = 0.0;
        leftTarget = 0.0;
      }
    }
    else {
      rightTarget = 0.0;
      leftTarget = 0.0;
    }

    Motor_SpeedControl(&Motor1, rightTarget);
    Motor_SpeedControl(&Motor2, leftTarget);

    Motor_Go(&Motor1);
    Motor_Go(&Motor2);

    PrintOdometry();
  }
}

// ===================== Read Serial Command =====================

void ReadSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      inputString.trim();

      if (inputString.length() > 0) {
        ParseCommand(inputString);
      }

      inputString = "";
    } else {
      inputString += c;
    }
  }
}

// ===================== Parse Serial Command =====================

void ParseCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  if (command == "RESET_ODOM") {
    ResetRoverOdometry();
    ResetMotorPID();
    return;
  }

  if (command == "MODE:MANUAL") {
    currentMode = MODE_MANUAL;
    linear_m_s = 0.0;
    angular_rad_s = 0.0;
    rightTarget = 0.0;
    leftTarget = 0.0;
    ResetMotorPID();
    ReseedRoverOdometry();
    return;
  }

  if (command == "MODE:SQUARE") {
    currentMode = MODE_SQUARE_PATH;
    ResetSquarePath();
    ResetMotorPID();
    return;
  }

  if (command == "MODE:STOP") {
    currentMode = MODE_STOP;
    linear_m_s = 0.0;
    angular_rad_s = 0.0;
    rightTarget = 0.0;
    leftTarget = 0.0;
    ResetMotorPID();
    return;
  }

  int vIndex = command.indexOf("V:");
  int wIndex = command.indexOf(",W:");

  if (vIndex == -1 || wIndex == -1) {
    return;
  }

  String vString = command.substring(vIndex + 2, wIndex);
  String wString = command.substring(wIndex + 3);

  linear_m_s = vString.toFloat();
  angular_rad_s = wString.toFloat();

  currentMode = MODE_MANUAL;
  lastCommandTime = millis();
}

// ===================== Manual Control Logic =====================
//
// ROS/Web convention:
// V positive = forward
// W positive = left
//
// Current robot physical result:
// V:-0.08 moves physical forward.
// V: 0.08 moves physical backward.
//
// Keep this for now because your 1-meter test is very good.

void RunManualControl() {
  float forward_in_s = linear_m_s * 39.37;
  float turn_in_s = angular_rad_s * wheelDistance_in * 0.5;

  rightTarget = -forward_in_s + turn_in_s;
  leftTarget  =  forward_in_s + turn_in_s;

  rightTarget = constrain(rightTarget, -maxWheelSpeed_in_s, maxWheelSpeed_in_s);
  leftTarget  = constrain(leftTarget, -maxWheelSpeed_in_s, maxWheelSpeed_in_s);
}

// ===================== Square Path Logic =====================

void RunSquarePath() {
  if (turningMode) {
    RunTurnMode();
  } else {
    RunStraightMode();
  }
}

// ===================== Turn Mode =====================

void RunTurnMode() {
  float headingError = wrapAngle(targetHeading_rad - roverHeading_rad);

  if (abs(headingError) < turnTolerance_rad) {
    turningMode = false;
    rightTarget = 0.0;
    leftTarget = 0.0;
    ResetMotorPID();
    ReseedRoverOdometry();
    return;
  }

  if (headingError > 0) {
    // Turn left
    rightTarget = turnSpeed_in_s;
    leftTarget  = turnSpeed_in_s;
  } else {
    // Turn right
    rightTarget = -turnSpeed_in_s;
    leftTarget  = -turnSpeed_in_s;
  }
}

// ===================== Straight Mode =====================

void RunStraightMode() {
  Point startPoint = path[segmentIndex];
  Point endPoint   = path[segmentIndex + 1];

  float segX = endPoint.x - startPoint.x;
  float segY = endPoint.y - startPoint.y;
  float segLength = sqrt(segX * segX + segY * segY);

  float unitX = segX / segLength;
  float unitY = segY / segLength;

  float robotX = roverX_in - startPoint.x;
  float robotY = roverY_in - startPoint.y;

  float progress = robotX * unitX + robotY * unitY;

  if (progress >= segLength - waypointTolerance_in) {
    segmentIndex++;

    if (segmentIndex >= numPoints - 1) {
      pathDone = true;
      rightTarget = 0.0;
      leftTarget = 0.0;
      ResetMotorPID();
    }
    else {
      targetHeading_rad = getSegmentHeading(segmentIndex);
      turningMode = true;
      rightTarget = 0.0;
      leftTarget = 0.0;
      ResetMotorPID();
      ReseedRoverOdometry();
    }

    return;
  }

  float desiredHeading = getSegmentHeading(segmentIndex);
  float headingError = wrapAngle(desiredHeading - roverHeading_rad);

  float correction = headingGain * headingError;

  // Forward in this robot motor convention:
  // right negative, left positive
  rightTarget = -pathSpeed_in_s - correction;
  leftTarget  =  pathSpeed_in_s + correction;

  rightTarget = constrain(rightTarget, -3.0, 3.0);
  leftTarget  = constrain(leftTarget, -3.0, 3.0);
}

// ===================== Reset Square Path =====================

void ResetSquarePath() {
  ResetRoverOdometry();

  segmentIndex = 0;
  turningMode = false;
  pathDone = false;
  targetHeading_rad = getSegmentHeading(0);

  rightTarget = 0.0;
  leftTarget = 0.0;
}

// ===================== Segment Heading =====================

float getSegmentHeading(int index) {
  Point start = path[index];
  Point end   = path[index + 1];

  float dx = end.x - start.x;
  float dy = end.y - start.y;

  return atan2(dy, dx);
}

// ===================== Print Odometry =====================
//
// ROS bridge needs exactly:
// ODOM:x,y,theta
//
// Do not add debug text after theta.

void PrintOdometry() {
  unsigned long currentTime = millis();

  if (currentTime - lastOdomPrintTime < odomPrintPeriod_ms) {
    return;
  }

  lastOdomPrintTime = currentTime;

  float x_m = roverX_in * 0.0254;
  float y_m = roverY_in * 0.0254;
  float thetaWrapped = wrapAngle(roverHeading_rad);

  Serial.print("ODOM:");
  Serial.print(x_m, 3);
  Serial.print(",");
  Serial.print(y_m, 3);
  Serial.print(",");
  Serial.println(thetaWrapped, 3);
}