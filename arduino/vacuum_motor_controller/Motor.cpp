#include "Motor.h"

// ===================== Encoder Objects =====================
//
// Encoder direction is corrected here, like encoderInvert in class code.
//
// Based on your stable test:
// Right encoder normal
// Left encoder inverted

EncoderCounter4x RightEncoder;
EncoderCounter4x LeftEncoder;

// ===================== Motor Objects =====================
//
// Motor direction is separate from encoder direction.
// Keep both motorSign = 1 because this was your stable setup.
// We fix "W forward" in RunManualControl(), not by randomly changing signs.

Motor Motor1 = {
  RIGHT_PWM,
  RIGHT_IN1,
  RIGHT_IN2,
  &RightEncoder,
  0,
  0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0,
  -1
};

Motor Motor2 = {
  LEFT_PWM,
  LEFT_IN1,
  LEFT_IN2,
  &LeftEncoder,
  0,
  0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0,
  1
};

// ===================== Encoder ISR Wrappers =====================

void rightEncoderISR_A() {
  RightEncoder.onInterruptA();
}

void rightEncoderISR_B() {
  RightEncoder.onInterruptB();
}

void leftEncoderISR_A() {
  LeftEncoder.onInterruptA();
}

void leftEncoderISR_B() {
  LeftEncoder.onInterruptB();
}

// ===================== Motor Init =====================

void Motor_Init() {
  pinMode(Motor1.pwmPin, OUTPUT);
  pinMode(Motor1.in1Pin, OUTPUT);
  pinMode(Motor1.in2Pin, OUTPUT);

  pinMode(Motor2.pwmPin, OUTPUT);
  pinMode(Motor2.in1Pin, OUTPUT);
  pinMode(Motor2.in2Pin, OUTPUT);

  // Right encoder: normal direction
  RightEncoder.init(RIGHT_ENCODER_A, RIGHT_ENCODER_B, true);

  // Left encoder: inverted direction
  LeftEncoder.init(LEFT_ENCODER_A, LEFT_ENCODER_B, true);

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), rightEncoderISR_B, CHANGE);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), leftEncoderISR_B, CHANGE);

  ResetMotorPID();

  Motor1.encoder->resetCount();
  Motor2.encoder->resetCount();

  Motor1.lastEncoderCount = 0;
  Motor2.lastEncoderCount = 0;

  Motor_Go(&Motor1);
  Motor_Go(&Motor2);
}

// ===================== Speed Feedback =====================

void Motor_UpdateSpeed(Motor* motor) {
  long currentCount;
  unsigned long edgeUs;

  motor->encoder->snapshot(currentCount, edgeUs);

  long deltaCount = currentCount - motor->lastEncoderCount;
  motor->lastEncoderCount = currentCount;

  float rev = (float)deltaCount / countsPerRev;
  float distance_in = rev * wheelCircumference_in;

  float dt_s = controlPeriod_ms / 1000.0;

  motor->measuredSpeed_in_s = distance_in / dt_s;

  // If encoder has not moved recently, force speed to zero.
  // This copies the class-code velocity-zero-timeout idea.
  if (edgeUs != 0 && (micros() - edgeUs) > 50000UL) {
    motor->measuredSpeed_in_s = 0.0;
  }

  motor->filteredSpeed_in_s =
    alpha * motor->measuredSpeed_in_s +
    (1.0 - alpha) * motor->filteredSpeed_in_s;
}

// ===================== Speed PID =====================
void Motor_SpeedControl(Motor* motor, float targetSpeed_in_s) {
  motor->targetSpeed_in_s = targetSpeed_in_s;

  if (abs(targetSpeed_in_s) < 0.01) {
    motor->error = 0.0;
    motor->lastError = 0.0;
    motor->integral = 0.0;
    motor->pwmOutput = 0;
    return;
  }

  float targetAbs = abs(targetSpeed_in_s);
  float speedAbs = abs(motor->filteredSpeed_in_s);

  motor->error = targetAbs - speedAbs;

  float dt_s = controlPeriod_ms / 1000.0;

  motor->integral += motor->error * dt_s;
  motor->integral = constrain(motor->integral, -20.0, 20.0);

  float derivative = (motor->error - motor->lastError) / dt_s;
  motor->lastError = motor->error;

  float feedForward = targetAbs * pwmPerInchPerSec;

  float output =
    feedForward +
    Kp * motor->error +
    Ki * motor->integral +
    Kd * derivative;

  int pwm = (int)output;

  pwm = constrain(pwm, minPWM, maxPWM);

  if (targetSpeed_in_s < 0) {
    pwm = -pwm;
  }

  motor->pwmOutput = pwm;
}
// ===================== Apply Motor Output =====================

void Motor_Go(Motor* motor) {
  int pwm = motor->pwmOutput * motor->motorSign;

  if (pwm > 0) {
    digitalWrite(motor->in1Pin, HIGH);
    digitalWrite(motor->in2Pin, LOW);
    analogWrite(motor->pwmPin, constrain(pwm, 0, 255));
  }
  else if (pwm < 0) {
    digitalWrite(motor->in1Pin, LOW);
    digitalWrite(motor->in2Pin, HIGH);
    analogWrite(motor->pwmPin, constrain(-pwm, 0, 255));
  }
  else {
    digitalWrite(motor->in1Pin, LOW);
    digitalWrite(motor->in2Pin, LOW);
    analogWrite(motor->pwmPin, 0);
  }
}

// ===================== Reset PID =====================

void ResetMotorPID() {
  Motor1.error = 0.0;
  Motor1.lastError = 0.0;
  Motor1.integral = 0.0;
  Motor1.pwmOutput = 0;
  Motor1.targetSpeed_in_s = 0.0;
  Motor1.filteredSpeed_in_s = 0.0;
  Motor1.measuredSpeed_in_s = 0.0;
  Motor1.lastEncoderCount = Motor1.encoder ? Motor1.encoder->getCount() : 0;

  Motor2.error = 0.0;
  Motor2.lastError = 0.0;
  Motor2.integral = 0.0;
  Motor2.pwmOutput = 0;
  Motor2.targetSpeed_in_s = 0.0;
  Motor2.filteredSpeed_in_s = 0.0;
  Motor2.measuredSpeed_in_s = 0.0;
  Motor2.lastEncoderCount = Motor2.encoder ? Motor2.encoder->getCount() : 0;
}