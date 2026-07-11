#include "Motor.h"

DC_Motor Motor1;
DC_Motor Motor2;

void Motor_Init() {
  // Right motor
  Motor1.PWM = RIGHT_PWM;
  Motor1.IN1 = RIGHT_IN1;
  Motor1.IN2 = RIGHT_IN2;
  Motor1.encoderA = RIGHT_ENCODER_A;
  Motor1.encoderB = RIGHT_ENCODER_B;
  Motor1.encoderSign = +1;

  // Left motor
  Motor2.PWM = LEFT_PWM;
  Motor2.IN1 = LEFT_IN1;
  Motor2.IN2 = LEFT_IN2;
  Motor2.encoderA = LEFT_ENCODER_A;
  Motor2.encoderB = LEFT_ENCODER_B;
  Motor2.encoderSign = -1;

  DC_Motor* motors[2] = {&Motor1, &Motor2};

  for (int i = 0; i < 2; i++) {
    DC_Motor* motor = motors[i];

    motor->encoderCount = 0;
    motor->lastEncoderCount = 0;

    motor->rawSpeed_in_s = 0.0;
    motor->filteredSpeed_in_s = 0.0;

    motor->error = 0.0;
    motor->lastError = 0.0;
    motor->integral = 0.0;

    motor->u_pwm = 0;

    pinMode(motor->PWM, OUTPUT);
    pinMode(motor->IN1, OUTPUT);
    pinMode(motor->IN2, OUTPUT);

    pinMode(motor->encoderA, INPUT_PULLUP);
    pinMode(motor->encoderB, INPUT_PULLUP);

    Motor_Stop(motor);
  }

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), RightEncoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), LeftEncoderISR_A, CHANGE);
}


void Motor_UpdateSpeed(DC_Motor* motor) {
  long currentCount = motor->encoderCount;
  long deltaCount = currentCount - motor->lastEncoderCount;
  motor->lastEncoderCount = currentCount;

  float dt = controlPeriod_ms / 1000.0;

  float rev = (deltaCount * motor->encoderSign) / countsPerRev;
  float distance_in = rev * wheelCircumference_in;

  motor->rawSpeed_in_s = distance_in / dt;

  motor->filteredSpeed_in_s =
    alpha * motor->rawSpeed_in_s + (1.0 - alpha) * motor->filteredSpeed_in_s;
}


void Motor_SpeedControl(DC_Motor* motor, float targetSpeed) {
  float dt = controlPeriod_ms / 1000.0;

  if (abs(targetSpeed) <= 0.05) {
    motor->u_pwm = 0;
    motor->integral = 0.0;
    motor->lastError = 0.0;
    motor->error = 0.0;
    return;
  }

  int directionSign;

  if (targetSpeed > 0) {
    directionSign = 1;
  } else {
    directionSign = -1;
  }

  float targetMag = abs(targetSpeed);
  float speedMag  = abs(motor->filteredSpeed_in_s);

  motor->error = targetMag - speedMag;

  motor->integral += motor->error * dt;
  motor->integral = constrain(motor->integral, -15.0, 15.0);

  float derivative = (motor->error - motor->lastError) / dt;

  float pidOutput = Kp * motor->error + Ki * motor->integral + Kd * derivative;

  int motorBasePWM;

  if (motor == &Motor1) {
    motorBasePWM = rightBasePWM;
  } else {
    motorBasePWM = leftBasePWM;
  }

  float pwmMagnitude = motorBasePWM + pidOutput;
  pwmMagnitude = constrain(pwmMagnitude, minPWM, maxPWM);

  motor->u_pwm = directionSign * (int)pwmMagnitude;

  motor->lastError = motor->error;
}


void Motor_Go(DC_Motor* motor) {
  if (motor->u_pwm == 0) {
    Motor_Stop(motor);
    return;
  }

  int pwmValue = abs(motor->u_pwm);

  if (motor->u_pwm > 0) {
    digitalWrite(motor->IN1, LOW);
    digitalWrite(motor->IN2, HIGH);
  }
  else {
    digitalWrite(motor->IN1, HIGH);
    digitalWrite(motor->IN2, LOW);
  }

  analogWrite(motor->PWM, pwmValue);
}


void Motor_Stop(DC_Motor* motor) {
  analogWrite(motor->PWM, 0);
  digitalWrite(motor->IN1, LOW);
  digitalWrite(motor->IN2, LOW);
}


void ResetMotorPID() {
  Motor1.integral = 0.0;
  Motor1.lastError = 0.0;
  Motor1.error = 0.0;

  Motor2.integral = 0.0;
  Motor2.lastError = 0.0;
  Motor2.error = 0.0;
}


void RightEncoderISR_A() {
  bool A = digitalRead(RIGHT_ENCODER_A);
  bool B = digitalRead(RIGHT_ENCODER_B);

  if (A == B) {
    Motor1.encoderCount++;
  } else {
    Motor1.encoderCount--;
  }
}


void LeftEncoderISR_A() {
  bool A = digitalRead(LEFT_ENCODER_A);
  bool B = digitalRead(LEFT_ENCODER_B);

  if (A == B) {
    Motor2.encoderCount++;
  } else {
    Motor2.encoderCount--;
  }
}