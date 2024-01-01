#include "Arduino.h"
#include "Encoder.h"
#include "MotorController.h"
#include "PID_v1_bc.h"

MotorController::MotorController(int encoderDirection, int encoderA, int encoderB, float pGain, float iGain, float dGain) : _encoder(encoderA, encoderB), _speedPID(&_speedTicksInput, &_speedPwmOutput, &_speedSetpoint, pGain, iGain, dGain, DIRECT) {
  _speedPID.SetMode(AUTOMATIC);
  _encoderDirection = encoderDirection;
  _numLastTicks = 0;
  _direction = 1;
  _lastError = 0;
  _accumulatedError = 0;
}

void MotorController::setup(int enable, int enableB, int pwm1, int pwm2) {
  _pwm1Pin = pwm1;
  _pwm2Pin = pwm2;

  pinMode(enable, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);

  digitalWrite(enable, HIGH);
  digitalWrite(enableB, LOW);
}

void MotorController::setLoopTime(int loopTime) {
  _speedPID.SetSampleTime(loopTime);
}

void MotorController::setControlLimits(int min, int max) {
  _speedPID.SetOutputLimits(min, max);
  _pwmMinLimit = min;
  _pwmMaxLimit = max;
}

void MotorController::loop() {
  bool pidEnabled = _speedPID.GetMode() == AUTOMATIC;

  if (pidEnabled) {
    if (_speedPID.Compute()) {
      motorControl(round(_speedPwmOutput * 8), _direction); // FIXME remove multiply by 8 when updating pid value to accomodate new 13 bit resolution
    }

    return;
  }

  motorControl(0, 0);
}

void MotorController::motorControl(int pwm, int direction) {
  analogWrite(_pwm1Pin, direction == 1 ? pwm : 0);
  analogWrite(_pwm2Pin, direction == -1 ? pwm : 0);
}

void MotorController::setDirection(int direction) {
  _direction = direction == 1 ? direction : -1;
}

MotorDebugData MotorController::getDebugData() {
  MotorDebugData debugData = {
    _speedSetpoint,
    _speedTicksInput,
    _speedPwmOutput,
  };

  return debugData;
}

void MotorController::setSpeed(int speed) {
  _speedPID.SetMode(AUTOMATIC);
  _speedSetpoint = speed;
}

void MotorController::setPwm(int pwm, int direction) {
  int contrainedPwm = constrain(pwm, _pwmMinLimit, _pwmMaxLimit);
  _speedPID.SetMode(MANUAL);

  motorControl(contrainedPwm, direction == 1 ? direction : -1);
}

void MotorController::stop() {
  _speedPID.SetMode(MANUAL);
  _speedSetpoint = 0;
  _speedPwmOutput = 0;
}

int MotorController::update() {
  unsigned int numTicks = _encoder.read() * _encoderDirection;
  int deltaTicks = numTicks - _numLastTicks;

  _speedTicksInput = abs(deltaTicks);
  _numLastTicks = numTicks;

  return numTicks;
}
