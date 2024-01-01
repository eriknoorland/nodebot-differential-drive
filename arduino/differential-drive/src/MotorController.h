#ifndef MotorController_h
#define MotorController_h

#define ENCODER_OPTIMIZE_INTERRUPTS

#include "Arduino.h"
#include "Encoder.h"
#include "PID_v1_bc.h"

struct MotorDebugData {
  int speedSetpoint;
  int speedTicksInput;
  int speedPwmOutput;
  unsigned int totalTicks;
};

class MotorController {
  public:
    MotorController(int encoderDirection, int encoderA, int encoderB, float pGain, float iGain, float dGain);
    void setup(int enable, int enableB, int pwm1, int pwm2);
    void setLoopTime(int loopTime);
    void setControlLimits(int min, int max);
    void loop();
    void setDirection(int direction);
    void setSpeed(int speed);
    void setPwm(int pwm, int direction);
    void stop();
    int update();
    MotorDebugData getDebugData();

  private:
    Encoder _encoder;
    PID _speedPID;
    double _speedTicksInput;
    double _speedPwmOutput;
    double _speedSetpoint;

    int _encoderDirection;
    int _pwm1Pin;
    int _pwm2Pin;
    int _pwmMinLimit = 0;
    int _pwmMaxLimit = 255;
    int _direction;
    float _lastError;
    float _accumulatedError;
    unsigned long _numLastTicks;
    void motorControl(int pwm1, int pwm2);
};

#endif
