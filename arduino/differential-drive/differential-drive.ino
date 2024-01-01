#include "PacketSerial.h"
#include "./src/MotorController.h"

const int MOTOR_LEFT_ENABLE_PIN = 0;
const int MOTOR_LEFT_ENABLEB_PIN = 1;
const int MOTOR_LEFT_PWM1_PIN = 3;
const int MOTOR_LEFT_PWM2_PIN = 2;
const int MOTOR_LEFT_ENCODER_A_PIN = 17;
const int MOTOR_LEFT_ENCODER_B_PIN = 16;

const int MOTOR_RIGHT_ENABLE_PIN = 9;
const int MOTOR_RIGHT_ENABLEB_PIN = 10;
const int MOTOR_RIGHT_PWM1_PIN = 11;
const int MOTOR_RIGHT_PWM2_PIN = 12;
const int MOTOR_RIGHT_ENCODER_A_PIN = 14;
const int MOTOR_RIGHT_ENCODER_B_PIN = 15;

const byte REQUEST_START_FLAG = 0xA3;
const byte REQUEST_IS_READY = 0x01;
const byte REQUEST_DEBUG_LEVEL = 0x02;
const byte REQUEST_RESET = 0x03;
const byte REQUEST_SET_MOTOR_DIRECTIONS = 0x10;
const byte REQUEST_SET_MOTOR_SPEEDS = 0x11;
const byte REQUEST_SET_MOTOR_PWM = 0x12;
const byte REQUEST_MOTOR_STOP = 0x13;

const byte RESPONSE_START_FLAG_1 = 0xA3;
const byte RESPONSE_START_FLAG_2 = 0x3A;
const byte RESPONSE_ODOMETRY = 0x30;
const byte RESPONSE_DEBUG = 0x35;
const byte RESPONSE_READY = 0xFF;

const int pwmResolution = 13;
const int pwmControlLimit = (pow(2, pwmResolution) / 8) - 1; // FIXME remove division by 8 when updating pid value to accomodate new 13 bit resolution
const int loopTime = 20;
unsigned long previousTime = 0;
bool isReady = false;
bool isDebugEnabled = false;

MotorController leftMotorController = MotorController(1, MOTOR_LEFT_ENCODER_A_PIN, MOTOR_LEFT_ENCODER_B_PIN, 3, 90, 0.1);
MotorController rightMotorController = MotorController(-1, MOTOR_RIGHT_ENCODER_A_PIN, MOTOR_RIGHT_ENCODER_B_PIN, 3, 90, 0.1);

PacketSerial serial;

void reset() {
  isReady = false;
}

/**
 * Serial packet received event handler
 * @param {uint8_t} buffer
 * @param {size_t} size
 */
void onPacketReceived(const uint8_t* buffer, size_t size) {
  byte startFlag = buffer[0];
  byte command = buffer[1];

  if (startFlag == REQUEST_START_FLAG) {
    switch (command) {
      case REQUEST_SET_MOTOR_SPEEDS: {
        leftMotorController.setSpeed(buffer[2]);
        rightMotorController.setSpeed(buffer[3]);
        break;
      }

      case REQUEST_SET_MOTOR_PWM: {
        leftMotorController.setPwm((buffer[2] << 8) + buffer[3], buffer[4]);
        rightMotorController.setPwm((buffer[5] << 8) + buffer[6], buffer[7]);
        break;
      }

      case REQUEST_MOTOR_STOP: {
        leftMotorController.stop();
        rightMotorController.stop();
        break;
      }

      case REQUEST_IS_READY: {
        responseReady();
        break;
      }

      case REQUEST_DEBUG_LEVEL: {
        setDebugLevel(buffer[2]);
        break;
      }

      case REQUEST_SET_MOTOR_DIRECTIONS: {
        leftMotorController.setDirection(buffer[2]);
        rightMotorController.setDirection(buffer[3]);
        break;
      }

      case REQUEST_RESET: {
        reset();
        break;
      }
    }
  }
}

/**
 * Send the ready response
 */
void responseReady() {
  uint8_t readyResponse[4] = {
    RESPONSE_START_FLAG_1,
    RESPONSE_START_FLAG_2,
    RESPONSE_READY,
    0x00,
  };

  serial.send(readyResponse, sizeof(readyResponse));
  isReady = true;
}

/**
 * Odometry response handler
 */
void handleOdometryResponse() {
  unsigned int leftMotorTicks = leftMotorController.update() + (1 << 31);
  unsigned int rightMotorTicks = rightMotorController.update() + (1 << 31);

  byte leftMotorTicks1 = leftMotorTicks >> 24;
  byte leftMotorTicks2 = leftMotorTicks >> 16;
  byte leftMotorTicks3 = leftMotorTicks >> 8;
  byte leftMotorTicks4 = leftMotorTicks;

  byte rightMotorTicks1 = rightMotorTicks >> 24;
  byte rightMotorTicks2 = rightMotorTicks >> 16;
  byte rightMotorTicks3 = rightMotorTicks >> 8;
  byte rightMotorTicks4 = rightMotorTicks;

  uint8_t response[12] = {
    RESPONSE_START_FLAG_1,
    RESPONSE_START_FLAG_2,
    RESPONSE_ODOMETRY,
    0x08,
    leftMotorTicks1,
    leftMotorTicks2,
    leftMotorTicks3,
    leftMotorTicks4,
    rightMotorTicks1,
    rightMotorTicks2,
    rightMotorTicks3,
    rightMotorTicks4,
  };

  serial.send(response, sizeof(response));
}

/**
 * Debug response handler
 * @param {int} deltaTime
 */
void handleDebugResponse(int deltaTime) {
  MotorDebugData leftDebugData = leftMotorController.getDebugData();
  MotorDebugData rightDebugData = rightMotorController.getDebugData();

  byte leftSpeedPwmOutputPartMsb = leftDebugData.speedPwmOutput >> 8;
  byte leftSpeedPwmOutputPartLsb = leftDebugData.speedPwmOutput;

  byte rightSpeedPwmOutputPartMsb = rightDebugData.speedPwmOutput >> 8;
  byte rightSpeedPwmOutputPartLsb = rightDebugData.speedPwmOutput;

  uint8_t response[13] = {
    RESPONSE_START_FLAG_1,
    RESPONSE_START_FLAG_2,
    RESPONSE_DEBUG,
    0x09,
    deltaTime,
    leftDebugData.speedSetpoint,
    leftDebugData.speedTicksInput,
    leftSpeedPwmOutputPartMsb,
    leftSpeedPwmOutputPartLsb,
    rightDebugData.speedSetpoint,
    rightDebugData.speedTicksInput,
    rightSpeedPwmOutputPartMsb,
    rightSpeedPwmOutputPartLsb,
  };

  serial.send(response, sizeof(response));
}

/**
 * Set config flags
 * @param {int} debugFlag
 */
void setDebugLevel(int debugFlag) {
  isDebugEnabled = debugFlag == 1;
}

/**
 * Setup
 */
void setup() {
  Serial.begin(115200);

  serial.setStream(&Serial);
  serial.setPacketHandler(&onPacketReceived);

  analogWriteResolution(pwmResolution);
  analogWriteFrequency(MOTOR_LEFT_PWM1_PIN, 18310.55);
  analogWriteFrequency(MOTOR_LEFT_PWM2_PIN, 18310.55);
  analogWriteFrequency(MOTOR_RIGHT_PWM1_PIN, 18310.55);
  analogWriteFrequency(MOTOR_RIGHT_PWM2_PIN, 18310.55);

  leftMotorController.setup(
    MOTOR_LEFT_ENABLE_PIN,
    MOTOR_LEFT_ENABLEB_PIN,
    MOTOR_LEFT_PWM1_PIN,
    MOTOR_LEFT_PWM2_PIN
  );

  leftMotorController.setControlLimits(0, pwmControlLimit);
  leftMotorController.setLoopTime(loopTime);

  rightMotorController.setup(
    MOTOR_RIGHT_ENABLE_PIN,
    MOTOR_RIGHT_ENABLEB_PIN,
    MOTOR_RIGHT_PWM1_PIN,
    MOTOR_RIGHT_PWM2_PIN
  );

  rightMotorController.setControlLimits(0, pwmControlLimit);
  rightMotorController.setLoopTime(loopTime);

  while (!Serial) {}

  responseReady();

  previousTime = millis();
}

/**
 * Loop
 */
void loop() {
  serial.update();

  leftMotorController.loop();
  rightMotorController.loop();

  long now = millis();
  int deltaTime = now - previousTime;

  if (isReady && deltaTime >= loopTime) {
    handleOdometryResponse();

    if (isDebugEnabled) {
      handleDebugResponse(deltaTime);
    }

    previousTime = now;
  }
}