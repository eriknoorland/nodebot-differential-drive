const LOOP_TIME = 20; // ms
const MOTOR_ENCODER_CPR = 64;
const MOTOR_GEAR_RATIO = 29;
const NUM_TICKS_PER_REVOLUTION = MOTOR_GEAR_RATIO * MOTOR_ENCODER_CPR;
const WHEEL_BASE = 190.825; // mm
const BASE_CIRCUMFERENCE = Math.PI * WHEEL_BASE;
const BASE_WHEEL_DIAMETER = 61.45;
const WHEEL_DIAMETER_DIFF_PERCENTAGE = 0; // %
const LEFT_WHEEL_DIAMETER = BASE_WHEEL_DIAMETER - (BASE_WHEEL_DIAMETER * WHEEL_DIAMETER_DIFF_PERCENTAGE / 100); // mm
const LEFT_WHEEL_CIRCUMFERENCE = Math.PI * LEFT_WHEEL_DIAMETER; // mm
const LEFT_DISTANCE_PER_TICK = LEFT_WHEEL_CIRCUMFERENCE / NUM_TICKS_PER_REVOLUTION; // mm
const RIGHT_WHEEL_DIAMETER = BASE_WHEEL_DIAMETER + (BASE_WHEEL_DIAMETER * WHEEL_DIAMETER_DIFF_PERCENTAGE / 100); // mm
const RIGHT_WHEEL_CIRCUMFERENCE = Math.PI * RIGHT_WHEEL_DIAMETER; // mm
const RIGHT_DISTANCE_PER_TICK = RIGHT_WHEEL_CIRCUMFERENCE / NUM_TICKS_PER_REVOLUTION; // mm
const ACCELERATION_STEP = 3; // mm/looptime
const ACCELERATION = ACCELERATION_STEP * (1000 / LOOP_TIME); // mm/s
const MIN_SPEED = 50; // mm/s
const MAX_SPEED = 350; // mm/s
const MAX_ROTATION_SPEED = MAX_SPEED / 2; // mm/s
const HEADING_KP = 22.5;
const HEADING_KI = 0.45;
const HEADING_KD = 0;

module.exports = {
  LOOP_TIME,
  MOTOR_ENCODER_CPR,
  MOTOR_GEAR_RATIO,
  NUM_TICKS_PER_REVOLUTION,
  WHEEL_BASE,
  BASE_CIRCUMFERENCE,
  LEFT_WHEEL_DIAMETER,
  LEFT_WHEEL_CIRCUMFERENCE,
  LEFT_DISTANCE_PER_TICK,
  RIGHT_WHEEL_DIAMETER,
  RIGHT_WHEEL_CIRCUMFERENCE,
  RIGHT_DISTANCE_PER_TICK,
  ACCELERATION_STEP,
  ACCELERATION,
  MIN_SPEED,
  MAX_SPEED,
  MAX_ROTATION_SPEED,
  HEADING_KP,
  HEADING_KI,
  HEADING_KD,
};
