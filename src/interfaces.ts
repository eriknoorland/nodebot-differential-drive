export interface Config {
  LOOP_TIME: number,
  WHEEL_BASE: number,
  LEFT_DISTANCE_PER_TICK: number,
  RIGHT_DISTANCE_PER_TICK: number,
  ACCELERATION_STEP: number,
  ACCELERATION: number,
  MIN_SPEED: number,
  MAX_SPEED: number,
  MAX_ROTATION_SPEED: number,
  HEADING_KP: number,
  HEADING_KI: number,
  HEADING_KD: number,
};

export interface Coordinate {
  x: number;
  y: number;
};

export interface DebugData {
  loopTime: number;
  left: {
    speedSetpoint: number;
    speedTicksInput: number;
    speedPwmOutput: number;
  }
  right: {
    speedSetpoint: number;
    speedTicksInput: number;
    speedPwmOutput: number;
  };
};

export interface Device {
  init?: Function;
  close?: Function;
  isReady?: Function;
  on?: Function;
  off?: Function;
};

export interface Options {
  imu?: Device;
  useIMU?: boolean;
  lidar?: Device;
  useICP?: boolean;
  icpReference?: Array<any>;
  icpOptions?: Object;
};

export interface OdometryData {
  leftTicks: number;
  rightTicks: number;
};

export interface PIDGain {
  Kp: number;
  Ki: number;
  Kd: number;
};

export interface Pose {
  x: number;
  y: number;
  phi: number;
};