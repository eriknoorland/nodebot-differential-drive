import nodebotUtils from '@eriknoorland/nodebot-utils';
import requests from '../requests';
import slope from '../utils/slope';
import { Config, OdometryData } from '../interfaces';

const { tickSpeedToSpeed, speedToTickSpeed } = nodebotUtils.math;

export default (config: Config, writeToSerialPort: Function) => {
  return (startLeftTicks: number, startRightTicks: number, resolve: Function) => {
    let lastLeftTicks = startLeftTicks;
    let lastRightTicks = startRightTicks;
    let leftSpeed = 0;
    let rightSpeed = 0;

    return ({ leftTicks, rightTicks }: OdometryData) => {
      const deltaLeftTicks = leftTicks - lastLeftTicks;
      const deltaRightTicks = rightTicks - lastRightTicks;

      lastLeftTicks = leftTicks;
      lastRightTicks = rightTicks;

      if (!leftSpeed) {
        leftSpeed = Math.abs(tickSpeedToSpeed(deltaLeftTicks, config.LEFT_DISTANCE_PER_TICK, config.LOOP_TIME));
        rightSpeed = Math.abs(tickSpeedToSpeed(deltaRightTicks, config.RIGHT_DISTANCE_PER_TICK, config.LOOP_TIME));
      }

      leftSpeed = slope(leftSpeed, 0, config.ACCELERATION_STEP);
      rightSpeed = slope(rightSpeed, 0, config.ACCELERATION_STEP);

      const leftTickSpeed = speedToTickSpeed(leftSpeed, config.LEFT_DISTANCE_PER_TICK, config.LOOP_TIME);
      const rightTickSpeed = speedToTickSpeed(rightSpeed, config.RIGHT_DISTANCE_PER_TICK, config.LOOP_TIME);

      writeToSerialPort([requests.START_FLAG, requests.SET_SPEED, leftTickSpeed, rightTickSpeed]);

      if (!leftTickSpeed && !rightTickSpeed) {
        resolve();
      }
    };
  };
};
