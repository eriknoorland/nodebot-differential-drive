import nodebotUtils from '@eriknoorland/nodebot-utils';
import requests from '../requests';
import motorDirections from '../motorDirections';
import slope from '../utils/slope';
import { Config, OdometryData, Pose } from '../interfaces';

const { constrain, speedToTickSpeed } = nodebotUtils.math;

export default (config: Config, writeToSerialPort: Function) => {
  return (speed: number, heading: number, callback: Function, resolve: Function) => {
    const isForward = speed > 0;
    const speedSetpoint = Math.abs(speed) - (10 * (isForward ? 1 : -1));
    const direction = isForward ? motorDirections.FORWARD : motorDirections.REVERSE;
    const KpDirection = isForward ? 1 : -1;

    let leftSpeed = 0;
    let rightSpeed = 0;
    let lastHeadingError = 0;
    let iAcc = 0;

    writeToSerialPort([requests.START_FLAG, requests.SET_DIRECTION, ...direction]);

    return (deltaTicks: OdometryData, pose: Pose) => {
      let headingError = Number((heading - pose.phi).toFixed(6));

      if (headingError > Math.PI) {
        headingError -= Math.PI * 2;
      }

      if (headingError <= -Math.PI) {
        headingError += Math.PI * 2;
      }

      leftSpeed = slope(leftSpeed, speedSetpoint, config.ACCELERATION_STEP);
      rightSpeed = slope(rightSpeed, speedSetpoint, config.ACCELERATION_STEP);

      const p = headingError * config.HEADING_KP;
      const i = (iAcc + (headingError * config.LOOP_TIME)) * config.HEADING_KI;
      const d = ((headingError - lastHeadingError) / config.LOOP_TIME) * config.HEADING_KD;

      lastHeadingError = headingError;
      iAcc = i;

      leftSpeed += Math.round(p + i + d) * KpDirection;
      rightSpeed -= Math.round(p + i + d) * KpDirection;

      leftSpeed = constrain(leftSpeed, 0, Math.abs(speed));
      rightSpeed = constrain(rightSpeed, 0, Math.abs(speed));

      const leftTickSpeed = speedToTickSpeed(leftSpeed, config.LEFT_DISTANCE_PER_TICK, config.LOOP_TIME);
      const rightTickSpeed = speedToTickSpeed(rightSpeed, config.RIGHT_DISTANCE_PER_TICK, config.LOOP_TIME);

      writeToSerialPort([requests.START_FLAG, requests.SET_SPEED, leftTickSpeed, rightTickSpeed]);

      if (callback()) {
        resolve();
      }
    };
  };
};
