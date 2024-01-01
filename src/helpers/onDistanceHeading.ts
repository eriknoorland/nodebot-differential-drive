import nodebotUtils from '@eriknoorland/nodebot-utils';
import requests from '../requests';
import motorDirections from '../motorDirections';
import calculateMaxSpeed from '../utils/calculateMaxSpeed';
import slope from '../utils/slope';
import { Config, OdometryData, Pose } from '../interfaces';

const { calculateDistance, constrain, speedToTickSpeed } = nodebotUtils.math;

export default (config: Config, writeToSerialPort: Function) => {
  return (distance: number, heading: number, startPose: Pose, resolve: Function) => {
    const absoluteDistance = Math.abs(distance);
    const direction = distance > 0 ? motorDirections.FORWARD : motorDirections.REVERSE;
    const { maxSpeed, accelerationDistance } = calculateMaxSpeed(absoluteDistance, config.MAX_SPEED, config.MIN_SPEED, config.ACCELERATION);
    const decelerationTarget = absoluteDistance - accelerationDistance;
    const KpDirection = distance > 0 ? 1 : -1;

    let speedSetpoint = maxSpeed;
    let hasPassedDecelerationTarget = false;
    let hasPassedStopTarget = false;
    let leftSpeed = 0;
    let rightSpeed = 0;
    let lastHeadingError = 0;
    let iAcc = 0;

    writeToSerialPort([requests.START_FLAG, requests.SET_DIRECTION, ...direction]);

    return (deltaTicks: OdometryData, pose: Pose) => {
      const distanceTravelled = calculateDistance(startPose, pose);
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

      if (!hasPassedDecelerationTarget && distanceTravelled >= decelerationTarget) {
        hasPassedDecelerationTarget = true;
        speedSetpoint = config.MIN_SPEED;
      }

      if (!hasPassedStopTarget && distanceTravelled >= absoluteDistance) {
        hasPassedStopTarget = true;
        speedSetpoint = 0;
        leftSpeed = speedSetpoint;
        rightSpeed = speedSetpoint;
      }

      leftSpeed = constrain(leftSpeed, 0, maxSpeed);
      rightSpeed = constrain(rightSpeed, 0, maxSpeed);

      const leftTickSpeed = speedToTickSpeed(leftSpeed, config.LEFT_DISTANCE_PER_TICK, config.LOOP_TIME);
      const rightTickSpeed = speedToTickSpeed(rightSpeed, config.RIGHT_DISTANCE_PER_TICK, config.LOOP_TIME);

      writeToSerialPort([requests.START_FLAG, requests.SET_SPEED, leftTickSpeed, rightTickSpeed]);

      if (hasPassedStopTarget) {
        resolve();
      }
    };
  };
};
