import nodebotUtils from '@eriknoorland/nodebot-utils';
import requests from '../requests';
import motorDirections from '../motorDirections';
import calculateMaxSpeed from '../utils/calculateMaxSpeed';
import slope from '../utils/slope';
import { Config, OdometryData, Pose } from '../interfaces';

const { constrain, speedToTickSpeed } = nodebotUtils.math;

export default (config: Config, writeToSerialPort: Function) => {
  return (angle: number, startPose: Pose, startLeftTicks: number, startRightTicks: number, resolve: Function) => {
    const distance = Math.abs((config.WHEEL_BASE / 2) * angle);
    const direction = angle > 0 ? motorDirections.ROTATE_RIGHT : motorDirections.ROTATE_LEFT;
    const { maxSpeed, accelerationDistance } = calculateMaxSpeed(distance, config.MAX_SPEED, config.MIN_SPEED, config.ACCELERATION);
    const decelerationTarget = distance - accelerationDistance;

    let lastLeftTicks = startLeftTicks;
    let lastRightTicks = startRightTicks;
    let leftDistanceTravelled = 0;
    let rightDistanceTravelled = 0;
    let lastLeftDistanceTravelled = 0;
    let lastRightDistanceTravelled = 0;
    let speedSetpointLeft = maxSpeed;
    let speedSetpointRight = maxSpeed;
    let hasPassedDecelerationTargetLeft = false;
    let hasPassedDecelerationTargetRight = false;
    let hasPassedStopTargetLeft = false;
    let hasPassedStopTargetRight = false;
    let leftSpeed = 0;
    let rightSpeed = 0;

    writeToSerialPort([requests.START_FLAG, requests.SET_DIRECTION, ...direction]);

    return ({ leftTicks, rightTicks }: OdometryData, pose: Pose) => {
      const deltaLeftTicks = leftTicks - lastLeftTicks;
      const deltaRightTicks = rightTicks - lastRightTicks;

      lastLeftTicks = leftTicks;
      lastRightTicks = rightTicks;

      leftDistanceTravelled += Math.abs(deltaLeftTicks * config.LEFT_DISTANCE_PER_TICK);
      rightDistanceTravelled += Math.abs(deltaRightTicks * config.RIGHT_DISTANCE_PER_TICK);

      const deltaLeftDistanceTravelled = leftDistanceTravelled - lastLeftDistanceTravelled;
      const deltaRightDistanceTravelled = rightDistanceTravelled - lastRightDistanceTravelled;

      lastLeftDistanceTravelled = leftDistanceTravelled;
      lastRightDistanceTravelled = rightDistanceTravelled;

      leftSpeed = slope(leftSpeed, speedSetpointLeft, config.ACCELERATION_STEP);
      rightSpeed = slope(rightSpeed, speedSetpointRight, config.ACCELERATION_STEP);

      if (!hasPassedDecelerationTargetLeft && leftDistanceTravelled >= decelerationTarget) {
        hasPassedDecelerationTargetLeft = true;
        speedSetpointLeft = config.MIN_SPEED;
      }

      if (!hasPassedDecelerationTargetRight && rightDistanceTravelled >= decelerationTarget) {
        hasPassedDecelerationTargetRight = true;
        speedSetpointRight = config.MIN_SPEED;
      }

      if (!hasPassedStopTargetLeft && leftDistanceTravelled >= distance - deltaLeftDistanceTravelled) {
        hasPassedStopTargetLeft = true;
        speedSetpointLeft = 0;
        leftSpeed = 0;
      }

      if (!hasPassedStopTargetRight && rightDistanceTravelled >= distance - deltaRightDistanceTravelled) {
        hasPassedStopTargetRight = true;
        speedSetpointRight = 0;
        rightSpeed = 0;
      }

      if (hasPassedStopTargetLeft && hasPassedStopTargetRight) {
        speedSetpointLeft = 0;
        leftSpeed = 0;
        speedSetpointRight = 0;
        rightSpeed = 0;
      }

      leftSpeed = constrain(leftSpeed, 0, maxSpeed);
      rightSpeed = constrain(rightSpeed, 0, maxSpeed);

      const leftTickSpeed = speedToTickSpeed(leftSpeed, config.LEFT_DISTANCE_PER_TICK, config.LOOP_TIME);
      const rightTickSpeed = speedToTickSpeed(rightSpeed, config.RIGHT_DISTANCE_PER_TICK, config.LOOP_TIME);

      writeToSerialPort([requests.START_FLAG, requests.SET_SPEED, leftTickSpeed, rightTickSpeed]);

      if (hasPassedStopTargetLeft && hasPassedStopTargetRight) {
        resolve();
      }
    };
  };
};
