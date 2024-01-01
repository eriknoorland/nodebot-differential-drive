import nodebotUtils from '@eriknoorland/nodebot-utils';
import requests from '../requests';
import motorDirections from '../motorDirections';
import calculateMaxSpeed from '../utils/calculateMaxSpeed';
import slope from '../utils/slope';
import { Config, OdometryData, Pose } from '../interfaces';

export default (config: Config, writeToSerialPort: Function) => {
  return (distance: number, resolve: Function) => {
    const absoluteDistance = Math.abs(distance);
    const direction = distance > 0 ? motorDirections.FORWARD : motorDirections.REVERSE;
    const { maxSpeed, accelerationDistance } = calculateMaxSpeed(absoluteDistance, config.MAX_SPEED, config.MIN_SPEED, config.ACCELERATION);
    const decelerationTarget = absoluteDistance - accelerationDistance;

    let totalLeftTicks = 0;
    let totalRightTicks = 0;
    let speedSetpoint = maxSpeed;
    let hasPassedDecelerationTarget = false;
    let hasPassedStopTarget = false;
    let leftSpeed = 0;
    let rightSpeed = 0;

    writeToSerialPort([requests.START_FLAG, requests.SET_DIRECTION, ...direction]);

    return ({ leftTicks, rightTicks }: OdometryData, pose: Pose) => {
      totalLeftTicks += leftTicks;
      totalRightTicks += rightTicks;

      const leftDistanceTravelled = totalLeftTicks * config.LEFT_DISTANCE_PER_TICK;
      const rightDistanceTravelled = totalRightTicks * config.RIGHT_DISTANCE_PER_TICK;
      const distanceTravelled = (leftDistanceTravelled + rightDistanceTravelled) / 2;

      leftSpeed = slope(leftSpeed, speedSetpoint, config.ACCELERATION_STEP);
      rightSpeed = slope(rightSpeed, speedSetpoint, config.ACCELERATION_STEP);

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

      leftSpeed = nodebotUtils.math.constrain(leftSpeed, 0, maxSpeed);
      rightSpeed = nodebotUtils.math.constrain(rightSpeed, 0, maxSpeed);

      const leftTickSpeed = nodebotUtils.math.speedToTickSpeed(leftSpeed, config.LEFT_DISTANCE_PER_TICK, config.LOOP_TIME);
      const rightTickSpeed = nodebotUtils.math.speedToTickSpeed(rightSpeed, config.RIGHT_DISTANCE_PER_TICK, config.LOOP_TIME);

      writeToSerialPort([requests.START_FLAG, requests.SET_SPEED, leftTickSpeed, rightTickSpeed]);

      if (hasPassedStopTarget) {
        resolve();
      }
    };
  };
};
