import nodebotUtils from '@eriknoorland/nodebot-utils';
import { DebugData } from '../interfaces';
import { DebugPacket } from '../types';

const { parseDecToBinary } = nodebotUtils.math;

export default (data: DebugPacket): DebugData => {
  const loopTime = data[0];

  const left = {
    speedSetpoint: data[1],
    speedTicksInput: data[2],
    speedPwmOutput: parseInt(`${parseDecToBinary(data[3])}${parseDecToBinary(data[4])}`, 2),
  };

  const right = {
    speedSetpoint: data[5],
    speedTicksInput: data[6],
    speedPwmOutput: parseInt(`${parseDecToBinary(data[7])}${parseDecToBinary(data[8])}`, 2),
  };

  return {
    loopTime,
    left,
    right,
  };
};
