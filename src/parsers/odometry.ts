import nodebotUtils from '@eriknoorland/nodebot-utils';
import { OdometryData } from '../interfaces';
import { OdometryPacket } from '../types';

const { parseDecToBinary } = nodebotUtils.math;

export default (data: OdometryPacket): OdometryData => {
  const leftTicks = parseInt([
    parseDecToBinary(data[0]),
    parseDecToBinary(data[1]),
    parseDecToBinary(data[2]),
    parseDecToBinary(data[3]),
  ].join(''), 2);

  const rightTicks = parseInt([
    parseDecToBinary(data[4]),
    parseDecToBinary(data[5]),
    parseDecToBinary(data[6]),
    parseDecToBinary(data[7]),
  ].join(''), 2);

  return {
    leftTicks,
    rightTicks,
  };
};
