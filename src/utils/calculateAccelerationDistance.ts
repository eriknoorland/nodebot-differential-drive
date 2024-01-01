import calculateAccelerationTime from './calculateAccelerationTime';

export default (targetSpeed: number, currentSpeed: number, acceleration: number) => {
  const time = calculateAccelerationTime(targetSpeed, currentSpeed, acceleration);
  const distance = Math.ceil(0.5 * (acceleration * Math.pow(time, 2)));

  return distance;
};