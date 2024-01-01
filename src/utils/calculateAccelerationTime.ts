export default (targetSpeed: number, currentSpeed: number, acceleration: number) => {
  const extraTimeOffset = 0.1;
  const time = ((targetSpeed - currentSpeed) / acceleration);

  return time + extraTimeOffset;
};