import { Config, Pose } from '../interfaces';

export default (config: Config, fixedDecimals: Function) => (lastPose: Pose, deltaLeftTicks: number, deltaRightTicks: number): Pose => {
  const distanceLeft = deltaLeftTicks * config.LEFT_DISTANCE_PER_TICK;
  const distanceRight = deltaRightTicks * config.RIGHT_DISTANCE_PER_TICK;
  const distanceCenter = (distanceLeft + distanceRight) / 2;
  const x = fixedDecimals(lastPose.x + (distanceCenter * Math.cos(lastPose.phi)), 4);
  const y = fixedDecimals(lastPose.y + (distanceCenter * Math.sin(lastPose.phi)), 4);
  const phi = Number((lastPose.phi - ((distanceRight - distanceLeft) / config.WHEEL_BASE)).toFixed(4));
  const normalizedPhi = Math.atan2(Math.sin(phi), Math.cos(phi)); // keep phi between -π and π

  return { x, y, phi: normalizedPhi };
};