import calculateAccelerationDistance from './calculateAccelerationDistance';

const calculateMaxSpeed = (distance: number, maxSpeed: number, minSpeed: number, acceleration: number): { maxSpeed: number, accelerationDistance: number } => {
  if (distance <= minSpeed * 2) {
    return {
      maxSpeed: minSpeed,
      accelerationDistance: 0,
    };
  }

  const accelerationDistance = calculateAccelerationDistance(maxSpeed, 0, acceleration);

  if (accelerationDistance * 2 > distance) {
    return calculateMaxSpeed(distance, maxSpeed - (minSpeed / 2), minSpeed, acceleration);
  }

  return {
    maxSpeed,
    accelerationDistance,
  };
};

export default calculateMaxSpeed;