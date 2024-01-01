 export default (input: number, setpoint: number, step: number) => {
  if (input < setpoint) {
    return Math.min(input += step, setpoint);
  }

  return Math.max(input -= step, setpoint);
};