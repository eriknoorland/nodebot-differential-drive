import EventEmitter from 'events';
import { SerialPort } from 'serialport';
import icpjs from '@eriknoorland/icpjs';
import nodebotUtils from '@eriknoorland/nodebot-utils';
import Parser from './Parser';
import requests from './requests';
import makeOnDistanceCalibrationTest from './helpers/onDistanceCalibrationTest';
import makeOnSpeedHeading from './helpers/onSpeedHeading';
import makeOnDistanceHeading from './helpers/onDistanceHeading';
import makeOnRotate from './helpers/onRotate';
import makeOnSoftStop from './helpers/onSoftStop';
import makeEstimatePoseOdom from './utils/estimatePoseOdom';
import makeEstimatePoseIMU from './utils/estimatePoseIMU';
import { Config, Coordinate, OdometryData, Options, PIDGain, Pose } from './interfaces';
import { LidarMeasurement } from './types';

const cobs = require('cobs');

const {
  deg2rad,
  speedToTickSpeed,
  calculateDistance,
  getHeadingFromPoseToCoordinate,
  fixedDecimals,
} = nodebotUtils.math;

export default (path: string, config: Config, options: Options = {}) => {
  const eventEmitter = new EventEmitter();
  const hasIMU = !!options.imu;
  const useIMU = hasIMU && options.useIMU;
  const hasLidar = !!options.lidar;
  const useICP = hasLidar && options.icpReference && options.useICP;
  const onDistanceCalibrationTest = makeOnDistanceCalibrationTest(config, writeToSerialPort);
  const onSpeedHeading = makeOnSpeedHeading(config, writeToSerialPort);
  const onDistanceHeading = makeOnDistanceHeading(config, writeToSerialPort);
  const onRotate = makeOnRotate(config, writeToSerialPort);
  const onSoftStop = makeOnSoftStop(config, writeToSerialPort);
  const estimatePoseOdom = makeEstimatePoseOdom(config, fixedDecimals);
  const estimatePoseIMU = makeEstimatePoseIMU(config, fixedDecimals);
  const requiredConfigProps = [
    'LOOP_TIME',
    'WHEEL_BASE',
    'LEFT_DISTANCE_PER_TICK',
    'RIGHT_DISTANCE_PER_TICK',
    'ACCELERATION_STEP',
    'ACCELERATION',
    'MIN_SPEED',
    'MAX_SPEED',
    'MAX_ROTATION_SPEED',
    'HEADING_KP',
    'HEADING_KI',
    'HEADING_KD',
  ];

  const lidarData: LidarMeasurement = {};
  let lastPoseOdom: Pose = { x: 0, y: 0, phi: 0 };
  let lastPoseIMU: Pose = { x: 0, y: 0, phi: 0 };
  let trackPose = false;
  let lastLeftTicks: number | null = null;
  let lastRightTicks: number | null = null;
  let lastHeading: number | null = null;
  let currentCommand: Function | null;
  let port: SerialPort;
  let parser;

  function constructor() {
    const isConfigComplete = requiredConfigProps.every(prop => config[prop as keyof Config] !== undefined);

    if (!isConfigComplete) {
      const missingProps = requiredConfigProps.filter(prop => config[prop as keyof Config] === undefined);

      throw new Error(`Config is not complete, missing: ${missingProps.join(', ')}`);
    }
  }

  function init(): Promise<void> {
    return new Promise((resolve, reject) => {
      if (port) {
        setTimeout(reject, 0);
        return;
      }

      let isTeensyReady = false;

      const isReadyTimeout = setTimeout(() => {
        if (!isTeensyReady) {
          writeToSerialPort([requests.START_FLAG, requests.IS_READY]);
        }
      }, 1000);

      if (useIMU && options.imu && options.imu.on) {
        options.imu.on('data', onIMUData);
      }

      if (useICP && options.lidar && options.lidar.on) {
        options.lidar.on('data', onLidarData);
      }

      port = new SerialPort({ path, baudRate: 115200 });
      parser = new Parser();

      port.pipe(parser);

      port.on('error', error => eventEmitter.emit('error', error));
      port.on('disconnect', () => eventEmitter.emit('disconnect'));
      port.on('close', () => eventEmitter.emit('close'));
      port.on('open', onPortOpen);

      parser.on('odometry', onOdometryData);
      parser.on('debug', data => eventEmitter.emit('debug', data));
      parser.on('ready', () => {
        clearTimeout(isReadyTimeout);
        isTeensyReady = true;
        resolve();
      });
    });
  }

  function isReady(): Promise<void> {
    writeToSerialPort([requests.START_FLAG, requests.IS_READY]);

    return Promise.resolve();
  }

  function setPIDGains(left: PIDGain, right: PIDGain) {
    // https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Float32Array
    // https://stackoverflow.com/questions/29629597/how-to-convert-float-into-byte-array
  }

  function setTrackPose(isEnabled: boolean): Promise<void> {
    trackPose = isEnabled;

    return Promise.resolve();
  }

  function setDebugLevel(debug = 0): Promise<void> {
    writeToSerialPort([requests.START_FLAG, requests.SET_DEBUG_LEVEL, debug]);

    return Promise.resolve();
  }

  function getPose(): Pose {
    if (useIMU){
      return lastPoseIMU;
    }

    return lastPoseOdom;
  }

  function appendPose(pose: Pose) {
    lastPoseOdom = pose;
    lastPoseIMU = pose;

    emitPose(pose);
  }

  function distanceCalibrationTest(distance: number): Promise<unknown> {
    const promise = new Promise(resolve => {
      currentCommand = onDistanceCalibrationTest(distance, resolve);
    });

    promise.then(resetCurrentCommand);

    return promise;
  }

  function speedLeftRight(speedLeft: number, speedRight: number) {
    const tickSpeedLeft = speedToTickSpeed(Math.abs(speedLeft), config.LEFT_DISTANCE_PER_TICK, config.LOOP_TIME);
    const tickSpeedRight = speedToTickSpeed(Math.abs(speedRight), config.RIGHT_DISTANCE_PER_TICK, config.LOOP_TIME);
    const directionLeft = speedLeft > 0 ? 1 : 0;
    const directionRight = speedRight > 0 ? 0 : 1;

    writeToSerialPort([requests.START_FLAG, requests.SET_DIRECTION, directionLeft, directionRight]);
    writeToSerialPort([requests.START_FLAG, requests.SET_SPEED, tickSpeedLeft, tickSpeedRight]);
  }

  function speedHeading(speed: number, heading: number, callback = () => {}): Promise<unknown> {
    const promise = new Promise(resolve => {
      currentCommand = onSpeedHeading(speed, heading, callback, resolve);
    });

    promise.then(resetCurrentCommand);

    return promise;
  }

  function distanceHeading(distance: number, heading: number): Promise<unknown> {
    const promise = new Promise(resolve => {
      currentCommand = onDistanceHeading(distance, heading, getPose(), resolve);
    });

    promise.then(resetCurrentCommand);

    return promise;
  }

  function rotate(angle: number): Promise<unknown> {
    const promise = new Promise(resolve => {
      currentCommand = onRotate(angle, getPose(), lastLeftTicks || 0, lastRightTicks || 0, resolve);
    });

    promise.then(resetCurrentCommand);

    return promise;
  }

  function stop(hardStop = false): Promise<unknown> {
    if (hardStop) {
      writeToSerialPort([requests.START_FLAG, requests.STOP]);

      return Promise.resolve();
    }

    const promise = new Promise(resolve => {
      currentCommand = onSoftStop(lastLeftTicks || 0, lastRightTicks || 0, resolve);
    });

    promise.then(resetCurrentCommand);

    return promise;
  }

  async function move2XY(coordinate: Coordinate, distanceOffset = 0): Promise<void> {
    const currentPose = getPose();
    const distance = calculateDistance(currentPose, coordinate) + distanceOffset;
    const heading = getHeadingFromPoseToCoordinate(currentPose, coordinate);

    await rotate(heading);
    await distanceHeading(distance, heading + currentPose.phi);

    return Promise.resolve();
  }

  async function move2XYPhi(coordinate: Coordinate, desiredHeading: number): Promise<void> {
    await move2XY(coordinate);

    const currentPose = getPose();
    const heading = desiredHeading - currentPose.phi;

    await rotate(heading);

    return Promise.resolve();
  }

  async function emergencyStop(): Promise<unknown> {
    resetCurrentCommand();

    return stop();
  }

  function close(): Promise<void> {
    return new Promise(resolve => {
      writeToSerialPort([requests.START_FLAG, 0x03]);

      port.flush(() => {
        port.close(() => {
          resolve();
        });
      });
    });
  }

  function onIMUData(heading: number) {
    lastHeading = deg2rad(heading);
  }

  function onLidarData({ angle, distance }: { angle: number, distance: number, quality: number }) {
    if (distance) {
      const index = Math.round(angle) % 360;

      lidarData[index] = distance;
    }
  }

  function onOdometryData(data: OdometryData) {
    const { leftTicks, rightTicks } = data;

    lastLeftTicks = lastLeftTicks || leftTicks;
    lastRightTicks = lastRightTicks || rightTicks;

    const deltaLeftTicks = leftTicks - lastLeftTicks;
    const deltaRightTicks = rightTicks - lastRightTicks;
    const poseOdom = estimatePoseOdom(lastPoseOdom, deltaLeftTicks, deltaRightTicks);
    const poseIMU = estimatePoseIMU(lastPoseIMU, deltaLeftTicks, deltaRightTicks, lastHeading || 0);
    let pose = useIMU ? poseIMU : poseOdom;

    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;

    if (useICP) {
      const { x, y, phi } = pose;
      const lidarPoints = Object
        .keys(lidarData)
        .map(angle => {
          const distance = lidarData[angle];
          const angleInRadians = deg2rad(parseInt(angle, 10));

          return {
            x: x + (Math.cos(phi + angleInRadians) * distance),
            y: y + (Math.sin(phi + angleInRadians) * distance),
          };
        });

      const { transformation } = icpjs.run(options.icpReference!, lidarPoints, pose, options.icpOptions);

      pose = {
        x: x + transformation.x,
        y: y + transformation.y,
        phi: phi + transformation.phi,
      };
    }

    const lastPose = useIMU ? lastPoseIMU : lastPoseOdom;
    const hasPoseChanged = JSON.stringify(pose) !== JSON.stringify(lastPose);

    if (hasPoseChanged) {
      lastPoseOdom = poseOdom;
      lastPoseIMU = poseIMU;

      emitPose(pose);
    }

    if (currentCommand) {
      currentCommand(data, pose);
    }

    eventEmitter.emit('odometry', data);
  }

  function emitPose(pose: Pose) {
    if (trackPose) {
      eventEmitter.emit('pose', pose);
      eventEmitter.emit('pose_odom', lastPoseOdom);

      if (hasIMU) {
        eventEmitter.emit('pose_imu', lastPoseIMU);
      }
    }
  }

  function writeToSerialPort(data: number[]) {
    port.write(cobs.encode(Buffer.from(data), true));
  }

  function resetCurrentCommand() {
    currentCommand = null;
  }

  function onPortOpen() {
    port.flush(error => {
      if (error) {
        eventEmitter.emit('error', error);
      }
    });
  }

  constructor();

  return Object.freeze({
    on: eventEmitter.on.bind(eventEmitter),
    off: eventEmitter.off.bind(eventEmitter),
    close,
    init,
    isReady,
    setPIDGains,
    setTrackPose,
    setDebugLevel,
    getPose,
    appendPose,
    distanceCalibrationTest,
    speedLeftRight,
    speedHeading,
    distanceHeading,
    rotate,
    stop,
    move2XY,
    move2XYPhi,
    emergencyStop,
  });
};
