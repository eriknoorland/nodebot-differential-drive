const MotionController = require('../dist/index.cjs');
const config = require('./config');
const motionController = MotionController('/dev/tty.usbmodem62586801', config);

async function init() {
  motionController.on('odometry', data => {
    const { leftTicks, rightTicks } = data;
    const deltaLeftRight = leftTicks - rightTicks;
    // console.log(data);
    console.log({ deltaLeftRight });
  });

  await motionController.init();
  await motionController.distanceHeading(500, 0);
  await motionController.close();
}

init();
