import { Transform } from 'stream';
import odometryParser from './parsers/odometry';
import debugParser from './parsers/debug';
import { OdometryPacket, DebugPacket } from './types';

const cobs = require('cobs');
const numDescriptorBytes = 4;

class Parser extends Transform {
  private startFlags: Buffer;
  private buffer: Buffer;
  
  constructor() {
    super();

    this.startFlags = Buffer.from([0xA3, 0x3A]);
    this.buffer = Buffer.alloc(0);
  }

  _transform(chunk: Buffer, encoding: string, callback: Function) {
    this.buffer = Buffer.concat([this.buffer, chunk]);

    for (let j = 0; j < this.buffer.length; j++) {
      if (this.buffer.indexOf(this.startFlags, 0, 'hex') !== -1) {
        const packetStart = this.buffer.indexOf(this.startFlags, 0, 'hex') - 1;

        if (this.buffer.length > packetStart + numDescriptorBytes) {
          const command = this.buffer[packetStart + 3];
          const dataLength = this.buffer[packetStart + 4];
          const packetEnd = packetStart + numDescriptorBytes + dataLength + 1;

          if (this.buffer.length > packetEnd) {
            const packet = this.buffer.slice(packetStart, packetEnd);
            const decodedPacket: Buffer = cobs.decode(packet);
            const packetData: number[] = [];

            this.buffer = this.buffer.slice(packetEnd);
            j = 0;

            for (let i = 0; i < dataLength; i++) {
              const index = numDescriptorBytes + i;
              packetData.push(decodedPacket[index]);
            }

            switch(command) {
              case 0xFF:
                this.emit('ready');
                break;

              case 0x30:
                this.emit('odometry', odometryParser(packetData as OdometryPacket));
                break;

              case 0x35:
                this.emit('debug', debugParser(packetData as DebugPacket));
                break;
            }
          }
        }
      }
    }

    callback();
  }
}

export default Parser;
