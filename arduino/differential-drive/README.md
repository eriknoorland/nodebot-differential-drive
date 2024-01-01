# Teensy Motor Controller
Firmware to manage two Pololu TB9051FTG motor drivers with a Teensy 4.0 microcontroller communicating over USB using COBS encoding.

## Packet Descriptions

### Request Packet Format

| Start Flag | Command | Payload Size |
|------------|---------|--------------|
| 1 byte     | 1 byte  | x bytes      |

### Response Packet Format

| Start Flag 1 | Start Flag 2 | Command | Response Data Length | Response |
|--------------|--------------|---------|----------------------|----------|
| `0xA3`       | `0x3A`       | 1 byte  | 1 byte               | x bytes  |

### Requests Overview

| Request              | Value  | Payload                                                         |
|----------------------|--------|-----------------------------------------------------------------|
| SET_MOTOR_DIRECTIONS | `0x10` | left motor direction (1 byte), right motor direction (1 byte)   |
| SET_MOTOR_SPEEDS     | `0x11` | left motor speed (1 byte), right motor speed (1 byte)           |
| SET_MOTOR_PWM        | `0x12` | left motor PWM value (2 bytes), right motor PWM value (2 bytes) |
| MOTOR_STOP           | `0x13` | N/A                                                             |
| IS_READY             | `0x01` | N/A                                                             |
| DEBUG_LEVEL          | `0x02` | debug (1 bit)                                                   |
| RESET                | `0x03` |                                                                 |

#### Set Motor Directions Request
Request: `0xA3` `0x10` `0x[0, 0, 0, 0, 0, 0, 0, direction]`

Sets the rotation direction of each motor.

#### Set Motor Speeds Request
Request: `0xA3` `0x11` `0x[left motor speed]` `0x[right motor speed]`

Sets the rotation speed of each motor.

#### Set Motor PWM Request
Request: `0xA3` `0x12` `0x[left motor pwm 15:8]` `0x[left motor pwm 7:0]` `0x[right motor pwm 15:8]` `0x[right motor pwm 7:0]`

Sets the PWM value for each motor. The value for each motor has a 13 bit resolution (0 - 8191).

#### Motor Stop Request
Request: `0xA3` `0x13`

Stops the rotation of both motors immediately.

#### Is ready Request
Request: `0xA3` `0x01`

Triggers the ready response to make sure the Teensy is ready for operation. Apparently the Teensy does not reset on USB connection like an Arduino seems to do.

#### Set Debug Level Request
Request: `0xA3` `0x02` `0x[0, 0, 0, 0, 0, 0, 0 , debug]`

Sets the config flags

#### Reset Request
Request: `0xA3` `0x03`

Resets the ready boolean to false to prevent data being sent out when not connected.

### Responses Overview

| Request  | Value  | Payload                                                                                                              |
|----------|--------|----------------------------------------------------------------------------------------------------------------------|
| IS_READY | `0xFF` | N/A                                                                                                                  |
| ODOMETRY | `0x30` | delta left ticks (1 byte), left motor direction (1 byte), delta right ticks (1 byte), right motor direction (1 byte) |
| DEBUG    | `0x35` | delta time (1 byte)                                                                                                  |

#### Ready Response
**Response:** `0xA3` `0x3A` `0xFF` `0x00`

This response will be sent when the Teensy is ready to be controlled.

#### Odometry Response
**Response:** `0xA3` `0x3A` `0x30` `0x04` `0x[delta ticks left]` `0x[0, 0, 0, 0, 0, 0, 0, left motor direction]` `0x[delta ticks right]` `0x[0, 0, 0, 0, 0, 0, 0, right motor direction]`

This response will be sent 50 times per second.

#### Debug Response
**Response:** `0xA3` `0x3A` `0x35` `0x06` `0x[delta time]` `some more bytes here...`

This response will be sent when the debug level flag is set to true.
