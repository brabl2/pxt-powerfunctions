/**
 * Power Functions IR Sender
 * Control your Power Functions motors using your micro:bit or Calliope-Mini, an infrared LED and MakeCode.
 *
 * (c) 2017-2020, Philipp Henkel
 */

enum PowerFunctionSpeedZero {
  //% block="float (default)"
  speed_0_float = 0,
  //% block="brake"
  speed_0_brake = 1,
}
enum PowerFunctionSendDelay {
  //% block="normal (default)"
  delay_normal = 1,
  //% block="short (16ms)"
  delay_short = 0,
}
enum PowerFunctionSendCount {
  //% block="5x (default)"
  five_times = 5,
  //% block="4x"
  four_times = 4,
  //% block="3x"
  three_times = 3,
  //% block="2x"
  twice = 2,
  //% block="1x"
  once = 1,
}
enum PowerFunctionsChannel {
  //% block="1"
  One = 0,
  //% block="2"
  Two = 1,
  //% block="3"
  Three = 2,
  //% block="4"
  Four = 3,
}

enum PowerFunctionsDirection {
  //% block="left"
  Left = 1,
  //% block="right"
  Right = -1,
}

enum PowerFunctionsOutput {
  //% block="red"
  Red = 0,
  //% block="blue"
  Blue = 1,
}

enum PowerFunctionsMotor {
  //% block="red | channel 1"
  Red1 = 0,
  //% block="red | channel 2"
  Red2 = 1,
  //% block="red | channel 3"
  Red3 = 2,
  //% block="red | channel 4"
  Red4 = 3,
  //% block="blue | channel 1"
  Blue1 = 4,
  //% block="blue | channel 2"
  Blue2 = 5,
  //% block="blue | channel 3"
  Blue3 = 6,
  //% block="blue | channel 4"
  Blue4 = 7,
}

enum PowerFunctionsCommand {
  //% block="float"
  Float = 0,
  //% block="forward"
  Forward = 1,
  //% block="backward"
  Backward = 2,
  //% block="brake"
  Brake = 3,
}

//% weight=99 color=#0fbc11 icon="\uf0e4" block="Power Functions"
namespace powerfunctions {
  interface PowerFunctionsState {
    irDevice: InfraredDevice;
    messageToggle : number;
    sendCount: PowerFunctionSendCount;
    sendDelay: PowerFunctionSendDelay;
    motorDirections: PowerFunctionsDirection[];
    motorSpeedZeros: PowerFunctionSpeedZero[];
  }

  let state: PowerFunctionsState;

  function getChannel(motor: PowerFunctionsMotor): PowerFunctionsChannel {
    const MOTOR_TO_CHANNEL = [
      PowerFunctionsChannel.One,
      PowerFunctionsChannel.Two,
      PowerFunctionsChannel.Three,
      PowerFunctionsChannel.Four,
      PowerFunctionsChannel.One,
      PowerFunctionsChannel.Two,
      PowerFunctionsChannel.Three,
      PowerFunctionsChannel.Four,
    ];
    return MOTOR_TO_CHANNEL[motor];
  }

  function getOutput(motor: PowerFunctionsMotor): PowerFunctionsOutput {
    const MOTOR_TO_OUTPUT = [
      PowerFunctionsOutput.Red,
      PowerFunctionsOutput.Red,
      PowerFunctionsOutput.Red,
      PowerFunctionsOutput.Red,
      PowerFunctionsOutput.Blue,
      PowerFunctionsOutput.Blue,
      PowerFunctionsOutput.Blue,
      PowerFunctionsOutput.Blue,
    ];
    return MOTOR_TO_OUTPUT[motor];
  }

  function sendSingleOutputCommand(
    channel: PowerFunctionsChannel,
    output: PowerFunctionsOutput,
    speed: number
  ) {
    const msg = message.createSingleOutputPwmMessage(channel, output, speed);
    if (state) {
      state.irDevice.sendMessage(msg);
    }
  }

  function sendSingleOutputIncDecCommand(
    channel: PowerFunctionsChannel,
    output: PowerFunctionsOutput,
    value: number
  ) {
    const msg = message.createSingleOutputIncDecMessage(channel, output, value);
    if (state) {
      state.irDevice.sendMessage(msg);
    }
  }

  /**
   * Configures the infrared LED pin. A 940 nm emitting diode is required.
   * @param pin pin an attached IR LED
   */
  //% blockId=pf_connect_ir_led
  //% block="connect IR LED at pin %pin"
  //% weight=90
  //% pin.fieldEditor="gridpicker" pin.fieldOptions.columns=4 pin.fieldOptions.tooltips="false"
  export function connectIrLed(pin: AnalogPin) {
    state = {
      irDevice: new InfraredDevice(pin),
      messageToggle: 0,
      sendCount: PowerFunctionSendCount.five_times,
      sendDelay: PowerFunctionSendDelay.delay_normal,
      motorDirections: [
        PowerFunctionsDirection.Left,
        PowerFunctionsDirection.Left,
        PowerFunctionsDirection.Left,
        PowerFunctionsDirection.Left,
        PowerFunctionsDirection.Left,
        PowerFunctionsDirection.Left,
        PowerFunctionsDirection.Left,
        PowerFunctionsDirection.Left,
      ],
      motorSpeedZeros: [
        PowerFunctionSpeedZero.speed_0_float,
        PowerFunctionSpeedZero.speed_0_float,
        PowerFunctionSpeedZero.speed_0_float,
        PowerFunctionSpeedZero.speed_0_float,
        PowerFunctionSpeedZero.speed_0_float,
        PowerFunctionSpeedZero.speed_0_float,
        PowerFunctionSpeedZero.speed_0_float,
        PowerFunctionSpeedZero.speed_0_float,
      ],
    };
  }

  /**
   * Configures send count and delay (the IR message is transmitted count-times with delay between messages).
   * @param count the send count
   * @param delay the send delay
   */
  //% blockId=pf_cfg_send_count_delay
  //% block="config send count to %count and delay to %delay"
  //% weight=89
  export function cfgSendCountDelay(count: PowerFunctionSendCount, delay: PowerFunctionSendDelay) {
    if (state) {
      state.sendCount = count;
      state.sendDelay = delay;
    }
  }

  /**
   * Configures a motor direction.
   * @param motor the motor
   * @param channel the channel
   * @param direction the direction
   */
  //% blockId=pf_cfg_motor_direction_ch
  //% block="config direction of motor %motor channel %channel to %direction"
  //% channel.min=1 channel.max=4 channel.defl=1
  //% weight=88
  export function cfgMotorDirectionCh(motor: PowerFunctionsOutput, channel: number, direction: PowerFunctionsDirection) {
    channel = Math.max(1, Math.min(4, channel));
    if (state) {
      state.motorDirections[motor * 4 + channel - 1] = direction;
    }
  }

  /**
   * Configures zero speed behaviour of the motor (float or brake).
   * @param motor the motor
   * @param channel the channel
   * @param behaviour the behaviour
   */
  //% blockId=pf_cfg_motor_speed_zero_ch
  //% block="config zero speed of motor %motor channel %channel to %behaviour"
  //% channel.min=1 channel.max=4 channel.defl=1
  //% behaviour.defl=0
  //% weight=87
  export function cfgMotorSpeedZeroCh(motor: PowerFunctionsOutput, channel: number, behaviour: PowerFunctionSpeedZero) {
    channel = Math.max(1, Math.min(4, channel));
    if (state) {
      state.motorSpeedZeros[motor * 4 + channel - 1] = behaviour;
    }
  }

  /**
   * Sets the speed of a motor.
   * @param motor the motor
   * @param channel the channel
   * @param speed speed of the motor
   */
  //% blockId=pf_set_speed_ch
  //% block="set motor %motor channel %channel speed to %speed"
  //% channel.min=1 channel.max=4 channel.defl=1
  //% speed.min=-7 speed.max=7 speed.defl=3
  //% weight=80
  export function setSpeedCh(motor: PowerFunctionsOutput, channel: number, speed: number) {
    channel = Math.max(1, Math.min(4, channel));
    speed = Math.max(-7, Math.min(7, speed));
    if (speed == 0) {
      if (state.motorSpeedZeros[motor * 4 + channel - 1] == PowerFunctionSpeedZero.speed_0_float) {
        speed = 8;
      }
    } else {
      speed = speed * state.motorDirections[motor * 4 + channel - 1];
    }
    if (state) {
      sendSingleOutputCommand(channel - 1, motor, speed);
    }
  }

  /**
   * Brakes then float.
   * The motor's power is quickly reversed and thus the motor will stop abruptly.
   * @param motor the motor
   * @param channel the channel
   */
  //% blockId=pf_brake_ch
  //% block="brake | motor %motor | channel %channel"
  //% channel.min=1 channel.max=4 channel.defl=1
  //% weight=70
  export function brakeCh(motor: PowerFunctionsOutput, channel: number) {
    channel = Math.max(1, Math.min(4, channel));
    if (state) {
      sendSingleOutputCommand(channel - 1, motor, 0);
    }
  }

  /**
   * Floats a motor to stop.
   * The motor's power is switched off and thus the motor will roll to a stop.
   * @param motor the motor
   * @param channel the channel
   */
  //% blockId=pf_float_ch
  //% block="float | motor %motor | channel %channel | to stop"
  //% channel.min=1 channel.max=4 channel.defl=1
  //% weight=60
  export function floatCh(motor: PowerFunctionsOutput, channel: number) {
    channel = Math.max(1, Math.min(4, channel));
    if (state) {
      sendSingleOutputCommand(channel - 1, motor, 8);
    }
  }

  /**
   * Increments motor speed.
   * @param motor the motor
   * @param channel the channel
   */
  //% blockId=pf_inc_ch
  //% block="increment motor %motor channel %channel speed"
  //% channel.min=1 channel.max=4 channel.defl=1
  //% weight=55
  export function incCh(motor: PowerFunctionsOutput, channel: number) {
    channel = Math.max(1, Math.min(4, channel));
    if (state) {
      sendSingleOutputIncDecCommand(channel - 1, motor, +1 * state.motorDirections[motor * 4 + channel - 1]);
    }
  }

  /**
   * Decrements motor speed.
   * @param motor the motor
   * @param channel the channel
   */
  //% blockId=pf_dec_ch
  //% block="decrement motor %motor channel %channel speed"
  //% channel.min=1 channel.max=4 channel.defl=1
  //% weight=50
  export function decCh(motor: PowerFunctionsOutput, channel: number) {
    channel = Math.max(1, Math.min(4, channel));
    if (state) {
      sendSingleOutputIncDecCommand(channel - 1, motor, -1 * state.motorDirections[motor * 4 + channel - 1]);
    }
  }

  //*********************************** original functions ***********************************

  /**
   * Sets the speed of a motor.
   * @param motor the motor
   * @param speed speed of the motor, eg: 3
   */
  //% blockId=powerfunctions_set_speed
  //% block="set | motor %motor | to %speed"
  //% subcategory=original
  //% speed.min=-7 speed.max=7
  //% weight=80
  //% motor.fieldEditor="gridpicker" motor.fieldOptions.columns=4 motor.fieldOptions.tooltips="false"
  export function setSpeed(motor: PowerFunctionsMotor, speed: number) {
    speed = Math.max(-7, Math.min(7, speed));
    if (speed == 0) {
      if (state.motorSpeedZeros[motor] == PowerFunctionSpeedZero.speed_0_float) {
        speed = 8;
      }
    } else {
      speed = speed * state.motorDirections[motor];
    }
    if (state) {
      sendSingleOutputCommand(getChannel(motor), getOutput(motor), speed);
    }
  }

  /**
   * Brakes then float.
   * The motor's power is quickly reversed and thus the motor will stop abruptly.
   * @param motor the motor
   */
  //% blockId=powerfunctions_brake
  //% block="brake| motor %motor"
  //% subcategory=original
  //% weight=75
  //% motor.fieldEditor="gridpicker" motor.fieldOptions.columns=4 motor.fieldOptions.tooltips="false"
  export function brake(motor: PowerFunctionsMotor) {
    if (state) {
      sendSingleOutputCommand(getChannel(motor), getOutput(motor), 0);
    }
  }

  /**
   * Floats a motor to stop.
   * The motor's power is switched off and thus the motor will roll to a stop.
   * @param motor the motor
   */
  //% blockId=pf_float
  //% block="float | motor %motor | to stop"
  //% subcategory=original
  //% weight=70
  //% motor.fieldEditor="gridpicker" motor.fieldOptions.columns=4 motor.fieldOptions.tooltips="false"
  export function float(motor: PowerFunctionsMotor) {
    if (state) {
      sendSingleOutputCommand(getChannel(motor), getOutput(motor), 8);
    }
  }

  /**
   * Configures a motor direction.
   * @param motor the motor
   * @param direction the direction
   */
  //% blockId=pf_set_motor_direction
  //% block="set direction | of motor %motor | to %direction"
  //% subcategory=original
  //% weight=20
  //% motor.fieldEditor="gridpicker" motor.fieldOptions.columns=4 motor.fieldOptions.tooltips="false"
  export function setMotorDirection(
    motor: PowerFunctionsMotor,
    direction: PowerFunctionsDirection
  ) {
    if (state) {
      state.motorDirections[motor] = direction;
    }
  }

  //*********************************** original functions end ***********************************

  namespace message {
    function mapValueToPwmElseFloat(value: number): number {
      switch (value) {
        case 7:
          return 0b0111;
        case 6:
          return 0b0110;
        case 5:
          return 0b0101;
        case 4:
          return 0b0100;
        case 3:
          return 0b0011;
        case 2:
          return 0b0010;
        case 1:
          return 0b0001;
        case 0:
          return 0b1000; // brake then float
        case -1:
          return 0b1111;
        case -2:
          return 0b1110;
        case -3:
          return 0b1101;
        case -4:
          return 0b1100;
        case -5:
          return 0b1011;
        case -6:
          return 0b1010;
        case -7:
          return 0b1001;
        default:
          return 0b0000; // float
      }
    }

    function createMessageFromNibbles(
      nibble1: number,
      nibble2: number,
      nibble3: number
    ) {
      state.messageToggle = ~state.messageToggle;
      nibble1 = nibble1 + (state.messageToggle & 0b1000); // Toggle bit is verified on receiver if inc/dec/toggle command is received.
      const lrc = 0xf ^ nibble1 ^ nibble2 ^ nibble3;
      return (nibble1 << 12) | (nibble2 << 8) | (nibble3 << 4) | lrc;
    }

    export function createSingleOutputPwmMessage(
      channel: PowerFunctionsChannel,
      output: PowerFunctionsOutput,
      value: number
    ) {
      const nibble1 = 0b0000 + channel;
      const nibble2 = 0b0100 + output;
      const nibble3 = mapValueToPwmElseFloat(value);
      return createMessageFromNibbles(nibble1, nibble2, nibble3);
    }

    export function createSingleOutputIncDecMessage(
      channel: PowerFunctionsChannel,
      output: PowerFunctionsOutput,
      value: number
    ) {
      const nibble1 = 0b0000 + channel;
      const nibble2 = 0b0110 + output;
      let nibble3;
      if (value == 1) {
        nibble3 = 0b0100; // Increment PWM
      } else {
        nibble3 = 0b0101; // Decrement PWM
      }
      return createMessageFromNibbles(nibble1, nibble2, nibble3);
    }

    export function createComboDirectMessage(
      channel: PowerFunctionsChannel,
      outputRed: PowerFunctionsCommand,
      outputBlue: PowerFunctionsCommand
    ) {
      const nibble1 = 0b0000 + channel;
      const nibble2 = 0b0001;
      const nibble3 = (outputBlue << 2) + outputRed;
      return createMessageFromNibbles(nibble1, nibble2, nibble3);
    }

    export function createComboPwmMessage(
      channel: PowerFunctionsChannel,
      outputRed: number,
      outputBlue: number
    ) {
      const nibble1 = 0b0100 + channel;
      const nibble2 = mapValueToPwmElseFloat(outputBlue);
      const nibble3 = mapValueToPwmElseFloat(outputRed);
      return createMessageFromNibbles(nibble1, nibble2, nibble3);
    }
  }

  const IR_MARK = Math.idiv(6 * 1000000, 38000);
  const START_STOP_PAUSE = Math.idiv((45 - 6) * 1000000, 38000);
  const LOW_PAUSE = Math.idiv((16 - 6) * 1000000, 38000);
  const HIGH_PAUSE = Math.idiv((27 - 6) * 1000000, 38000);

  export class InfraredDevice {
    private pin: AnalogPin;
    private waitCorrection: number;

    constructor(pin: AnalogPin, pwmPeriod = 26) {
      this.pin = pin;
      pins.analogWritePin(this.pin, 0);
      pins.analogSetPeriod(this.pin, pwmPeriod);

      // Measure the time we need for a minimal bit (analogWritePin and waitMicros)
      {
        const start = input.runningTimeMicros();
        const runs = 8;
        for (let i = 0; i < runs; i++) {
          this.transmitBit(1, 1);
        }
        const end = input.runningTimeMicros();
        this.waitCorrection = Math.idiv(end - start - runs * 2, runs * 2);
      }

      // Insert a pause between callibration and first message
      control.waitMicros(2000);
    }

    public transmitBit(highMicros: number, lowMicros: number): void {
      pins.analogWritePin(this.pin, 511);
      control.waitMicros(highMicros);
      pins.analogWritePin(this.pin, 1);
      control.waitMicros(lowMicros);
    }

    public sendMessage(message: number): void {
      const MAX_LENGTH_MS = 16;
      const channel = 1 + ((message >> 12) & 0b0011);
      const ir_mark = IR_MARK - this.waitCorrection;
      const high_pause = HIGH_PAUSE - this.waitCorrection;
      const low_pause = LOW_PAUSE - this.waitCorrection;
      const start_stop_pause = START_STOP_PAUSE - this.waitCorrection;

      for (let sendCount = (5 - state.sendCount); sendCount < 5; sendCount++) {
        const MESSAGE_BITS = 16;

        let mask = 1 << (MESSAGE_BITS - 1);

        // start bit
        this.transmitBit(ir_mark, start_stop_pause);

        // low and high bits
        while (mask > 0) {
          if (message & mask) {
            this.transmitBit(ir_mark, high_pause);
          } else {
            this.transmitBit(ir_mark, low_pause);
          }
          mask >>= 1;
        }

        // stop bit
        this.transmitBit(ir_mark, start_stop_pause);

        if (state.sendDelay == PowerFunctionSendDelay.delay_short) {
          basic.pause(MAX_LENGTH_MS); //16ms
        } else if (sendCount == 0 || sendCount == 1) {
          basic.pause(5 * MAX_LENGTH_MS);
        } else {
          basic.pause((6 + 2 * channel) * MAX_LENGTH_MS);
        }
      }
    }
  }

  export function runTests() {
    {
      state.messageToggle = 1;
      const c1RedFullForward = message.createSingleOutputPwmMessage(
        PowerFunctionsChannel.One,
        PowerFunctionsOutput.Red,
        7
      );
      const expectedC1RedFullForward = 0b0000010001111100; // 1148
      control.assert(
        c1RedFullForward === expectedC1RedFullForward,
        "createSingleOutputPwmMessage motor Red1 with speed 7"
      );
    }

    {
      state.messageToggle = 1;
      const c1ComboRedForwardBlueBackward = message.createComboDirectMessage(
        PowerFunctionsChannel.One,
        PowerFunctionsCommand.Forward,
        PowerFunctionsCommand.Backward
      );
      const expectedC1ComboRedForwardBlueBackward = 0b0000000110010111; // 407
      control.assert(
        c1ComboRedForwardBlueBackward === expectedC1ComboRedForwardBlueBackward,
        "createComboDirectMessage Red1 forward, Blue1 backward full speed"
      );
    }

    {
      state.messageToggle = 1;
      const c1ComboRedFloatBlueBrake = message.createComboPwmMessage(
        PowerFunctionsChannel.One,
        8,
        0
      );
      const expectedC1ComboRedFloatBlueBrake = 0b0100100000000011; // 18435
      control.assert(
        c1ComboRedFloatBlueBrake === expectedC1ComboRedFloatBlueBrake,
        "createComboPwmMessage Red1 float, Blue1 brake"
      );
    }
  }
}
