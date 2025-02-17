# Power Functions IR Sender

![IR sender+receiver](icon.png)

Control your LEGO® Power Functions motors using your micro:bit or Calliope-Mini, an infrared LED and MakeCode.
This extension turns your device into a remote control for your Power Functions devices.

A project using this extension is documented on [hackster.io](https://www.hackster.io/philipp-henkel/lego-power-functions-ir-sender-for-micro-bit-aecc10)

## Installation

Open MakeCode and select '+ Extensions' in the 'Advanced' menu. You need to enter our project URL https://github.com/brabl2/pxt-powerfunctions in the search field, hit return and then select the powerfunctions extension.

# Documentation

## powerfunctions.connectIrLed

Configures the infrared LED analog pin. Using hardware PWM mode. A 940 nm emitting diode is required.

```sig
powerfunctions.connectIrLed(AnalogPin.P0)
```
#### Parameters
- `pin` - analog pin with an attached IR-emitting LED

## powerfunctions.connectIrLedBitBang

Configures the infrared LED digital pin. Using software bit-bang mode. A 940 nm emitting diode is required.

```sig
powerfunctions.connectIrLed(DigitalPin.P0, PowerFunctionMicroBitVer.Ver2)
```
#### Parameters
- `pin` - digital pin with an attached IR-emitting LED
- `ver` - micro:bit version

## powerfunctions.cfgSendCountDelay

Configures send count and delay (the IR message is transmitted count-times with delay between messages).
Sending of the complete message in default setup (`count` = five-times and `delay` = delay_normal) can take around 0.8 second in worst case.
Setting these parameters to values different than default can speed-up data sending at cost of lower reliability.
Sending of the complete message in short setup (`count` = five-times and `delay` = delay_short) can take around 0.16 second in worst case.

```sig
powerfunctions.cfgSendCountDelay(PowerFunctionSendCount.five_times, PowerFunctionSendDelay.delay_normal)
```
#### Parameters
- `count` - the IR message is transmitted count-times
- `delay` - the delay between messages. It is safe to set this parameter to 'delay_short' in case of single LEGO® IR Receiver.

## powerfunctions.cfgMotorDirectionCh

Configures a motor direction.

```sig
powerfunctions.cfgMotorDirectionCh(PowerFunctionsOutput.Red, 1, PowerFunctionsDirection.Right)
```
#### Parameters
- `motor` - the motor
- `channel` - the channel of the motor from `1` to `4`
- `direction` - the direction of the motor

## powerfunctions.cfgMotorSpeedZeroCh

Configures zero speed behaviour of the motor (float or brake).

```sig
powerfunctions.cfgMotorSpeedZeroCh(PowerFunctionsOutput.Red, 1, PowerFunctionSpeedZero.speed_0_brake)
```
#### Parameters
- `motor` - the motor
- `channel` - the channel of the motor from `1` to `4`
- `behaviour` - the behaviour of the motor when speed is set to zero (float or brake)

## powerfunctions.incCh

Increments motor speed.

```sig
powerfunctions.incCh(PowerFunctionsOutput.Red, 1)
```
#### Parameters
- `motor` - the motor
- `channel` - the channel of the motor from `1` to `4`

## powerfunctions.decCh

Decrements motor speed.

```sig
powerfunctions.decCh(PowerFunctionsOutput.Red, 1)
```
#### Parameters
- `motor` - the motor
- `channel` - the channel of the motor from `1` to `4`

## powerfunctions.setSpeedCh

Sets the speed of a motor.

```sig
powerfunctions.setSpeedCh(PowerFunctionsOutput.Red, 1, 3)
```
#### Parameters
- `motor` - the motor
- `channel` - the channel of the motor from `1` to `4`
- `speed` - the speed of the motor from `-7` to `7`

## powerfunctions.brakeCh

Brakes then float. The motor's power is quickly reversed and thus the motor will stop abruptly.

```sig
powerfunctions.brakeCh(PowerFunctionsOutput.Red, 1)
```
#### Parameters
- `motor` - the motor
- `channel` - the channel of the motor from `1` to `4`

## powerfunctions.floatCh

Floats a motor to stop. The motor's power is switched off and thus the motor will roll to a stop.

```sig
powerfunctions.floatCh(PowerFunctionsOutput.Red, 1)
```
#### Parameters
- `motor` - the motor
- `channel` - the channel of the motor from `1` to `4`

# Original functions

## powerfunctions.setSpeed

Sets the speed of a motor.

```sig
powerfunctions.setSpeed(PowerFunctionsMotor.Red1, 3)
```
#### Parameters
- `motor` - the motor
- `speed` - the speed of the motor from `-7` to `7`.

## powerfunctions.brake

Brakes then float. The motor's power is quickly reversed and thus the motor will stop abruptly.

```sig
powerfunctions.brake(PowerFunctionsMotor.Red1)
```
#### Parameters
- `motor` - the motor

## powerfunctions.float

Floats a motor to stop. The motor's power is switched off and thus the motor will roll to a stop.

```sig
powerfunctions.float(PowerFunctionsMotor.Red1)
```
#### Parameters
- `motor` - the motor

## powerfunctions.setMotorDirection

Configures a motor direction.

```sig
powerfunctions.setMotorDirection(PowerFunctionsMotor.Red1, PowerFunctionsDirection.Right)
```
#### Parameters
- `motor` - the motor
- `direction` - the direction of the motor

## MakeCode Example

```blocks
basic.showIcon(IconNames.Heart);
powerfunctions.connectIrLed(AnalogPin.P1);
powerfunctions.cfgMotorDirectionCh(PowerFunctionsOutput.Blue, 1, PowerFunctionsDirection.Right);

input.onButtonPressed(Button.A, () => {
  powerfunctions.setSpeedCh(PowerFunctionsOutput.Blue, 1, 3);
});

input.onButtonPressed(Button.B, () => {
  powerfunctions.floatCh(PowerFunctionsOutput.Blue, 1);
});

basic.forever(() => {
  led.plotBarGraph(input.lightLevel(), 255);

  if (input.lightLevel() > 200) {
    powerfunctions.floatCh(PowerFunctionsOutput.Blue, 1);
    basic.pause(5000);
    powerfunctions.setSpeedCh(PowerFunctionsOutput.Blue, 1, 2);
    basic.pause(3000);
  }
});
```

## Disclaimer

LEGO® is a trademark of the LEGO Group of companies which does not sponsor, authorize or endorse this project.

## License

Copyright (C) 2017-2020 Philipp Henkel

Licensed under the MIT License (MIT). See LICENSE file for more details.

## Supported targets

- for PXT/microbit
