# Teensy SBUS Deployment Controller

This project is a Teensy-based Arduino control system for an SBUS radio-controlled deployment mechanism. It controls a camera gimbal, a BTS7960-driven linear actuator, antenna petal servos, and radar antenna servos using non-blocking state machines.

The system is designed for safe deployment and recovery using SBUS failsafe handling, actuator limit switches, servo interlocks, and startup homing logic.

## Features

* Reads SBUS receiver data from `Serial5`
* Supports 16 SBUS channels
* Uses PWM-style channel values around `1000–2000 us`
* Non-blocking logic using `millis()` / `micros()`
* No `delay()` calls
* Main system state machine
* Linear actuator state machine
* Smooth servo movement for all servos
* SBUS failsafe and timeout handling
* Startup actuator homing
* Limit switch debounce
* Actuator timeout protection
* Debug output over USB serial

## Controlled Hardware

### Camera Gimbal

* 2 servos:

  * Pan / yaw
  * Tilt / pitch
* Controlled directly from assigned SBUS channels
* Configurable:

  * Minimum position
  * Maximum position
  * Center position
  * Safe position
  * Servo inversion
  * Movement speed

### Linear Actuator

* Controlled with a BTS7960 H-bridge motor driver
* Uses `GyverMotor2`
* Configured for two direction pins plus one PWM speed pin
* Includes:

  * Extend command
  * Retract command
  * Stop command
  * Brake command
  * Retracted limit switch
  * Extended limit switch
  * Timeout protection
  * Error detection if both limit switches are active

### Antenna Petals

* 2 servos
* Logical states:

  * CLOSED
  * OPEN
* Petals only open when the actuator is fully extended
* Configured for approximately 180° servo movement

### Radar Antennas

* 2 servos
* Logical states:

  * CLOSED / PARKED
  * OPEN / DEPLOYED
* Radar servos only move when:

  * The linear actuator is fully extended
  * The petal servos are already in the open position
* Configured for approximately 90° servo movement

## Safety Logic

The system includes several safety protections:

* Deployable servos stay closed unless the actuator is fully extended
* Radar servos stay closed unless the actuator is extended and petals are open
* Actuator stops automatically at limit switches
* Actuator cannot drive farther into an already active limit switch
* If both limit switches are active, the system enters an error state
* If actuator movement timeout expires, the system enters an error state
* Radio commands are ignored when the system is disarmed
* On SBUS failsafe or timeout, the system returns to the safest closed/base state
* If actuator position is unknown, the system performs safe homing

## Main System States

```cpp
SYS_BOOT
SYS_HOMING
SYS_DISARMED_SAFE
SYS_ARMED_READY
SYS_DEPLOYING
SYS_DEPLOYED
SYS_RETRACTING
SYS_FAILSAFE
SYS_ERROR
```

## Actuator States

```cpp
ACTUATOR_UNKNOWN
ACTUATOR_RETRACTED
ACTUATOR_EXTENDING
ACTUATOR_EXTENDED
ACTUATOR_RETRACTING
ACTUATOR_ERROR
```

## Servo Group States

```cpp
PETALS_CLOSED
PETALS_OPEN
RADAR_CLOSED
RADAR_OPEN
```

## Pin Configuration

The sketch has a user configuration section at the top where all pins are defined.

Example pin assignments:

| Function               | Teensy Pin |
| ---------------------- | ---------: |
| Camera pan servo       |         32 |
| Camera tilt servo      |         31 |
| Petal left servo       |         33 |
| Petal right servo      |         34 |
| Radar left servo       |         35 |
| Radar right servo      |         36 |
| BTS7960 PWM            |         37 |
| BTS7960 direction A    |         38 |
| BTS7960 direction B    |         39 |
| Retracted limit switch |         40 |
| Extended limit switch  |         41 |
| Status LED             |         13 |
| SBUS receiver          |        RX5 |

## SBUS Channel Configuration

SBUS channels are zero-based in the code.

For example:

```cpp
channel_pwm[0] = SBUS CH1
channel_pwm[1] = SBUS CH2
channel_pwm[2] = SBUS CH3
```

Current channel assignment example:

```cpp
CH_CAMERA_PAN     = 3;  // SBUS CH4
CH_CAMERA_TILT    = 2;  // SBUS CH3
CH_ARM_ENABLE     = 4;  // SBUS CH5
CH_ACTUATOR_CMD   = 5;  // SBUS CH6
CH_PETALS_CMD     = 6;  // SBUS CH7
CH_RADAR_CMD      = 7;  // SBUS CH8
```

## Wiring Notes

### SBUS Receiver

Connect the SBUS signal wire to Teensy RX5.

Do not use RX5 for any other device.

### Servos

The servo signal wires connect to the assigned Teensy pins.

Do not power large HV servos from the Teensy.

Use a separate high-current servo power supply or BEC. Connect all grounds together:

* Teensy GND
* Servo power GND
* SBUS receiver GND
* BTS7960 logic GND
* Actuator power GND

### BTS7960 Driver

The sketch uses a three-wire motor driver style:

```cpp
DIR_A
DIR_B
PWM
```

Generic wiring:

| Teensy    | BTS7960 Function   |
| --------- | ------------------ |
| DIR_A pin | Direction input A  |
| DIR_B pin | Direction input B  |
| PWM pin   | Speed / enable PWM |
| GND       | Logic GND          |


If actuator direction is reversed, change:

```cpp
ACTUATOR_INVERT_DIRECTION
```

## Limit Switches

The actuator uses two normally-open limit switches:

* Retracted / start limit switch
* Extended / end limit switch

For switches wired to GND using internal pullups:

```cpp
LIMIT_ACTIVE_LOW = true;
```

For active-high wiring:

```cpp
LIMIT_ACTIVE_LOW = false;
```

## Startup Homing

On startup, the system reads both actuator limit switches.

If the retracted switch is active, the actuator state becomes:

```cpp
ACTUATOR_RETRACTED
```

If the extended switch is active, the actuator state becomes:

```cpp
ACTUATOR_EXTENDED
```

If neither switch is active, the actuator position is unknown. The system then performs a safe homing sequence:

1. Move all servos to safe closed positions
2. Extend actuator until the extended limit switch is reached
3. Keep petals and radar servos closed
4. Retract actuator until the retracted limit switch is reached
5. Enter the base closed state

The homing sequence is fully non-blocking.

## Failsafe Behavior

If SBUS failsafe is active, SBUS signal is lost, SBUS times out, or channel values are invalid:

* Camera gimbal moves to safe position
* Petals close
* Radar antennas close / park
* Actuator retracts to the start position if safe
* System ignores normal radio commands
* If actuator position is unknown, safe homing/recovery logic is used

## Debug Output

The sketch prints debug information over USB serial, including:

* SBUS health
* ARM state
* Main system state
* Actuator state
* Limit switch states
* Actuator command
* Petal state
* Radar state
* Servo target positions

## Calibration

### Servo Endpoints

Start with conservative pulse values, for example:

```cpp
1000 us
1500 us
2000 us
```

Increase or decrease only after checking that the linkage does not bind.

### Servo Inversion

Each servo can be inverted in the configuration section.

Example:

```cpp
CAMERA_PAN_INVERT = true;
```

### Petal and Radar Travel

Petal servos are configured for approximately 180° movement.

Radar servos are configured for approximately 90° movement.

Final values should be calibrated mechanically using servo pulse widths.

### Actuator Direction

If the actuator extends when it should retract, change the actuator direction inversion setting:

```cpp
ACTUATOR_INVERT_DIRECTION = true;
```

### Limit Switch Polarity

For normally-open switches connected to ground with `INPUT_PULLUP`:

```cpp
LIMIT_ACTIVE_LOW = true;
```

## Safe Testing Procedure

Before connecting the real mechanism:

1. Upload the sketch to the Teensy
2. Open Serial Monitor
3. Verify SBUS channel values
4. Verify ARM switch behavior
5. Press each limit switch manually and check debug output
6. Test actuator direction with the actuator disconnected from the mechanism
7. Confirm that each limit switch stops the actuator
8. Confirm that both switches active causes an error state
9. Test servo movement with linkages disconnected
10. Connect the real mechanism only after all safety checks pass

## Required Libraries

* `Servo.h`
* `GyverMotor2.h`
* SBUS library included as:

```cpp
#include "src/SBUS/SBUS.h"
```
