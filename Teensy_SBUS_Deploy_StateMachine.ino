/*
  Teensy SBUS Deployment Controller

  Hardware summary:
    - SBUS receiver on Teensy Serial5 RX pin.
    - 6 servo outputs, controlled in microseconds with Servo.h.
    - Linear actuator through BTS7960 using GyverMotor2 DRIVER3WIRE mode.
    - Two normally-open limit switches: retracted/start and extended/end.

  Non-blocking design:
    - No delay().
    - SBUS, actuator state machine, servo smoothing, switch debounce, homing,
      failsafe, and debug printing all run from loop().
*/

#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <GyverMotor2.h>
#include "src/SBUS/SBUS.h"

// ============================================================================
// USER CONFIGURATION
// ============================================================================

// -------------------- Serial / debug --------------------
const unsigned long USB_BAUD = 500000;
const uint16_t DEBUG_PRINT_INTERVAL_MS = 250;

// -------------------- SBUS configuration --------------------
// SBUS receiver signal goes to Teensy RX5. Do not use the RX5 pin for anything else.
SBUS sbus(Serial5);

const uint8_t SBUS_CHANNEL_COUNT = 16;
const unsigned long SBUS_TIMEOUT_US = 200000;   // 200 ms without valid frame = radio timeout

// SBUS scaling from your existing dRehmFlight-style radio code.
const float SBUS_SCALE = 0.615f;
const float SBUS_BIAS  = 895.0f;
const float RADIO_FILTER_BETA = 0.7f;           // CH1-CH4 low-pass filter, 0.0 slow, 1.0 raw

// Valid scaled PWM-style channel range. Anything outside this trips radio failsafe logic.
const uint16_t RADIO_VALID_MIN_US = 800;
const uint16_t RADIO_VALID_MAX_US = 2200;
const uint16_t RADIO_INPUT_MIN_US = 1000;
const uint16_t RADIO_INPUT_MID_US = 1500;
const uint16_t RADIO_INPUT_MAX_US = 2000;

// -------------------- Zero-based SBUS channel assignments --------------------
// SBUS CH1 is index 0, CH2 is index 1, etc.
const uint8_t CH_CAMERA_PAN     = 3;    // SBUS CH4
const uint8_t CH_CAMERA_TILT    = 2;    // SBUS CH3
const uint8_t CH_ARM_ENABLE     = 4;    // SBUS CH5
const uint8_t CH_ACTUATOR_CMD   = 5;    // SBUS CH6
const uint8_t CH_PETALS_CMD     = 6;    // SBUS CH7
const uint8_t CH_RADAR_CMD      = 7;    // SBUS CH8

// -------------------- Radio deadbands and switch thresholds --------------------
const uint16_t RADIO_STICK_DEADBAND_US = 15;    // camera stick deadband around 1500
const uint16_t SWITCH_LOW_THRESHOLD_US  = 1300; // low switch position
const uint16_t SWITCH_HIGH_THRESHOLD_US = 1700; // high switch position

// ARM hysteresis: below OFF = disarmed, above ON = armed, between = hold previous state.
const uint16_t ARM_OFF_THRESHOLD_US = 1500;
const uint16_t ARM_ON_THRESHOLD_US  = 1700;

// -------------------- Pin definitions --------------------
// Pick PWM-capable pins for servos and the actuator PWM pin.
const uint8_t PIN_SERVO_CAMERA_PAN   = 32; // пока не подлкючал но надо будет подключить к камере 
const uint8_t PIN_SERVO_CAMERA_TILT  = 31; 

const uint8_t PIN_SERVO_PETAL_LEFT   = 33;
const uint8_t PIN_SERVO_PETAL_RIGHT  = 34;
const uint8_t PIN_SERVO_RADAR_LEFT   = 35;
const uint8_t PIN_SERVO_RADAR_RIGHT  = 36;

// BTS7960 in DRIVER3WIRE mode: two direction pins + one PWM speed pin.
const uint8_t PIN_ACTUATOR_DIR_A = 38;    // BTS7960 direction pin A
const uint8_t PIN_ACTUATOR_DIR_B = 39;    // BTS7960 direction pin B
const uint8_t PIN_ACTUATOR_PWM   = 37;   // BTS7960 speed PWM pin

// Limit switches. Пока не подкючены надо НАДО 
const uint8_t PIN_LIMIT_RETRACTED = 40;  // start/min/retracted switch
const uint8_t PIN_LIMIT_EXTENDED  = 41;  // end/max/extended switch
const uint8_t PIN_STATUS_LED      = 13;

// -------------------- BTS7960 wiring notes --------------------
/*
  This sketch uses GyverMotor2 DRIVER3WIRE mode: GPIO, GPIO, PWM.

  Generic BTS7960 breakout wiring for this mode:
    Teensy PIN_ACTUATOR_DIR_A -> direction input A
    Teensy PIN_ACTUATOR_DIR_B -> direction input B
    Teensy PIN_ACTUATOR_PWM   -> PWM speed input
    Teensy GND                -> BTS7960 logic GND

  Many BTS7960 modules expose RPWM, LPWM, R_EN, and L_EN instead of a single
  DIR_A / DIR_B / PWM interface. To use a 3-wire abstraction safely, the usual
  adapter wiring is:
    - Tie R_EN and L_EN together and drive them from PIN_ACTUATOR_PWM,
      or tie them high only if you accept full driver enable and use PWM on
      the direction-side logic as supported by your module.
    - Use PIN_ACTUATOR_DIR_A and PIN_ACTUATOR_DIR_B as the two complementary
      direction commands.

  Confirm your specific BTS7960 board logic with the motor disconnected first.
  If EXTEND and RETRACT are swapped, change ACTUATOR_INVERT_DIRECTION.
*/

// -------------------- Limit switch configuration --------------------
// true  = switch closes to GND, pinMode(INPUT_PULLUP), active when digitalRead() == LOW
// false = active-high wiring, pinMode(INPUT), active when digitalRead() == HIGH
const bool LIMIT_ACTIVE_LOW = true;
const uint16_t LIMIT_DEBOUNCE_MS = 30;

// -------------------- Actuator calibration --------------------
const int16_t ACTUATOR_SPEED_PWM = 180;             // 0..255 for default 8-bit GyverMotor2 PWM
const uint16_t ACTUATOR_MIN_DUTY_PWM = 0;           // raise if motor stalls at low PWM
const uint32_t ACTUATOR_ACCEL_PWM_PER_SEC = 700;    // 0 disables acceleration; tick() still safe to call
const uint16_t ACTUATOR_DEADTIME_US = 5;            // H-bridge deadtime between direction changes
const uint32_t ACTUATOR_MOVE_TIMEOUT_MS = 15000;    // timeout per extend/retract movement
const bool ACTUATOR_INVERT_DIRECTION = false;       // flip if extend/retract are physically reversed
const bool ACTUATOR_BRAKE_ON_LIMIT_OR_ERROR = true; // true = active brake at limits/errors, false = coast stop

// -------------------- Servo smoothing --------------------
const uint32_t SERVO_UPDATE_INTERVAL_US = 20000;    // 50 Hz servo writes
const float CAMERA_SERVO_SPEED_US_PER_SEC = 600.0f;
const float DEPLOY_SERVO_SPEED_US_PER_SEC = 500.0f;
const int SERVO_POSITION_TOLERANCE_US = 10;         // considered "at position" within this pulse error

// 9imod BLS-HV70MG / similar 180-degree HV digital servos:
// Start conservatively with 1000..2000 us as the usable 180-degree range.
// If your exact servo safely supports a wider pulse range, recalibrate MIN/MAX
// and SERVO_180_DEG_TRAVEL_US after testing with the linkage disconnected.
const int SERVO_180_DEG_TRAVEL_US = 1000;
const int PETAL_TRAVEL_US = SERVO_180_DEG_TRAVEL_US;       // petals move about 180 degrees
const int RADAR_TRAVEL_US = SERVO_180_DEG_TRAVEL_US / 2;   // radar moves about 90 degrees

// -------------------- Camera gimbal servo calibration --------------------
const int CAMERA_PAN_MIN_US    = 1000;
const int CAMERA_PAN_MAX_US    = 2000;
const int CAMERA_PAN_CENTER_US = 1500;
const int CAMERA_PAN_SAFE_US   = 1500;
const bool CAMERA_PAN_INVERT   = false;

const int CAMERA_TILT_MIN_US    = 1000;
const int CAMERA_TILT_MAX_US    = 2000;
const int CAMERA_TILT_CENTER_US = 1500;
const int CAMERA_TILT_SAFE_US   = 1500;
const bool CAMERA_TILT_INVERT   = false;

// -------------------- Petal servo calibration --------------------
// Petals are configured for approximately 180 degrees of motion.
// The right petal is mechanically mirrored here, so its open pulse moves downward.
const int PETAL_LEFT_MIN_US    = 1000;
const int PETAL_LEFT_MAX_US    = 2000;
const int PETAL_LEFT_CLOSED_US = 1000;
const int PETAL_LEFT_OPEN_US   = PETAL_LEFT_CLOSED_US + PETAL_TRAVEL_US;
const bool PETAL_LEFT_INVERT   = false;

const int PETAL_RIGHT_MIN_US    = 1000;
const int PETAL_RIGHT_MAX_US    = 2000;
const int PETAL_RIGHT_CLOSED_US = 2000;
const int PETAL_RIGHT_OPEN_US   = PETAL_RIGHT_CLOSED_US - PETAL_TRAVEL_US;
const bool PETAL_RIGHT_INVERT   = false;

// -------------------- Radar antenna servo calibration --------------------
// Radar antennas are configured for approximately 90 degrees of motion.
// The right radar servo is mechanically mirrored here, so its deployed pulse moves downward.
const int RADAR_LEFT_MIN_US    = 1000;
const int RADAR_LEFT_MAX_US    = 2000;
const int RADAR_LEFT_CLOSED_US = 1000;
const int RADAR_LEFT_OPEN_US   = RADAR_LEFT_CLOSED_US + RADAR_TRAVEL_US;
const bool RADAR_LEFT_INVERT   = false;

const int RADAR_RIGHT_MIN_US    = 1000;
const int RADAR_RIGHT_MAX_US    = 2000;
const int RADAR_RIGHT_CLOSED_US = 2000;
const int RADAR_RIGHT_OPEN_US   = RADAR_RIGHT_CLOSED_US - RADAR_TRAVEL_US;
const bool RADAR_RIGHT_INVERT   = false;

// -------------------- Radio failsafe channel values --------------------
// These are applied whenever SBUS failsafe, timeout, or invalid channel values occur.
unsigned long channel_fs[SBUS_CHANNEL_COUNT] = {
  1500, 1500, 1500, 1500,
  1000, 1000, 1000, 1000,
  1500, 1500, 1500, 1500,
  1500, 1500, 1500, 1500
};

// ============================================================================
// STATE DEFINITIONS
// ============================================================================

enum SystemState {
  SYS_BOOT,
  SYS_HOMING,
  SYS_DISARMED_SAFE,
  SYS_ARMED_READY,
  SYS_DEPLOYING,
  SYS_DEPLOYED,
  SYS_RETRACTING,
  SYS_FAILSAFE,
  SYS_ERROR
};

enum ActuatorState {
  ACTUATOR_UNKNOWN,
  ACTUATOR_RETRACTED,
  ACTUATOR_EXTENDING,
  ACTUATOR_EXTENDED,
  ACTUATOR_RETRACTING,
  ACTUATOR_ERROR
};

enum PetalState {
  PETALS_CLOSED,
  PETALS_OPEN
};

enum RadarState {
  RADAR_CLOSED,
  RADAR_OPEN
};

enum ActuatorTarget {
  ACT_TARGET_HOLD,
  ACT_TARGET_RETRACTED,
  ACT_TARGET_EXTENDED
};

enum HomingStage {
  HOMING_NONE,
  HOMING_EXTEND_FIRST,
  HOMING_RETRACT_SECOND,
  HOMING_DONE
};

enum ServoIndex {
  SERVO_CAMERA_PAN = 0,
  SERVO_CAMERA_TILT,
  SERVO_PETAL_LEFT,
  SERVO_PETAL_RIGHT,
  SERVO_RADAR_LEFT,
  SERVO_RADAR_RIGHT,
  SERVO_COUNT
};

// ============================================================================
// GLOBALS
// ============================================================================

// Radio state, compatible with your existing code style.
unsigned long channel_pwm[SBUS_CHANNEL_COUNT];
unsigned long channel_pwm_prev[4];
uint16_t sbusChannels[SBUS_CHANNEL_COUNT];
bool sbusFailSafe = false;
bool sbusLostFrame = false;
bool sbusFrameReceived = false;
bool sbusTimedOut = true;
bool sbusInvalidRange = false;
bool radioHealthy = false;
unsigned long lastSbusFrameMicros = 0;

// System state.
SystemState systemState = SYS_BOOT;
ActuatorState actuatorState = ACTUATOR_UNKNOWN;
PetalState petalState = PETALS_CLOSED;
RadarState radarState = RADAR_CLOSED;
ActuatorTarget actuatorTarget = ACT_TARGET_RETRACTED;
HomingStage homingStage = HOMING_NONE;
bool homingIsFailsafeRecovery = false;
bool armedLatched = false;
const char* lastError = "none";

// Actuator timing.
uint32_t actuatorMoveStartMs = 0;
ActuatorTarget actuatorMoveExpectedTarget = ACT_TARGET_HOLD;

// Debug timing.
uint32_t lastDebugPrintMs = 0;

// Motor driver object. DRIVER3WIRE = direction GPIO, direction GPIO, PWM.
GyverMotor2<GM2::DIR_DIR_PWM> actuatorMotor(PIN_ACTUATOR_DIR_A, PIN_ACTUATOR_DIR_B, PIN_ACTUATOR_PWM);

struct DebouncedLimitSwitch {
  uint8_t pin;
  bool rawActive;
  bool lastRawActive;
  bool stableActive;
  uint32_t lastRawChangeMs;
};

DebouncedLimitSwitch limitRetracted = { PIN_LIMIT_RETRACTED, false, false, false, 0 };
DebouncedLimitSwitch limitExtended  = { PIN_LIMIT_EXTENDED,  false, false, false, 0 };

struct SmoothServo {
  Servo servo;
  uint8_t pin;
  int minUs;
  int maxUs;
  int safeUs;
  bool invert;
  float speedUsPerSec;
  float currentUs;
  int targetUs;
  int lastWrittenUs;
  uint32_t lastUpdateUs;
};

SmoothServo smoothServos[SERVO_COUNT];

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void radioSetup();
void readRadio();
bool radioIsHealthy();
bool isArmed();
void setRadioFailsafeValues();
unsigned long sbusToPwm(uint16_t sbusValue);
unsigned long filterRadioChannel(unsigned long previousValue, unsigned long newValue);
bool channelsAreValid();
void applyRadioFailsafeValues();

void setupLimitSwitches();
void readLimitSwitches();
void updateLimitSwitchDebounce();
bool rawLimitActive(uint8_t pin);
bool retractedLimitActive();
bool extendedLimitActive();
bool bothLimitsActive();

void setupServos();
void configureServo(ServoIndex index, uint8_t pin, int minUs, int maxUs, int safeUs, bool invert, float speedUsPerSec);
void updateServoTargets();
void updateSmoothServos();
void setAllServosSafe();
void setServoTarget(ServoIndex index, int targetUs);
int logicalToPhysicalServoUs(ServoIndex index, int logicalUs);
bool servoAtLogicalPosition(ServoIndex index, int logicalUs);
bool petalsAtOpenPosition();
int mapChannelToServoUs(uint8_t ch, int minUs, int maxUs, int centerUs, bool invert);
int clampInt(int value, int low, int high);

void setupActuator();
void updateSystemState();
void updateActuatorState();
void extendActuator();
void retractActuator();
void stopActuator();
void brakeActuator();
void beginActuatorExtend();
void beginActuatorRetract();
void enterActuatorError(const char* reason);
void startSafeHoming(bool failsafeRecovery);
void updateSafeHomingSequence();

bool channelSwitchHigh(uint8_t ch);
bool channelSwitchLow(uint8_t ch);
const char* actuatorCommandName();

void printDebug();
const char* systemStateName(SystemState state);
const char* actuatorStateName(ActuatorState state);
const char* actuatorTargetName(ActuatorTarget target);
const char* homingStageName(HomingStage stage);

// ============================================================================
// ARDUINO SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(USB_BAUD);

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  setRadioFailsafeValues();
  radioSetup();

  setupLimitSwitches();
  setupServos();
  setupActuator();

  readLimitSwitches();
  updateLimitSwitchDebounce();

  // Determine initial actuator position from debounced switch states.
  if (bothLimitsActive()) {
    actuatorState = ACTUATOR_ERROR;
    systemState = SYS_ERROR;
    lastError = "both limit switches active at boot";
    brakeActuator();
  } else if (retractedLimitActive()) {
    actuatorState = ACTUATOR_RETRACTED;
    actuatorTarget = ACT_TARGET_RETRACTED;
  } else if (extendedLimitActive()) {
    actuatorState = ACTUATOR_EXTENDED;
    actuatorTarget = ACT_TARGET_RETRACTED;  // base state is retracted/closed
  } else {
    actuatorState = ACTUATOR_UNKNOWN;
    actuatorTarget = ACT_TARGET_HOLD;
  }

  setAllServosSafe();
  updateSmoothServos();

  Serial.println(F("Teensy SBUS deployment controller started"));
  Serial.println(F("SBUS on Serial5; channels are zero-based in channel_pwm[]"));
}

void loop() {
  readRadio();
  readLimitSwitches();
  updateLimitSwitchDebounce();

  // Required when GyverMotor2 acceleration is used. Safe to call even if accel is 0.
  actuatorMotor.tick();

  updateSystemState();
  updateActuatorState();
  updateServoTargets();
  updateSmoothServos();
  printDebug();
}

// ============================================================================
// RADIO
// ============================================================================

void radioSetup() {
  sbus.begin();
}

void setRadioFailsafeValues() {
  for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++) {
    channel_pwm[i] = channel_fs[i];
  }
  for (uint8_t i = 0; i < 4; i++) {
    channel_pwm_prev[i] = channel_pwm[i];
  }
}

void readRadio() {
  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame)) {
    sbusFrameReceived = true;
    lastSbusFrameMicros = micros();

    for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++) {
      channel_pwm[i] = sbusToPwm(sbusChannels[i]);
    }

    // Low-pass CH1-CH4 like your existing radioComm code.
    for (uint8_t i = 0; i < 4; i++) {
      channel_pwm[i] = filterRadioChannel(channel_pwm_prev[i], channel_pwm[i]);
      channel_pwm_prev[i] = channel_pwm[i];
    }
  }

  sbusTimedOut = !sbusFrameReceived || ((micros() - lastSbusFrameMicros) > SBUS_TIMEOUT_US);
  sbusInvalidRange = !channelsAreValid();
  radioHealthy = !(sbusFailSafe || sbusTimedOut || sbusInvalidRange);

  if (!radioHealthy) {
    applyRadioFailsafeValues();
  }
}

bool radioIsHealthy() {
  return radioHealthy;
}

bool isArmed() {
  if (!radioIsHealthy()) {
    armedLatched = false;
    return false;
  }

  const uint16_t armValue = channel_pwm[CH_ARM_ENABLE];
  if (armValue >= ARM_ON_THRESHOLD_US) {
    armedLatched = true;
  } else if (armValue <= ARM_OFF_THRESHOLD_US) {
    armedLatched = false;
  }
  return armedLatched;
}

unsigned long sbusToPwm(uint16_t sbusValue) {
  return (unsigned long)(sbusValue * SBUS_SCALE + SBUS_BIAS);
}

unsigned long filterRadioChannel(unsigned long previousValue, unsigned long newValue) {
  return (unsigned long)((1.0f - RADIO_FILTER_BETA) * previousValue + RADIO_FILTER_BETA * newValue);
}

bool channelsAreValid() {
  for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++) {
    if (channel_pwm[i] < RADIO_VALID_MIN_US || channel_pwm[i] > RADIO_VALID_MAX_US) {
      return false;
    }
  }
  return true;
}

void applyRadioFailsafeValues() {
  for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++) {
    channel_pwm[i] = channel_fs[i];
  }
  for (uint8_t i = 0; i < 4; i++) {
    channel_pwm_prev[i] = channel_pwm[i];
  }
}

// ============================================================================
// LIMIT SWITCHES
// ============================================================================

void setupLimitSwitches() {
  pinMode(PIN_LIMIT_RETRACTED, LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  pinMode(PIN_LIMIT_EXTENDED,  LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);

  const uint32_t nowMs = millis();
  limitRetracted.rawActive = rawLimitActive(limitRetracted.pin);
  limitRetracted.lastRawActive = limitRetracted.rawActive;
  limitRetracted.stableActive = limitRetracted.rawActive;
  limitRetracted.lastRawChangeMs = nowMs;

  limitExtended.rawActive = rawLimitActive(limitExtended.pin);
  limitExtended.lastRawActive = limitExtended.rawActive;
  limitExtended.stableActive = limitExtended.rawActive;
  limitExtended.lastRawChangeMs = nowMs;
}

void readLimitSwitches() {
  limitRetracted.rawActive = rawLimitActive(limitRetracted.pin);
  limitExtended.rawActive  = rawLimitActive(limitExtended.pin);
}

void updateLimitSwitchDebounce() {
  const uint32_t nowMs = millis();

  DebouncedLimitSwitch* switches[2] = { &limitRetracted, &limitExtended };
  for (uint8_t i = 0; i < 2; i++) {
    DebouncedLimitSwitch* sw = switches[i];

    if (sw->rawActive != sw->lastRawActive) {
      sw->lastRawActive = sw->rawActive;
      sw->lastRawChangeMs = nowMs;
    }

    if ((nowMs - sw->lastRawChangeMs) >= LIMIT_DEBOUNCE_MS) {
      sw->stableActive = sw->rawActive;
    }
  }
}

bool rawLimitActive(uint8_t pin) {
  const bool levelHigh = (digitalRead(pin) == HIGH);
  return LIMIT_ACTIVE_LOW ? !levelHigh : levelHigh;
}

bool retractedLimitActive() {
  return limitRetracted.stableActive;
}

bool extendedLimitActive() {
  return limitExtended.stableActive;
}

bool bothLimitsActive() {
  return retractedLimitActive() && extendedLimitActive();
}

// ============================================================================
// SERVOS
// ============================================================================

void setupServos() {
  configureServo(SERVO_CAMERA_PAN,  PIN_SERVO_CAMERA_PAN,  CAMERA_PAN_MIN_US,  CAMERA_PAN_MAX_US,  CAMERA_PAN_SAFE_US,  CAMERA_PAN_INVERT,  CAMERA_SERVO_SPEED_US_PER_SEC);
  configureServo(SERVO_CAMERA_TILT, PIN_SERVO_CAMERA_TILT, CAMERA_TILT_MIN_US, CAMERA_TILT_MAX_US, CAMERA_TILT_SAFE_US, CAMERA_TILT_INVERT, CAMERA_SERVO_SPEED_US_PER_SEC);

  configureServo(SERVO_PETAL_LEFT,  PIN_SERVO_PETAL_LEFT,  PETAL_LEFT_MIN_US,   PETAL_LEFT_MAX_US,   PETAL_LEFT_CLOSED_US,  PETAL_LEFT_INVERT,  DEPLOY_SERVO_SPEED_US_PER_SEC);
  configureServo(SERVO_PETAL_RIGHT, PIN_SERVO_PETAL_RIGHT, PETAL_RIGHT_MIN_US,  PETAL_RIGHT_MAX_US,  PETAL_RIGHT_CLOSED_US, PETAL_RIGHT_INVERT, DEPLOY_SERVO_SPEED_US_PER_SEC);

  configureServo(SERVO_RADAR_LEFT,  PIN_SERVO_RADAR_LEFT,  RADAR_LEFT_MIN_US,   RADAR_LEFT_MAX_US,   RADAR_LEFT_CLOSED_US,  RADAR_LEFT_INVERT,  DEPLOY_SERVO_SPEED_US_PER_SEC);
  configureServo(SERVO_RADAR_RIGHT, PIN_SERVO_RADAR_RIGHT, RADAR_RIGHT_MIN_US,  RADAR_RIGHT_MAX_US,  RADAR_RIGHT_CLOSED_US, RADAR_RIGHT_INVERT, DEPLOY_SERVO_SPEED_US_PER_SEC);
}

void configureServo(ServoIndex index, uint8_t pin, int minUs, int maxUs, int safeUs, bool invert, float speedUsPerSec) {
  SmoothServo& s = smoothServos[index];
  s.pin = pin;
  s.minUs = minUs;
  s.maxUs = maxUs;
  s.invert = invert;
  const int logicalSafeUs = clampInt(safeUs, minUs, maxUs);
  s.safeUs = invert ? (minUs + maxUs - logicalSafeUs) : logicalSafeUs;
  s.speedUsPerSec = speedUsPerSec;
  s.currentUs = s.safeUs;
  s.targetUs = s.safeUs;
  s.lastWrittenUs = s.safeUs;
  s.lastUpdateUs = micros();

  s.servo.attach(pin, minUs, maxUs);
  s.servo.writeMicroseconds(s.safeUs);
}

void updateServoTargets() {
  const bool safeOnly =
    (systemState == SYS_BOOT) ||
    (systemState == SYS_HOMING) ||
    (systemState == SYS_DISARMED_SAFE) ||
    (systemState == SYS_FAILSAFE) ||
    (systemState == SYS_ERROR) ||
    !radioIsHealthy() ||
    !isArmed();

  if (safeOnly) {
    setAllServosSafe();
    return;
  }

  // Camera is allowed only when armed and radio is healthy.
  setServoTarget(SERVO_CAMERA_PAN,
                 mapChannelToServoUs(CH_CAMERA_PAN, CAMERA_PAN_MIN_US, CAMERA_PAN_MAX_US, CAMERA_PAN_CENTER_US, false));
  setServoTarget(SERVO_CAMERA_TILT,
                 mapChannelToServoUs(CH_CAMERA_TILT, CAMERA_TILT_MIN_US, CAMERA_TILT_MAX_US, CAMERA_TILT_CENTER_US, false));

  // Deployable servos are mechanically locked closed unless actuator is exactly fully extended.
  const bool actuatorFullyExtended = (actuatorState == ACTUATOR_EXTENDED);
  const bool petalOpenCommandAllowed = actuatorFullyExtended && channelSwitchHigh(CH_PETALS_CMD);

  if (petalOpenCommandAllowed) {
    setServoTarget(SERVO_PETAL_LEFT,  PETAL_LEFT_OPEN_US);
    setServoTarget(SERVO_PETAL_RIGHT, PETAL_RIGHT_OPEN_US);
    petalState = PETALS_OPEN;
  } else {
    setServoTarget(SERVO_PETAL_LEFT,  PETAL_LEFT_CLOSED_US);
    setServoTarget(SERVO_PETAL_RIGHT, PETAL_RIGHT_CLOSED_US);
    petalState = PETALS_CLOSED;
  }

  // Radar has a stricter interlock:
  //   1. actuator must be fully extended,
  //   2. petals must be commanded open,
  //   3. both petal servos must have physically reached their open pulse targets.
  // This prevents the radar servos from moving until the petals have cleared them.
  const bool radarOpenCommandAllowed =
    actuatorFullyExtended &&
    petalOpenCommandAllowed &&
    petalsAtOpenPosition() &&
    channelSwitchHigh(CH_RADAR_CMD);

  if (radarOpenCommandAllowed) {
    setServoTarget(SERVO_RADAR_LEFT,  RADAR_LEFT_OPEN_US);
    setServoTarget(SERVO_RADAR_RIGHT, RADAR_RIGHT_OPEN_US);
    radarState = RADAR_OPEN;
  } else {
    setServoTarget(SERVO_RADAR_LEFT,  RADAR_LEFT_CLOSED_US);
    setServoTarget(SERVO_RADAR_RIGHT, RADAR_RIGHT_CLOSED_US);
    radarState = RADAR_CLOSED;
  }
}

void updateSmoothServos() {
  const uint32_t nowUs = micros();

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    SmoothServo& s = smoothServos[i];
    const uint32_t elapsedUs = nowUs - s.lastUpdateUs;

    if (elapsedUs < SERVO_UPDATE_INTERVAL_US) {
      continue;
    }

    s.lastUpdateUs = nowUs;

    const float maxStep = s.speedUsPerSec * ((float)elapsedUs / 1000000.0f);
    const float delta = (float)s.targetUs - s.currentUs;

    if (fabs(delta) <= maxStep || maxStep <= 0.0f) {
      s.currentUs = (float)s.targetUs;
    } else if (delta > 0.0f) {
      s.currentUs += maxStep;
    } else {
      s.currentUs -= maxStep;
    }

    const int writeUs = clampInt((int)(s.currentUs + 0.5f), s.minUs, s.maxUs);
    if (writeUs != s.lastWrittenUs) {
      s.servo.writeMicroseconds(writeUs);
      s.lastWrittenUs = writeUs;
    }
  }
}

void setAllServosSafe() {
  setServoTarget(SERVO_CAMERA_PAN,  CAMERA_PAN_SAFE_US);
  setServoTarget(SERVO_CAMERA_TILT, CAMERA_TILT_SAFE_US);
  setServoTarget(SERVO_PETAL_LEFT,  PETAL_LEFT_CLOSED_US);
  setServoTarget(SERVO_PETAL_RIGHT, PETAL_RIGHT_CLOSED_US);
  setServoTarget(SERVO_RADAR_LEFT,  RADAR_LEFT_CLOSED_US);
  setServoTarget(SERVO_RADAR_RIGHT, RADAR_RIGHT_CLOSED_US);

  petalState = PETALS_CLOSED;
  radarState = RADAR_CLOSED;
}

void setServoTarget(ServoIndex index, int targetUs) {
  smoothServos[index].targetUs = logicalToPhysicalServoUs(index, targetUs);
}

int logicalToPhysicalServoUs(ServoIndex index, int logicalUs) {
  SmoothServo& s = smoothServos[index];
  const int clampedLogicalUs = clampInt(logicalUs, s.minUs, s.maxUs);
  const int physicalUs = s.invert ? (s.minUs + s.maxUs - clampedLogicalUs) : clampedLogicalUs;
  return clampInt(physicalUs, s.minUs, s.maxUs);
}

bool servoAtLogicalPosition(ServoIndex index, int logicalUs) {
  SmoothServo& s = smoothServos[index];
  const int physicalTargetUs = logicalToPhysicalServoUs(index, logicalUs);
  const int currentUs = clampInt((int)(s.currentUs + 0.5f), s.minUs, s.maxUs);
  return abs(currentUs - physicalTargetUs) <= SERVO_POSITION_TOLERANCE_US;
}

bool petalsAtOpenPosition() {
  return servoAtLogicalPosition(SERVO_PETAL_LEFT, PETAL_LEFT_OPEN_US) &&
         servoAtLogicalPosition(SERVO_PETAL_RIGHT, PETAL_RIGHT_OPEN_US);
}

int mapChannelToServoUs(uint8_t ch, int minUs, int maxUs, int centerUs, bool invert) {
  if (ch >= SBUS_CHANNEL_COUNT) {
    return clampInt(centerUs, minUs, maxUs);
  }

  int value = (int)channel_pwm[ch];
  if (abs(value - (int)RADIO_INPUT_MID_US) <= (int)RADIO_STICK_DEADBAND_US) {
    return clampInt(centerUs, minUs, maxUs);
  }

  value = clampInt(value, RADIO_INPUT_MIN_US, RADIO_INPUT_MAX_US);
  long mapped = map(value, RADIO_INPUT_MIN_US, RADIO_INPUT_MAX_US, minUs, maxUs);

  if (invert) {
    mapped = (long)minUs + (long)maxUs - mapped;
  }

  return clampInt((int)mapped, minUs, maxUs);
}

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

// ============================================================================
// SYSTEM STATE MACHINE
// ============================================================================

void updateSystemState() {
  if (actuatorState == ACTUATOR_ERROR) {
    systemState = SYS_ERROR;
  }

  if (systemState == SYS_ERROR) {
    actuatorTarget = ACT_TARGET_HOLD;
    setAllServosSafe();
    stopActuator();
    digitalWrite(PIN_STATUS_LED, HIGH);
    return;
  }

  digitalWrite(PIN_STATUS_LED, radioIsHealthy() ? HIGH : LOW);

  if (systemState == SYS_BOOT) {
    setAllServosSafe();

    if (actuatorState == ACTUATOR_UNKNOWN) {
      startSafeHoming(false);
      systemState = SYS_HOMING;
    } else {
      actuatorTarget = ACT_TARGET_RETRACTED;
      systemState = SYS_DISARMED_SAFE;
    }
    return;
  }

  // Continue any active startup or failsafe homing/recovery sequence without accepting radio commands.
  if (homingStage != HOMING_NONE && homingStage != HOMING_DONE) {
    setAllServosSafe();
    updateSafeHomingSequence();
    systemState = homingIsFailsafeRecovery ? SYS_FAILSAFE : SYS_HOMING;
    return;
  }

  if (homingStage == HOMING_DONE) {
    homingStage = HOMING_NONE;
    homingIsFailsafeRecovery = false;
    actuatorTarget = ACT_TARGET_RETRACTED;
  }

  if (!radioIsHealthy()) {
    systemState = SYS_FAILSAFE;
    setAllServosSafe();

    if (actuatorState == ACTUATOR_UNKNOWN) {
      startSafeHoming(true);
      updateSafeHomingSequence();
    } else if (actuatorState == ACTUATOR_RETRACTED) {
      actuatorTarget = ACT_TARGET_RETRACTED;
    } else {
      actuatorTarget = ACT_TARGET_RETRACTED;
    }
    return;
  }

  if (!isArmed()) {
    systemState = SYS_DISARMED_SAFE;
    setAllServosSafe();
    actuatorTarget = ACT_TARGET_RETRACTED;
    return;
  }

  // Armed and radio healthy: interpret actuator switch as target state.
  if (channelSwitchHigh(CH_ACTUATOR_CMD)) {
    actuatorTarget = ACT_TARGET_EXTENDED;
  } else if (channelSwitchLow(CH_ACTUATOR_CMD)) {
    actuatorTarget = ACT_TARGET_RETRACTED;
  }
  // Middle/deadband switch position leaves actuatorTarget unchanged.

  if (actuatorTarget == ACT_TARGET_EXTENDED) {
    if (actuatorState == ACTUATOR_EXTENDED) {
      systemState = SYS_DEPLOYED;
    } else {
      systemState = SYS_DEPLOYING;
    }
  } else if (actuatorTarget == ACT_TARGET_RETRACTED) {
    if (actuatorState == ACTUATOR_RETRACTED) {
      systemState = SYS_ARMED_READY;
    } else {
      systemState = SYS_RETRACTING;
    }
  } else {
    systemState = SYS_ARMED_READY;
  }
}

void startSafeHoming(bool failsafeRecovery) {
  homingIsFailsafeRecovery = failsafeRecovery;
  homingStage = HOMING_EXTEND_FIRST;
  actuatorTarget = ACT_TARGET_EXTENDED;
  setAllServosSafe();
}

void updateSafeHomingSequence() {
  if (actuatorState == ACTUATOR_ERROR) {
    systemState = SYS_ERROR;
    return;
  }

  if (homingStage == HOMING_EXTEND_FIRST) {
    setAllServosSafe();

    if (actuatorState == ACTUATOR_EXTENDED) {
      homingStage = HOMING_RETRACT_SECOND;
      actuatorTarget = ACT_TARGET_RETRACTED;
    } else {
      actuatorTarget = ACT_TARGET_EXTENDED;
    }
    return;
  }

  if (homingStage == HOMING_RETRACT_SECOND) {
    setAllServosSafe();

    if (actuatorState == ACTUATOR_RETRACTED) {
      homingStage = HOMING_DONE;
      actuatorTarget = ACT_TARGET_RETRACTED;
    } else {
      actuatorTarget = ACT_TARGET_RETRACTED;
    }
    return;
  }
}

// ============================================================================
// ACTUATOR STATE MACHINE
// ============================================================================

void setupActuator() {
  actuatorMotor.setMinDuty(ACTUATOR_MIN_DUTY_PWM);
  actuatorMotor.setDeadtime(ACTUATOR_DEADTIME_US);
  actuatorMotor.setReverse(ACTUATOR_INVERT_DIRECTION);
  actuatorMotor.setAccel(ACTUATOR_ACCEL_PWM_PER_SEC);
  actuatorMotor.stop();
}

void updateActuatorState() {
  if (bothLimitsActive()) {
    enterActuatorError("both limit switches active");
    return;
  }

  if (actuatorState == ACTUATOR_ERROR) {
    stopActuator();
    return;
  }

  // If position was unknown but a limit is now active, recover known state.
  if (actuatorState == ACTUATOR_UNKNOWN) {
    if (retractedLimitActive()) {
      actuatorState = ACTUATOR_RETRACTED;
      actuatorTarget = ACT_TARGET_RETRACTED;
      stopActuator();
    } else if (extendedLimitActive()) {
      actuatorState = ACTUATOR_EXTENDED;
      actuatorTarget = ACT_TARGET_EXTENDED;
      stopActuator();
    }
  }

  switch (actuatorState) {
    case ACTUATOR_RETRACTED:
      stopActuator();
      if (actuatorTarget == ACT_TARGET_EXTENDED) {
        beginActuatorExtend();
      }
      break;

    case ACTUATOR_EXTENDED:
      stopActuator();
      if (actuatorTarget == ACT_TARGET_RETRACTED) {
        beginActuatorRetract();
      }
      break;

    case ACTUATOR_EXTENDING:
      if (extendedLimitActive()) {
        actuatorState = ACTUATOR_EXTENDED;
        actuatorTarget = ACT_TARGET_EXTENDED;
        brakeActuator();
      } else if ((millis() - actuatorMoveStartMs) > ACTUATOR_MOVE_TIMEOUT_MS) {
        enterActuatorError("extend timeout");
      } else if (actuatorTarget == ACT_TARGET_RETRACTED) {
        beginActuatorRetract();
      } else {
        extendActuator();
      }
      break;

    case ACTUATOR_RETRACTING:
      if (retractedLimitActive()) {
        actuatorState = ACTUATOR_RETRACTED;
        actuatorTarget = ACT_TARGET_RETRACTED;
        brakeActuator();
      } else if ((millis() - actuatorMoveStartMs) > ACTUATOR_MOVE_TIMEOUT_MS) {
        enterActuatorError("retract timeout");
      } else if (actuatorTarget == ACT_TARGET_EXTENDED) {
        beginActuatorExtend();
      } else {
        retractActuator();
      }
      break;

    case ACTUATOR_UNKNOWN:
      stopActuator();
      if (actuatorTarget == ACT_TARGET_EXTENDED) {
        beginActuatorExtend();
      } else if (actuatorTarget == ACT_TARGET_RETRACTED) {
        beginActuatorRetract();
      }
      break;

    case ACTUATOR_ERROR:
    default:
      stopActuator();
      break;
  }
}

void beginActuatorExtend() {
  if (bothLimitsActive()) {
    enterActuatorError("both limit switches active before extend");
    return;
  }

  // Never drive further into the extended/end limit.
  if (extendedLimitActive()) {
    actuatorState = ACTUATOR_EXTENDED;
    actuatorTarget = ACT_TARGET_EXTENDED;
    brakeActuator();
    return;
  }

  actuatorState = ACTUATOR_EXTENDING;
  actuatorMoveStartMs = millis();
  actuatorMoveExpectedTarget = ACT_TARGET_EXTENDED;
  extendActuator();
}

void beginActuatorRetract() {
  if (bothLimitsActive()) {
    enterActuatorError("both limit switches active before retract");
    return;
  }

  // Never drive further into the retracted/start limit.
  if (retractedLimitActive()) {
    actuatorState = ACTUATOR_RETRACTED;
    actuatorTarget = ACT_TARGET_RETRACTED;
    brakeActuator();
    return;
  }

  actuatorState = ACTUATOR_RETRACTING;
  actuatorMoveStartMs = millis();
  actuatorMoveExpectedTarget = ACT_TARGET_RETRACTED;
  retractActuator();
}

void extendActuator() {
  if (extendedLimitActive()) {
    actuatorState = ACTUATOR_EXTENDED;
    actuatorTarget = ACT_TARGET_EXTENDED;
    brakeActuator();
    return;
  }

  actuatorMotor.runSpeed(ACTUATOR_SPEED_PWM);
}

void retractActuator() {
  if (retractedLimitActive()) {
    actuatorState = ACTUATOR_RETRACTED;
    actuatorTarget = ACT_TARGET_RETRACTED;
    brakeActuator();
    return;
  }

  actuatorMotor.runSpeed(-ACTUATOR_SPEED_PWM);
}

void stopActuator() {
  actuatorMotor.stop();
}

void brakeActuator() {
  if (ACTUATOR_BRAKE_ON_LIMIT_OR_ERROR) {
    actuatorMotor.brake();
  } else {
    actuatorMotor.stop();
  }
}

void enterActuatorError(const char* reason) {
  lastError = reason;
  actuatorState = ACTUATOR_ERROR;
  systemState = SYS_ERROR;
  actuatorTarget = ACT_TARGET_HOLD;
  brakeActuator();
  setAllServosSafe();
}

// ============================================================================
// RADIO SWITCH HELPERS
// ============================================================================

bool channelSwitchHigh(uint8_t ch) {
  if (ch >= SBUS_CHANNEL_COUNT) return false;
  return channel_pwm[ch] >= SWITCH_HIGH_THRESHOLD_US;
}

bool channelSwitchLow(uint8_t ch) {
  if (ch >= SBUS_CHANNEL_COUNT) return false;
  return channel_pwm[ch] <= SWITCH_LOW_THRESHOLD_US;
}

const char* actuatorCommandName() {
  if (channelSwitchHigh(CH_ACTUATOR_CMD)) return "HIGH/EXTEND";
  if (channelSwitchLow(CH_ACTUATOR_CMD)) return "LOW/RETRACT";
  return "MID/HOLD";
}

// ============================================================================
// DEBUG
// ============================================================================

void printDebug() {
  const uint32_t nowMs = millis();
  if ((nowMs - lastDebugPrintMs) < DEBUG_PRINT_INTERVAL_MS) {
    return;
  }
  lastDebugPrintMs = nowMs;

  Serial.print(F("SBUS healthy:"));
  Serial.print(radioIsHealthy() ? 1 : 0);
  Serial.print(F(" frame:"));
  Serial.print(sbusFrameReceived ? 1 : 0);
  Serial.print(F(" fs:"));
  Serial.print(sbusFailSafe ? 1 : 0);
  Serial.print(F(" lost:"));
  Serial.print(sbusLostFrame ? 1 : 0);
  Serial.print(F(" timeout:"));
  Serial.print(sbusTimedOut ? 1 : 0);
  Serial.print(F(" invalid:"));
  Serial.print(sbusInvalidRange ? 1 : 0);

  Serial.print(F(" | ARM:"));
  Serial.print(armedLatched ? F("ON") : F("OFF"));
  Serial.print(F(" ch:"));
  Serial.print(channel_pwm[CH_ARM_ENABLE]);

  Serial.print(F(" | SYS:"));
  Serial.print(systemStateName(systemState));
  Serial.print(F(" home:"));
  Serial.print(homingStageName(homingStage));

  Serial.print(F(" | ACT:"));
  Serial.print(actuatorStateName(actuatorState));
  Serial.print(F(" target:"));
  Serial.print(actuatorTargetName(actuatorTarget));
  Serial.print(F(" cmd:"));
  Serial.print(actuatorCommandName());

  Serial.print(F(" | LIM ret:"));
  Serial.print(retractedLimitActive() ? 1 : 0);
  Serial.print(F(" ext:"));
  Serial.print(extendedLimitActive() ? 1 : 0);

  Serial.print(F(" | PETALS:"));
  Serial.print(petalState == PETALS_OPEN ? F("OPEN") : F("CLOSED"));
  Serial.print(F(" atOpen:"));
  Serial.print(petalsAtOpenPosition() ? 1 : 0);
  Serial.print(F(" RADAR:"));
  Serial.print(radarState == RADAR_OPEN ? F("OPEN") : F("CLOSED"));

  Serial.print(F(" | Servo targets us [pan,tilt,PL,PR,RL,RR]:"));
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    Serial.print(i == 0 ? F(" ") : F(","));
    Serial.print(smoothServos[i].targetUs);
  }

  if (systemState == SYS_ERROR) {
    Serial.print(F(" | ERROR:"));
    Serial.print(lastError);
  }

  Serial.println();
}

const char* systemStateName(SystemState state) {
  switch (state) {
    case SYS_BOOT:          return "SYS_BOOT";
    case SYS_HOMING:        return "SYS_HOMING";
    case SYS_DISARMED_SAFE: return "SYS_DISARMED_SAFE";
    case SYS_ARMED_READY:   return "SYS_ARMED_READY";
    case SYS_DEPLOYING:     return "SYS_DEPLOYING";
    case SYS_DEPLOYED:      return "SYS_DEPLOYED";
    case SYS_RETRACTING:    return "SYS_RETRACTING";
    case SYS_FAILSAFE:      return "SYS_FAILSAFE";
    case SYS_ERROR:         return "SYS_ERROR";
    default:                return "SYS_UNKNOWN";
  }
}

const char* actuatorStateName(ActuatorState state) {
  switch (state) {
    case ACTUATOR_UNKNOWN:    return "ACTUATOR_UNKNOWN";
    case ACTUATOR_RETRACTED:  return "ACTUATOR_RETRACTED";
    case ACTUATOR_EXTENDING:  return "ACTUATOR_EXTENDING";
    case ACTUATOR_EXTENDED:   return "ACTUATOR_EXTENDED";
    case ACTUATOR_RETRACTING: return "ACTUATOR_RETRACTING";
    case ACTUATOR_ERROR:      return "ACTUATOR_ERROR";
    default:                  return "ACTUATOR_UNKNOWN_STATE";
  }
}

const char* actuatorTargetName(ActuatorTarget target) {
  switch (target) {
    case ACT_TARGET_HOLD:      return "HOLD";
    case ACT_TARGET_RETRACTED: return "RETRACTED";
    case ACT_TARGET_EXTENDED:  return "EXTENDED";
    default:                   return "UNKNOWN";
  }
}

const char* homingStageName(HomingStage stage) {
  switch (stage) {
    case HOMING_NONE:          return "NONE";
    case HOMING_EXTEND_FIRST:  return "EXTEND_FIRST";
    case HOMING_RETRACT_SECOND:return "RETRACT_SECOND";
    case HOMING_DONE:          return "DONE";
    default:                   return "UNKNOWN";
  }
}