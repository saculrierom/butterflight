/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/pwm_output.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/time.h"

#include "io/motors.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/fc_core.h"
#include "fc/fc_rc.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/battery.h"


#ifdef USE_DSHOT
#ifdef USE_DSHOT_DMAR
    #ifdef FORCE_DSHOT_DMAR
        #define MIXER_USE_DSHOT_DMAR  true
    #else
        #define MIXER_USE_DSHOT_DMAR  false
    #endif //FORCE_DSHOT_DMAR
#endif //USE_DSHOT_DMAR       
#else
    #define MIXER_USE_DSHOT_DMAR  false
#endif //USE_DSHOT

#define GET_DSHOT_THROTTLE(min, low, offset) min + ((low - DSHOT_MIN_THROTTLE) / 100.0f) * CONVERT_PARAMETER_TO_PERCENT(offset);


PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixerMode = TARGET_DEFAULT_MIXER,
    .yaw_motors_reversed = false,
);

PG_REGISTER_WITH_RESET_FN(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);

void pgResetFn_motorConfig(motorConfig_t *motorConfig)
{
    motorConfig->minthrottle = MIXER_MIN_THROTTLE;
    motorConfig->maxthrottle = MIXER_MAX_THROTTLE;
    motorConfig->dev.motorPwmRate = MIXER_PWM_RATE;
    motorConfig->dev.motorPwmProtocol = MIXER_PWM_PROTOCOL;
    motorConfig->dev.useUnsyncedPwm = MIXER_SYNC_PWM;
    motorConfig->mincommand = MIXER_MIN_COMMAND;
    motorConfig->digitalIdleOffsetValue = MIXER_IDLE_OFFSET;
    motorConfig->dev.useBurstDshot = MIXER_USE_DSHOT_DMAR;
    
    int motorIndex = 0;
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && motorIndex < MAX_SUPPORTED_MOTORS; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_MOTOR) {
            motorConfig->dev.ioTags[motorIndex] = timerHardware[i].tag;
            motorIndex++;
        }
    }
}

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);

#define PWM_RANGE_MID 1500

static FAST_RAM uint8_t motorCount;
static FAST_RAM float motorMixRange;

float FAST_RAM motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};
static const motorMixer_t mixerQuadX1234[] = {
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
};

// Keep synced with mixerMode_e
const mixer_t mixers[] = {
    // motors, use servo, motor mixer
    { 4, false, mixerQuadX },          // MIXER_QUADX
    { 4, false, mixerQuadX1234 },
    { 0, false, NULL },                // MIXER_CUSTOM
};

FAST_RAM float motorOutputHigh, motorOutputLow;

static FAST_RAM float disarmMotorOutput, deadbandMotor3dHigh, deadbandMotor3dLow;
static FAST_RAM uint16_t rcCommand3dDeadBandLow;
static FAST_RAM uint16_t rcCommand3dDeadBandHigh;
static FAST_RAM float rcCommandThrottleRange, rcCommandThrottleRange3dLow, rcCommandThrottleRange3dHigh;

uint8_t getMotorCount(void)
{
    return motorCount;
}

float getMotorMixRange(void)
{
    return motorMixRange;
}

bool areMotorsRunning(void)
{
    bool motorsRunning = false;
    if (ARMING_FLAG(ARMED)) {
        motorsRunning = true;
    } else {
        for (int i = 0; i < motorCount; i++) {
            if (motor_disarmed[i] != disarmMotorOutput) {
                motorsRunning = true;

                break;
            }
        }
    }

    return motorsRunning;
}

bool mixerIsOutputSaturated(int axis, float errorRate)
{
    (void)axis;
    (void)errorRate;
    return motorMixRange >= 1.0f;
}

// All PWM motor scaling is done to standard PWM range of 1000-2000 for easier tick conversion with legacy code / configurator
// DSHOT scaling is done to the actual dshot range
void initEscEndpoints(void)
{
    bool is3dEnabled = feature(FEATURE_3D);
    // Can't use 'isMotorProtocolDshot()' here since motors haven't been initialised yet
    if ((motorConfig()->dev.motorPwmProtocol >> 8) > 0) {
        #ifdef USE_DSHOT
        disarmMotorOutput = DSHOT_DISARM_COMMAND;
        uint32_t low = is3dEnabled ? DSHOT_3D_DEADBAND_LOW : DSHOT_MAX_THROTTLE;
        deadbandMotor3dLow = DSHOT_3D_DEADBAND_LOW;
        motorOutputHigh = DSHOT_MAX_THROTTLE;
        motorOutputLow = GET_DSHOT_THROTTLE(DSHOT_MIN_THROTTLE, low, motorConfig()->digitalIdleOffsetValue);
        deadbandMotor3dHigh = GET_DSHOT_THROTTLE(DSHOT_3D_DEADBAND_HIGH, DSHOT_MAX_THROTTLE, motorConfig()->digitalIdleOffsetValue);
        #endif        
    } else {
        motorOutputHigh = motorConfig()->maxthrottle;
        deadbandMotor3dHigh = flight3DConfig()->deadband3d_high;
        deadbandMotor3dLow = flight3DConfig()->deadband3d_low;
        if (is3dEnabled) {
            disarmMotorOutput = flight3DConfig()->neutral3d;
            motorOutputLow = PWM_RANGE_MIN;
        } else {
            disarmMotorOutput = motorConfig()->mincommand;
            motorOutputLow = motorConfig()->minthrottle;
        }
    }
    rcCommandThrottleRange = PWM_RANGE_MAX - rxConfig()->mincheck;

    rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
    rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;

    rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
    rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;
}

void mixerInit(mixerMode_e mixerMode)
{
    currentMixerMode = mixerMode;
    initEscEndpoints();
}

void mixerConfigureOutput(void)
{
    motorCount = 0;

    if (currentMixerMode == MIXER_CUSTOM) {
        // load custom mixer into currentMixer
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            // check if done
            if (customMotorMixer(i)->throttle == 0.0f) {
                break;
            }
            currentMixer[i] = *customMotorMixer(i);
            motorCount++;
        }
    } else {
        motorCount = mixers[currentMixerMode].motorCount;
        if (motorCount > MAX_SUPPORTED_MOTORS) {
            motorCount = MAX_SUPPORTED_MOTORS;
        }
        // copy motor-based mixers
        if (mixers[currentMixerMode].motor) {
            for (int i = 0; i < motorCount; i++)
                currentMixer[i] = mixers[currentMixerMode].motor[i];
        }
    }

    // in 3D mode, mixer gain has to be halved
    if (feature(FEATURE_3D)) {
        if (motorCount > 1) {
            for (int i = 0; i < motorCount; i++) {
                currentMixer[i].pitch *= 0.5f;
                currentMixer[i].roll *= 0.5f;
                currentMixer[i].yaw *= 0.5f;
            }
        }
    }

    mixerResetDisarmedMotors();
}

void mixerLoadMix(int index, motorMixer_t *customMixers)
{
    // we're 1-based
    index++;
    // clear existing
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        customMixers[i].throttle = 0.0f;
    }
    // do we have anything here to begin with?
    if (mixers[index].motor != NULL) {
        for (int i = 0; i < mixers[index].motorCount; i++) {
            customMixers[i] = mixers[index].motor[i];
        }
    }
}

void mixerResetDisarmedMotors(void)
{
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = disarmMotorOutput;
    }
}

void writeMotors(void)
{
    if (pwmAreMotorsEnabled()) {
        for (int i = 0; i < motorCount; i++) {
            pwmWriteMotor(i, motor[i]);
        }
        pwmCompleteMotorUpdate(motorCount);
    }
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

void stopPwmAllMotors(void)
{
    pwmShutdownPulsesForAllMotors(motorCount);
    delayMicroseconds(1500);
}

static FAST_RAM float throttle = 0;
static FAST_RAM float motorOutputMin;
static FAST_RAM float motorRangeMin;
static FAST_RAM float motorRangeMax;
static FAST_RAM float motorOutputRange;
static FAST_RAM int8_t motorOutputMixSign;

static inline void update3DNormal(float newThrottle, float currentThrottleInputRange, timeUs_t currentTimeUs, timeUs_t reversalTimeUs) {
    (void)currentThrottleInputRange;
    (void)reversalTimeUs;
    motorRangeMin = deadbandMotor3dHigh;
    motorRangeMax = motorOutputHigh;
    motorOutputMin = deadbandMotor3dHigh;
    motorOutputRange = motorOutputHigh - deadbandMotor3dHigh;
    if (motorOutputMixSign != 1) {
        reversalTimeUs = currentTimeUs;
    }
    motorOutputMixSign = 1;
    throttle = newThrottle;
    currentThrottleInputRange = rcCommandThrottleRange3dHigh;
}

static inline void update3DInverted(float newThrottle, float currentThrottleInputRange, timeUs_t currentTimeUs, timeUs_t reversalTimeUs) {
    (void)currentThrottleInputRange;
    (void)reversalTimeUs;    
    motorRangeMin = motorOutputLow;
    motorRangeMax = deadbandMotor3dLow;
    if (isMotorProtocolDshot()) {
        motorOutputMin = motorOutputLow;
        motorOutputRange = deadbandMotor3dLow - motorOutputLow;
    } else {
        motorOutputMin = deadbandMotor3dLow;
        motorOutputRange = motorOutputLow - deadbandMotor3dLow;
    }
    if (motorOutputMixSign != -1) {
        reversalTimeUs = currentTimeUs;
    }
    motorOutputMixSign = -1;
    throttle = newThrottle;
    currentThrottleInputRange = rcCommandThrottleRange3dLow;
}

static inline void calculate3d(timeUs_t currentTimeUs, float currentThrottleInputRange) {
    static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    if (!ARMING_FLAG(ARMED)) {
        rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
    }
    if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow) {
        // INVERTED
        update3DInverted((float)rcCommand3dDeadBandLow - rcCommand[THROTTLE], currentThrottleInputRange, currentTimeUs, reversalTimeUs);
        rcThrottlePrevious = rcCommand[THROTTLE];
    } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
        // NORMAL
        update3DNormal((float)rcCommand[THROTTLE] - rcCommand3dDeadBandHigh, currentThrottleInputRange, currentTimeUs, reversalTimeUs);
        rcThrottlePrevious = rcCommand[THROTTLE];
    } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
            !flight3DConfigMutable()->switched_mode3d) ||
            isMotorsReversed()) {
        // INVERTED_TO_DEADBAND
        update3DInverted(0.0f, currentTimeUs, currentThrottleInputRange, reversalTimeUs);
    } else {
        // NORMAL_TO_DEADBAND
        update3DNormal(0.0f, currentTimeUs, currentThrottleInputRange, reversalTimeUs);
    }
    if (currentTimeUs - reversalTimeUs < 250000) {
        // keep ITerm zero for 250ms after motor reversal
        pidResetITerm();
    }
}

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    float currentThrottleInputRange = 0.0f;

    if (feature(FEATURE_3D)) {
        calculate3d(currentTimeUs, currentThrottleInputRange);
    } else {
        throttle = rcCommand[THROTTLE] - rxConfig()->mincheck;
        currentThrottleInputRange = rcCommandThrottleRange;
        motorRangeMin = motorOutputLow;
        motorRangeMax = motorOutputHigh;
        motorOutputMin = motorOutputLow;
        motorOutputRange = motorOutputHigh - motorOutputLow;
        motorOutputMixSign = 1;
    }

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

static void applyFlipOverAfterCrashModeToMotors(void)
{
    if (ARMING_FLAG(ARMED)) {
        float motorMix[MAX_SUPPORTED_MOTORS];
        for (int i = 0; i < motorCount; i++) {
            if (getRcDeflectionAbs(FD_ROLL) > getRcDeflectionAbs(FD_PITCH)) {
                motorMix[i] = getRcDeflection(FD_ROLL) * currentMixer[i].roll * -1;
            } else {
                motorMix[i] = getRcDeflection(FD_PITCH) * currentMixer[i].pitch * -1;
            }
            // Apply the mix to motor endpoints
            float motorOutput =  motorOutputMin + motorOutputRange * motorMix[i];
            // Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
            motorOutput = (motorOutput < motorOutputMin + CRASH_FLIP_DEADBAND ) ? disarmMotorOutput : motorOutput - CRASH_FLIP_DEADBAND;
            motor[i] = motorOutput;
        }
    } else {
        // Disarmed mode
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS])
{
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (int i = 0; i < motorCount; i++) {
        float motorOutput = motorOutputMin + (motorOutputRange * (motorOutputMixSign * motorMix[i] + throttle * currentMixer[i].throttle));
        if (failsafeIsActive()) {
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
            motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrain(motorOutput, motorRangeMin, motorRangeMax);
        }
        // Motor stop handling
        if (feature(FEATURE_MOTOR_STOP) && ARMING_FLAG(ARMED) && !feature(FEATURE_3D) && !isAirmodeActive()) {
            if (((rcData[THROTTLE]) < rxConfig()->mincheck)) {
                motorOutput = disarmMotorOutput;
            }
        }
        motor[i] = motorOutput;
    }

    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

void mixTable(timeUs_t currentTimeUs, uint8_t vbatPidCompensation)
{
    if (isFlipOverAfterCrashMode()) {
        applyFlipOverAfterCrashModeToMotors();
        return;
    }

    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    // Calculate and Limit the PIDsum
    const float scaledAxisPidRoll =
        constrainf(axisPIDSum[FD_ROLL], -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(axisPIDSum[FD_PITCH], -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    float scaledAxisPidYaw =
        constrainf(axisPIDSum[FD_YAW], -currentPidProfile->pidSumLimitYaw, currentPidProfile->pidSumLimitYaw) / PID_MIXER_SCALING;
    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    // Calculate voltage compensation
    const float vbatCompensationFactor = vbatPidCompensation ? calculateVbatPidCompensation() : 1.0f;

    // Find roll/pitch/yaw desired output
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < motorCount; i++) {
        float mix =
            scaledAxisPidRoll  * currentMixer[i].roll +
            scaledAxisPidPitch * currentMixer[i].pitch +
            scaledAxisPidYaw   * currentMixer[i].yaw;

        mix *= vbatCompensationFactor;  // Add voltage compensation

        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

    motorMixRange = motorMixMax - motorMixMin;

    if (motorMixRange > 1.0f) {
        for (int i = 0; i < motorCount; i++) {
            motorMix[i] /= motorMixRange;
        }
        // Get the maximum correction by setting offset to center when airmode enabled
        if (isAirmodeActive()) {
            throttle = 0.5f;
        }
    } else {
        if (isAirmodeActive() || throttle > 0.5f) {  // Only automatically adjust throttle when airmode enabled. Airmode logic is always active on high throttle
            const float throttleLimitOffset = motorMixRange / 2.0f;
            throttle = constrainf(throttle, 0.0f + throttleLimitOffset, 1.0f - throttleLimitOffset);
        }
    }

    // Apply the mix to motor endpoints
    applyMixToMotors(motorMix);
}

static inline uint16_t calculateDshotValue(float motorValue) {
    if (feature(FEATURE_3D)) {
        if (motorValue == DSHOT_DISARM_COMMAND || motorValue < DSHOT_MIN_THROTTLE) {
            return (uint16_t)PWM_RANGE_MID;
        } else if (motorValue <= DSHOT_3D_DEADBAND_LOW) {
            return (uint16_t)scaleRange(motorValue, DSHOT_MIN_THROTTLE, DSHOT_3D_DEADBAND_LOW, PWM_RANGE_MID - 1, PWM_RANGE_MIN);
        } else {
            return (uint16_t)scaleRange(motorValue, DSHOT_3D_DEADBAND_HIGH, DSHOT_MAX_THROTTLE, PWM_RANGE_MID + 1, PWM_RANGE_MAX);
        }
    } else {
        return (uint16_t)(motorValue < DSHOT_MIN_THROTTLE) ? PWM_RANGE_MIN : scaleRange(motorValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIN + 1, PWM_RANGE_MAX);
    }
}

float convertExternalToMotor(uint16_t externalValue)
{
    #ifdef USE_DSHOT
    if ((int)isMotorProtocolDshot()) {
        return (float)calculateDshotValue(constrain(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX));
    }
    #endif
    return (float)externalValue;
}

uint16_t convertMotorToExternal(float motorValue)
{
    #ifdef USE_DSHOT
    if ((int)isMotorProtocolDshot()) {
        return (uint16_t)calculateDshotValue(motorValue);
    }
    #endif
    return (uint16_t)motorValue;
}
