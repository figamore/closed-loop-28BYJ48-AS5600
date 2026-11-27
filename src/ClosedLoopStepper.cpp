// SPDX-License-Identifier: GPL-3.0-only
#include "AS5600ClosedLoopStepper.h"

#include <Arduino.h>
#include <algorithm>
#include <esp_arduino_version.h>
#include <math.h>
#include <stdlib.h>

#include "as5600_analog.h"
#include "as5600_bus.h"
#include "stepper_motion.h"

constexpr int ClosedLoopStepper::countsPerEncoderTurn;

namespace {
constexpr int countsPerEncoderTurn = ClosedLoopStepper::countsPerEncoderTurn;
constexpr int maxEncoderJumpCounts = countsPerEncoderTurn / 4;
constexpr uint32_t controlPeriodUs = 10000;
constexpr uint32_t controlDeadlineUs = 60000;
constexpr uint32_t startupSampleGapUs = 100000;
constexpr uint32_t feedbackTimeoutUs = 250000;
constexpr uint32_t encoderStartupTimeoutUs = 500000;
constexpr uint8_t validSamplesToArm = 3;
constexpr uint32_t driverSettleMs = 20;
// Leave room for backlash and load disturbances beyond the commanded speed.
constexpr double encoderJumpMarginCounts = 32.0;
constexpr double holdEncoderMotionCountsPerSecond = 4.0 * countsPerEncoderTurn;
constexpr uint32_t settleTimeUs = 200000;
constexpr int analogHoldConfirmSamples = 5;
constexpr int directionLearnCounts = 4;
constexpr double directionLearnSpeed = 16.0;

constexpr double velocityResponseSeconds = 0.06;
constexpr double brakingResponseSeconds = 0.04;

constexpr uint32_t minStallTimeUs = 750000;
constexpr uint32_t slowStallTimeUs = 5000000;
constexpr double stallTravelFullSteps = 48.0;
constexpr double slowStallTravelFullSteps = 4.0;
constexpr double bypassNoProgressFullSteps = 16.0;

double limit(double value, double magnitude) {
    return fmin(fmax(value, -magnitude), magnitude);
}

int encoderDelta(int previous, int current) {
    const int delta = current - previous;
    if (delta > countsPerEncoderTurn / 2) {
        return delta - countsPerEncoderTurn;
    }
    if (delta < -countsPerEncoderTurn / 2) {
        return delta + countsPerEncoderTurn;
    }
    return delta;
}

int64_t signedPulseDifference(uint32_t current, uint32_t origin) {
    const uint32_t difference = current - origin;
    return difference <= INT32_MAX ? difference :
        static_cast<int64_t>(difference) - (int64_t{1} << 32);
}

bool validFeedbackPins(const ClosedLoopStepper::Config& config, ClosedLoopStepper::FeedbackMode mode) {
    const auto& driver = config.driver;
    const auto& encoder = config.encoder;
    const auto usedForDriver = [&driver](uint8_t pin) {
        return pin == driver.stepPin || pin == driver.dirPin || pin == driver.enablePin;
    };
    if (mode == ClosedLoopStepper::FeedbackMode::Analog) {
        return encoder.analogPin >= 32 && encoder.analogPin <= 39 && !usedForDriver(encoder.analogPin);
    }
    if (mode == ClosedLoopStepper::FeedbackMode::I2C) {
        return encoder.sharedWire || (encoder.i2cBusNumber < 2 && encoder.sdaPin != encoder.sclPin &&
            StepperMotion::isOutputPin(encoder.sdaPin) && StepperMotion::isOutputPin(encoder.sclPin) &&
            !usedForDriver(encoder.sdaPin) && !usedForDriver(encoder.sclPin));
    }
    return false;
}

uint64_t pinsFor(const ClosedLoopStepper::Config& config, ClosedLoopStepper::FeedbackMode mode) {
    const auto& driver = config.driver;
    const auto& encoder = config.encoder;
    const auto bit = [](uint8_t pin) { return uint64_t{1} << pin; };
    const uint64_t driverPins = bit(driver.stepPin) | bit(driver.dirPin) | bit(driver.enablePin);
    if (mode == ClosedLoopStepper::FeedbackMode::Analog) {
        return driverPins | bit(encoder.analogPin);
    }
    return encoder.sharedWire ? driverPins : driverPins | bit(encoder.sdaPin) | bit(encoder.sclPin);
}
}  // namespace

struct ClosedLoopStepper::Hardware {
    StepperMotion motion;
    As5600Bus encoder;
    uint64_t pins = 0;
    TwoWire* wire = nullptr;

    // All controllers reserve and release resources from the same application task.
    static Hardware slots[StepperMotion::timerCount];

    static Hardware* available(uint8_t timerNumber) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        (void)timerNumber;
        for (auto& slot : slots) {
            if (!slot.pins) return &slot;
        }
        return nullptr;
#else
        auto& slot = slots[timerNumber];
        return slot.pins ? nullptr : &slot;
#endif
    }

    static bool conflicts(uint64_t pins, TwoWire* wire, const Hardware* current = nullptr) {
        for (const auto& slot : slots) {
            if (&slot != current && ((slot.pins & pins) || (wire && wire == slot.wire))) {
                return true;
            }
        }
        return false;
    }
};

ClosedLoopStepper::Hardware ClosedLoopStepper::Hardware::slots[StepperMotion::timerCount];

bool ClosedLoopStepper::validConfig(const Config& config) const {
    const auto& encoder = config.encoder;
    const auto& motion = config.motion;
    const double positive[] = {motion.fullStepsPerOutputTurn, motion.encoderTurnsPerOutputTurn,
        motion.maxFullStepsPerSecond, motion.maxAcceleration, motion.maxDeceleration, motion.maxJerk,
        motion.maxBrakingJerk, motion.brakingAcceleration, motion.positionGain, motion.holdPositionGain,
        motion.holdMaxAcceleration, motion.holdMaxJerk, motion.stallRecoveryFullStepsPerSecond};
    for (double value : positive) {
        if (!isfinite(value) || value <= 0.0) {
            return false;
        }
    }
    const double countsPerTurn = countsPerEncoderTurn * motion.encoderTurnsPerOutputTurn;
    const double stepsPerCount = motion.fullStepsPerOutputTurn / countsPerTurn;
    const double countsPerSecond = motion.maxFullStepsPerSecond / stepsPerCount;
    return isfinite(stepsPerCount) && stepsPerCount > 0.0 &&
        countsPerSecond * (controlPeriodUs / 1000000.0) < maxEncoderJumpCounts &&
        StepperMotion::validConfig(config.driver, motion.maxFullStepsPerSecond) &&
        validFeedbackPins(config, encoder.mode) && encoder.adcMin < encoder.adcMax &&
        encoder.adcMax <= As5600Analog::adcMax && encoder.polarity >= -1 && encoder.polarity <= 1 &&
        (encoder.sharedWire || (encoder.i2cFrequency >= 10000 && encoder.i2cFrequency <= 400000 &&
                                 encoder.i2cTimeoutMs > 0 && encoder.i2cTimeoutMs <= 50)) &&
        motion.analogTargetTolerance <= motion.analogHoldTolerance &&
        motion.i2cTargetTolerance <= motion.i2cHoldTolerance &&
        motion.analogHoldTolerance < maxEncoderJumpCounts && motion.i2cHoldTolerance < maxEncoderJumpCounts;
}

bool ClosedLoopStepper::usingAnalogEncoder() const {
    return config_.encoder.mode == FeedbackMode::Analog;
}

int ClosedLoopStepper::targetToleranceCounts() const {
    return usingAnalogEncoder() ? config_.motion.analogTargetTolerance : config_.motion.i2cTargetTolerance;
}

int ClosedLoopStepper::holdToleranceCounts() const {
    return usingAnalogEncoder() ? config_.motion.analogHoldTolerance : config_.motion.i2cHoldTolerance;
}

int64_t ClosedLoopStepper::errorCounts() const {
    return targetOffset - encoderPosition * encoderPolarity;
}

const char* ClosedLoopStepper::readFeedback(uint16_t& rawAngle) {
    if (!usingAnalogEncoder()) {
        return hardware_->encoder.readFeedback(rawAngle);
    }
    As5600Analog::Sample sample{};
    const char* error = As5600Analog::readSample(config_.encoder, sample);
    lastAdc = sample.adc;
    if (error) return error;
    rawAngle = sample.angle;
    return nullptr;
}

bool ClosedLoopStepper::qualifyAngle(uint16_t rawAngle, uint32_t sampledAt) {
    if (sampledAt - qualificationLastUs >= startupSampleGapUs ||
        abs(encoderDelta(qualificationFirstAngle, rawAngle)) > encoderJumpMarginCounts) {
        qualificationSamples = 0;
    }
    if (qualificationSamples == 0) {
        qualificationFirstAngle = rawAngle;
    } else if (qualificationSamples == 1) {
        qualificationSecondAngle = rawAngle;
    }
    qualificationLastUs = sampledAt;
    if (qualificationSamples < validSamplesToArm) {
        ++qualificationSamples;
    }
    return qualificationSamples == validSamplesToArm;
}

ClosedLoopStepper::AngleResult ClosedLoopStepper::trackEncoder(uint16_t rawAngle, uint32_t sampledAt) {
    const int delta = encoderDelta(lastRawAngle, rawAngle);
    // A held shaft can be displaced by hand faster than the motor can drive it.
    // A newly lowered speed limit can be below the speed of the interval just sampled.
    const double driveRate = fmax(config_.motion.maxFullStepsPerSecond, fabs(velocity)) / fullStepsPerCount;
    const double rate = reachedOnce ? fmax(driveRate, holdEncoderMotionCountsPerSecond) : driveRate;
    const uint32_t ageUs = sampledAt - lastValidAngleUs;
    const double elapsed = ageUs / 1000000.0;
    if (abs(delta) > maxEncoderJumpCounts) return AngleResult::Rejected;
    if (abs(delta) > encoderJumpMarginCounts + rate * elapsed) {
        // A released rotor can catch up faster than the commanded STEP rate. Keep the
        // current pulse train for one more observation if both positions leave room.
        if (feedback.paused || velocity == 0.0 || encoderPolarity == 0 || ageUs > 3 * controlPeriodUs) {
            return AngleResult::Rejected;
        }
        if (feedback.pendingAngle < 0) {
            const int direction = velocity > 0.0 ? 1 : -1;
            const double nextPollTravel = fabs(velocity) / fullStepsPerCount * controlPeriodUs / 1e6;
            const int64_t remaining = errorCounts() * direction;
            const int64_t candidateRemaining = remaining - delta * encoderPolarity * direction;
            if (std::min(remaining, candidateRemaining) <= targetToleranceCounts() + nextPollTravel) {
                return AngleResult::Rejected;
            }
            feedback.pendingAngle = rawAngle;
            feedback.pendingAtUs = sampledAt;
            return AngleResult::Pending;
        }
        // Confirm against the candidate on the circular scale, then commit the
        // displacement from the last trusted angle exactly once, including at wrap.
        const double interval = (sampledAt - feedback.pendingAtUs) / 1e6;
        const int firstDelta = encoderDelta(lastRawAngle, feedback.pendingAngle);
        const int followDelta = encoderDelta(feedback.pendingAngle, rawAngle);
        const double releaseRate = fmax(driveRate, holdEncoderMotionCountsPerSecond);
        const double followRate = firstDelta * followDelta > 0 ? releaseRate : rate;
        if (abs(delta) > encoderJumpMarginCounts + releaseRate * elapsed ||
            abs(followDelta) > encoderJumpMarginCounts + followRate * interval) {
            return AngleResult::Rejected;
        }
    }
    feedback.pendingAngle = -1;
    encoderPosition += delta;
    recordEncoderAngle(rawAngle, sampledAt);
    return AngleResult::Accepted;
}

void ClosedLoopStepper::recordEncoderAngle(uint16_t rawAngle, uint32_t sampledAt) {
    lastRawAngle = rawAngle;
    lastValidAngleUs = sampledAt;
    pulsesAtLastAngle = hardware_->motion.pulsesSent();
}

void ClosedLoopStepper::anchorEncoder(uint16_t rawAngle, uint32_t sampledAt) {
    recordEncoderAngle(rawAngle, sampledAt);
    directionCandidate = 0;
    netPulsesAtOrigin = hardware_->motion.netPulsesSent();
    qualificationSamples = 0;
    encoderPosition = 0;
    if (usingAnalogEncoder()) {
        // Anchor to the median of the three stationary polls, unwrapped around the first.
        // Keep the last physical reading as the baseline for the next sample delta.
        const int lastOffset = encoderDelta(qualificationFirstAngle, rawAngle);
        int offsets[] = {0, encoderDelta(qualificationFirstAngle, qualificationSecondAngle), lastOffset};
        std::sort(offsets, offsets + 3);
        encoderPosition = lastOffset - offsets[1];
    }
}

void ClosedLoopStepper::haltDrive() {
    feedback.pendingAngle = -1;
    hardware_->motion.setVelocity(0.0);
    velocity = 0.0;
    acceleration = 0.0;
    settledFeedbackUs = 0;
    holdOutsideSamples = 0;
}

void ClosedLoopStepper::fail(const char* cause) {
    haltDrive();
    hardware_->motion.setDriverEnabled(false);
    starting = false;
    targetActive = false;
    holding = false;
    stalled = false;
    bypassPhase = BypassPhase::idle;
    feedback.expired = feedback.paused = false;
    lastFault = cause;
    emit(Event::Fault);
}

void ClosedLoopStepper::resetProgress(uint32_t now) {
    bestError = llabs(errorCounts());
    lastProgressUs = now;
    pulsesAtProgress = hardware_->motion.pulsesSent();
}

void ClosedLoopStepper::discardFeedback(const char* cause) {
    feedback.cause = cause;
    ++feedback.discarded;
    haltDrive();
    if (!feedback.paused) {
        feedback.paused = true;
        emit(Event::FeedbackPaused);
    }
}

void ClosedLoopStepper::loseFeedback(const char* cause) {
    haltDrive();
    feedback.cause = cause;
    feedback.paused = true;
    feedback.expired = true;
    feedback.lostAtUs = micros();
    qualificationSamples = 0;
    emit(Event::FeedbackLost);
}

void ClosedLoopStepper::checkProgress(uint32_t now) {
    const int64_t absoluteError = llabs(errorCounts());
    const int progressCounts = stalled ? targetToleranceCounts() + 2 : 2;
    if (encoderPolarity != 0 && (absoluteError + progressCounts <= bestError ||
                                 (stalled && absoluteError <= targetToleranceCounts()))) {
        stalled = false;
        resetProgress(now);
        return;
    }
    if (stalled) {
        // A load can push the shaft farther from the target while it is blocked.
        // Measure renewed forward progress from that displaced position.
        if (absoluteError > bestError) bestError = absoluteError;
        if (config_.motion.stallFaultMs && now - stallReportedUs >= config_.motion.stallFaultMs * 1000UL) {
            fail("No sustained encoder progress after stall");
        }
        return;
    }
    if (encoderPolarity != 0 && absoluteError <= targetToleranceCounts() && velocity == 0.0) {
        return;
    }

    const double fullStepsSinceProgress = static_cast<double>(hardware_->motion.pulsesSent() - pulsesAtProgress) /
                                          config_.driver.microsteps;
    const uint32_t elapsed = now - lastProgressUs;
    const bool noProgress = (elapsed >= minStallTimeUs && fullStepsSinceProgress >= stallTravelFullSteps) ||
        (elapsed >= slowStallTimeUs && fullStepsSinceProgress >= slowStallTravelFullSteps);
    if (noProgress) {
        // Use this position to detect renewed progress, even after backtracking.
        bestError = absoluteError;
        stalled = true;
        stallReportedUs = now;
        haltDrive();
        emit(Event::Stalled);
    }
}

void ClosedLoopStepper::rampVelocity(double desiredVelocity, double dt) {
    const auto& motion = config_.motion;
    const double speedLimit = stalled ? fmin(motion.maxFullStepsPerSecond, motion.stallRecoveryFullStepsPerSecond) :
                                        motion.maxFullStepsPerSecond;
    desiredVelocity = limit(desiredVelocity, speedLimit);
    const double velocityError = desiredVelocity - velocity;
    const bool braking = velocity * velocityError < 0.0;
    const double responseSeconds = braking ? brakingResponseSeconds : velocityResponseSeconds;
    const double accelerationLimit = braking ? motion.maxDeceleration :
        (reachedOnce ? motion.holdMaxAcceleration : motion.maxAcceleration);
    const double jerkLimit = braking ? motion.maxBrakingJerk : (reachedOnce ? motion.holdMaxJerk : motion.maxJerk);
    const double desiredAcceleration = limit(velocityError / responseSeconds, accelerationLimit);
    acceleration += limit(desiredAcceleration - acceleration, jerkLimit * dt);
    // When already above a new limit, allow only the ramp back toward it.
    const double upper = fmax(speedLimit, velocity);
    const double lower = fmin(-speedLimit, velocity);
    const double next = fmax(lower, fmin(velocity + acceleration * dt, upper));
    if (desiredVelocity == 0.0 &&
        (fabs(next) < 0.05 || (velocity > 0.0 && next < 0.0) || (velocity < 0.0 && next > 0.0))) {
        velocity = 0.0;
        acceleration = 0.0;
    } else {
        velocity = next;
    }
}

double ClosedLoopStepper::approachSpeed(double distance) const {
    const auto& motion = config_.motion;
    const double brakingSpeed = sqrt(2.0 * fmin(motion.brakingAcceleration, motion.maxDeceleration) * distance);
    const double positionGain = reachedOnce ? config_.motion.holdPositionGain : config_.motion.positionGain;
    return fmin(fmin(positionGain * distance, brakingSpeed),
                config_.motion.maxFullStepsPerSecond);
}

void ClosedLoopStepper::controlBypass(double dt) {
    if (bypassPhase == BypassPhase::starting) {
        if (!hardware_->motion.hasStopped()) {
            return;
        }
        hardware_->motion.setPulseLimit(bypassPulseCount);
        bypassLastRemaining = bypassPulseCount;
        bypassFullStepsSinceProgress = 0.0;
        bypassPhase = BypassPhase::moving;
    }
    if (bypassPhase == BypassPhase::finishing) {
        if (hardware_->motion.hasStopped()) {
            hardware_->motion.setDriverEnabled(false);
            bypassPhase = BypassPhase::idle;
            emit(Event::OpenLoopComplete);
        }
        return;
    }

    const uint32_t remaining = hardware_->motion.pulsesRemaining();
    if (remaining == 0) {
        haltDrive();
        bypassPhase = BypassPhase::finishing;
        return;
    }
    if (remaining < bypassLastRemaining) {
        bypassLastRemaining = remaining;
        bypassFullStepsSinceProgress = 0.0;
    } else {
        bypassFullStepsSinceProgress += fmax(velocity, 0.0) * dt;
        if (bypassFullStepsSinceProgress >= bypassNoProgressFullSteps) {
            fail("STEP generator is not producing pulses");
            return;
        }
    }

    const double distance = static_cast<double>(remaining) / config_.driver.microsteps;
    rampVelocity(approachSpeed(distance), dt);
    const double speed = fmax(velocity, 0.0);
    hardware_->motion.setVelocity(requestedTurns > 0.0 ? speed : -speed);
}

void ClosedLoopStepper::controlPosition(uint32_t now, double dt) {
    if (encoderPolarity == 0) {
        const int threshold = usingAnalogEncoder() ? targetToleranceCounts() + directionLearnCounts : directionLearnCounts;
        const int64_t pulses = signedPulseDifference(hardware_->motion.netPulsesSent(), netPulsesAtOrigin);
        const double expected = fabs(static_cast<double>(pulses)) / config_.driver.microsteps / fullStepsPerCount;
        const bool enoughTravel = llabs(encoderPosition) >= threshold && expected >= threshold / 2.0;
        const int candidate = enoughTravel ? ((pulses > 0) == (encoderPosition > 0) ? 1 : -1) : 0;
        if (candidate != directionCandidate) {
            directionCandidate = candidate;
            directionCandidateUs = now;
        }
        if (!candidate || now - directionCandidateUs < 40000) {
            rampVelocity(targetOffset > 0 ? directionLearnSpeed : -directionLearnSpeed, dt);
            hardware_->motion.setVelocity(velocity);
            checkProgress(now);
            return;
        }
        encoderPolarity = candidate;
        stalled = false;
        resetProgress(now);
    }

    const int64_t error = errorCounts();
    const int64_t absoluteError = llabs(error);
    if (holding) {
        if (absoluteError <= holdToleranceCounts()) {
            holdOutsideSamples = 0;
            return;
        }
        if (usingAnalogEncoder()) {
            const int side = error > 0 ? 1 : -1;
            if (side * holdOutsideSamples < 0) holdOutsideSamples = 0;
            holdOutsideSamples += side;
            if (abs(holdOutsideSamples) < analogHoldConfirmSamples) return;
        }
        holdOutsideSamples = 0;
        holding = false;
        resetProgress(now);
    }

    const int tolerance = targetToleranceCounts();
    double desiredVelocity = 0.0;
    if (absoluteError > tolerance) {
        // Slow a hold correction as it approaches the acceptable position band.
        const double distanceCounts = reachedOnce ? absoluteError - tolerance : absoluteError;
        const double speed = approachSpeed(distanceCounts * fullStepsPerCount);
        desiredVelocity = error > 0 ? speed : -speed;
    }
    rampVelocity(desiredVelocity, dt);
    hardware_->motion.setVelocity(velocity);
    if (absoluteError <= tolerance && velocity == 0.0) {
        // Count only fresh, in-tolerance feedback time. Discarded polls
        // neither confirm arrival nor erase the valid observations so far.
        settledFeedbackUs += static_cast<uint32_t>(dt * 1000000.0 + 0.5);
        if (settledFeedbackUs >= settleTimeUs) {
            holding = true;
            stalled = false;
            const Event event = reachedOnce ? Event::Recovered : Event::Reached;
            reachedOnce = true;
            if (!holdAtTarget) {
                holding = false;
                targetActive = false;
                hardware_->motion.setDriverEnabled(false);
            }
            emit(event);
        }
    } else {
        settledFeedbackUs = 0;
    }
    checkProgress(now);
}

bool ClosedLoopStepper::moveInProgress() const {
    return starting || bypassPhase != BypassPhase::idle ||
           (targetActive && (!holding || feedback.cause));
}

ClosedLoopStepper::~ClosedLoopStepper() {
    end();
}

ClosedLoopStepper::Result ClosedLoopStepper::begin() {
    return begin(Config{});
}

ClosedLoopStepper::Result ClosedLoopStepper::begin(const Config& config) {
    if (hardware_) {
        return Result::Busy;
    }
    if (!validConfig(config)) {
        return Result::InvalidConfig;
    }

    auto* slot = Hardware::available(config.driver.timerNumber);
    const auto pins = pinsFor(config, config.encoder.mode);
    auto* wire = config.encoder.mode == FeedbackMode::I2C ? As5600Bus::wireFor(config.encoder) : nullptr;
    if (!slot || Hardware::conflicts(pins, wire)) {
        return Result::InUse;
    }

    slot->motion.begin(config.driver, static_cast<uint8_t>(slot - Hardware::slots));
    if (wire) {
        if (!slot->encoder.begin(wire, config.encoder)) {
            slot->motion.end();
            return Result::EncoderError;
        }
    } else {
        As5600Analog::begin(config.encoder.analogPin);
    }
    config_ = config;
    countsPerOutputTurn = countsPerEncoderTurn * config.motion.encoderTurnsPerOutputTurn;
    fullStepsPerCount = config.motion.fullStepsPerOutputTurn / countsPerOutputTurn;
    encoderPolarity = config.encoder.polarity;
    pulsesAtMoveStart = slot->motion.pulsesSent();
    slot->pins = pins;
    slot->wire = wire;
    hardware_ = slot;
    return Result::Ok;
}

void ClosedLoopStepper::end() {
    if (!hardware_) {
        return;
    }
    stop();
    hardware_->encoder.end();
    hardware_->motion.end();
    hardware_->wire = nullptr;
    hardware_->pins = 0;
    hardware_ = nullptr;
}

ClosedLoopStepper::Result ClosedLoopStepper::setMaxSpeed(double fullStepsPerSecond) {
    if (!hardware_) return Result::NotInitialized;
    Config candidate = config_;
    candidate.motion.maxFullStepsPerSecond = fullStepsPerSecond;
    if (!validConfig(candidate)) return Result::InvalidConfig;
    config_.motion.maxFullStepsPerSecond = fullStepsPerSecond;
    return Result::Ok;
}

ClosedLoopStepper::Result ClosedLoopStepper::setMaxAcceleration(double fullStepsPerSecondSquared) {
    if (!hardware_) return Result::NotInitialized;
    if (!isfinite(fullStepsPerSecondSquared) || fullStepsPerSecondSquared <= 0.0) return Result::InvalidConfig;
    config_.motion.maxAcceleration = fullStepsPerSecondSquared;
    return Result::Ok;
}

ClosedLoopStepper::Result ClosedLoopStepper::setMaxDeceleration(double fullStepsPerSecondSquared) {
    if (!hardware_) return Result::NotInitialized;
    if (!isfinite(fullStepsPerSecondSquared) || fullStepsPerSecondSquared <= 0.0) return Result::InvalidConfig;
    config_.motion.maxDeceleration = fullStepsPerSecondSquared;
    return Result::Ok;
}

ClosedLoopStepper::Result ClosedLoopStepper::setFeedbackMode(FeedbackMode mode) {
    if (!hardware_) {
        return Result::NotInitialized;
    }
    if (mode == config_.encoder.mode) {
        return Result::Ok;
    }
    if (targetActive || moveInProgress() || !hardware_->motion.hasStopped()) {
        return Result::Busy;
    }
    if (!validFeedbackPins(config_, mode)) {
        return Result::InvalidConfig;
    }
    const auto pins = pinsFor(config_, mode);
    auto* wire = mode == FeedbackMode::I2C ? As5600Bus::wireFor(config_.encoder) : nullptr;
    if (Hardware::conflicts(pins, wire, hardware_)) {
        return Result::InUse;
    }

    hardware_->motion.pausePulseGeneratorWhenStopped();
    if (wire) {
        if (!hardware_->encoder.begin(wire, config_.encoder)) {
            return Result::EncoderError;
        }
    } else {
        hardware_->encoder.end();
        As5600Analog::begin(config_.encoder.analogPin);
    }
    hardware_->pins = pins;
    hardware_->wire = wire;
    config_.encoder.mode = mode;
    encoderPolarity = config_.encoder.polarity;
    lastAdc = lastRawAngle = -1;
    lastFault = nullptr;
    feedback = {};
    return Result::Ok;
}

void ClosedLoopStepper::resetMove(double turns, uint32_t now) {
    haltDrive();
    targetActive = false;
    holding = false;
    reachedOnce = false;
    stalled = false;
    lastAdc = lastRawAngle = -1;
    lastFault = nullptr;
    feedback = {};
    requestedTurns = turns;
    pulsesAtMoveStart = hardware_->motion.pulsesSent();
    lastSampleUs = now;
    startupBeganUs = now;
}

ClosedLoopStepper::Result ClosedLoopStepper::moveRelative(double turns) {
    return startClosedLoopMove(turns, false);
}

ClosedLoopStepper::Result ClosedLoopStepper::holdRelative(double turns) {
    return startClosedLoopMove(turns, true);
}

ClosedLoopStepper::Result ClosedLoopStepper::startClosedLoopMove(double turns, bool explicitHold) {
    if (!hardware_) {
        return Result::NotInitialized;
    }
    if (moveInProgress() || !hardware_->motion.hasStopped()) {
        return Result::Busy;
    }
    const double counts = turns * countsPerOutputTurn;
    const bool holdInPlace = explicitHold && turns == 0.0;
    if (!isfinite(counts) || fabs(counts) > INT32_MAX || (!holdInPlace && fabs(counts) < 0.5)) {
        return Result::InvalidMove;
    }
    if (holdInPlace && encoderPolarity == 0) {
        return Result::EncoderPolarityUnknown;
    }

    resetMove(turns, micros());
    holdAtTarget = explicitHold || config_.motion.holdOnTarget;
    hardware_->motion.pausePulseGeneratorWhenStopped();
    hardware_->motion.setDriverEnabled(true);
    targetOffset = static_cast<int32_t>(llround(counts));
    qualificationSamples = 0;
    starting = true;
    return Result::Ok;
}

ClosedLoopStepper::Result ClosedLoopStepper::moveOpenLoop(double turns) {
    if (!hardware_) {
        return Result::NotInitialized;
    }
    if (moveInProgress()) {
        return Result::Busy;
    }
    const double pulses = fabs(turns) * config_.motion.fullStepsPerOutputTurn * config_.driver.microsteps;
    if (!isfinite(pulses) || pulses > StepperMotion::maxCountedPulses || pulses < 0.5) {
        return Result::InvalidMove;
    }
    resetMove(turns, micros());
    if (!hardware_->motion.startPulseGenerator()) {
        fail("STEP timer is not running");
        return Result::TimerError;
    }

    hardware_->motion.setDriverEnabled(true);
    startupBeganUs = lastSampleUs = micros();
    bypassPulseCount = static_cast<uint32_t>(llround(pulses));
    bypassPhase = BypassPhase::starting;
    return Result::Ok;
}

void ClosedLoopStepper::updateStartup(uint32_t now) {
    constexpr uint32_t settleUs = driverSettleMs * 1000;
    constexpr uint32_t deadlineUs = settleUs + encoderStartupTimeoutUs;
    if (now - startupBeganUs < settleUs) {
        return;
    }
    uint16_t candidate = 0;
    const char* error = readFeedback(candidate);
    const uint32_t sampledAt = micros();
    const bool sampleValid = !error && sampledAt - now < controlDeadlineUs;
    if (!sampleValid || !qualifyAngle(candidate, sampledAt)) {
        const char* cause = "AS5600 angle unstable while stopped";
        if (!sampleValid) {
            cause = error ? error : "AS5600 startup read arrived late";
            qualificationSamples = 0;
            ++feedback.discarded;
        }
        if (!feedback.expired && sampledAt - startupBeganUs >= deadlineUs) {
            loseFeedback(cause);
        }
        return;
    }
    if (!hardware_->motion.startPulseGenerator()) {
        fail("STEP timer is not running");
        return;
    }
    hardware_->motion.clearPulseLimit();
    anchorEncoder(candidate, sampledAt);
    lastControlUs = lastSampleUs = micros();
    resetProgress(lastSampleUs);
    starting = false;
    targetActive = true;
    const bool recovered = feedback.expired;
    feedback.expired = feedback.paused = false;
    feedback.cause = nullptr;
    if (recovered) {
        emit(Event::FeedbackRestored);
    }
    if (targetOffset == 0) {
        holding = true;
        reachedOnce = true;
        emit(Event::Reached);
    }
}

void ClosedLoopStepper::update() {
    if (!hardware_) {
        return;
    }
    if (!starting && !targetActive && bypassPhase == BypassPhase::idle) {
        hardware_->motion.pausePulseGeneratorWhenStopped();
        return;
    }
    const uint32_t now = micros();
    const uint32_t elapsed = now - lastSampleUs;
    if (elapsed < controlPeriodUs) {
        return;
    }
    lastSampleUs = now;
    const double dt = elapsed / 1000000.0;
    if (feedback.expired && config_.motion.feedbackLossFaultMs &&
        now - feedback.lostAtUs >= config_.motion.feedbackLossFaultMs * 1000UL) {
        fail("Encoder feedback did not recover");
        return;
    }
    if (starting) {
        updateStartup(now);
        return;
    }
    if (bypassPhase != BypassPhase::idle) {
        if (elapsed >= controlDeadlineUs) {
            fail("motion update missed its deadline");
        } else if (!hardware_->motion.isPulseGeneratorRunning()) {
            fail("STEP timer stopped");
        } else if (bypassPhase != BypassPhase::starting || now - startupBeganUs >= driverSettleMs * 1000) {
            controlBypass(dt);
        }
        return;
    }

    const uint32_t previousValidAngleUs = lastValidAngleUs;
    uint16_t rawAngle = 0;
    const char* sensorFault = readFeedback(rawAngle);
    const uint32_t sampledAt = micros();
    if (sampledAt - lastControlUs > controlDeadlineUs) {
        // A late service can trip the independent STEP watchdog. Restart its
        // ramp from rest, measuring the gap between completed observations.
        haltDrive();
    }
    lastControlUs = sampledAt;
    if (!hardware_->motion.isPulseGeneratorRunning()) {
        fail("STEP timer stopped");
        return;
    }
    if (!sensorFault && sampledAt - now >= controlDeadlineUs) {
        sensorFault = "AS5600 feedback sample arrived late";
    }
    // Beyond half a turn the single-turn angle cannot identify a unique position.
    const double travel = (hardware_->motion.pulsesSent() - pulsesAtLastAngle) /
                          fullStepsPerCount / config_.driver.microsteps;
    if (travel >= countsPerEncoderTurn / 2) {
        fail("Unobserved STEP travel reached half an encoder turn");
        return;
    }
    if (!feedback.expired && sampledAt - lastValidAngleUs >= feedbackTimeoutUs) {
        ++feedback.discarded;
        loseFeedback(sensorFault ? sensorFault : "Encoder feedback timed out");
        return;
    }
    if (sensorFault) {
        qualificationSamples = 0;
        discardFeedback(sensorFault);
        return;
    }
    if (feedback.expired && (!hardware_->motion.hasStopped() || !qualifyAngle(rawAngle, sampledAt))) {
        return;
    }
    const auto angle = trackEncoder(rawAngle, sampledAt);
    if (angle == AngleResult::Pending) {
        ++feedback.discarded;
        return;  // Keep the existing rate. Only trusted feedback can update the ramp or target.
    }
    if (angle == AngleResult::Rejected) {
        discardFeedback("AS5600 angle changed too far between readings");
        return;
    }
    feedback.expired = false;
    qualificationSamples = 0;
    ++feedback.accepted;
    const bool recovered = feedback.paused;
    feedback.paused = false;
    feedback.cause = nullptr;
    if (recovered) {
        if (stalled) {
            // Keep the stall's error baseline so the recovered angle can show progress.
            // Exclude the unobserved interval from the stall timeout.
            stallReportedUs += sampledAt - previousValidAngleUs;
        } else {
            lastProgressUs += sampledAt - previousValidAngleUs;
        }
        emit(Event::FeedbackRestored);
    }
    controlPosition(sampledAt, fmin(dt, controlPeriodUs / 1000000.0));
}

void ClosedLoopStepper::stop() {
    if (!hardware_) {
        return;
    }
    haltDrive();
    hardware_->motion.setDriverEnabled(false);
    hardware_->motion.pausePulseGeneratorWhenStopped();
    starting = targetActive = holding = false;
    stalled = false;
    bypassPhase = BypassPhase::idle;
    lastFault = nullptr;
    lastAdc = lastRawAngle = -1;
    feedback = {};
    requestedTurns = 0;
    pulsesAtMoveStart = hardware_->motion.pulsesSent();
}

ClosedLoopStepper::Status ClosedLoopStepper::status() const {
    Status snapshot;
    if (!hardware_) {
        return snapshot;
    }
    snapshot.state = State::Idle;
    if (lastFault) snapshot.state = State::Fault;
    else if (feedback.expired) snapshot.state = State::FeedbackLost;
    else if (starting) snapshot.state = State::Starting;
    else if (bypassPhase != BypassPhase::idle) snapshot.state = State::OpenLoop;
    else if (targetActive) snapshot.state = holding ? State::Holding : State::Moving;

    snapshot.feedbackMode = config_.encoder.mode;
    snapshot.driverEnabled = hardware_->motion.isDriverEnabled();
    snapshot.positionKnown = lastRawAngle >= 0 && encoderPolarity != 0 && !lastFault && !feedback.expired;
    snapshot.feedbackPaused = feedback.paused;
    snapshot.requestedTurns = requestedTurns;
    if (snapshot.positionKnown) {
        snapshot.measuredTurns = encoderPosition * encoderPolarity / countsPerOutputTurn;
        snapshot.errorTurns = requestedTurns - snapshot.measuredTurns;
    }
    snapshot.commandedFullStepsPerSecond = bypassPhase != BypassPhase::idle && requestedTurns < 0.0 ?
                                          -velocity : velocity;
    snapshot.pulsesSent = hardware_->motion.pulsesSent() - pulsesAtMoveStart;
    if (bypassPhase != BypassPhase::idle) {
        snapshot.pulsesRemaining = bypassPhase == BypassPhase::starting ? bypassPulseCount :
                                                                        hardware_->motion.pulsesRemaining();
    }
    snapshot.encoderPolarity = encoderPolarity;
    snapshot.lastAdc = lastAdc;
    snapshot.encoderAngle = lastRawAngle;
    if (lastRawAngle >= 0) {
        snapshot.feedbackAgeMs = (micros() - lastValidAngleUs) / 1000;
    }
    snapshot.acceptedSamples = feedback.accepted;
    snapshot.discardedSamples = feedback.discarded;
    snapshot.fault = lastFault;
    snapshot.feedbackError = feedback.cause;
    snapshot.stalled = stalled;
    return snapshot;
}

void ClosedLoopStepper::onEvent(EventHandler handler, void* context) {
    eventHandler_ = handler;
    eventContext_ = context;
}

void ClosedLoopStepper::emit(Event event) const {
    if (eventHandler_) {
        eventHandler_(event, status(), eventContext_);
    }
}

ClosedLoopStepper::Result ClosedLoopStepper::readEncoder(EncoderReading& reading) {
    reading = {};
    if (!hardware_) {
        return Result::NotInitialized;
    }
    if ((!feedback.expired && (starting || targetActive)) ||
        (!usingAnalogEncoder() && bypassPhase != BypassPhase::idle)) {
        return Result::Busy;
    }

    const uint32_t startedAt = micros();
    if (usingAnalogEncoder()) {
        As5600Analog::Sample sample{};
        reading.error = As5600Analog::readSample(config_.encoder, sample);
        reading.adc = sample.adc;
        if (!reading.error) reading.angle = sample.angle;
    } else {
        As5600Bus::Sample sample{};
        reading.error = hardware_->encoder.readSample(sample);
        if (reading.error) {
            reading.i2cFailure = hardware_->encoder.lastReadFailure();
        } else {
            reading.angle = sample.rawAngle;
            reading.magnetStatus = sample.status;
        }
    }
    reading.elapsedUs = micros() - startedAt;
    return reading.error ? Result::EncoderError : Result::Ok;
}

ClosedLoopStepper::Result ClosedLoopStepper::readAutomaticGain(uint8_t& gain) {
    if (!hardware_) {
        return Result::NotInitialized;
    }
    if (usingAnalogEncoder()) {
        return Result::InvalidConfig;
    }
    if (starting || targetActive || bypassPhase != BypassPhase::idle) {
        return Result::Busy;
    }
    return hardware_->encoder.readGain(gain) ? Result::EncoderError : Result::Ok;
}

const char* ClosedLoopStepper::stateMessage(State state) {
    switch (state) {
        case State::Uninitialized: return "Uninitialized";
        case State::Idle: return "Idle";
        case State::Starting: return "Starting";
        case State::Moving: return "Moving";
        case State::Holding: return "Holding";
        case State::OpenLoop: return "Open loop";
        case State::Fault: return "Fault";
        case State::FeedbackLost: return "Feedback lost";
    }
    return "Unknown state";
}

const char* ClosedLoopStepper::resultMessage(Result result) {
    switch (result) {
        case Result::Ok: return "OK";
        case Result::NotInitialized: return "Call begin first";
        case Result::Busy: return "Controller is busy";
        case Result::InUse: return "Drive timer slot, GPIO, or I2C bus already in use";
        case Result::InvalidConfig: return "Invalid or incompatible configuration";
        case Result::InvalidMove: return "Move is non-finite, out of range, or rounds to zero";
        case Result::TimerError: return "STEP timer unavailable";
        case Result::EncoderError: return "Encoder operation failed";
        case Result::EncoderPolarityUnknown: return "Move once to learn encoder direction or configure encoder.polarity";
    }
    return "Unknown result";
}
