// SPDX-License-Identifier: GPL-3.0-only
#include "stepper_motion.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_arduino_version.h>
#include <soc/gpio_struct.h>

#include <math.h>

#if !CONFIG_IDF_TARGET_ESP32
#error "AS5600ClosedLoopStepper currently targets the original ESP32."
#endif

namespace {
constexpr uint32_t timerFrequencyHz = 1000000;
constexpr uint32_t tickPeriodUs = 25;
constexpr uint32_t ticksPerSecond = timerFrequencyHz / tickPeriodUs;
constexpr uint32_t milliPerUnit = 1000;
constexpr uint32_t phasePerPulse = ticksPerSecond * milliPerUnit;
constexpr uint32_t commandTimeoutUs = 60000;
constexpr uint32_t commandTimeoutTicks = commandTimeoutUs / tickPeriodUs;
constexpr uint32_t stopTicks = 3;  // Finish the high pulse and allow DIR hold time.
}  // namespace

StepperMotion* StepperMotion::interruptOwners[timerCount]{};

// Each no-argument callback routes to its timer's drive and stays in IRAM.
void IRAM_ATTR StepperMotion::stepTimerInterrupt0() { interruptOwners[0]->tick(); }
void IRAM_ATTR StepperMotion::stepTimerInterrupt1() { interruptOwners[1]->tick(); }
void IRAM_ATTR StepperMotion::stepTimerInterrupt2() { interruptOwners[2]->tick(); }
void IRAM_ATTR StepperMotion::stepTimerInterrupt3() { interruptOwners[3]->tick(); }

void IRAM_ATTR StepperMotion::tick() {
    const uint32_t now = ++timerTick;
    tickCount.store(now, std::memory_order_relaxed);

    if (stepHigh) {
        stepHigh = false;
        *stepClearRegister = stepMask;
    }

    const uint32_t rate = requestedRate.load(std::memory_order_acquire);
    const uint32_t remaining = pulseBudget.load(std::memory_order_relaxed);
    if (rate == 0 || remaining == 0) {
        phase = 0;
        return;
    }

    const uint32_t updated = rateUpdatedTick.load(std::memory_order_relaxed);
    if (updated != observedUpdateTick) {
        observedUpdateTick = updated;
        rateExpired = false;
    }
    if (now - updated > commandTimeoutTicks) {
        rateExpired = true;
    }
    if (rateExpired) {
        phase = 0;
        return;
    }

    phase += rate;
    if (phase >= phasePerPulse) {
        phase -= phasePerPulse;
        *stepSetRegister = stepMask;
        stepHigh = true;
        totalPulses.store(totalPulses.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
        netPulses.store(netPulses.load(std::memory_order_relaxed) +
                       pulseIncrement.load(std::memory_order_relaxed), std::memory_order_relaxed);
        if (remaining != unlimitedPulses) {
            pulseBudget.store(remaining - 1, std::memory_order_release);
        }
    }
}

bool StepperMotion::initializePulseTimer() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    pulseTimer = timerBegin(timerFrequencyHz);
#else
    pulseTimer = timerBegin(driver.timerNumber, 80, true);  // ESP32 APB clock is 80 MHz.
#endif
    if (!pulseTimer) {
        return false;
    }

    static constexpr void (*handlers[timerCount])() = {
        stepTimerInterrupt0, stepTimerInterrupt1, stepTimerInterrupt2, stepTimerInterrupt3
    };
    interruptOwners[interruptSlot] = this;
    const auto handler = handlers[interruptSlot];
    // Set the first alarm before the timer starts counting from zero.
    timerStop(pulseTimer);
    timerWrite(pulseTimer, 0);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAttachInterrupt(pulseTimer, handler);
    timerAlarm(pulseTimer, tickPeriodUs, true, 0);
#else
    timerAttachInterrupt(pulseTimer, handler, false);
    timerAlarmWrite(pulseTimer, tickPeriodUs, true);
    timerAlarmEnable(pulseTimer);
#endif
    return true;
}

bool StepperMotion::isOutputPin(uint8_t pin) {
    return pin < GPIO_NUM_MAX && GPIO_IS_VALID_OUTPUT_GPIO(pin) && (pin < 6 || pin > 11);
}

bool StepperMotion::validConfig(const ClosedLoopStepper::DriverConfig& config, double maxSpeed) {
    const uint8_t microsteps = config.microsteps;
    const bool validMicrosteps = microsteps == 1 || microsteps == 2 || microsteps == 4 ||
                                 microsteps == 8 || microsteps == 16;
    return validMicrosteps && maxSpeed >= 0.05 && maxSpeed * microsteps <= ticksPerSecond / 2 &&
           config.timerNumber < timerCount && isOutputPin(config.stepPin) && isOutputPin(config.dirPin) &&
           isOutputPin(config.enablePin) && config.stepPin != config.dirPin &&
           config.stepPin != config.enablePin && config.dirPin != config.enablePin;
}

void StepperMotion::begin(const ClosedLoopStepper::DriverConfig& config, uint8_t slot) {
    driver = config;
    interruptSlot = slot;
    stepMask = 1UL << (driver.stepPin % 32);
    stepSetRegister = driver.stepPin < 32 ? &GPIO.out_w1ts : &GPIO.out1_w1ts.val;
    stepClearRegister = driver.stepPin < 32 ? &GPIO.out_w1tc : &GPIO.out1_w1tc.val;
    clockwise = true;
    pulseIncrement.store(1, std::memory_order_relaxed);
    // External pull-up on EN keeps the driver off before setup runs.
    gpio_set_level(static_cast<gpio_num_t>(driver.enablePin), HIGH);
    pinMode(driver.enablePin, OUTPUT);
    digitalWrite(driver.enablePin, HIGH);
    driverEnabled = false;
    gpio_set_level(static_cast<gpio_num_t>(driver.stepPin), LOW);
    pinMode(driver.stepPin, OUTPUT);
    digitalWrite(driver.stepPin, LOW);
    pinMode(driver.dirPin, OUTPUT);
    digitalWrite(driver.dirPin, driver.clockwiseDirHigh ? HIGH : LOW);
}

void StepperMotion::end() {
    setDriverEnabled(false);
    if (pulseTimer) {
        if (pulseTimerActive) timerStop(pulseTimer);
        timerDetachInterrupt(pulseTimer);
        timerEnd(pulseTimer);
        pulseTimer = nullptr;
        interruptOwners[interruptSlot] = nullptr;
    }
    pulseTimerActive = false;
    digitalWrite(driver.stepPin, LOW);
    timerTick = phase = observedUpdateTick = 0;
    stepHigh = false;
    rateExpired = false;
    requestedRate.store(0, std::memory_order_relaxed);
    tickCount.store(0, std::memory_order_relaxed);
    rateUpdatedTick.store(0, std::memory_order_relaxed);
    totalPulses.store(0, std::memory_order_relaxed);
    netPulses.store(0, std::memory_order_relaxed);
    clearPulseLimit();
}

void StepperMotion::setDriverEnabled(bool enabled) {
    if (!enabled) {
        setVelocity(0.0);
    }
    digitalWrite(driver.enablePin, enabled ? LOW : HIGH);
    driverEnabled = enabled;
}

bool StepperMotion::isDriverEnabled() const {
    return driverEnabled;
}

bool StepperMotion::startPulseGenerator() {
    if (pulseTimerActive) {
        return isPulseGeneratorRunning();
    }
    if (!pulseTimer && !initializePulseTimer()) {
        return false;
    }
    lastObservedTick = tickCount.load(std::memory_order_relaxed);
    lastDriveStopTick = lastObservedTick - stopTicks;
    lastTickObservedUs = micros();
    timerStart(pulseTimer);
    pulseTimerActive = true;
    return true;
}

void StepperMotion::pausePulseGeneratorWhenStopped() {
    if (!pulseTimerActive) {
        return;
    }
    if (hasStopped()) {
        timerStop(pulseTimer);
        pulseTimerActive = false;
    } else {
        isPulseGeneratorRunning();
    }
}

void StepperMotion::setVelocity(double fullStepsPerSecond) {
    if (!driverEnabled) {
        fullStepsPerSecond = 0.0;
    }
    const double pulsesPerSecond = fmin(fabs(fullStepsPerSecond) * driver.microsteps, ticksPerSecond / 2.0);
    const uint32_t rate = static_cast<uint32_t>(pulsesPerSecond * milliPerUnit + 0.5);
    const bool newClockwise = fullStepsPerSecond > 0.0;

    if (rate == 0 || newClockwise != clockwise) {
        if (driving) {
            requestedRate.store(0, std::memory_order_release);
            driving = false;
            lastDriveStopTick = tickCount.load(std::memory_order_relaxed);
        }
        if (rate == 0 || !hasStopped()) {
            return;
        }
        clockwise = newClockwise;
        digitalWrite(driver.dirPin, clockwise == driver.clockwiseDirHigh ? HIGH : LOW);
        pulseIncrement.store(clockwise ? 1U : UINT32_MAX, std::memory_order_relaxed);
        delayMicroseconds(5);
    }

    driving = true;
    rateUpdatedTick.store(tickCount.load(std::memory_order_relaxed), std::memory_order_relaxed);
    requestedRate.store(rate, std::memory_order_release);
}

bool StepperMotion::isPulseGeneratorRunning() {
    if (!pulseTimerActive) {
        return false;
    }
    const uint32_t now = micros();
    const uint32_t ticks = tickCount.load(std::memory_order_relaxed);
    if (ticks != lastObservedTick) {
        lastObservedTick = ticks;
        lastTickObservedUs = now;
    }
    if (now - lastTickObservedUs >= commandTimeoutUs) {
        setDriverEnabled(false);
        timerStop(pulseTimer);
        pulseTimerActive = false;
        digitalWrite(driver.stepPin, LOW);
        return false;
    }
    return true;
}

bool StepperMotion::hasStopped() const {
    return !driving && (!pulseTimerActive ||
        tickCount.load(std::memory_order_relaxed) - lastDriveStopTick >= stopTicks);
}

// The motion controller changes the budget only after hasStopped().
void StepperMotion::setPulseLimit(uint32_t pulses) {
    pulseBudget.store(pulses, std::memory_order_release);
}

void StepperMotion::clearPulseLimit() {
    setPulseLimit(unlimitedPulses);
}

uint32_t StepperMotion::pulsesRemaining() const {
    return pulseBudget.load(std::memory_order_acquire);
}

uint32_t StepperMotion::pulsesSent() const {
    return totalPulses.load(std::memory_order_relaxed);
}

uint32_t StepperMotion::netPulsesSent() const {
    return netPulses.load(std::memory_order_relaxed);
}
