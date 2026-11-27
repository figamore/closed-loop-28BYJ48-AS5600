// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <atomic>
#include <esp32-hal-timer.h>
#include <stdint.h>
#include "AS5600ClosedLoopStepper.h"

class StepperMotion {
public:
    static constexpr uint32_t maxCountedPulses = UINT32_MAX - 1;
    static constexpr uint8_t timerCount = 4;

    static bool isOutputPin(uint8_t pin);
    static bool validConfig(const ClosedLoopStepper::DriverConfig& config, double maxFullStepsPerSecond);
    void begin(const ClosedLoopStepper::DriverConfig& config, uint8_t slot);
    void end();
    void setDriverEnabled(bool enabled);
    bool isDriverEnabled() const;
    bool startPulseGenerator();
    void pausePulseGeneratorWhenStopped();
    void setVelocity(double fullStepsPerSecond);
    bool isPulseGeneratorRunning();
    bool hasStopped() const;
    void setPulseLimit(uint32_t pulses);
    void clearPulseLimit();
    uint32_t pulsesRemaining() const;
    uint32_t pulsesSent() const;
    uint32_t netPulsesSent() const;  // CW increments, CCW decrements, modulo 2^32.

private:
    static constexpr uint32_t unlimitedPulses = UINT32_MAX;
    static StepperMotion* interruptOwners[timerCount];

    bool initializePulseTimer();
    static void stepTimerInterrupt0();
    static void stepTimerInterrupt1();
    static void stepTimerInterrupt2();
    static void stepTimerInterrupt3();
    void tick();

    ClosedLoopStepper::DriverConfig driver;
    uint8_t interruptSlot = 0;
    uint32_t stepMask = 0;
    volatile uint32_t* stepSetRegister = nullptr;
    volatile uint32_t* stepClearRegister = nullptr;
    hw_timer_t* pulseTimer = nullptr;
    std::atomic<uint32_t> requestedRate{0};  // Millipulses per second.
    std::atomic<uint32_t> rateUpdatedTick{0};
    std::atomic<uint32_t> pulseBudget{unlimitedPulses};
    std::atomic<uint32_t> tickCount{0};
    std::atomic<uint32_t> totalPulses{0};
    std::atomic<uint32_t> netPulses{0};
    std::atomic<uint32_t> pulseIncrement{1};

    // Only the timer interrupt changes these while the timer is running.
    uint32_t timerTick = 0;
    uint32_t phase = 0;
    bool stepHigh = false;
    uint32_t observedUpdateTick = 0;
    bool rateExpired = false;

    // Only the motion controller task changes these.
    bool pulseTimerActive = false;
    bool clockwise = true;
    bool driving = false;
    bool driverEnabled = false;
    uint32_t lastDriveStopTick = 0;
    uint32_t lastObservedTick = 0;
    uint32_t lastTickObservedUs = 0;
};
