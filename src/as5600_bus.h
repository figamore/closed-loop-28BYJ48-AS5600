// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <stdint.h>
#include "AS5600ClosedLoopStepper.h"

class As5600Bus {
public:
    struct Sample {
        uint8_t status;
        uint16_t rawAngle;
    };

    static TwoWire* wireFor(const ClosedLoopStepper::EncoderConfig& config);
    bool begin(TwoWire* wire, const ClosedLoopStepper::EncoderConfig& config);
    void end();
    const char* readSample(Sample& sample);
    const char* readFeedback(uint16_t& rawAngle);
    const char* readGain(uint8_t& gain);
    ClosedLoopStepper::I2cFailure lastReadFailure() const;

private:
    void recordFailure(uint8_t reg, uint8_t code, uint8_t expected, uint8_t received, uint32_t startedAt);
    bool readRegisters(uint8_t reg, uint8_t* data, uint8_t length);

    ClosedLoopStepper::I2cFailure lastFailure;
    TwoWire* bus = nullptr;
    bool ownsBus = false;
};
