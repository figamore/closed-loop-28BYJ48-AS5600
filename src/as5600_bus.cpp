// SPDX-License-Identifier: GPL-3.0-only
#include "as5600_bus.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t address = 0x36;
constexpr uint8_t statusRegister = 0x0B;
constexpr uint8_t gainRegister = 0x1A;
}  // namespace

void As5600Bus::recordFailure(uint8_t reg, uint8_t code, uint8_t expected, uint8_t received, uint32_t startedAt) {
    lastFailure.reg = reg;
    lastFailure.transmissionCode = code;
    lastFailure.expectedBytes = expected;
    lastFailure.receivedBytes = received;
    lastFailure.elapsedUs = micros() - startedAt;
}

bool As5600Bus::readRegisters(uint8_t reg, uint8_t* data, uint8_t length) {
    const uint32_t startedAt = micros();
    bus->beginTransmission(address);
    if (bus->write(reg) != 1) {
        // Finish the transaction so a shared Wire bus is available to its owner.
        const uint8_t code = bus->endTransmission(true);
        recordFailure(reg, code, length, 0, startedAt);
        return false;
    }
    const uint8_t code = bus->endTransmission(false);
    const uint8_t received = code == 0 ? bus->requestFrom(address, length) : 0;
    if (code != 0 || received != length) {
        recordFailure(reg, code, length, received, startedAt);
        while (bus->available()) {
            bus->read();
        }
        return false;
    }

    for (uint8_t index = 0; index < length; ++index) {
        const int value = bus->read();
        if (value < 0) {
            recordFailure(reg, code, length, index, startedAt);
            return false;
        }
        data[index] = static_cast<uint8_t>(value);
    }
    return true;
}

TwoWire* As5600Bus::wireFor(const ClosedLoopStepper::EncoderConfig& config) {
    return config.sharedWire ? config.sharedWire : (config.i2cBusNumber == 0 ? &Wire : &Wire1);
}

bool As5600Bus::begin(TwoWire* wire, const ClosedLoopStepper::EncoderConfig& config) {
    if (!config.sharedWire) {
        if (!wire->begin(config.sdaPin, config.sclPin, config.i2cFrequency)) {
            return false;
        }
        wire->setTimeOut(config.i2cTimeoutMs);
    }
    lastFailure = {};
    bus = wire;
    ownsBus = !config.sharedWire;
    return true;
}

void As5600Bus::end() {
    if (ownsBus) {
        bus->end();
    }
    bus = nullptr;
    ownsBus = false;
}

const char* As5600Bus::readSample(Sample& sample) {
    // Read STATUS and RAW ANGLE together from their contiguous registers.
    uint8_t data[3];
    if (!readRegisters(statusRegister, data, sizeof(data))) {
        return "AS5600 STATUS/RAW ANGLE read failed at 0x36";
    }
    sample = {data[0], static_cast<uint16_t>(((data[1] & 0x0F) << 8) | data[2])};
    return nullptr;
}

const char* As5600Bus::readFeedback(uint16_t& rawAngle) {
    Sample sample{};
    if (const char* error = readSample(sample)) {
        return error;
    }
    if (!(sample.status & 0x20)) {
        return "AS5600 magnet not detected";
    }
    if (sample.status & 0x10) {
        return "AS5600 magnetic field too weak";
    }
    if (sample.status & 0x08) {
        return "AS5600 magnetic field too strong";
    }
    rawAngle = sample.rawAngle;
    return nullptr;
}

const char* As5600Bus::readGain(uint8_t& gain) {
    return readRegisters(gainRegister, &gain, 1) ? nullptr : "AS5600 AGC read failed at 0x36";
}

ClosedLoopStepper::I2cFailure As5600Bus::lastReadFailure() const {
    return lastFailure;
}
