// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <stdint.h>
#include "AS5600ClosedLoopStepper.h"

namespace As5600Analog {
constexpr uint16_t adcMax = 4095;

struct Sample {
    uint16_t adc;
    uint16_t angle;
};

void begin(uint8_t pin);
const char* readSample(const ClosedLoopStepper::EncoderConfig& config, Sample& sample);
}
