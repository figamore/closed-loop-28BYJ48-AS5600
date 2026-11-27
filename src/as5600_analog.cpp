// SPDX-License-Identifier: GPL-3.0-only
#include "as5600_analog.h"

#include <Arduino.h>
#include <algorithm>
#include <cstdlib>

namespace As5600Analog {
namespace {
constexpr int adcRangeMargin = 32;  // Allow small excursions past calibrated endpoints.
}  // namespace

void begin(uint8_t pin) {
    analogReadResolution(12);
    analogSetPinAttenuation(pin, ADC_11db);
}

const char* readSample(const ClosedLoopStepper::EncoderConfig& config, Sample& sample) {
    // Median of three rejects an isolated ADC spike without delaying a control poll.
    uint16_t readings[3];
    for (auto& reading : readings) {
        reading = analogRead(config.analogPin);
    }
    std::sort(readings, readings + 3);
    sample.adc = readings[1];
    const int bounded = std::min<int>(std::max<int>(sample.adc, config.adcMin), config.adcMax);
    if (std::abs(sample.adc - bounded) > adcRangeMargin) {
        return "AS5600 ADC value outside calibrated range";
    }
    const int span = config.adcMax - config.adcMin;
    sample.angle = ((bounded - config.adcMin) * (ClosedLoopStepper::countsPerEncoderTurn - 1) + span / 2) / span;
    return nullptr;
}
}
