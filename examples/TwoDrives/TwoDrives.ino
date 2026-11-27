// SPDX-License-Identifier: GPL-3.0-only
#include <AS5600ClosedLoopStepper.h>

ClosedLoopStepper first;
ClosedLoopStepper second;

void setup() {
    Serial.begin(115200);

    ClosedLoopStepper::Config a;
    a.motion.fullStepsPerOutputTurn = 200;
    a.motion.encoderTurnsPerOutputTurn = 1;
    a.motion.maxFullStepsPerSecond = 100;

    auto b = a;
    b.driver.stepPin = 18;
    b.driver.dirPin = 19;
    b.driver.enablePin = 23;
    b.driver.timerNumber = 1;
    b.encoder.analogPin = 35;

    using Result = ClosedLoopStepper::Result;
    auto result = first.begin(a);
    if (result == Result::Ok) result = second.begin(b);
    if (result == Result::Ok) result = first.moveRelativeDegrees(90);
    if (result == Result::Ok) result = second.moveRelativeDegrees(-45);

    if (result != Result::Ok) {
        first.end();
        second.end();
        Serial.println(ClosedLoopStepper::resultMessage(result));
    }
}

void loop() {
    first.update();
    second.update();
}
