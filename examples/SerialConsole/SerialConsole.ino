// SPDX-License-Identifier: GPL-3.0-only
#include <AS5600ClosedLoopStepper.h>
#include "Console.h"

ClosedLoopStepper motor;
using Console::report;

void printHelp() {
    report("Move      0.25, 90deg, b-45deg (open loop)\n"
           "Hold      hold, hold 90deg, stop (release)\n"
           "Limits    speed 100, accel 80, decel 120 (full-step units)\n"
           "Status    status, raw\n"
           "Encoder   encoder analog, encoder i2c\n");
}

void printStatus() {
    const auto s = motor.status();
    report("v%s | %s | %s | driver %s | %lu pulses\n", ClosedLoopStepper::version(), ClosedLoopStepper::stateMessage(s.state),
           s.feedbackMode == ClosedLoopStepper::FeedbackMode::Analog ? "analog" : "I2C",
           s.driverEnabled ? "enabled" : "released", static_cast<unsigned long>(s.pulsesSent));
    if (s.positionKnown) report("Position %+.5f turns | error %+.5f turns (%+.2f deg)\n",
                                s.measuredTurns, s.errorTurns, s.errorTurns * 360.0);
    if (s.lastAdc >= 0) report("ADC %d | ", s.lastAdc);
    if (s.encoderAngle >= 0) report("Last angle %d | age %lu ms | %lu rejected polls\n",
                                    s.encoderAngle, static_cast<unsigned long>(s.feedbackAgeMs),
                                    static_cast<unsigned long>(s.discardedSamples));
    else if (s.lastAdc >= 0) report("Waiting for a qualified angle\n");
    const auto& motion = motor.config().motion;
    report("Command %+.2f | max %.2f full steps/s | accel %.2f | decel %.2f full steps/s^2\n",
           s.commandedFullStepsPerSecond, motion.maxFullStepsPerSecond, motion.maxAcceleration, motion.maxDeceleration);
    if (s.state == ClosedLoopStepper::State::OpenLoop) {
        report("%lu pulses remaining\n", static_cast<unsigned long>(s.pulsesRemaining));
    }
    if (s.stalled) report("Stalled\n");
    if (s.feedbackError) report("%s\n", s.feedbackError);
    if (s.fault) report("%s\n", s.fault);
}

void onMotionEvent(ClosedLoopStepper::Event event, const ClosedLoopStepper::Status& s, void*) {
    using Event = ClosedLoopStepper::Event;
    switch (event) {
        case Event::Reached:          report("Target reached (%+.5f turns).\n", s.measuredTurns); break;
        case Event::Recovered:        report("Hold position recovered.\n"); break;
        case Event::OpenLoopComplete: report("Open-loop move complete.\n"); break;
        case Event::FeedbackPaused:
        case Event::FeedbackLost:     report("Feedback paused: %s\n", s.feedbackError); break;
        case Event::FeedbackRestored: report("Feedback restored.\n"); break;
        case Event::Stalled:          report("Stall detected. Retrying at low speed. Send stop to release.\n"); break;
        case Event::Fault:            report("Fault: %s\n", s.fault); break;
    }
}

void handleCommand(const char* command) {
    using Mode = ClosedLoopStepper::FeedbackMode;
    using Result = ClosedLoopStepper::Result;
    Result result = Result::Ok;
    if (strcmp(command, "help") == 0) {
        printHelp();
        return;
    } else if (strcmp(command, "stop") == 0) {
        motor.stop();
    } else if (strcmp(command, "status") == 0) {
        printStatus();
        return;
    } else if (strcmp(command, "raw") == 0) {
        ClosedLoopStepper::EncoderReading reading;
        result = motor.readEncoder(reading);
        if (result == Result::Busy) {
            const auto s = motor.status();
            if (s.encoderAngle >= 0) {
                report("Last angle %d counts (%lu ms ago)\n", s.encoderAngle,
                       static_cast<unsigned long>(s.feedbackAgeMs));
                return;
            }
        } else if (result == Result::Ok) {
            if (reading.adc >= 0) report("ADC %d | ", reading.adc);
            else report("Magnet status 0x%02X | ", unsigned(reading.magnetStatus));
            report("Angle %d counts\n", reading.angle);
            return;
        } else if (reading.error) {
            report("%s\n", reading.error);
            return;
        }
    } else if (strcmp(command, "encoder analog") == 0 || strcmp(command, "encoder i2c") == 0) {
        result = motor.setFeedbackMode(strcmp(command, "encoder analog") == 0 ? Mode::Analog : Mode::I2C);
    } else if (strcmp(command, "hold") == 0) {
        result = motor.holdRelative();
    } else if (strncmp(command, "speed ", 6) == 0 || strncmp(command, "accel ", 6) == 0 ||
               strncmp(command, "decel ", 6) == 0 || strncmp(command, "acceleration ", 13) == 0 ||
               strncmp(command, "deceleration ", 13) == 0) {
        const char* number = strchr(command, ' ') + 1;
        char* end;
        const double value = strtod(number, &end);
        if (end == number || *end) {
            report("Enter a positive numeric limit.\n");
            return;
        }
        if (command[0] == 's') result = motor.setMaxSpeed(value);
        else if (command[0] == 'a') result = motor.setMaxAcceleration(value);
        else result = motor.setMaxDeceleration(value);
    } else {
        const bool hold = strncmp(command, "hold ", 5) == 0;
        const bool openLoop = command[0] == 'b';
        const char* number = command + (hold ? 5 : openLoop ? 1 : 0);
        char* end;
        const double distance = strtod(number, &end);
        const bool hasNumber = end != number && isfinite(distance);
        while (*end == ' ') ++end;
        const bool degrees = strcmp(end, "deg") == 0;
        if (!hasNumber || (*end && !degrees)) {
            report("Invalid command. Type help.\n");
            return;
        }
        if (hold) result = degrees ? motor.holdRelativeDegrees(distance) : motor.holdRelative(distance);
        else if (openLoop) result = degrees ? motor.moveOpenLoopDegrees(distance) : motor.moveOpenLoop(distance);
        else result = degrees ? motor.moveRelativeDegrees(distance) : motor.moveRelative(distance);
    }
    if (result == Result::Busy) report("Controller is busy. Use status to inspect or stop to cancel.\n");
    else report("%s\n", ClosedLoopStepper::resultMessage(result));
}

void setup() {
    Serial.begin(115200);
    ClosedLoopStepper::Config config;
    // On the example gearbox, encoder counts fall during positive output travel.
    config.encoder.polarity = -1;
    // Keep retrying after a stall until stop.
    config.motion.stallFaultMs = 0;
    const auto result = motor.begin(config);
    if (result != ClosedLoopStepper::Result::Ok) {
        report("%s\n", ClosedLoopStepper::resultMessage(result));
        return;
    }
    motor.onEvent(onMotionEvent);
    printHelp();
}

void loop() {
    motor.update();
    if (const char* command = Console::readCommand()) handleCommand(command);
    Console::drainOutput();
    delay(1);
}
