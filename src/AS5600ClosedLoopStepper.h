// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <stdint.h>

class TwoWire;

class ClosedLoopStepper {
public:
    static const char* version() { return "0.3.0"; }
    static constexpr int countsPerEncoderTurn = 4096;

    enum class FeedbackMode { Analog, I2C };
    enum class State { Uninitialized, Idle, Starting, Moving, Holding, OpenLoop, Fault, FeedbackLost };
    enum class Result {
        Ok, NotInitialized, Busy, InUse, InvalidConfig, InvalidMove, TimerError, EncoderError, EncoderPolarityUnknown
    };
    enum class Event { FeedbackPaused, FeedbackRestored, Stalled, Reached, Recovered, OpenLoopComplete, Fault, FeedbackLost };

    struct DriverConfig {
        uint8_t stepPin = 26;
        uint8_t dirPin = 27;
        uint8_t enablePin = 33;
        uint8_t microsteps = 1;
        bool clockwiseDirHigh = true;
        uint8_t timerNumber = 0;  // Arduino-ESP32 2.x hardware timer index.
    };

    struct EncoderConfig {
        FeedbackMode mode = FeedbackMode::Analog;
        uint8_t analogPin = 34;
        uint16_t adcMin = 0;
        uint16_t adcMax = 4095;
        uint8_t sdaPin = 21;
        uint8_t sclPin = 22;
        uint32_t i2cFrequency = 100000;
        uint16_t i2cTimeoutMs = 50;
        // When supplied, the application initializes and owns the bus, pins and timeout.
        TwoWire* sharedWire = nullptr;
        int8_t polarity = 1;  // +1 means counts rise CW. Set 0 to learn on the first move.
        uint8_t i2cBusNumber = 0;  // Library-owned I2C: 0 uses Wire, 1 uses Wire1.
    };

    struct MotionConfig {
        double fullStepsPerOutputTurn = 2048.0 * 32.0 / 9.0;
        double encoderTurnsPerOutputTurn = 2.0;
        // Motor full steps/s, full steps/s² and full steps/s³, independent of microstepping.
        double maxFullStepsPerSecond = 600.0;
        double maxAcceleration = 300.0;
        double maxDeceleration = 650.0;
        double maxJerk = 3600.0;
        double maxBrakingJerk = 12000.0;
        double brakingAcceleration = 400.0;
        double positionGain = 6.0;
        uint16_t analogTargetTolerance = 8;
        uint16_t analogHoldTolerance = 16;
        uint16_t i2cTargetTolerance = 2;
        uint16_t i2cHoldTolerance = 4;
        bool holdOnTarget = true;
        // Recovery after reaching a held target. Speed and braking limits still apply.
        double holdPositionGain = 4.0;
        double holdMaxAcceleration = 600.0;
        double holdMaxJerk = 14400.0;
        // Time allowed after a stall warning or feedback-loss event. Zero keeps retrying.
        uint16_t stallFaultMs = 3000;
        uint16_t feedbackLossFaultMs = 2000;
        // Motor full steps/s while seeking renewed encoder progress after a stall.
        double stallRecoveryFullStepsPerSecond = 16.0;
    };

    struct Config {
        DriverConfig driver;
        EncoderConfig encoder;
        MotionConfig motion;
    };

    struct I2cFailure {
        uint8_t reg = 0;
        uint8_t transmissionCode = 0;
        uint8_t expectedBytes = 0;
        uint8_t receivedBytes = 0;
        uint32_t elapsedUs = 0;
    };

    struct EncoderReading {
        int angle = -1;  // Counts in [0, 4095] when available.
        int adc = -1;    // Raw ADC conversion in analog mode.
        int magnetStatus = -1;  // AS5600 STATUS byte in I2C mode.
        uint32_t elapsedUs = 0;
        const char* error = nullptr;
        I2cFailure i2cFailure;
    };

    struct Status {
        State state = State::Uninitialized;
        FeedbackMode feedbackMode = FeedbackMode::Analog;
        bool driverEnabled = false;
        bool positionKnown = false;
        bool feedbackPaused = false;
        double requestedTurns = 0;
        double measuredTurns = 0;
        double errorTurns = 0;
        double commandedFullStepsPerSecond = 0;
        uint32_t pulsesSent = 0;
        uint32_t pulsesRemaining = 0;
        int encoderAngle = -1;
        uint32_t feedbackAgeMs = 0;
        uint32_t acceptedSamples = 0;
        uint32_t discardedSamples = 0;
        const char* fault = nullptr;
        const char* feedbackError = nullptr;
        bool stalled = false;
        int lastAdc = -1;  // Latest analog poll, including rejected feedback.
        int encoderPolarity = 0;  // 0 until direction has been learned.
    };

    using EventHandler = void (*)(Event event, const Status& status, void* context);

    ClosedLoopStepper() = default;
    ~ClosedLoopStepper();
    ClosedLoopStepper(const ClosedLoopStepper&) = delete;
    ClosedLoopStepper& operator=(const ClosedLoopStepper&) = delete;

    Result begin();
    Result begin(const Config& config);
    void end();
    // Service all controllers from the same application task that calls begin().
    void update();
    // Live limits for this and subsequent moves. Call from the task servicing update().
    Result setMaxSpeed(double fullStepsPerSecond);
    Result setMaxAcceleration(double fullStepsPerSecondSquared);
    Result setMaxDeceleration(double fullStepsPerSecondSquared);
    // An accepted move starts qualifying during update(). Motion faults arrive in status().
    Result moveRelative(double outputTurns);
    // Hold at a signed output offset from the qualified starting position. Zero holds in place.
    Result holdRelative(double outputTurns = 0.0);
    Result moveOpenLoop(double outputTurns);
    // Degree commands use the same signed output-shaft reference as turn commands.
    Result moveRelativeDegrees(double degrees) { return moveRelative(degrees / 360.0); }
    Result holdRelativeDegrees(double degrees = 0.0) { return holdRelative(degrees / 360.0); }
    Result moveOpenLoopDegrees(double degrees) { return moveOpenLoop(degrees / 360.0); }
    void stop();
    Status status() const;
    const Config& config() const { return config_; }
    // Callbacks run synchronously. Defer actions on the controller to the application loop.
    void onEvent(EventHandler handler, void* context = nullptr);

    Result setFeedbackMode(FeedbackMode mode);
    Result readEncoder(EncoderReading& reading);
    Result readAutomaticGain(uint8_t& gain);

    static const char* stateMessage(State state);
    static const char* resultMessage(Result result);

private:
    struct Hardware;
    enum class BypassPhase { idle, starting, moving, finishing };
    enum class AngleResult { Accepted, Pending, Rejected };

    bool validConfig(const Config& config) const;
    bool usingAnalogEncoder() const;
    bool moveInProgress() const;
    Result startClosedLoopMove(double turns, bool explicitHold);
    const char* readFeedback(uint16_t& rawAngle);
    bool qualifyAngle(uint16_t rawAngle, uint32_t sampledAt);
    void updateStartup(uint32_t now);
    void anchorEncoder(uint16_t rawAngle, uint32_t sampledAt);
    void recordEncoderAngle(uint16_t rawAngle, uint32_t sampledAt);
    AngleResult trackEncoder(uint16_t rawAngle, uint32_t sampledAt);
    int64_t errorCounts() const;
    int targetToleranceCounts() const;
    int holdToleranceCounts() const;
    void haltDrive();
    void fail(const char* cause);
    void emit(Event event) const;
    void discardFeedback(const char* cause);
    void loseFeedback(const char* cause);
    void resetProgress(uint32_t now);
    void checkProgress(uint32_t now);
    void rampVelocity(double desiredVelocity, double dt);
    double approachSpeed(double distance) const;
    void controlPosition(uint32_t now, double dt);
    void controlBypass(double dt);
    void resetMove(double turns, uint32_t now);

    Config config_;
    EventHandler eventHandler_ = nullptr;
    void* eventContext_ = nullptr;
    Hardware* hardware_ = nullptr;
    bool starting = false;
    bool targetActive = false;
    bool holdAtTarget = false;
    bool holding = false;
    int holdOutsideSamples = 0;  // Signed consecutive analog polls outside the hold band.
    bool reachedOnce = false;
    bool stalled = false;
    uint32_t startupBeganUs = 0;
    uint32_t qualificationLastUs = 0;
    uint16_t qualificationFirstAngle = 0;
    uint16_t qualificationSecondAngle = 0;
    uint8_t qualificationSamples = 0;
    const char* lastFault = nullptr;
    int lastRawAngle = -1;
    int lastAdc = -1;
    int64_t encoderPosition = 0;
    int encoderPolarity = 1;
    int directionCandidate = 0;
    uint32_t directionCandidateUs = 0;
    uint32_t lastSampleUs = 0;
    uint32_t lastControlUs = 0;
    uint32_t lastValidAngleUs = 0;
    uint32_t pulsesAtLastAngle = 0;

    struct FeedbackState {
        int pendingAngle = -1;
        uint32_t pendingAtUs = 0;
        const char* cause = nullptr;
        bool paused = false;
        bool expired = false;
        uint32_t lostAtUs = 0;
        uint32_t accepted = 0;
        uint32_t discarded = 0;
    } feedback;

    uint32_t pulsesAtMoveStart = 0;
    uint32_t netPulsesAtOrigin = 0;
    int32_t targetOffset = 0;
    double requestedTurns = 0;
    BypassPhase bypassPhase = BypassPhase::idle;
    uint32_t bypassPulseCount = 0;
    uint32_t bypassLastRemaining = 0;
    double bypassFullStepsSinceProgress = 0;
    double velocity = 0;
    double acceleration = 0;
    uint32_t settledFeedbackUs = 0;
    int64_t bestError = 0;
    uint32_t lastProgressUs = 0;
    uint32_t pulsesAtProgress = 0;
    uint32_t stallReportedUs = 0;

    double countsPerOutputTurn = 0;
    double fullStepsPerCount = 0;
};
