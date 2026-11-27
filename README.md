# AS5600 Closed Loop Stepper

Control up to four A4988-driven steppers on the original ESP32 with AS5600 analog or I2C feedback. Supports multi-turn moves, active hold, and pulse-counted open-loop moves.

## Install

- **Arduino IDE** — Install the release ZIP via **Sketch → Include Library → Add .ZIP Library**. Select an original ESP32 board and open `SerialConsole` or `TwoDrives` from the library examples.
- **PlatformIO** — Add the release tarball to `lib_deps` with `file:///path/to/release.tar.gz`. Run `pio run` from the repository to build `SerialConsole`.

## Wiring

| Connection | Default ESP32 GPIO |
| --- | --- |
| A4988 STEP / DIR / active-low EN | 26 / 27 / 33 |
| AS5600 OUT (analog, default) | 34 |
| AS5600 SDA / SCL (I2C) | 21 / 22 |

Use 3.3 V logic, a common ground, and a 10 kΩ pull-up on EN. Set the driver current limit and match `driver.microsteps` to the MS pins. A five-wire 28BYJ-48 requires conversion to two isolated windings. [A4988 wiring guide](https://www.pololu.com/product/1182)

## Usage

```cpp
#include <AS5600ClosedLoopStepper.h>

ClosedLoopStepper motor;

void setup() {
    Serial.begin(115200);
    ClosedLoopStepper::Config config;
    config.motion.fullStepsPerOutputTurn = 200;
    config.motion.encoderTurnsPerOutputTurn = 1;
    config.motion.maxFullStepsPerSecond = 100;

    auto result = motor.begin(config);
    if (result == ClosedLoopStepper::Result::Ok) result = motor.moveRelativeDegrees(90);
    Serial.println(ClosedLoopStepper::resultMessage(result));
}

void loop() {
    motor.update();
}
```

This sketch assumes a direct-drive 1.8° motor with the encoder on its shaft. Set the gearing to match your hardware. Set `encoder.polarity` to `+1` for counts that rise on positive travel, `-1` for falling counts, or `0` to learn on the next nonzero move. For analog calibration, set `adcMin` and `adcMax`.

Call `update()` every loop from the task that calls `begin()`. The control period is 10 ms. The STEP watchdog stops pulses after 60 ms without a rate update.

Moves use signed output turns or degrees and hold on arrival by default. Use `holdRelative()` to hold in place, `moveOpenLoop()` for pulse-counted travel, and `stop()` to release the driver. Speed, acceleration, and deceleration limits use motor full-step units and can change during a move. Use `status()` to check faults and `positionKnown` before reading measured travel. [API and configuration defaults](src/AS5600ClosedLoopStepper.h)

Run `SerialConsole`, open the serial monitor at **115200 baud**, and enter `help`. [SerialConsole sketch](examples/SerialConsole/SerialConsole.ino)

## Multiple drives

Give each drive its own STEP, DIR, EN, and encoder connections. Call every controller from the same task and service each with `update()`. On Arduino-ESP32 2.x, assign a distinct free `driver.timerNumber` (0–3). On 3.x, the core allocates timers. `begin()` reports resource conflicts as `Result::InUse`.

Use a separate ADC pin for each analog encoder. For I2C, set `encoder.mode` to `FeedbackMode::I2C`. The AS5600 uses address `0x36`, so each I2C encoder needs its own SDA/SCL pair. Set `encoder.i2cBusNumber` to `0` for `Wire` or `1` for `Wire1`. For an application-managed bus, pass an initialized `encoder.sharedWire` and reserve its pins in your application.

[TwoDrives](examples/TwoDrives/TwoDrives.ino) starts two analog-feedback moves at boot.

## License

[GPL-3.0-only](LICENSE)
