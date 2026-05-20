# TeensyStepDriver

Non-blocking stepper motor library for **Teensy 4.1**, built on
[TeensyTimerTool](https://github.com/luni64/TeensyTimerTool) QTIMER hardware
channels for precise, low-jitter step pulses and true multi-motor parallelism.

---

## Features

| Feature | Detail |
|---|---|
| **Motion profile** | Trapezoidal: accel → cruise → decel |
| **Multi-motor** | Up to 8 independent motors (4 QTIMER modules × 4 channels, 2 chanels per motor) |
| **Non-blocking** | All motion runs in ISR; `loop()` stays completely free |
| **Direction invert** | Per-instance flag for wiring flexibility |
| **Runtime commands** | `moveTo`, `moveRelative`, `stop` (ramped), `forceStop`, `runSpeed` |
| **Status query** | `isRunning()`, `currentPosition()`, `targetPosition()` |
| **Homing support** | `setCurrentPosition()` resets the internal counter |

---

## Dependencies

Install via Arduino Library Manager or PlatformIO before using this library:

```
TeensyTimerTool   by luni64   (>=1.3.0)
```

PlatformIO `platformio.ini`:
```ini
[env:teensy41]
platform  = teensy
board     = teensy41
framework = arduino
lib_deps  = luni64/TeensyTimerTool
```

---

## Installation

1. Download / clone this repository.
2. Copy the `TeensyStepDriver` folder into your Arduino `libraries/` directory.
3. Restart the Arduino IDE.

---

## API Reference

### Constructor
```cpp
StepMotor(uint8_t stepPin, uint8_t dirPin, TimerGenerator periodicTmr, TimerGenerator oneshotTmr, bool invertDir = false);
```

### Initialisation
```cpp
void begin();
```
Call once in `setup()`.

### Configuration
```cpp
void setMaxSpeed(float stepsPerSec);      // cruising speed
void setAcceleration(float stepsPerSec2); // ramp rate (accel = decel)
```

### Motion Commands
```cpp
void moveTo(int32_t absolutePosition);   // absolute target in steps
void moveRelative(int32_t deltaSteps);   // relative move
void stop();                              // controlled decel → stop
void forceStop();                         // immediate stop (no ramp)
```

### Status
```cpp
bool    isRunning()        const;
int32_t currentPosition()  const;
int32_t targetPosition()   const;
```

### Homing / Position Reset
```cpp
void setCurrentPosition(int32_t position);
```

---

## Multi-Motor Usage

Each `StepMotor` instance automatically claims two QTIMER channel from
TeensyTimerTool.  The Teensy 4.1 provides four QTIMER modules (TMR1–TMR4),
each with 4 independent channels, for up to **8 simultaneous motors**.

---

## Wiring

```
Teensy 4.1 Pin (stepPin)  ──►  Driver STEP
Teensy 4.1 Pin (dirPin)   ──►  Driver DIR
Teensy 4.1 GND            ──►  Driver GND  (common ground is essential)
```

Active-high STEP pulse, ~3 µs wide — compatible with A4988, DRV8825, TMC2208,
TMC2209, and most other common stepper drivers.

---

## Tuning Constants

In `TeensyStepDriver.h` you can adjust three compile-time constants:

```cpp
static constexpr uint32_t MIN_PERIOD_US = 5;   // Minimum step-timer period in µs → maximum step frequency
static constexpr uint32_t PULSE_WIDTH_US = 3;  // STEP high-time in µs
static constexpr float MIN_SPEED_SPS = 1.0f; // Sets the floor for minimum speed
```

Reduce `MIN_PERIOD_US` with caution — the Cortex-M7 at 600 MHz can handle
very short ISR intervals, but your driver datasheet specifies a minimum
step-pulse period you must not violate.

---

## License

MIT — see LICENSE file.
