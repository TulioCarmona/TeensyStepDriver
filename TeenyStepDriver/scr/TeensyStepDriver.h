/**
 * TeensyStepDriver.h
 * ==================
 * Non-blocking stepper-motor driver library for Teensy 4.1.
 *
 * Uses TeensyTimerTool (TMR / QTIMER hardware timers) for precise,
 * low-jitter pulse generation.  Each StepMotor instance owns TWO
 * independent hardware-timer channels:
 *   • _stepTimer  — periodic, fires every 1/speed µs to advance the profile
 *   • _pulseTimer — one-shot, fires PULSE_WIDTH_US after the STEP pin goes
 *                   HIGH to pull it LOW again (non-blocking pulse, safe at
 *                   25 000 steps/s)
 *
 * Speed range: ~0.5 steps/s  →  25 000 steps/s  (40 µs period minimum)
 * Motion profile: trapezoidal  (accel → cruise → decel)
 *
 * Wiring assumption
 * -----------------
 *   Driver STEP pin  ←→  stepPin   (any digital-output capable pin)
 *   Driver DIR  pin  ←→  dirPin    (any digital-output capable pin)
 *   Active-high STEP pulse, PULSE_WIDTH_US wide (default 3 µs).
 *
 * Dependencies
 * ------------
 *   - TeensyTimerTool  https://github.com/luni64/TeensyTimerTool
 *     Install via Arduino Library Manager or platformio.
 */
#pragma once

#include <Arduino.h>
#include <TeensyTimerTool.h>

using namespace TeensyTimerTool;

//-----Tuneable constants
/* Minimum step-timer period in µs → maximum step frequency.
   40 µs = 25 000 steps/s. Reduce only if your driver datasheet allows it.*/
static constexpr uint32_t MIN_PERIOD_US = 40;

/* Step pulse high-time in µs
   3 µs satisfies A4988 (1 µs min), DRV8825 (1.9 µs min), TMC22xx (≥100 ns).*/
static constexpr uint32_t PULSE_WIDTH_US = 3;

/* Minimum meaningful spped in steps/s
   At 1 step/s the timer period is 1 000 000 µs (1 s) — fine for uint32_t.
   For lower speed change the variable type of the period. */
static constexpr float MIN_SPEED_SPS = 1.0f;

class StepMotor{

public:
	/* @param stepTimer  TeensyTimerTool timer module for the step cadence.
								Use TMR1, TMR2, TMR3 or TMR4.
		@param pulseTimer TeensyTimerTool timer module for the step pulse width.
								Can be the same module as stepTimer - TeensyTimerToool
								picks the next free chanel automatically.
		2 motors can share one module safely since each module has 4 chanels and
		each motor uses 2.
		For Teensy 4.1 with 4 modules, it is possible to control up to 8 motors.
		@param invertDir  Set true if your driver/motor needs the direction logic
								flipped.
	*/
	/*explicit StepMotor(uint8_t stepPin, uint8_t dirPin, TimerType stepTimerType,
							 TimerType pulseTimerType, bool invertDir = false); // explicit: avoids accidental or confusing conversions.*/
	explicit StepMotor(uint8_t stepPin, uint8_t dirPin, bool invertDir = false); // explicit: avoids accidental or confusing conversions.

	void begin();
	void setMaxSpeed(float stepsPerSec);
	void setAccel(float accel);
	//----Motion Comands
	//	Position movement
	void moveTo(int32_t absPos); // Move to an absolute position (in steps from the home/zero point)
	void moveRelative(int32_t deltaSteps); // Move by a relative number of steps from the current position.
	//	Constant speed movement
	void runSpeed(float spd);
	//  Stop motor
	void stop(); // Perform a controlled deceleration to a stop, then stay idle.
	void forceStop(); // Immediate stop — no deceleration ramp.  Use only for emergencies.
	//---Motor status
	bool isRunning() const;
	int32_t currentPosition() const; // Reurns current position in steps
	int32_t targetPosition() const; // Returns target position of the current (or last) move.
	//---Position management
	void setCurrentPosition(int32_t position); // Override the internal step counter without moving the motor.

private:
	// Pin config
	const uint8_t _stepPin;
	const uint8_t _dirPin;
	const bool _invertDir;

	// Motion parameters (Modify by user via functions)
	float _maxSpeed = 1000.0f; // steps/s
	float _accel = 5000.0f; // steps/s²

	// Position tracking
	volatile int32_t _currentPos = 0; // High capacity variable for position tarcking in velocity mode
	volatile int32_t _targetPos = 0;

	// Trapezoidal-profile state (ISR-owned)
	volatile float _currentSpeed = 0.0f; // steps/s, signed
	volatile float _targetSpeed = 0.0f; // Cruise speed for runSpeed() mode, signed
	volatile bool _running = false;
	volatile bool _stopRequest = false;
	volatile bool _continuous = false; // true while in runSpeed() mode

	/* TeensyTimerTool timers:
       _stepTimer  — periodic, sets cadence of the motion profile (1 per step)
       _pulseTimer — one-shot, pulls STEP pin LOW after PULSE_WIDTH_US
                     This decouples the pulse width from the step period,
                     making high speeds (25 000 steps/s) safe and clean.*/
    PeriodicTimer _stepTimer;
    OneShotTimer _pulseTimer;

    // ISR (Interrupt Service Routine) callbacks (static, receive intance pointer)
    // Class level functions
    static void _stepISR(StepMotor* self); 
    static void _pulseISR(StepMotor* self); // just pulls STEP LOW

    // Internal helpers (called from ISR only)
    //void _stepForward();
    //void _stepBackward();
    //void _updateTimerPeriod(float stepsPerSec);
    void _startMove();

};