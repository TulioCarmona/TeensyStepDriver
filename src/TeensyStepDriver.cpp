/*
* Implementation of the StepMotor class.
*
* Two-timer pulse architecture
* ----------------------------
* 	_stepTimer(Periodic Timer)
*		Fires every 1/speed µs
*		Runs trapezoidal profile math
*		Pulls STEP pin HIGH
*		Arms _pulseTimer for PULSE_WIDTH_US later, then returns inmediately
*
*	_pulseTimer(OneShotTimer)
*		Fires exactly PULSE_WIDTH_US after the rising edge
*		Pulls STEP pin LOW
*
* Speed range
* -----------
*	Low end: MIN_SPEED_SPS = 1 steps/s  -> period = 1'000'000 µs
*	High end: MIN_PERIOD_US = 40 µs  -> speed = 25'000 steps/s
*/
#include "TeensyStepDriver.h"

/*StepMotor::StepMotor(uint8_t stepPin, uint8_t dirPin, TimerType stepTimerType,
					 TimerType pulseTimerType, bool invertDir)
	: _stepPin(stepPin),
	  _dirPin(dirPin),
	  _invertDir(invertDir),
	  _stepTimer(stepTimerType),
	  _pulseTimer(pulseTimerType)
{}*/

StepMotor::StepMotor(uint8_t stepPin, uint8_t dirPin, bool invertDir)
	: _stepPin(stepPin),
	  _dirPin(dirPin),
	  _invertDir(invertDir)
{}

//----Helper
static inline uint32_t speed2Period_us(float absSpd){
	if (absSpd < MIN_SPEED_SPS) absSpd = MIN_SPEED_SPS;
	uint32_t period = (uint32_t)(1'000'000.0f/absSpd);
	if(period < MIN_PERIOD_US) period = MIN_PERIOD_US; // Enforce hardware ceiling
	if(period > 1'000'000u) period = 1'000'000u; // Cap at 1 step/s
	return period;
}

//----Public API

void StepMotor::begin(){
	pinMode(_stepPin, OUTPUT);
	pinMode(_dirPin, OUTPUT);
	digitalWriteFast(_stepPin, LOW);
	digitalWriteFast(_dirPin, _invertDir ? HIGH : LOW);

	// Periodic step timer - long placeholder period; stoppped until a movement starts
	_stepTimer.begin([this]() {_stepISR(this); }, 1'000'000 /*us*/);
	_stepTimer.stop();
	// One-shot pulse timer — fires on demand to end each STEP pulse
	_pulseTimer.begin([this]() {_pulseISR(this); });

	_running = false;
	_stopRequest = false;
	_continuous = false;
	_targetSpeed = 0.0f;
	_currentSpeed = 0.0f;
	_currentPos = 0;
	_targetPos = 0;
}

void StepMotor::setMaxSpeed(float stepsPerSec){
	if (stepsPerSec > 0.0) _maxSpeed = stepsPerSec;
}

void StepMotor::setAccel(float accel){
	if (accel > 0.0) _accel = accel;
}

void StepMotor::moveTo(int32_t absPos){
	_continuous = false; // Cancels any active running
	_targetPos = absPos;
	_stopRequest = false;
	_startMove();
}

void StepMotor::moveRelative(int32_t detalSteps){
	moveTo(_currentPos + detalSteps);
}

void StepMotor::runSpeed(float spd){
	if (spd == 0.0f) {stop(); return; }

	// Clamp magnitude to valid range
	const float sign = (spd > 0.0f) ? 1.0f : -1.0f;
	float magnitude = fabsf(spd);
	if (magnitude < MIN_SPEED_SPS) magnitude = MIN_SPEED_SPS;
	if (magnitude > _maxSpeed) magnitude = _maxSpeed;
	const bool forward = (sign > 0);
	digitalWriteFast(_dirPin, (forward ^ _invertDir) ? LOW : HIGH); // XOR operation
	delayMicroseconds(1); // DIR setup: most drivers need >= 200 ns

	_targetSpeed = sign*magnitude; // Store the target cruise speed so the ISR can read it.
	_continuous = true;
	_stopRequest = false;

	if(!_running){ 
		// Cold-start: seed from minimum speed in the requested direction
		_currentSpeed = sign*MIN_SPEED_SPS;
		_running = true;
		//digitalWriteFast(_dirPin, (sign > 0.0f) ? LOW : HIGH);
		_stepTimer.setPeriod(speed2Period_us(fabsf(_currentSpeed)));
		_stepTimer.start();
	}
	// If already running the ISR picks up _continuous and _targetSpeed on its very next tick.
}

void StepMotor::stop(){
	_stopRequest = true; // ISR starts decel ramp on next tick
}

void StepMotor::forceStop(){
	_stepTimer.stop();
	_running = false;
	_stopRequest = false;
	_continuous = false;
	_currentSpeed = 0.0f;
	digitalWriteFast(_stepPin, LOW);
}

bool StepMotor::isRunning() const {return _running; }
int32_t StepMotor::currentPosition() const {return _currentPos; }
int32_t StepMotor::targetPosition() const {return _targetPos; }

void StepMotor::setCurrentPosition(int32_t position){
	if (_running) forceStop();
	_currentPos = position;
	_targetPos = position;
}

//----Internal helpers

void StepMotor::_startMove(){
	const int32_t delta = _targetPos - _currentPos;
	if (delta == 0) return;

	// Set direction pin with setup-time margin	
	const bool forward = (delta > 0);
	digitalWriteFast(_dirPin, (forward ^ _invertDir) ? LOW : HIGH); // XOR operation
	delayMicroseconds(1); // DIR setup: most drivers need >= 200 ns

	/*float minDistance = fabsf(_maxSpeed*_maxSpeed/_accel); // Min distance required to run a trapezoidal speed profile
	int32_t absDelta = abs(delta);
	if (absDelta < ((int)minDistance + 1)) {
		Serial.println("Distance to short.");
		Serial.printf("Min distance = %.4f, Delta = %d\n", minDistance, absDelta);
	}*/

	if(!_running){
		_currentSpeed = forward ? MIN_SPEED_SPS : -MIN_SPEED_SPS;
		_running = true;
		_stepTimer.setPeriod(speed2Period_us(fabsf(_currentSpeed)));
		_stepTimer.start();
	}
	// If already running: _targetPos was updated above; the ISR re-evaluates on its very next tick automatically.
}

//----ISR

// static
void StepMotor::_pulseISR(StepMotor* self){
	digitalWriteFast(self->_stepPin, LOW); // End of step pulse
}

// static
void StepMotor::_stepISR(StepMotor* self){
	// 1. Sanpshot state
	const int32_t currentPos = self->_currentPos;
	const int32_t targetPos = self->_targetPos;
	const float speed = self->_currentSpeed;
	bool stopRequest = self->_stopRequest;
	bool continuous = self->_continuous;

	// In continuous mode there is no target position — the only way to stop is via stop() / forceStop().
	//const int32_t remaining = (continuous || stopRequest) ? (stopRequest ? 0 : INT32_MAX) : (targetPos - currentPos);
	int32_t remaining = stopRequest ? 0 : (targetPos - currentPos);

	// 2. Check completion (positional moves only)
	if(!continuous && remaining == 0 /*&& fabsf(speed) <= MIN_SPEED_SPS*/){
		self->_running = false;
		self->_currentSpeed = 0.0f;
		self->_stopRequest = false;
		self->_continuous = false;
		self->_stepTimer.stop();
		return;
	}
	// Catch the stoprequest path reaching zero speed
	if(stopRequest && fabsf(speed) <= MIN_SPEED_SPS){
		self->_stepTimer.stop();
		 self->_running = false;
		 self->_currentSpeed = 0.0f;
		 self->_stopRequest = false;
		 self->_continuous = false;
		 return;
	}

	// 3. trapezoidal speed update
	float absVel = fabsf(speed);
	if(absVel < MIN_SPEED_SPS) absVel = MIN_SPEED_SPS;
	const float dt = 1.0f/absVel;
	 // In continuous mode: direction and cruise target come from _targetSpeed.
     // In positional mode: direction comes from remaining distance.
	float cruiseSpeed, dirSign;
	if(continuous){
		cruiseSpeed = fabsf(self->_targetSpeed);
		dirSign =  (self->_targetSpeed >= 0.0f) ? 1.0f : -1.0f;
	}
	else{
		cruiseSpeed = self->_maxSpeed;
		dirSign = (remaining >= 0) ? 1.0f : -1.0f;
	}

	// Braking distance: d = v^2 / 2a, same as accel distance
	const float dStop = (absVel*absVel) / (2.0f*self->_accel);
	const float absRemaining = continuous ? 1e9f : fabsf((float)remaining);

	float newSpeed;
	float temp_newSpeed = 0.0f;

	if (stopRequest || (!continuous && absRemaining <= dStop)){
		// Decelerate
		temp_newSpeed = absVel - self->_accel*dt;
		newSpeed = dirSign*(temp_newSpeed > MIN_SPEED_SPS ? temp_newSpeed : MIN_SPEED_SPS);
	}
	else if (absVel < cruiseSpeed){
		// Accelerate
		temp_newSpeed = absVel + self->_accel*dt;
		newSpeed = dirSign*(temp_newSpeed < cruiseSpeed ? temp_newSpeed : cruiseSpeed);
	}
	else{
		// Cruise mode
		newSpeed = dirSign*cruiseSpeed;
	}

	self->_currentSpeed = newSpeed;

	// 4. Emit step pulse (non-blocking)
	if(newSpeed > 0.0f) self->_currentPos++;
	else self->_currentPos--;
	
	digitalWriteFast(self->_stepPin, HIGH);
	self->_pulseTimer.trigger(PULSE_WIDTH_US); // Schedule the falling edge PULSE_WIDTH_US from now — no blocking wait.

	// 5. Reschedule step timer
	self->_stepTimer.setPeriod(speed2Period_us(fabsf(self->_currentSpeed)));

}
