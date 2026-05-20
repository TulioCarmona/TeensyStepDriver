#include "TeensyStepDriver.h"

StepMotor motor(2, 3, TMR1, TMR1);   // STEP=pin2, DIR=pin3, TMR1 chanel 1, TMR1 chanel 2 (automatic asignation)

void setup() {
    motor.begin();
    motor.setMaxSpeed(4000);     // steps / second
    motor.setAccel(8000); // steps / second²
    motor.moveTo(3200);          // non-blocking absolute move
}

void loop() {
    // motor running entirely in the background
}
