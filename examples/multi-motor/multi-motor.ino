#include "TeensyStepDriver.h"

StepMotor motor1(2, 3, TMR1, TMR1, true);
StepMotor motor2(4, 5, TMR2, TMR2);
StepMotor motor3(6, 7, TMR3, TMR3);

void setup() {
    motor1.begin();  motor1.setMaxSpeed(3200); motor1.setAcceleration(6400);
    motor2.begin();  motor2.setMaxSpeed(8000); motor2.setAcceleration(16000);
    motor3.begin();  motor3.setMaxSpeed(800);  motor3.setAcceleration(1600);

    // All three start simultaneously, each on its own hardware timer
    motor1.moveTo(6400);
    motor2.moveTo(-3200);
    motor3.moveTo(1600);
}

void loop() {
    // motor running entirely in the background
}
