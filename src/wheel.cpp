#include <math.h>
#include "wheel.h"

Wheel::Wheel(double dist, double ang, AccelStepper& stepper) {
    distanceToCenter = dist;
    angle = ang;
    motor = stepper;
}


// Updates the speed of a single wheel given the commanded linear and angular velocities.
void Wheel::update(double commandedLinear, double commandedAngular) {
    // Convert the wheel's mounting angle to radians.
    double rad = angle * M_PI / 180.0;
    
    // Compute the wheel speed by projecting the commanded linear velocity and adding the contribution of rotational velocity.
    double wheelSpeed = -sin(rad) * commandedLinear + distanceToCenter * commandedAngular;
    setSpeed(wheelSpeed);
}

// Simulate sending a command to a motor controller by printing the speed.
void Wheel::setSpeed(double speed) {
    // Set the motor speed.
    motor.setSpeed(speed);
}