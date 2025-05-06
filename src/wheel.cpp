#include <math.h>
#include "wheel.h"

Wheel::Wheel(double dist, double ang, double diameter, ContinuousStepper<StepperDriver>* stepper) {
    distanceToCenter = dist;
    angle = ang;
    motor = stepper;
    diam = diameter;
}


// Updates the speed of a single wheel given the commanded linear and angular velocities.
// linear: linear speed of robot in mm/s
// theta: current error angle to target in degrees
// angular: angular speed of robot in degrees/s
void Wheel::update(double linear, double theta, double angular) {
    double rad = (angle + theta) * M_PI / 180.0;

    double circumference = M_PI * diam; // in mm
    double robotCircumference = 2 * M_PI * distanceToCenter; // in mm
    
    // Compute the wheel speed by projecting the commanded linear velocity and adding the contribution of rotational velocity.
    // The wheel speed is in mm/s.
    double wheelSpeed = linear * -sin(rad) + robotCircumference * angular / 360;
    setSpeed(wheelSpeed / circumference); // Convert to revolutions per second (rps)
}

// speed is in rps
void Wheel::setSpeed(double speed) {
    double stepsPerRevolution = 200;
    double microstepping = 8;
    double stepsPerSecond = speed * stepsPerRevolution * microstepping; // Convert rps to steps/s
    // Set the motor speed
    motor->spin(-stepsPerSecond);

    /*
    int steps = abs(stepsPerSecond/200);
    for (int i = 0; i < steps; i++) {
        // turn on and off the stepPin_
        digitalWrite(stepPin_, HIGH);
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();
        digitalWrite(stepPin_, LOW);        
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();
        _NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();_NOP();
    }
        */

    // rpm
    // int rpm = speed * 60;
    // motor->spin(rpm);
}