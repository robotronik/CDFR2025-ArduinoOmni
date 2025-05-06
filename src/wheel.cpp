#include <math.h>
#include "wheel.h"

Wheel::Wheel(double dist, double ang, double diameter, ContinuousStepper<StepperDriver, KhoihTicker<AVR_PWM>>* stepper) {
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
    if (abs(stepsPerSecond) < 250)
        stepsPerSecond = 0;
    motor->spin(-stepsPerSecond);

    Serial.print("speed: ");
    Serial.println(stepsPerSecond);

    // rpm
    // int rpm = speed * 60;
    // motor->spin(rpm);
}