#pragma once
#include <AccelStepper.h>

// A simple class that represents an individual wheel.
class Wheel {
public:
    AccelStepper motor;

    // Constructor to initialize wheel geometry.
    Wheel(double dist, double ang, AccelStepper& stepper);
    void update(double commandedLinear, double commandedAngular);

private:
    // The distance from the center of the robot to the wheel (in mm)
    double distanceToCenter;
    // The mounting angle of the wheel relative to the robot's frame (in degrees)
    double angle;
    // Simulate sending a command to a motor controller by printing the speed.
    void setSpeed(double speed);
};