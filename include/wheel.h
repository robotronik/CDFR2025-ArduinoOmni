#pragma once
#include <ContinuousStepper.h>
#include <ContinuousStepper/Tickers/TimerOne.hpp>

// A simple class that represents an individual wheel.
class Wheel {
public:
    ContinuousStepper<StepperDriver>* motor;

    // Constructor to initialize wheel geometry.
    Wheel(double dist, double ang, double diameter, ContinuousStepper<StepperDriver>* stepper);

    void update(double linear, double theta, double angular);
    void setSpeed(double speed);

private:
    // The distance from the center of the robot to the wheel (in mm)
    double distanceToCenter;
    // The mounting angle of the wheel relative to the robot's frame (in degrees)
    double angle;
    // The wheel's diameter (in mm)
    double diam;
};