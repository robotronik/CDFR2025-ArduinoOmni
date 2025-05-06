#include "control.h"
#include <math.h>

// Updates the speeds of three wheels based on the current and target positions.
// The controller computes the distance and orientation error and then determines
// commanded speeds using proportional gains.
void updateWheels(const position_t& current, const position_t& target,
                  Wheel& wheelA, Wheel& wheelB, Wheel& wheelC)
{
    // Compute differences in position.
    double dx = target.x - current.x; // in mm
    double dy = target.y - current.y; // in mm

    double errorDistance = sqrt(dx * dx + dy * dy);
    double errorTheta = target.a - current.a;

    double angleToTarget = current.a - atan2(dy, dx) * 180 / M_PI; // in degrees

    double kP_linear = 1.0;   // Gain for linear speed (mm/s per mm error)
    double kP_angular = 1.0;  // Gain for angular speed (deg/s per deg error)
    
    // Compute commanded speeds.
    double commandedLinear = kP_linear * errorDistance; // mm/s
    double commandedAngular = kP_angular * errorTheta;  // degs/s

    double maxLinearSpeed = 1000; // mm/s
    double maxAngularSpeed = 180;  // deg/s

    commandedLinear = fmin(fmax(commandedLinear, -maxLinearSpeed), maxLinearSpeed); // Limit linear speed
    commandedAngular = fmin(fmax(commandedAngular, -maxAngularSpeed), maxAngularSpeed); // Limit angular speed
    
    // Update each wheel with the computed linear and angular speed commands.
    wheelA.update(commandedLinear, angleToTarget, commandedAngular);
    wheelB.update(commandedLinear, angleToTarget, commandedAngular);
    wheelC.update(commandedLinear, angleToTarget, commandedAngular);

    /*
    Serial.print("commandedLinear");
    Serial.print(commandedLinear);
    Serial.print("commandedAngular");
    Serial.print(commandedAngular);
    */
}