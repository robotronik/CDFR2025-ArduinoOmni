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

    // Compute the Euclidean distance error in meters (mm to m conversion).
    double errorDistance = sqrt(dx * dx + dy * dy) / 1000.0;

    // Compute the orientation error.
    // Difference in theta (in degrees) and then convert to radians.
    double errorTheta = (target.theta - current.theta) * (M_PI / 180.0);

    // Proportional gains (adjust these gains as needed).
    double kP_linear = 1.0;   // Gain for linear speed (m/s per meter error)
    double kP_angular = 1.0;  // Gain for angular speed (rad/s per rad error)
    
    // Compute commanded speeds.
    double commandedLinear = kP_linear * errorDistance;
    double commandedAngular = kP_angular * errorTheta;
    
    // Update each wheel with the computed linear and angular speed commands.
    wheelA.update(commandedLinear, commandedAngular);
    wheelB.update(commandedLinear, commandedAngular);
    wheelC.update(commandedLinear, commandedAngular);
}