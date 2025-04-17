#pragma once

#include "wheel.h"
#include "common/structs.h"

// Updates the speeds of three wheels based on the current and target positions.
// The controller computes the distance and orientation error and then determines
// commanded speeds using proportional gains.
void updateWheels(const position_t& current, const position_t& target,
                  Wheel& wheelA, Wheel& wheelB, Wheel& wheelC);