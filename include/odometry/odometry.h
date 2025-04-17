#pragma once
#include "I2CDevice.h"
#include "OTOS.h"
#include "common/structs.h"

/// Arduino class for the SparkFun Qwiic Optical Tracking Odometry Sensor
/// (OTOS)
class Odometry : private OTOS
{
  public:
    Odometry();

    // Returns true if the device is connected and ready to use
    return_t begin(I2CDevice &i2c);
    return_t getPosition(position_t &pos);
    return_t setOffset(position_t &pos);
    return_t setPosition(position_t &pos);

  private:
    position_t fromPose(const otos_pose2d_t &pose)
    {
        position_t pos;
        pos.x = pose.x / 1000.0;
        pos.y = pose.y / 1000.0;
        pos.theta = pose.h;
        return pos;
    }
    otos_pose2d_t toPose(const position_t &pos)
    {
        otos_pose2d_t pose;
        pose.x = pos.x * 1000.0;
        pose.y = pos.y * 1000.0;
        pose.h = pos.theta;
        return pose;
    }

  protected:
    void delayMs(uint32_t ms)
    {
        delay(ms);
    }
};