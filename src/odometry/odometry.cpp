#include "odometry/odometry.h"
#include "odometry/OTOS.h"

Odometry::Odometry() : OTOS()
{
    // Constructor implementation
}
return_t Odometry::begin(I2CDevice &i2c)
{
    i2c.setAddress(kDefaultAddress);
    // Initialize the OTOS with the provided I2C device
    return OTOS::begin(i2c);
}

return_t Odometry::getPosition(position_t &pos){
    otos_pose2d_t pose;
    if (OTOS::getPosition(pose) != ret_OK) {
        return ret_FAIL;
    }
    pos = fromPose(pose);
    return ret_OK;
}

return_t Odometry::setOffset(position_t &pos){
    otos_pose2d_t pose = toPose(pos);
    return OTOS::setOffset(pose);
}

return_t Odometry::setPosition(position_t &pos){
    otos_pose2d_t pose = toPose(pos);
    return OTOS::setPosition(pose);
}

void Odometry::delayMs(uint32_t ms)
{
    delay(ms);
}