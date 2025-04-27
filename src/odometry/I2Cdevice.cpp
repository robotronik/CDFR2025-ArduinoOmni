#include "odometry/I2Cdevice.h"

I2CDevice::I2CDevice(SoftWire &wirePort, uint8_t address)
{
    _wire = &wirePort;
    setAddress(address);
    setTimeout(1000);
    setRetries(3);
    setClockSpeed(400000);
    setTimeout(1000);
}

void I2CDevice::begin(){
    _wire->begin();
}

int I2CDevice::readRegister(uint8_t reg, uint8_t *data, uint8_t size, int &bytesRead)
{
    _wire->beginTransmission(_address);
    _wire->write(reg);
    int err = _wire->endTransmission();
    if (err != 0)
        return err;

    _wire->requestFrom(_address, size);
    bytesRead = _wire->available();
    for (int i = 0; i < bytesRead; i++)
        data[i] = _wire->read();

    return 0;
}

int I2CDevice::readRegister(uint8_t reg, uint8_t &data)
{
    int bytesRead;
    return readRegister(reg, &data, 1, bytesRead);
}

int I2CDevice::writeRegister(uint8_t reg, const uint8_t *data, uint8_t size)
{
    _wire->beginTransmission(_address);
    _wire->write(reg);
    for (int i = 0; i < size; i++)
        _wire->write(data[i]);
    return _wire->endTransmission();
}

int I2CDevice::writeRegister(uint8_t reg, const uint8_t &data)
{
    return writeRegister(reg, &data, 1);
}

void I2CDevice::setClockSpeed(uint32_t clockSpeed)
{
    _clockSpeed = clockSpeed;
    _wire->setClock(_clockSpeed);
}

// Timeout in ms
void I2CDevice::setTimeout(uint32_t i2cTimeout)
{
    _i2cTimeout = i2cTimeout;
    _wire->setTimeout(_i2cTimeout);
}

void I2CDevice::setAddress(uint8_t address)
{
    _address = address;
}