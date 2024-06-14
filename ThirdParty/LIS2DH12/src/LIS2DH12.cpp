/* Include ----------------------------------------------------------------- */
#include "LIS2DH12.h"

/* Constant defines -------------------------------------------------------- */
// Registers
#define LIS2DH12_STATUS_REG_AUX        0x07U
#define LIS2DH12_OUT_TEMP_L            0x0CU
#define LIS2DH12_WHO_AM_I_REG          0X0FU
#define LIS2DH12_TEMP_CFG_REG          0x1FU
#define LIS2DH12_CTRL_REG1             0x20U
#define LIS2DH12_CTRL_REG4             0x23U
#define LIS2DH12_STATUS_REG            0x27U
#define LIS2DH12_OUT_X_L               0x28U

// Register masks
#define LIS2DH12_STATUS_REG_ZYXOR_MASK   0x80u
#define LIS2DH12_STATUS_REG_AUX_TDA_MASK 0x04u

LIS2DH12Class::LIS2DH12Class(TwoWire& wire, uint8_t slaveAddress) :
    _wire(&wire),
    _slaveAddress(slaveAddress)
{
}

LIS2DH12Class::~LIS2DH12Class() { /* Nothing to do */ }

int LIS2DH12Class::begin()
{
    int8_t type;
    uint8_t ret;
    _wire->begin();

    type = readRegister(LIS2DH12_WHO_AM_I_REG);
    if (type != MOTION_SENSOR_LIS2DH12) {
        return 0;
    }

    // Data rate: 100Hz, enable all axis, high-resolution mode
    writeRegister(LIS2DH12_CTRL_REG1, 0x57);
    
    //Enable block data update, full-scale 2g, hr 1
    writeRegister(LIS2DH12_CTRL_REG4, 0x88);

    //Enable temperature sensor
    writeRegister(LIS2DH12_TEMP_CFG_REG, 0xC0);

    return type;
}

void LIS2DH12Class::end()
{
    _wire->end();
}

int LIS2DH12Class::readAcceleration(float& x, float& y, float& z)
{
    int16_t data[3];

    if (!readRegisters(LIS2DH12_OUT_X_L, (uint8_t*)data, sizeof(data))) {
        x = NAN;
        y = NAN;
        z = NAN;

        return 0;
    }

    /* First convert fs2 hr to mg and then to g */
    x = ((data[0] / 16.0f) * 1.0f) / 1000.0f;
    y = ((data[1] / 16.0f) * 1.0f) / 1000.0f;
    z = ((data[2] / 16.0f) * 1.0f) / 1000.0f;

    return 1;
}

int LIS2DH12Class::accelerationAvailable()
{
    uint8_t data;
    if (readRegisters(LIS2DH12_STATUS_REG, &data, 1) != 1) {
        return 0;
    }

    return (data & LIS2DH12_STATUS_REG_ZYXOR_MASK) ? 1 : 0;
}

float LIS2DH12Class::accelerationSampleRate()
{
    return 100.0F;
}

int LIS2DH12Class::readTemperature(int & temperature_deg)
{
    /* Read the raw temperature from the sensor. */
    int16_t temperature_raw = 0;

    if (readRegisters(LIS2DH12_OUT_TEMP_L, reinterpret_cast<uint8_t*>(&temperature_raw), sizeof(temperature_raw)) != 1) {
        return 0;
    }

    temperature_deg = (((float)temperature_raw / 64.0f ) / 4.0f ) + 25.0f;

    return 1;
}

int LIS2DH12Class::temperatureAvailable()
{
    uint8_t data;

    if (readRegisters(LIS2DH12_STATUS_REG_AUX, &data, 1) != 1) {
        return 0;
    }
    return (data & LIS2DH12_STATUS_REG_AUX_TDA_MASK) ? 1 : 0;
}

int LIS2DH12Class::readRegister(uint8_t address)
{
uint8_t value;

if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
}

return value;
}

int LIS2DH12Class::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
    if (length > 1)
    {
        //For multi byte reads we must set the first bit to 1
        address |= 0x80;
    }
    
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);

    if (_wire->endTransmission(false) != 0) {
        return -1;
    }

    if (_wire->requestFrom(_slaveAddress, length) != length) {
        return 0;
    }

    for (size_t i = 0; i < length; i++) {
        *data++ = _wire->read();
    }
    return 1;
}

int LIS2DH12Class::writeRegister(uint8_t address, uint8_t value)
{
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);
    _wire->write(value);
    if (_wire->endTransmission() != 0) {
        return 0;
    }
    return 1;
}

LIS2DH12Class MOTION(Wire, LIS2DH12_ADDRESS);
