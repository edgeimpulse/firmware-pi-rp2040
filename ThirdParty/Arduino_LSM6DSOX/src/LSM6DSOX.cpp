/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "LSM6DSOX.h"

#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_WHO_AM_I_REG       0X0F

#define  LSM6DS3_WHO_AM_I           0x6A    //Grove LSM6DS3 module
#define  LSM6DOX_WHO_AM_I           0x6C

#define LSM6DSOX_CTRL1_XL           0X10
#define LSM6DSOX_CTRL2_G            0X11

#define LSM6DSOX_STATUS_REG         0X1E

#define LSM6DSOX_CTRL6_C            0X15
#define LSM6DSOX_CTRL7_G            0X16
#define LSM6DSOX_CTRL8_XL           0X17

#define LSM6DSOX_OUT_TEMP_L         0X20
#define LSM6DSOX_OUT_TEMP_H         0X21

#define LSM6DSOX_OUTX_L_G           0X22
#define LSM6DSOX_OUTX_H_G           0X23
#define LSM6DSOX_OUTY_L_G           0X24
#define LSM6DSOX_OUTY_H_G           0X25
#define LSM6DSOX_OUTZ_L_G           0X26
#define LSM6DSOX_OUTZ_H_G           0X27

#define LSM6DSOX_OUTX_L_XL          0X28
#define LSM6DSOX_OUTX_H_XL          0X29
#define LSM6DSOX_OUTY_L_XL          0X2A
#define LSM6DSOX_OUTY_H_XL          0X2B
#define LSM6DSOX_OUTZ_L_XL          0X2C
#define LSM6DSOX_OUTZ_H_XL          0X2D


LSM6DSOXClass::LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress) :
  _wire(&wire),
  _slaveAddress(slaveAddress)
{
}

LSM6DSOXClass::~LSM6DSOXClass()
{
}

int LSM6DSOXClass::begin()
{
  uint8_t type;
  _wire->begin();

  if (readRegister(LSM6DSOX_WHO_AM_I_REG) == LSM6DOX_WHO_AM_I) {
    type = 1;
  }
  else if (readRegister(LSM6DSOX_WHO_AM_I_REG) == LSM6DS3_WHO_AM_I) {
    type = 2;
  }
  else {
    end();
    return 0;
  }

  //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
  writeRegister(LSM6DSOX_CTRL2_G, 0x4C);

  // Set the Accelerometer control register to work at 104 Hz, 2g,and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DSOX's datasheet)
  writeRegister(LSM6DSOX_CTRL1_XL, 0x42);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DSOX_CTRL7_G, 0x00);

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DSOX_CTRL8_XL, 0x09);

  return type;
}

void LSM6DSOXClass::end()
{
  writeRegister(LSM6DSOX_CTRL2_G, 0x00);
  writeRegister(LSM6DSOX_CTRL1_XL, 0x00);
  _wire->end();
}

int LSM6DSOXClass::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2.0 / 32768.0;
  y = data[1] * 2.0 / 32768.0;
  z = data[2] * 2.0 / 32768.0;

  return 1;
}

int LSM6DSOXClass::accelerationAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::accelerationSampleRate()
{
  return 104.0F;
}

int LSM6DSOXClass::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}

int LSM6DSOXClass::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

int LSM6DSOXClass::readTemperature(int & temperature_deg)
{
  /* Read the raw temperature from the sensor. */
  int16_t temperature_raw = 0;

  if (readRegisters(LSM6DSOX_OUT_TEMP_L, reinterpret_cast<uint8_t*>(&temperature_raw), sizeof(temperature_raw)) != 1) {
    return 0;
  }

  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  temperature_deg = (static_cast<int>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

  return 1;
}

int LSM6DSOXClass::temperatureAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::gyroscopeSampleRate()
{
  return 104.0F;
}

int LSM6DSOXClass::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int LSM6DSOXClass::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
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

int LSM6DSOXClass::writeRegister(uint8_t address, uint8_t value)
{
  _wire->beginTransmission(_slaveAddress);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return 0;
  }
  return 1;
}

LSM6DSOXClass IMU(Wire, LSM6DSOX_ADDRESS);
LSM6DSOXClass IMU1(Wire1, LSM6DSOX_ADDRESS);