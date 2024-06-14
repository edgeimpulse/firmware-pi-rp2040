  /*
    This file is part of the Arduino_LIS2DH12 library.
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

  #include <Wire.h>
  #include <math.h>
  #include <stdint.h>
  #include <stdlib.h>
  #include <string.h>
  #include <stdio.h>

  #define MOTION_SENSOR_LIS2DH12  0x33
  #define LIS2DH12_ADDRESS        0x19U

  class LIS2DH12Class {
    public:
      LIS2DH12Class(TwoWire& wire, uint8_t slaveAddress);
      ~LIS2DH12Class();

      int begin();
      void end();

      // Accelerometer
      int readAcceleration(float& x, float& y, float& z);
      float accelerationSampleRate();
      int accelerationAvailable();

      // Temperature
      int readTemperature(int & temperature_deg);
      int temperatureAvailable();

    private:
      int readRegister(uint8_t address);
      int readRegisters(uint8_t address, uint8_t* data, size_t length);
      int writeRegister(uint8_t address, uint8_t value);


    private:
      TwoWire* _wire;
      uint8_t _slaveAddress;
  };

  extern LIS2DH12Class MOTION;
