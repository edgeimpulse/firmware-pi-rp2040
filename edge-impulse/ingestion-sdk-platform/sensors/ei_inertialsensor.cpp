/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_inertialsensor.h"
#include <LSM6DSOX.h>

#include "ei_device_raspberry_rp2040.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f

static float imu_data[INERTIAL_AXIS_SAMPLED];

bool ei_inertial_sensor_init(void)
{
    uint8_t acc_type = IMU.begin();
    if (!acc_type) { 
        acc_type = IMU1.begin(); 
        IMU = IMU1;
    }

    if (!acc_type) {
        return false;
    }
    else if (acc_type == 1) {
        DEBUG_PRINT("Using LSM6DSOX\n");
    }
    else if (acc_type == 2) {
        DEBUG_PRINT("Using LSM6DS3\n");        
    }

    ei_add_sensor_to_fusion_list(inertial_sensor);
    return true;
}

float *ei_fusion_inertial_sensor_read_data(int n_samples)
{

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(imu_data[0], imu_data[1], imu_data[2]);

        imu_data[0] *= CONVERT_G_TO_MS2;
        imu_data[1] *= CONVERT_G_TO_MS2;
        imu_data[2] *= CONVERT_G_TO_MS2;
    }

    if (n_samples > 3 && IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(imu_data[3], imu_data[4], imu_data[5]);
    }

#ifdef DEBUG
    for (int i = 0; i < INERTIAL_AXIS_SAMPLED; i++) {
        ei_printf("%f ", imu_data[i]);
    }
    ei_printf("\n");
#endif
    return imu_data;
}
